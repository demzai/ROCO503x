####################################################
################### DEPENDENCIES ###################
####################################################
import helper_functions
import graph as gr
import filter as fl
import pyqtgraph.Qt
import time
import dead_reckoning as dr
import sys
import signal
import os as opsys
import argument_parser
import pickle

clp = argument_parser.commandline_argument_parser()
args = clp.parser.parse_args()
argument_parser.validate(args)

####################################################
################# GLOBAL CONSTANTS #################
####################################################
inputType = args.dataSource
print("Input type:", inputType)
if inputType == "live":
    import Spatial_simple_cl as spatialC
# fileLocale = "UpDown2.txt"
fileLocale = "IMU_Stationary.txt"
# fileLocale = args.fileLocation
print("File location:", fileLocale)

sleepTime = 0.0001
graphWindow = gr.newWindow("Graphs", 640, 480)
graphChart = gr.newGraph(graphWindow, "Test")
graphAccX = gr.newPlot(graphChart, [], [], 'r', None, None, 3, 'o')
graphAccY = gr.newPlot(graphChart, [], [], 'g', None, None, 3, 'o')
graphAccZ = gr.newPlot(graphChart, [], [], 'b', None, None, 3, 'o')
updateEvery = 10
cutoffFrequency = [20, 5]  # [Accel Low Pass, Gyro High Pass]

####################################################
################# GLOBAL VARIABLES #################
####################################################
dataFile = None
listRaw = []
listRawDR = []
listFiltered = []
listFilteredDR = []
count = 0
calibrationCount = 0
dcm = helper_functions.np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])  # @todo Incorrect initial orientation!


####################################################
############## CALIBRATION CONSTANTS ###############
####################################################
calGyroOffset = [-0.35, -0.3, -0.45]
calAccelOffset = [0.0, 0.0, 0.0]
calAccelScale = [1.0, 1.0, 1.0]


####################################################
################# CUSTOM FUNCTIONS #################
####################################################
# Apply scaling and offset factors
def apply_calibration(data):
    temp = data * 1  # Dealing with pointers?!?!
    for i in range(0, 3):
        # Offset then scale accelerometer values
        temp[i + 1] -= calAccelOffset[i]
        temp[i + 1] /= calAccelScale[i]

        # Offset the gyroscope values
        temp[i + 4] -= calGyroOffset[i]
    return temp


# Extracts gyroscope calibration parameters from a hard-coded text file
def read_calibration_file_gyro():
    file = open('calibrationGyro.txt', 'rb')
    calibration_variables = pickle.load(file) # todo convert string to float?!
    file.close()
    return calibration_variables


# Determines the gyroscope offset parameters
def set_calibration_gyro(data):
    global start, calibrationCount, args
    time_elapsed = time.time() - start
    print("start time", args.startTime)
    print("end time", args.startTime + args.durationTime)
    if time_elapsed > args.startTime:
        calibrationCount += 1
        calGyroOffset[0] += data[4]
        calGyroOffset[1] += data[5]
        calGyroOffset[2] += data[6]
    if time_elapsed > args.startTime + args.durationTime:
        calGyroOffset[0] = calGyroOffset[0] / calibrationCount
        calGyroOffset[1] = calGyroOffset[1] / calibrationCount
        calGyroOffset[2] = calGyroOffset[2] / calibrationCount
        print("Vx", calGyroOffset[0])
        print("Vy", calGyroOffset[1])
        print("Vz", calGyroOffset[2])

        # @todo word better - collects then eventually sets
        pickle.dump(calGyroOffset, open("calibrationGyro.txt", "wb"))
        close_nicely()


# Extracts accelerometer calibration parameters from a hard-coded text file
def read_calibration_file_accel():
    file = open('calibrationAccel.txt', 'rb')
    calibration_variables = pickle.load(file)
    file.close()
    return calibration_variables


# Determines the accelerometer offset parameters
def set_calibration_accel_offset(calibration_variables):
    offsets = [0.0, 0.0, 0.0]
    for i in range(0, 3):
        offsets[i] = (calibration_variables[i] + calibration_variables[i + 3]) / 2
    return offsets


# Determines the accelerometer scaling parameters
def set_calibration_accel_scale(offset, data):
    output = [0.0, 0.0, 0.0]
    for i in range(0, 3):
        output[i] = data[i] - offset[i]
    return output


# Retrieve the next input of raw data
def get_next_data():
    global inputType, imuObj, calGyroOffset, calAccelOffset, calAccelScale
    data = None

    # If reading from a file:
    if inputType == "file":
        # Considered an array of chars, so [:-1] removes the last character
        data = dataFile.readline()[:-1]
        try:
            data = [float(i) for i in data.split(",")]
        except:
            data = None
        # Calibrate the data
        if not args.calibrate:
            data = apply_calibration(data)

    # Else if streaming from the IMU directly
    elif inputType == "live":
        while not imuObj.dataReady:
            pass
        if imuObj.dataReady:
            data = imuObj.getData()
            # Calibrate the data
            if not args.calibrate:
                data = apply_calibration(data)

    # If not then you don't know what you want
    else:
        print("Error: invalid input type specified. Please set either \"file\" or \"live\"")

    # Test whether data is being collected for calibration purposes
    if args.calibrate:
        set_calibration_gyro(data)
    print(data)
    return data


# Destroy the IMU object properly on exit
def close_nicely():
    global imuObj
    # Close down sockets before exiting
    if inputType == "live":
        imuObj.stopIMU()
    opsys.system('stty sane')
    sys.exit(0)


# If CTRL+C is passed, then close the program
def handle_ctrl_c(signal, frame):
    close_nicely()
    # sys.exit(130) # 130 is standard exit code for ctrl-c


####################################################
############ INITIALIZATION FUNCTION(S) ############
####################################################
def init():
    # Initialize start time variable, IMU object and any input files
    global start, calibration_variables_gyro, calibration_variables_accel, dataFile, fileLocale, imuObj
    global calGyroOffset, calAccelScale, calAccelOffset

    # If we're not in calibration mode, apply the last calibration values
    if not args.calibrate:
        calGyroOffset = read_calibration_file_gyro()
        calibration_variables_accel = read_calibration_file_accel()
        calAccelOffset = set_calibration_accel_offset(calibration_variables_accel)  # generate offsets
        calAccelScale = set_calibration_accel_scale(calAccelOffset, calibration_variables_accel)

        print("Applying calibration from file.")
        print("If you wish to re-run calibration, please call this program with -c or --calibrate.")
        print("Please press enter when ready.")

    # If we are in calibration mode...
    else:
        duration = args.durationTime
        print("Entering calibration mode. Please try and isolate the IMU from noise and vibrations.")
        print("Calibration data will be saved to calibration.txt in the local directory.")
        print("Calibration will be applied the next time you run this program without the calibration flag.")
        if duration == 1:
            print("Calibration will start at t =", args.startTime, "seconds and finish at t =",
                  args.startTime + args.durationTime, "seconds, covering a duration of", duration, "second.")
        else:
            print("Calibration will start at t =", args.startTime, "seconds and finish at t =",
                  args.startTime + args.durationTime, "seconds, covering a duration of", duration, "seconds.")
        print("Please press enter when ready.")
        # Press enter to continue
        sys.stdin.read(1)

    # This line tells python to call the 'handle_ctrl_c' function if control+c is pressed (allows us to exit nicely)
    signal.signal(signal.SIGINT, handle_ctrl_c)

    ###
    # Open the data file or connect to the IMU
    ###

    if inputType == "file":
        dataFile = open(fileLocale, "r")
    elif inputType == "live":
        imuObj = spatialC.IMU('some')
        time.sleep(1)
    # Remember start time so that we can calculate durations later
    start = time.time()
    ###
    # Initialise the lists
    ###
    global listRaw, listFiltered
    # [time, ax,  ay,  az,  gx,  gy,  gz]
    listRaw.append(get_next_data())

    # [time, ax,  ay,  az,  vx,  vy,  vz,  px,  py,  pz,
    #   gx,  gy,  gz,  tx,  ty,  tz,  qw,  qx,  qy,  qz]
    tempList = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
    listRawDR.append(dr.doDeadReckoning(tempList, listRaw[0]))
    listRawDR[0] = [listRawDR[0][0] - 0.004] + \
                   listRawDR[0][1:4] + [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] + \
                   listRawDR[0][4:7] + [0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]

    listFiltered.append(listRaw[0])
    listFilteredDR.append(listRawDR[0])

    return


####################################################
##################### MAIN CODE ####################
####################################################
def main():
    global listRaw, count, sleepTime, inputType, listFiltered
    count += 1

    # Get raw data
    next_data = get_next_data()
    listRaw.append(next_data)
    listRaw = helper_functions.limitSize(listRaw)

    # Get filtered data
    if next_data is not None:
        global listRawDR, listFilteredDR, listFiltered
        listFiltered.append([listRaw[-1][0]] +
                            fl.filterData(listRaw, [1, 2, 3], ['butter', 'low', cutoffFrequency[0], 4]) +
                            fl.filterData(listRaw, [4, 5, 6], ['butter', 'high', cutoffFrequency[1], 4])
                            )

        listFiltered = helper_functions.limitSize(listFiltered)

        # Get dead reckoned data
        listRawDR.append(dr.doDeadReckoning(listRawDR[-1], listRaw[-1]))
        listRawDR = helper_functions.limitSize(listRawDR)
        listFilteredDR.append(dr.doDeadReckoning(listFilteredDR[-1], listFiltered[-1]))
        listFilteredDR = helper_functions.limitSize(listFilteredDR)

        # listRawDR:
        #  [time, ax,  ay,  az,  vx,  vy,  vz,  px,  py,  pz,
        #   gx,  gy,  gz,  tx,  ty,  tz,  qw,  qx,  qy,  qz]
        """
        set triplet to:
        acceleration = 0
        velocity = 1
        position = 2
        angular velocity = 3
        angular position = 4
        quaternion = 5
        """
    # Plot data if appropriate
    if args.calibrate:
        triplet = 3
    else:
        triplet = args.triplet

    # Plot data if appropriate
    if count == updateEvery:
        timeCol = helper_functions.getCol(listFilteredDR, 0)
        gr.updatePlot(graphAccX, helper_functions.getCol(listFilteredDR, 1 + 3 * triplet), timeCol)
        gr.updatePlot(graphAccY, helper_functions.getCol(listFilteredDR, 2 + 3 * triplet), timeCol)
        gr.updatePlot(graphAccZ, helper_functions.getCol(listFilteredDR, 3 + 3 * triplet), timeCol)
    count = count % updateEvery
    if inputType == 'file':
        time.sleep(sleepTime)
    if inputType == 'live':
        time.sleep(sleepTime)
        # gr.newPlot(graphChart, getCol(listRaw, 1)[-3:-1], getCol(listRaw, 0)[-3:-1], 'g', 'r', 'b', 5, 'o')


# Cannot put into a function because QtCore.QTimer() is a dick
# Initialize the system
init()

# Update the graph data
timer = pyqtgraph.Qt.QtCore.QTimer()
timer.timeout.connect(main)
timer.start(0)
