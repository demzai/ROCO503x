####################################################
################### DEPENDENCIES ###################
####################################################
import numpy as np
import graph as gr
import filter as fl
from pyqtgraph.Qt import QtCore
import time
import dead_reckoning as dr
import sys, signal, os
import argument_parser
import pickle
clp = argument_parser.commandline_argument_parser()
args = clp.parser.parse_args()
argument_parser.validate(args)



####################################################
################# GLOBAL CONSTANTS #################
####################################################
inputType = args.dataSource
print "Input type:",inputType
if (inputType == "live"):
    import Spatial_simple_cl as spatialC
#fileLocale = "IMU_Stationary.txt"
fileLocale = args.fileLocation
print "File location:",fileLocale

sleepTime = 0.0001
numSamplesMax = 100
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
calibCount = 0
dcm = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])  # @todo Incorrect initial orientation!

####################################################
############## CALIBRATION CONSTANTS ###############
####################################################
calGyroOffset  = [-0.35, -0.3, -0.45]
calAccelOffset = [0.0, 0.0, 0.0]
calAccelScale  = [1.0, 1.0, 1.0]
calV = [0,0,0]


####################################################
################# CUSTOM FUNCTIONS #################
####################################################
#Apply scaling and offset factors
# def demiApplyCalibration(data, calGyroOffset, calAccelOffset, calAccelScale):
#     temp = data*1
#     for i in range(0, 3):
#         # Offset then scale accelerometer values
#         temp[i+1] -= calAccelOffset[i]
#         temp[i+1] /= calAccelScale[i]
#
#         # Offset the gyroscope values
#         temp[i+4] -= calGyroOffset[i]
#     return temp

def demiApplyCalibration(data, calGyroOffset, calAccelOffset, calAccelScale):
    temp = data*1
    for i in range(0, 3):
        # Offset then scale accelerometer values
        data[i+1] -= calAccelOffset[i]
        data[i+1] /= calAccelScale[i]

        # Offset the gyroscope values
        data[i+4] -= calGyroOffset[i]
    return data


def read_calibration_file():
    file = open('calibration.txt', 'rb')
    calibration_variables = pickle.load(file)
    file.close()
    return calibration_variables




def collect_calibration(data):
    global start, calibCount, calV, args
    time_esapsed = time.time() - start
    print "start time", args.startTime
    print "end time", args.startTime + args.durationTime
    if (time_esapsed > args.startTime):
        calibCount += 1
        calGyroOffset[0] += data[4]
        calGyroOffset[1] += data[5]
        calGyroOffset[2] += data[6]
    if (time_esapsed > args.startTime + args.durationTime):
        calGyroOffset[0] = calGyroOffset[0] / calibCount
        calGyroOffset[1] = calGyroOffset[1] / calibCount
        calGyroOffset[2] = calGyroOffset[2] / calibCount
        print "Vx", calGyroOffset[0]
        print "Vy", calGyroOffset[1]
        print "Vz", calGyroOffset[2]
        pickle.dump(calGyroOffset, open("calibration.txt", "wb"))
        close_nicely()






def read_calibration_file_accel():
    file = open('calibrationAccel.txt', 'rb')
    calibration_variables = pickle.load(file)
    file.close()
    return calibration_variables



def generate_accel_offsets(calibration_variables_accel):
    offsets = [0.0, 0.0, 0.0]
    for i in range(0, 3):
        offsets[i] = (calibration_variables_accel[i] + calibration_variables_accel[i + 3]) / 2
    return offsets



def generate_accel_scalars(offset, data):
    output = [0.0, 0.0, 0.0]
    for i in range(0, 3):
        output[i] = data[i] - offset[i]
    return output



    print "start time", args.startTime
    print "end time", args.startTime + args.durationTime



printflag = True
# Retrieve the next input of raw data
def getNextData():
    """
    # Extract the next piece of data
    :param type: Determines whether the function tries to read from a file or directly from an IMU
    :return:
    """
    global inputType, imuObj, calGyroOffset, calAccelOffset, calAccelScale, args, start, printflag
    time_esapsed = time.time() - start
    if (args.use_time)and(printflag):
        print "start time", args.startTime
        print "end time", args.startTime + args.durationTime
        print "waiting", args.startTime, "seconds to start..."
        printflag = False
    if (inputType == "file"):
        # Considered an array of chars, so [:-1] removes the last character
        data = dataFile.readline()[:-1]

    elif (inputType == "live"):
        while (imuObj.dataReady == False):
            pass
        if (imuObj.dataReady):
            data = imuObj.getData()
            if not args.calibrate:
                data = demiApplyCalibration(data, calGyroOffset, calAccelOffset, calAccelScale)
    else:
        print("Error: invalid input type specified. Please set either \"file\" or \"live\"")
    # Try to convert the data into an array of floats
    # This should only fail if the package is corrupt
    if (inputType == "file"):
        try:
            data = [float(i) for i in data.split(",")]
        except:
            data = None

    if (args.calibrate):
        collect_calibration(data)

    if (args.use_time):
        if (time_esapsed > args.startTime):
            print(data)
        if (time_esapsed > args.startTime + args.durationTime):
            close_nicely()
    else:
        print(data)
        return data
    return data


# Extract a single column and return it as a python list
def getCol(data, column):
    return np.array(data)[:, column].tolist()


# Normalise a given list to a given size
def limitSize(data, maxLength=numSamplesMax):
    returnVal = data
    if (returnVal[-1] == None):
        returnVal = returnVal[:-1]
    if (returnVal.__len__() >= maxLength):
        returnVal = returnVal[-maxLength:]
    return returnVal

def close_nicely():
    global imuObj
    # close down sockets before exiting
    if (inputType == "live"):
        imuObj.stopIMU()
    os.system('stty sane')
    sys.exit(0)

def handle_ctrl_c(signal, frame):
    close_nicely()
    #sys.exit(130) # 130 is standard exit code for ctrl-c




####################################################
############ INITIALIZATION FUNCTION(S) ############
####################################################
def init():
    #Initialize start time variable, IMU object and any input files
    global start, calibration_variables_gyro, calibration_variables_accel, dataFile, fileLocale, imuObj
    global calGyroOffset, calAccelScale, calAccelOffset

    #If we're not in calibration mode, apply the last calibration values
    if not args.calibrate:
        calGyroOffset = read_calibration_file()
        calibration_variables_accel = read_calibration_file_accel()
        calAccelOffset = generate_accel_offsets(calibration_variables_accel)  # generate offsets
        calAccelScale = generate_accel_scalars(calAccelOffset, calibration_variables_accel)

        print "Applying calibration from file. If you wish to re-run calibration, please call this program with -c or --calibrate."
        print "Please press enter when ready."
        # Press enter to continue
        #chr = sys.stdin.read(1)
    #If we are in calibration mode...
    else:
        duration = args.durationTime
        print "Entering calibration mode. Please try and isolate the IMU from noise and vibrations."
        print "Calibration data will be saved to calibration.txt in the local directory, and will be automatically applied the next time you run this program without the calibration flag."
        if duration == 1:
            print "Calibration will start at t =", args.startTime, "seconds and finish at t =", args.startTime + args.durationTime, "seconds, covering a duration of", duration, "second."
        else:
            print "Calibration will start at t =", args.startTime, "seconds and finish at t =", args.startTime + args.durationTime, "seconds, covering a duration of", duration, "seconds."
        print "Please press enter when ready."
        #Press enter to continue
        chr = sys.stdin.read(1)


    #This line tells python to call the 'handle_ctrl_c' function if control+c is pressed (allows us to exit nicely)
    signal.signal(signal.SIGINT, handle_ctrl_c)

    ###
    # Open the data file or connect to the IMU
    ###

    if (inputType == "file"):
        dataFile = open(fileLocale, "r")
    elif (inputType == "live"):
        imuObj = spatialC.IMU('some')
        time.sleep(1)
    # Remember start time so that we can calculate durations later
    start = time.time()
    ###
    # Initialise the lists
    ###
    global listRaw, listCrude, listFiltered
    # [time, ax,  ay,  az,  gx,  gy,  gz]
    listRaw.append(getNextData())

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
    nextData = getNextData()
    listRaw.append(nextData)
    listRaw = limitSize(listRaw)

    # Get filtered data
    if (nextData != None):
        global listRawDR, listFilteredDR, listFiltered
        listFiltered.append([listRaw[-1][0]] +
                            fl.filterData(listRaw, [1, 2, 3], ['butter', 'low', cutoffFrequency[0], 4]) +
                            fl.filterData(listRaw, [4, 5, 6], ['butter', 'high', cutoffFrequency[1], 4])
                            )

        listFiltered = limitSize(listFiltered)

        # Get dead reckoned data
        listRawDR.append(dr.doDeadReckoning(listRawDR[-1], listRaw[-1]))
        listRawDR = limitSize(listRawDR)
        listFilteredDR.append(dr.doDeadReckoning(listFilteredDR[-1], listFiltered[-1]))
        listFilteredDR = limitSize(listFilteredDR)

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

    if args.calibrate:
        triplet=3
    else:
        triplet = args.triplet

    # Plot data if appropriate
    if (count == updateEvery):
        timeCol = getCol(listFilteredDR, 0)
        gr.updatePlot(graphAccX, getCol(listFilteredDR, 1 + 3 * triplet), timeCol)
        gr.updatePlot(graphAccY, getCol(listFilteredDR, 2 + 3 * triplet), timeCol)
        gr.updatePlot(graphAccZ, getCol(listFilteredDR, 3 + 3 * triplet), timeCol)
    count = count % updateEvery
    if (inputType == 'file'):
        time.sleep(sleepTime)
    if (inputType == 'live'):
        time.sleep(sleepTime)
    # gr.newPlot(graphChart, getCol(listRaw, 1)[-3:-1], getCol(listRaw, 0)[-3:-1], 'g', 'r', 'b', 5, 'o')



# Cannot put into a function because QtCore.QTimer() is a dick
# Initialize the system
init()

# Update the graph data
timer = QtCore.QTimer()
timer.timeout.connect(main)
timer.start(0)
