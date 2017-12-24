import graph as gr
import pickle

####################################################
################# GLOBAL CONSTANTS #################
####################################################
# Inputs
inputType = "live"
fileLocale = "UpDown2.txt"
calibrationLocale = "hard_coded"
calibrationLocale = "file"
gyroFile = "calibration.txt"
accelFile = "calibrationAccel.txt"

# Graphs
graphWindow = gr.newWindow("Graphs", 640, 480)
graphChart = gr.newGraph(graphWindow, "Test")
graphAccX = gr.newPlot(graphChart, [], [], 'r', None, None, 3, 'o')
graphAccY = gr.newPlot(graphChart, [], [], 'g', None, None, 3, 'o')
graphAccZ = gr.newPlot(graphChart, [], [], 'b', None, None, 3, 'o')
g1 = gr.newPlot(graphChart, [], [], 'c', None, None, 3, 'o')
g2 = gr.newPlot(graphChart, [], [], 'm', None, None, 3, 'o')
g3 = gr.newPlot(graphChart, [], [], 'y', None, None, 3, 'o')
g4 = gr.newPlot(graphChart, [], [], 'k', None, None, 3, 'o')
g5 = gr.newPlot(graphChart, [], [], 'w', None, None, 3, 'o')
# House keeping
sleepTime = 0.001
updateEvery = 10
startTime = 0.0
numSamplesMax = 100

# Complementary filter
beta = 0.25


####################################################
############## CALIBRATION CONSTANTS ###############
####################################################
calGyroOffset  = [-0.20854434285714282, -0.12702977142857147, -0.2672430000000005]
calAccelOffset = [ 0.01725754327857845, -0.00160451736868370,  0.0145056265099980]
calAccelScale  = [ 0.99893057626429155,  1.00137490276358830,  1.0021011922022352]




def read_calibration_file():
    file = open('calibration.txt', 'rb')
    calibration_variables = pickle.load(file)
    file.close()
    return calibration_variables




def collect_calibration(data):
    global start, calibCount, calV, args
    time_esapsed = time.time() - start
    print("start time", args.startTime)
    print ("end time", args.startTime + args.durationTime)
    if (time_esapsed > args.startTime):
        calibCount += 1
        calGyroOffset[0] += data[4]
        calGyroOffset[1] += data[5]
        calGyroOffset[2] += data[6]
    if (time_esapsed > args.startTime + args.durationTime):
        calGyroOffset[0] = calGyroOffset[0] / calibCount
        calGyroOffset[1] = calGyroOffset[1] / calibCount
        calGyroOffset[2] = calGyroOffset[2] / calibCount
        print("Vx", calGyroOffset[0])
        print("Vy", calGyroOffset[1])
        print("Vz", calGyroOffset[2])
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

if calibrationLocale == "file":
    calGyroOffset = read_calibration_file()
    calibration_variables_accel = read_calibration_file_accel()
    calAccelOffset = generate_accel_offsets(calibration_variables_accel)  # generate offsets
    calAccelScale = generate_accel_scalars(calAccelOffset, calibration_variables_accel)
    print("Values loaded from files")
    print("Gyro offset:", calGyroOffset)
    print("Accel offset:", calAccelOffset)
    print("Accel scale:", calAccelScale)