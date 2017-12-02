
####################################################
################### DEPENDENCIES ###################
####################################################
import numpy as np
import quaternion as qt
import graph as gr
import filter as fl
from pyqtgraph.Qt import QtCore
import time


####################################################
################# GLOBAL CONSTANTS #################
####################################################
inputType = "file"
fileLocale = "IMU_Stationary.txt"
sleepTime = 0.0001
numSamplesMax = 100
minSamples  = 10
graphWindow = gr.newWindow("Graphs", 640, 480)
graphChart  = gr.newGraph(graphWindow, "Test")
graphAccX   = gr.newPlot(graphChart, [], [], 'r', None, None, 3, 'o')
graphAccY   = gr.newPlot(graphChart, [], [], 'g', None, None, 3, 'o')
graphAccZ   = gr.newPlot(graphChart, [], [], 'b', None, None, 3, 'o')
updateEvery = 10
cutoffFrequency = [0.5, 20] # [Accel Low Pass, Gyro High Pass]


####################################################
################# GLOBAL VARIABLES #################
####################################################
dataFile = None
listRaw = []
listRawDR = []
listFiltered = []
listFilteredDR = []
count = 0


####################################################
################# CUSTOM FUNCTIONS #################
####################################################
# Retrieve the next input of raw data
def getNextData():
    """
    # Extract the next piece of data
    :param type: Determines whether the function tries to read from a file or directly from an IMU
    :return:
    """
    global inputType
    if (inputType == "file"):
        # Considered an array of chars, so [:-1] removes the last character
        data = dataFile.readline()[:-1]

    else:
        # @todo get input directly from the IMU
        return None

    # Try to convert the data into an array of floats
    # This should only fail if the package is corrupt
    try:
        data = [float(i) for i in data.split(",")]
    except:
        data = None
    return data


# Perform dead reckoning on the provided raw data
def doDeadReckoning(prevComplete, raw):
    """
    # Perform dead reckoning to acquire the orientation, position & velocity of the IMU
    # Raw data format = [time, ax, ay, az, gx, gy, gz]
    # Complete data format = [time, ax, ay, az, vx, vy, vz, px, py, pz,
    #                           gx, gy, gz, tx, ty, tz, qw, qx, qy, qz]
    """

    # Ensure that prevComplete isn't modified anywhere by copying the data over beforehand
    delTime = raw[0] - prevComplete[0]
    complete = prevComplete * 1 # Deep copy items over

    # Update the time
    complete[0] = raw[0]

    # #                [time, ax,  ay,  az,  vx,  vy,  vz,  px,  py,  pz,
    # listRawDR.append([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    #                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    # #                  gx,  gy,  gz,  tx,  ty,  tz,  qw,  qx,  qy,  qz]
    # #              [time, ax,  ay,  az,  gx,  gy,  gz]
    # listRaw.append([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])

    # Update the Euler orientation
    for i in range(0, 3):
        complete[i+10] = raw[i+4]
        complete[i+13] += raw[i+4] * delTime  # Theta += Gyro * time

    # Update the Quaternion orientation
    gyroQuat = qt.euler_to_quat(raw[4:7])
    currQuat = [complete[16], complete[17], complete[18], complete[19]]
    complete[16], complete[17], complete[18], complete[19] = qt.q_mult(currQuat, gyroQuat)

    # Re-orientate the accelerometer values based on the IMU orientation
    currQuat = [complete[16], complete[17], complete[18], complete[19]]
    dcm = np.array(qt.quat_to_dcm(currQuat))
    acc = np.array([raw[1], raw[2], raw[3]]).transpose()
    acc = dcm.dot(acc)

    # Update the acceleration, velocity & position info
    for i in range(0, 2):
        # Acceleration
        complete[i+1] = acc[i]
        # Position += u*t + 0.5*a*t^2
        complete[i+7] += (complete[i+4] + 0.5*acc[i]*delTime) * delTime
        # Velocity += a*t
        complete[i+4] += acc[i] * delTime

    return complete


# Extract a single column and return it as a python list
def getCol(data, column):
    return np.array(data)[:,column].tolist()


# Normalise a given list to a given size
def limitSize(data, maxLength=numSamplesMax):
    returnVal = data
    if (returnVal[-1] == None):
        returnVal = returnVal[:-1]
    if (returnVal.__len__() >= maxLength):
        returnVal = returnVal[-maxLength:]
    return returnVal


# Effectively the main function...
def update():
    global listRaw, count, sleepTime, inputType, listFiltered
    count += 1

    # Get raw data
    nextData = getNextData()
    listRaw.append(nextData)
    listRaw = limitSize(listRaw)

    # Get filtered data
    if (nextData != None):
        global  listRawDR, listFilteredDR, listFiltered
        listFiltered.append(
            fl.filterData(listRaw,
                          ['butter', 'low' , cutoffFrequency[0], 4],
                          ['butter', 'high', cutoffFrequency[1], 4]))
        listFiltered = limitSize(listFiltered)

    # Get dead reckoned data
        listRawDR.append(doDeadReckoning(listRawDR[-1], listRaw[-1]))
        listRawDR = limitSize(listRawDR)
        listFilteredDR.append(doDeadReckoning(listFilteredDR[-1], listFiltered[-1]))
        listFilteredDR = limitSize(listFilteredDR)

# #                [time, ax,  ay,  az,  vx,  vy,  vz,  px,  py,  pz,
# listRawDR.append([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
#                   0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
# #                  gx,  gy,  gz,  tx,  ty,  tz,  qw,  qx,  qy,  qz]

    # Plot data if appropriate
    if (count == updateEvery):
        gr.updatePlot(graphAccX, getCol(listFilteredDR, 13), getCol(listFilteredDR, 0))
        gr.updatePlot(graphAccY, getCol(listFilteredDR, 14), getCol(listFilteredDR, 0))
        gr.updatePlot(graphAccZ, getCol(listFilteredDR, 15), getCol(listFilteredDR, 0))
    count = count%updateEvery
    if (inputType == 'file'):
        time.sleep(sleepTime)
    # gr.newPlot(graphChart, getCol(listRaw, 1)[-3:-1], getCol(listRaw, 0)[-3:-1], 'g', 'r', 'b', 5, 'o')


####################################################
############ INITIALIZATION FUNCTION(S) ############
####################################################
def init():
    ###
    # Initialise the lists
    ###
    global  listRaw, listCrude, listFiltered
    #              [time, ax,  ay,  az,  gx,  gy,  gz]
    listRaw.append([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])

    #                [time, ax,  ay,  az,  vx,  vy,  vz,  px,  py,  pz,
    listRawDR.append([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    #                  gx,  gy,  gz,  tx,  ty,  tz,  qw,  qx,  qy,  qz]

    listFiltered.append(listRaw[0])
    listFilteredDR.append(listRawDR[0])

    ###
    # Open the data file or connect to the IMU
    ###
    global dataFile, fileLocale
    if (inputType == "file"):
        dataFile = open(fileLocale, "r")

    return


####################################################
##################### MAIN CODE ####################
####################################################
# Cannot put into a function because QtCore.QTimer() is a dick
# Initialize the system
init()

# Acquire an initial set of data
while True:

    listRaw.append(getNextData())
    if (listRaw[-1] == None):
        listRaw = listRaw[:-1]

    # If enough samples have been collected, do more analysis
    if (listRaw.__len__() >= minSamples):
        break
    print(listRaw.__len__())


# Update the graph data
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(0)










































































