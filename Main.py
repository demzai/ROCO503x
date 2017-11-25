#@todo initialise the complete data set
#@todo create a main function? or whatever pythons version of it is
#@todo plot the pretties
#@todo filter the data...

####################################################
################### DEPENDENCIES ###################
####################################################
import numpy as np
import quaternion as qt
import graph as gr
from pyqtgraph.Qt import QtGui, QtCore
#import time as time


####################################################
################# GLOBAL CONSTANTS #################
####################################################
inputType = "file"
fileLocale = "C:/Users/Student/Desktop/Uni/IMU/ROCO503x/IMU_Stationary.txt"
numSamplesMax = 1000
graphWindow = gr.newWindow("Graphs", 640, 480)
graphChart  = gr.newGraph(graphWindow, "Test")
graphData   = gr.newPlot(graphChart, [], [], 'g', 'r', 'b', 5, 'o')
updateEvery = 10


####################################################
################# GLOBAL VARIABLES #################
####################################################
dataFile = None
listRaw = []
listCrude = []
listFiltered = []
count = 0


####################################################
################# CUSTOM FUNCTIONS #################
####################################################
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
        return None

    # Try to convert the data into an array of floats
    # This should only fail if the package is corrupt
    try:
        data = [float(i) for i in data.split(",")]
    except:
        data = None
    return data


def doDeadReckoning(prevComplete, raw):
    """
    # Perform dead reckoning to acquire the orientation, position & velocity of the IMU
    # Raw data format = [time, ax, ay, az, gx, gy, gz]
    # Complete data format = [time, ax, ay, az, vx, vy, vz, px, py, pz,
    #                           gx, gy, gz, tx, ty, tz, qw, qx, qy, qz]
    :param prevComplete:
    :param raw:
    :return: list of updated values
    """
    delTime = raw[0] - prevComplete[0]
    complete = prevComplete

    # Update the Euler orientation
    for i in range(-1, -3):
        complete[i-7] = raw[i]
        complete[i-4] += raw[i] * delTime  # Theta += Gyro * time

    # Update the Quaternion orientation
    gyroQuat = qt.euler_to_quat(raw[-3:-1])
    currQuat = [complete[-4], complete[-3], complete[2], complete[-1]]
    complete[-4], complete[-3], complete[2], complete[-1] = qt.q_mult(currQuat, gyroQuat)

    # Re-orientate the accelerometer values based on the IMU orientation
    currQuat = [complete[-4], complete[-3], complete[2], complete[-1]]
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

    return


def getCol(data, column):
    return np.array(data)[:,column].tolist()


# Effectively the main function...
def update():
    global listRaw, count
    count += 1
    listRaw.append(getNextData())
    if (listRaw[-1] == None):
        listRaw = listRaw[:-1]
    if (listRaw.__len__() >= numSamplesMax):
        listRaw = listRaw[-numSamplesMax:]

    if (count == updateEvery):
        gr.updatePlot(graphData, getCol(listRaw, 1), getCol(listRaw, 0))
    count = count%updateEvery
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
    listCrude.append([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    #                  gx,  gy,  gz,  tx,  ty,  tz,  qw,  qx,  qy,  qz]

    listFiltered.append(listCrude[0])

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

# Perform the main code
while True:

    listRaw.append(getNextData())
    if (listRaw[-1] == None):
        listRaw = listRaw[:-1]

    # If enough samples have been collected, do more analysis
    if (listRaw.__len__() >= numSamplesMax):
        listRaw = listRaw[-numSamplesMax:]
        break
    print(listRaw.__len__())


# Update the graph data
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(0)










































































