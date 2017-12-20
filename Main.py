####################################################
################### DEPENDENCIES ###################
####################################################
import numpy as np
import quaternion as qt
import graph as gr
import filter as fl
from pyqtgraph.Qt import QtCore
import time
from math import *

####################################################
################# GLOBAL CONSTANTS #################
####################################################
inputType = "file"
fileLocale = "IMU_Stationary.txt"
sleepTime = 0.0001
numSamplesMax = 100
minSamples = 10
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
dcm = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])  # @todo Incorrect initial orientation!


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


# Convert degrees to radians
def d2r(angle):
    return angle * pi / 180


# Generate a rotation matrix from the roll pitch & yaw values
def getRotationMatrix(x, y, z):
    rotX = [[1, 0, 0], [0, cos(d2r(x)), -sin(d2r(x))], [0, sin(d2r(x)), cos(d2r(x))]]
    rotY = [[cos(d2r(y)), 0, sin(d2r(y))], [0, 1, 0], [-sin(d2r(y)), 0, cos(d2r(y))]]
    rotZ = [[cos(d2r(z)), -sin(d2r(z)), 0], [sin(d2r(z)), cos(d2r(z)), 0], [0, 0, 1]]
    rot = np.matrix(rotX) *  np.matrix(rotY) *  np.matrix(rotZ)
    return rot.tolist()


def integrateGyro(prevData, currData):
    delTime = currData[0] - prevData[0]
    returnList = []
    for i in range(0, 3):
        returnList += [prevData[i + 13] + currData[i + 4] * delTime]  # Theta += Gyro * time
    return returnList


# Perform dead reckoning on the provided raw data
def doDeadReckoning(prevComplete, raw):
    # Ensure that prevComplete isn't modified anywhere by copying the data over beforehand
    delTime = raw[0] - prevComplete[0]
    complete = prevComplete * 1  # Deep copy items over

    # Update the time
    complete[0] = raw[0]

    # Update the Euler orientation
    orientation = integrateGyro(prevComplete, raw)
    for i in range(0, 3):
        complete[i + 10] = raw[i + 4]
        complete[i + 13] = orientation[i]

    # UPDATE THE QUATERNION ORIENTATION
    # Small angle Euler angles do not work over time
    # Quaternions do work, and are safer as they don't suffer from gimbal lock
    # Choose between the 2 methods below:

    # global dcm
    # dcm = np.array(np.matrix(dcm) *
    #                np.matrix(getRotationMatrix(raw[4], raw[5], raw[6])))

    gyroQuat = qt.euler_to_quat(raw[4:7])
    currQuat = [complete[16], complete[17], complete[18], complete[19]]
    complete[16], complete[17], complete[18], complete[19] = qt.q_mult(currQuat, gyroQuat)
    currQuat = [complete[16], complete[17], complete[18], complete[19]]
    dcm = np.array(qt.quat_to_dcm(currQuat))

    # Re-orientate the accelerometer values based on the IMU orientation
    acc = np.array([raw[1], raw[2], raw[3]]).transpose()
    acc = dcm.dot(acc) * 9.81

    # Update the acceleration, velocity & position info
    for i in range(0, 3):
        # Acceleration
        complete[i + 1] = acc[i]
        # Subtract gravity
        if (i == 2):
            acc[i] -= 9.81
        # Position += u*t + 0.5*a*t^2
        complete[i + 7] += (complete[i + 4] + 0.5 * acc[i] * delTime) * delTime
        # Velocity += a*t
        complete[i + 4] += acc[i] * delTime

    return complete


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


####################################################
############ INITIALIZATION FUNCTION(S) ############
####################################################
def init():
    ###
    # Open the data file or connect to the IMU
    ###
    global dataFile, fileLocale
    if (inputType == "file"):
        dataFile = open(fileLocale, "r")

    ###
    # Initialise the lists
    ###
    global listRaw, listCrude, listFiltered
    # [time, ax,  ay,  az,  gx,  gy,  gz]
    listRaw.append(getNextData())

    # [time, ax,  ay,  az,  vx,  vy,  vz,  px,  py,  pz,
    #   gx,  gy,  gz,  tx,  ty,  tz,  qw,  qx,  qy,  qz]
    tempList = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
    listRawDR.append(doDeadReckoning(tempList, listRaw[0]))
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
        listRawDR.append(doDeadReckoning(listRawDR[-1], listRaw[-1]))
        listRawDR = limitSize(listRawDR)
        listFilteredDR.append(doDeadReckoning(listFilteredDR[-1], listFiltered[-1]))
        listFilteredDR = limitSize(listFilteredDR)

        # listRawDR:
        # [time, ax,  ay,  az,  vx,  vy,  vz,  px,  py,  pz,
        #   gx,  gy,  gz,  tx,  ty,  tz,  qw,  qx,  qy,  qz]

    # Plot data if appropriate
    triplet = 0
    if (count == updateEvery):
        timeCol = getCol(listFilteredDR, 0)
        gr.updatePlot(graphAccX, getCol(listFilteredDR, 1 + 3 * triplet), timeCol)
        gr.updatePlot(graphAccY, getCol(listFilteredDR, 2 + 3 * triplet), timeCol)
        gr.updatePlot(graphAccZ, getCol(listFilteredDR, 3 + 3 * triplet), timeCol)
    count = count % updateEvery
    if (inputType == 'file'):
        time.sleep(sleepTime)
        # gr.newPlot(graphChart, getCol(listRaw, 1)[-3:-1], getCol(listRaw, 0)[-3:-1], 'g', 'r', 'b', 5, 'o')


# Cannot put into a function because QtCore.QTimer() is a dick
# Initialize the system
init()

# Update the graph data
timer = QtCore.QTimer()
timer.timeout.connect(main)
timer.start(0)
