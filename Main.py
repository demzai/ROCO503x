####################################################
################### DEPENDENCIES ###################
####################################################
import numpy as np
import quaternion as qt
import graph as gr
import filter as fl
from pyqtgraph.Qt import QtCore
import Spatial_simple_cl as spatialC
import time
#from math import *

####################################################
################# GLOBAL CONSTANTS #################
####################################################
inputType = "file"
fileLocale = "IMU_Stationary.txt"
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
dcm = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])  # @todo Incorrect initial orientation!

####################################################
############## CALIBRATION CONSTANTS ###############
####################################################
calGyroOffset  = [-0.35, -0.3, -0.45]
calAccelOffset = [0.0, 0.0, 0.0]
calAccelScale  = [1.0, 1.0, 1.0]


####################################################
################# CUSTOM FUNCTIONS #################
####################################################
# Apply scaling and offset factors
def applyCalibration(data):
    for i in range(0, 3):
        # Offset then scale accelerometer values
        data[i+1] -= calAccelOffset[i]
        data[i+1] /= calAccelScale[i]

        # Offset the gyroscope values
        data[i+4] -= calGyroOffset[i]
    return data

# Retrieve the next input of raw data
def getNextData():
    """
    # Extract the next piece of data
    :param type: Determines whether the function tries to read from a file or directly from an IMU
    :return:
    """
    global inputType, imuObj
    if (inputType == "file"):
        # Considered an array of chars, so [:-1] removes the last character
        data = dataFile.readline()[:-1]

    elif (inputType == "live"):
        # @todo get input directly from the IMU
        while (imuObj.dataReady == False):
            pass
        if (imuObj.dataReady):
            data = imuObj.getData()
            #data = applyCalibration(data)
    else:
        print("Error: invalid input type specified. Please set either \"file\" or \"live\"")
    # Try to convert the data into an array of floats
    # This should only fail if the package is corrupt
    if (inputType == "file"):
        try:
            data = [float(i) for i in data.split(",")]
        except:
            data = None

    print(data)
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


# Update the orientation estimation
def integrateGyro(prevOrientation, gyroData, delTime):
    returnList = []
    for i in range(0, 3):
        returnList += [prevOrientation[i] + gyroData[i] * delTime]  # Theta += Gyro * time
    return returnList


# Perform dead reckoning on the provided raw data
def doDeadReckoning(prevComplete, raw):
    # Ensure that prevComplete isn't modified anywhere by copying the data over beforehand
    delTime = raw[0] - prevComplete[0]
    complete = prevComplete * 1  # Deep copy items over

    # Update the time
    complete[0] = raw[0]

    # Update the Euler orientation
    orientation = integrateGyro(prevComplete[13:16], raw[4:7], delTime)
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
    acc = np.array([raw[1], raw[2], raw[3]]).transpose()
    acc = dcm.dot(acc)*9.81

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
    global dataFile, fileLocale, imuObj
    if (inputType == "file"):
        dataFile = open(fileLocale, "r")
    elif (inputType == "live"):
        imuObj = spatialC.IMU('some')
        time.sleep(1)



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
    triplet = 3

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
