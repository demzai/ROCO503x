# Dependencies
from math import *
import quaternion as qt
from helper_functions import *
import filter as fl
import dead_reckoning as dr

# Global constants
cutoffFrequency = [10.0, 10.0, 10.0]
beta = 0.95

# Global variables
accelTheta = []
gyroTheta = []
errorTheta = []
outputTheta = []

def getOrientationFromGravity(data):
    # Reference: https://goo.gl/eChMxp
    # [Roll, Pitch, Yaw]
    orientation = [
        atan2(data[2], data[3]) * 180 / pi,
        atan2(-data[1], sqrt(data[2] ** 2 + data[3] ** 2)) * 180 / pi,
        0.0]
    return orientation

# Perform a complementary filter on the data
def doComplementaryFilter(prevDataFull, newDataRaw):
    global accelTheta, gyroTheta, errorTheta
    # Create the axis from the previous datas gravity vector
        # Gravity always points down and is equal to 1g
        # Therefore, rotating a unit vector to the same orientation gives the gravity vector
    prevQuaternion = prevDataFull[16:20]
    dcm = np.matrix(qt.quat_to_dcm(prevQuaternion)).transpose()
    gravityVector = dcm * np.matrix([0.0, 0.0, 1.0]).transpose()

    # Obtain the theta approximations from the accelerometer (x & y only)
    # accelThetaX = atan2( gravityVector[1], gravityVector[2]) * 180/pi
    # accelThetaY = atan2(-gravityVector[0], gravityVector[2]) * 180/pi
    # accelTheta.append([accelThetaX, accelThetaY, 0.0])
    accelTheta.append(getOrientationFromGravity(gravityVector))


    # Obtain the updated gyroscope-based orientation
    prevTheta = prevDataFull[13:16]
    delTime = newDataRaw[0] - prevDataFull[0]
    gyroTheta.append(dr.integrateGyro(prevDataFull[13:16], newDataRaw[4:7], delTime))
    accelTheta[-1][2] = gyroTheta[-1][2]


    # Obtain the error between the theta values
    tempList = []
    for i in range(0, 3):
        tempList += [accelTheta[-1][i]-gyroTheta[-1][i]]
    errorTheta.append(tempList)


    # Ensure no list gets too long
    accelTheta = limitSize(accelTheta)
    gyroTheta  = limitSize(gyroTheta)
    errorTheta = limitSize(errorTheta)


    # Filter the data
    filteredAccelTheta = fl.filterData(accelTheta, [0, 1, 2],
                                        ['butter', 'low', cutoffFrequency[0], 4])
    filteredGyroTheta  = fl.filterData(gyroTheta,  [0, 1, 2],
                                        ['butter', 'low', cutoffFrequency[1], 4])
    filteredErrorTheta = fl.filterData(errorTheta, [0, 1, 2],
                                        ['butter', 'low', cutoffFrequency[2], 4])

    
    # Sensor fusion
    outputTheta.append(gyroTheta[-1])
    for i in range(0, 3):
        outputTheta[-1][i] = filteredGyroTheta[i] * beta + filteredAccelTheta[i] * (1 - beta)
        # OR
        # outputTheta[-1][i] += filteredErrorTheta[i]


    # Differentiate for the new gyroscope values
    outputGyro = prevTheta * 1
    for i in range(0, 3):
        outputGyro[i] = outputTheta[-1][i] - prevTheta[i]

    return outputTheta + outputGyro
