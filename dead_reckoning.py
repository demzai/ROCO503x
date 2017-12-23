import numpy as np
import quaternion as qt
from math import *
import complementary_filter as cf


# Convert degrees to radians
def d2r(angle):
    return angle * pi / 180


# Generate a rotation matrix from the roll pitch & yaw values
def getRotationMatrix(x, y, z):
    rotX = [[1, 0, 0], [0, cos(d2r(x)), -sin(d2r(x))], [0, sin(d2r(x)), cos(d2r(x))]]
    rotY = [[cos(d2r(y)), 0, sin(d2r(y))], [0, 1, 0], [-sin(d2r(y)), 0, cos(d2r(y))]]
    rotZ = [[cos(d2r(z)), -sin(d2r(z)), 0], [sin(d2r(z)), cos(d2r(z)), 0], [0, 0, 1]]
    rot = np.matrix(rotZ) * np.matrix(rotY) * np.matrix(rotX)
    return rot.tolist()


# Update the orientation estimation
def integrateGyro(prevOrientation, gyroData, delTime):
    returnList = []
    for i in range(0, 3):
        returnList += [prevOrientation[i] + gyroData[i] * delTime]  # Theta += Gyro * time
    return returnList


# Perform dead reckoning on the provided raw data
def doDeadReckoning(prevComplete, raw, useComplimentaryFilter=False):
    # Ensure that prevComplete isn't modified anywhere by copying the data over beforehand
    delTime = raw[0] - prevComplete[0]
    complete = prevComplete * 1  # Deep copy items over

    # Update the time
    complete[0] = raw[0]

    # Update the Euler orientation
    if(useComplimentaryFilter == False):
        orientation = integrateGyro(prevComplete[13:16], raw[4:7], delTime)
        for i in range(0, 3):
            complete[i + 10] = raw[i + 4]
            complete[i + 13] = orientation[i]
    else:
        orientation = cf.doComplementaryFilter(prevComplete, raw)
        for i in range(0, 3):
            complete[i + 10] = orientation[i+3]
            complete[i + 13] = orientation[i]
    complete[16:20] = qt.euler_to_quat(orientation[0:3])



    # UPDATE THE QUATERNION ORIENTATION
    # Small angle Euler angles do not work over time
    # Quaternions do work, and are safer as they don't suffer from gimbal lock
    # Choose between the 2 methods below:

    # dcm = np.array(getRotationMatrix(orientation[0], orientation[1], orientation[2]))
    dcm = np.array(qt.quat_to_dcm(complete[16:20]))

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
