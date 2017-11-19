import numpy as np
import quaternion as qt
from math import *

file = open("C:/Users/Student/Desktop/IMU_Stationary.txt", "r")

#@todo initialise the complete data set
#@todo create a main function? or whatever pythons version of it is
#@todo plot the pretties
#@todo filter the data...

####################################################
################# CUSTOM FUNCTIONS #################
####################################################
def getNextData(type):
    """
    # Extract the next piece of data
    :param type: Determines whether the function tries to read from a file or directly from an IMU
    :return:
    """
    if (type == "file"):
        data = file.readline()[:-1]  # Considered an array of chars, so [:-1] removes the last character
    else:
        data = None
    data = np.array([float(i) for i in data.split(",")])
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


