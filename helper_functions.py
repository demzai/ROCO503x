import numpy as np
from constants import *
from math import *

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


# Convert degrees to radians
def d2r(angle):
    return angle * pi / 180


# Update the orientation estimation
def integrateGyro(prevOrientation, gyroData, delTime):
    returnList = []
    for i in range(0, 3):
        returnList += [prevOrientation[i] + gyroData[i] * delTime]  # Theta += Gyro * time
    return returnList


# Perform the exponential moving average over a given scalar
def expAvg(prevValue, nextValue, beta=alpha):
    return beta * prevValue + (1 - beta) * nextValue