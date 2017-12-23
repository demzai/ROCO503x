import numpy as np
from constants import *

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
