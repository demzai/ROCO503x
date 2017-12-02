
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
graphData   = gr.newPlot(graphChart, [], [], 'b', 'b', 'b', 3, 'o')
graphFilt   = gr.newPlot(graphChart, [], [], 'r', 'r', 'r', 3, 'o')
updateEvery = 10
cutoffFrequency = [5, 1]


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
    complete = prevComplete

    # Update the time
    complete[0] = raw[0]

    # Update the Euler orientation
    for i in range(-1, -3):
        complete[i-7] = raw[i]
        complete[i-4] += raw[i] * delTime  # Theta += Gyro * time

    # Update the Quaternion orientation
    gyroQuat = qt.euler_to_quat(raw[-4:-1])
    currQuat = [complete[-4], complete[-3], complete[-2], complete[-1]]
    complete[-4], complete[-3], complete[-2], complete[-1] = qt.q_mult(currQuat, gyroQuat)

    # Re-orientate the accelerometer values based on the IMU orientation
    currQuat = [complete[-4], complete[-3], complete[-2], complete[-1]]
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
    listFiltered.append(
        fl.filterData(listRaw,
                      ['butter', 'low' , cutoffFrequency[0], 4],
                      ['butter', 'high', cutoffFrequency[1], 4]))
    if (nextData == None):
        listFiltered[-1] = nextData
    listFiltered = limitSize(listFiltered)

    # Get dead reckoned data

    ##################################################################################################
    ######################################### BUG IN HERE!!!!!!!!!!! #################################
    ##################################################################################################
    ##################################################################################################
    ##################################################################################################
    ##################################################################################################
    print(listRawDR)    # Prints before anything happens
    __temp = listRawDR  # Stored in a brand new and unique variable to ensure isolation
    doDeadReckoning(listRawDR[-1], listRaw[-1]) # Call to dead reckoning function
    print(__temp)       # This should print out exactly the same as the previous print but it doesn't!
    ##################################################################################################
    ##################################################################################################
    ##################################################################################################
    ##################################################################################################
    # listFilteredDR.append(doDeadReckoning(listFilteredDR[-1], listFiltered[-1]))

    # Plot data if appropriate
    if (count == updateEvery):
        gr.updatePlot(graphData, getCol(listRaw, 1), getCol(listRaw, 0))
        gr.updatePlot(graphFilt, getCol(listRawDR, 1), getCol(listRawDR, 0))
        # gr.updatePlot(graphFilt, getCol(listFilteredDR, 1), getCol(listFilteredDR, 0))
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










































































