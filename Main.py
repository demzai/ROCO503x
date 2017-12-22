####################################################
################### DEPENDENCIES ###################
####################################################
from helper_functions import *
import graph as gr
import filter as fl
from pyqtgraph.Qt import QtCore
import time
import dead_reckoning as dr

####################################################
################# GLOBAL CONSTANTS #################
####################################################
if (inputType == "live"):
    import Spatial_simple_cl as spatialC
fileLocale = "UpDown2.txt"
sleepTime = 0.0001
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


####################################################
############## CALIBRATION CONSTANTS ###############
####################################################
calGyroOffset  = [-0.35, -0.3, -0.45]
calAccelOffset = [0.0, 0.0, 0.0]
calAccelScale  = [1.0, 1.0, 1.0]


####################################################
################# CUSTOM FUNCTIONS #################
####################################################
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
    if (inputType == "file"):
        # Considered an array of chars, so [:-1] removes the last character
        data = dataFile.readline()[:-1]

    elif (inputType == "live"):
        while (imuObj.dataReady == False):
            pass
        if (imuObj.dataReady):
            data = imuObj.getData()
    else:
        print("Error: invalid input type specified. Please set either \"file\" or \"live\"")
    # Try to convert the data into an array of floats
    # This should only fail if the package is corrupt
    if (inputType == "file"):
        try:
            data = [float(i) for i in data.split(",")]
        except:
            data = None

    # print(data)
    return data


####################################################
############ INITIALIZATION FUNCTION(S) ############
####################################################
def init():
    ###
    # Open the data file or connect to the IMU
    ###
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
    listRawDR.append(dr.doDeadReckoning(tempList, listRaw[0]))
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
        listRawDR.append(dr.doDeadReckoning(listRawDR[-1], listRaw[-1]))
        listRawDR = limitSize(listRawDR)
        listFilteredDR.append(dr.doDeadReckoning(listFilteredDR[-1], listFiltered[-1]))
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
    triplet = 0

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
