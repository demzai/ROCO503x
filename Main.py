####################################################
################### DEPENDENCIES ###################
####################################################
from helper_functions import *
from constants import *
import graph as gr
import filter as fl
from pyqtgraph.Qt import QtCore
import time
import dead_reckoning as dr
import quaternion as qt
import complementary_filter as cf


####################################################
################# GLOBAL VARIABLES #################
####################################################
dataFile = None
listRaw = []
listRawDR = []
listFiltered = []
listFilteredDR = []

listRawT0 = []
listRawT1 = []
listFiltT0 = []
listFiltT1 = []

count = 0
cutoffFrequency = [20, 5]  # [Accel Low Pass, Gyro High Pass]


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
    global inputType, imuObj
    data = None

    # If collecting data from a file then...
    if (inputType == "file"):
        # Considered an array of chars, so [:-1] removes the last character
        data = dataFile.readline()[:-1]

    # Else if collecting data from the IMU in real time then...
    elif (inputType == "live"):
        # @todo get input directly from the IMU
        while (imuObj.dataReady == False):
            pass
        if (imuObj.dataReady):
            data = imuObj.getData()
            #data = applyCalibration(data)

    # Otherwise the user doesn't know what they want
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
    global dataFile, fileLocale, imuObj
    if (inputType == "file"):
        dataFile = open(fileLocale, "r")
    elif (inputType == "live"):
        imuObj = spatialC.IMU('some')
        time.sleep(1)

    ###
    # Remove noisy start to filtered data
    ###
    global listRaw, listRawDR, listFiltered, listFilteredDR
    while True:
        # Get raw data
        # [time, ax,  ay,  az,  gx,  gy,  gz]
        nextData = getNextData()
        listRaw.append(nextData)
        listRaw = limitSize(listRaw)

        # Get filtered data
        if (nextData != None):
            listFiltered.append([listRaw[-1][0]] +
                                fl.filterData(listRaw, [1, 2, 3], ['butter', 'low', cutoffFrequency[0], 4]) +
                                fl.filterData(listRaw, [4, 5, 6], ['butter', 'high', cutoffFrequency[1], 4])
                                )

            listFiltered = limitSize(listFiltered)

        else:
            listRaw = listRaw[:-1]

        if (listRaw[-1][0] >= startTime):
            break

    ###
    # Initialise orientation
    ###
    # Assume IMU is stationary and derive initial orientation
    # Reference: https://goo.gl/eChMxp
    # [Roll, Pitch, Yaw]
    orientation = cf.getOrientationFromGravity(listRaw[-1][1:4])
    quaternion = qt.euler_to_quat(orientation)
    for i in range(0, 100):
        cf.accelTheta.append(orientation)
        cf.gyroTheta.append(orientation)

    ###
    # Initialise the dead reckoning lists
    ###
    # [time, ax,  ay,  az,  vx,  vy,  vz,  px,  py,  pz,
    #   gx,  gy,  gz,  tx,  ty,  tz,  qw,  qx,  qy,  qz]
    empty = [0.0, 0.0, 0.0]
    tempList = [startTime] + empty + empty + empty + empty + orientation + quaternion
    listRawDR.append(dr.doDeadReckoning(tempList, listRaw[0], False))
    listRawDR[0] = [listRawDR[0][0] - 0.004] + \
                   listRawDR[0][1:4] + empty + empty + \
                   listRawDR[0][10:]

    listFilteredDR.append(listRawDR[0])

    return


####################################################
##################### MAIN CODE ####################
####################################################
def main():
    global count, sleepTime, inputType
    global listRaw, listRawDR, listRawT0, listRawT1
    global listFiltered, listFilteredDR, listFiltT0, listFiltT1
    count += 1

    # Get raw data
    nextData = getNextData()
    mult = 1.0
    nextData[4] *= mult
    nextData[5] *= mult
    nextData[6] *= mult
    listRaw.append(nextData)
    listRaw = limitSize(listRaw)

    # Get filtered data
    if(listRaw.__len__() >= 300):
        if (nextData != None):
            global listRawDR, listFilteredDR, listFiltered
            listFiltered[-1][0] = listRaw[-2][0]
            listFiltered.append([listRaw[-1][0]] +
                                fl.filterData(listRaw, [1, 2, 3], ['butter', 'low', 2.5, 1]) +
                                fl.filterData(listRaw, [4, 5, 6], ['butter', 'low', 2.5, 4])
                                )

            listFiltered = limitSize(listFiltered)

            # Get dead reckoned data
            listRawDR.append(dr.doDeadReckoning(listRawDR[-1], listRaw[-1], True))
            listRawDR = limitSize(listRawDR)
            listFilteredDR[-1][0] = listRaw[-2][0]
            listFilteredDR.append(dr.doDeadReckoning(listFilteredDR[-1], listFiltered[-1], True))
            listFilteredDR = limitSize(listFilteredDR)

            # Remove low frequency components
            listRawT0.append(listRawDR[-1]*1)
            listRawT1.append(listRawDR[-1]*1)
            listFiltT0.append(listFilteredDR[-1]*1)
            listFiltT1.append(listFilteredDR[-1]*1)

            for i in range( 1, 20 ):
                if listRawT0.__len__() > 1:
                    listRawT0[-1][i] = alpha*listRawT0[-1][i] + (1-alpha)*listRawT0[-2][i]
                    listFiltT0[-1][i] = alpha*listFiltT0[-1][i] + (1-alpha)*listFiltT0[-2][i]
                listRawT1[-1][i] -= listRawT0[-1][i]
                listFiltT1[-1][i] -= listFiltT0[-1][i]

            listRawT0 = limitSize(listRawT0)
            listRawT1 = limitSize(listRawT1)
            listFiltT0 = limitSize(listFiltT0)
            listFiltT1 = limitSize(listFiltT1)


        else:
            quit(1)

        # listRawDR:
        #  [time, ax,  ay,  az,  vx,  vy,  vz,  px,  py,  pz,
        #    gx,  gy,  gz,  tx,  ty,  tz,  qw,  qx,  qy,  qz]
        """
        set triplet to:
        acceleration = 0
        velocity = 1
        position = 2
        angular velocity = 3
        angular position = 4
        quaternion = 5
        """

        triplet = 1
        axis = 1
        useList1 = listRawDR
        useList2 = listRawT1
        # print(listFilteredDR[-1])
        if (count == updateEvery):
            # gr.updatePlot(graphAccX, getCol(useList1, axis + 3 * triplet), getCol(useList1, 0))
            gr.updatePlot(graphAccY, getCol(useList2, axis + 3 * triplet), getCol(useList2, 0))

        count = count % updateEvery
        if (inputType == 'file'):
            time.sleep(sleepTime)


# Initialize the system
init()

# Update the graph data
timer = QtCore.QTimer()
timer.timeout.connect(main)
timer.start(0)
