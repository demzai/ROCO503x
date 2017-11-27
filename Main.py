
####################################################
################### DEPENDENCIES ###################
####################################################
import numpy as np
import quaternion as qt
import graph as gr
import filter as fl
from pyqtgraph.Qt import QtCore

import sys
from ctypes import *
#Phidget specific imports
from Phidgets.Phidget import Phidget
from Phidgets.PhidgetException import PhidgetErrorCodes, PhidgetException
from Phidgets.Events.Events import SpatialDataEventArgs, AttachEventArgs, DetachEventArgs, ErrorEventArgs
from Phidgets.Devices.Spatial import Spatial, SpatialEventData, TimeSpan
from Phidgets.Phidget import PhidgetLogLevel
#import time


####################################################
################# GLOBAL CONSTANTS #################
####################################################
inputType = "live"
fileLocale = "IMU_Stationary.txt"
numSamplesMax = 1000
graphWindow = gr.newWindow("Graphs", 640, 480)
graphChart  = gr.newGraph(graphWindow, "Test")
graphData   = gr.newPlot(graphChart, [], [], 'g', 'r', 'b', 5, 'o')
updateEvery = 10
filter10Hz  = fl.getFilter(1000, 30, 62.5)


####################################################
################# GLOBAL VARIABLES #################
####################################################
dataFile = None
listRaw = []
listCrude = []
listFiltered = []
count = 0


####################################################
################# CUSTOM FUNCTIONS #################
####################################################
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

        # Try to convert the data into an array of floats
        # This should only fail if the package is corrupt
        try:
            data = [float(i) for i in data.split(",")]
        except:
            data = None
        return data
    else:
        try:
            data = [spatialData.Timestamp.seconds, spatialData.Acceleration[0], spatialData.Acceleration[1],
                    spatialData.Acceleration[2], spatialData.AngularRate[0], spatialData.AngularRate[1],
                    spatialData.AngularRate[2]]
        except:
            data = None
        return data


if (inputType != "file"):
    try:
        spatial = Spatial()
    except RuntimeError as e:
        print("Runtime Exception: %s" % e.details)
        print("Exiting....")
        exit(1)


#Information Display Function
def DisplayDeviceInfo():
    print("|------------|----------------------------------|--------------|------------|")
    print("|- Attached -|-              Type              -|- Serial No. -|-  Version -|")
    print("|------------|----------------------------------|--------------|------------|")
    print("|- %8s -|- %30s -|- %10d -|- %8d -|" % (spatial.isAttached(), spatial.getDeviceName(), spatial.getSerialNum(), spatial.getDeviceVersion()))
    print("|------------|----------------------------------|--------------|------------|")
    print("Number of Acceleration Axes: %i" % (spatial.getAccelerationAxisCount()))
    print("Number of Gyro Axes: %i" % (spatial.getGyroAxisCount()))
    print("Number of Compass Axes: %i" % (spatial.getCompassAxisCount()))


#Event Handler Callback Functions
def SpatialAttached(e):
    attached = e.device
    print("Spatial %i Attached!" % (attached.getSerialNum()))

def SpatialDetached(e):
    detached = e.device
    print("Spatial %i Detached!" % (detached.getSerialNum()))

def SpatialError(e):
    try:
        source = e.device
        print("Spatial %i: Phidget Error %i: %s" % (source.getSerialNum(), e.eCode, e.description))
    except PhidgetException as e:
        print("Phidget Exception %i: %s" % (e.code, e.details))


def SpatialData(e):
    source = e.device
    print("Spatial %i: Amount of data %i" % (source.getSerialNum(), len(e.spatialData)))
    for index, spatialData in enumerate(e.spatialData):
        print("=== Data Set: %i ===" % (index))
        if len(spatialData.Acceleration) > 0:
            print("Acceleration> x: %6f  y: %6f  z: %6f" % (
            spatialData.Acceleration[0], spatialData.Acceleration[1], spatialData.Acceleration[2]))
        if len(spatialData.AngularRate) > 0:
            print("Angular Rate> x: %6f  y: %6f  z: %6f" % (
            spatialData.AngularRate[0], spatialData.AngularRate[1], spatialData.AngularRate[2]))
        if len(spatialData.MagneticField) > 0:
            print("Magnetic Field> x: %6f  y: %6f  z: %6f" % (
            spatialData.MagneticField[0], spatialData.MagneticField[1], spatialData.MagneticField[2]))
        print("Time Span> Seconds Elapsed: %i  microseconds since last packet: %i" % (
        spatialData.Timestamp.seconds, spatialData.Timestamp.microSeconds))

    print("------------------------------------------")


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


def getCol(data, column):
    return np.array(data)[:,column].tolist()


# Effectively the main function...
def update():
    global listRaw, count
    count += 1
    listRaw.append(getNextData())
    if (listRaw[-1] == None):
        listRaw = listRaw[:-1]
    if (listRaw.__len__() >= numSamplesMax):
        listRaw = listRaw[-numSamplesMax:]

    if (count == updateEvery):
#uncomment to see filtered

        gr.updatePlot(graphData, fl.runLowPassFilter(filter10Hz, getCol(listRaw, 1)), getCol(listRaw, 0))
#comment to not see raw
    #    gr.updatePlot(graphData, getCol(listRaw, 1), getCol(listRaw, 0))
    count = count%updateEvery
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
    listCrude.append([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    #                  gx,  gy,  gz,  tx,  ty,  tz,  qw,  qx,  qy,  qz]

    listFiltered.append(listCrude[0])

    ###
    # Open the data file or connect to the IMU
    ###
    global dataFile, fileLocale
    if (inputType == "file"):
        dataFile = open(fileLocale, "r")
    else:
        try:
            # logging example, uncomment to generate a log file
            # spatial.enableLogging(PhidgetLogLevel.PHIDGET_LOG_VERBOSE, "phidgetlog.log")

            spatial.setOnAttachHandler(SpatialAttached)
            spatial.setOnDetachHandler(SpatialDetached)
            spatial.setOnErrorhandler(SpatialError)
            spatial.setOnSpatialDataHandler(SpatialData)
        except PhidgetException as e:
            print("Phidget Exception %i: %s" % (e.code, e.details))
            print("Exiting....")
            exit(1)

        print("Opening phidget object....")

        try:
            spatial.openPhidget()
        except PhidgetException as e:
            print("Phidget Exception %i: %s" % (e.code, e.details))
            print("Exiting....")
            exit(1)

        print("Waiting for attach....")

        try:
            spatial.waitForAttach(10000)
        except PhidgetException as e:
            print("Phidget Exception %i: %s" % (e.code, e.details))
            try:
                spatial.closePhidget()
            except PhidgetException as e:
                print("Phidget Exception %i: %s" % (e.code, e.details))
                print("Exiting....")
                exit(1)
            print("Exiting....")
            exit(1)
        else:
            spatial.setDataRate(4)
            DisplayDeviceInfo()
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
    if (listRaw.__len__() >= numSamplesMax):
        listRaw = listRaw[-numSamplesMax:]
        break
    #print(listRaw.__len__())
    if (inputType != "file"):

        print("Press Enter to quit....")

        chr = sys.stdin.read(1)

        print("Closing...")

        try:
            spatial.closePhidget()
        except PhidgetException as e:
            print("Phidget Exception %i: %s" % (e.code, e.details))
            print("Exiting....")
            exit(1)

        print("Done.")


# Update the graph data
timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(0)










































































