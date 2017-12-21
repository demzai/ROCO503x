#!/usr/bin/env python

"""Copyright 2010 Phidgets Inc.
This work is licensed under the Creative Commons Attribution 2.5 Canada License. 
To view a copy of this license, visit http://creativecommons.org/licenses/by/2.5/ca/
"""

__author__ = 'Adam Stelmack'
__version__ = '2.1.8'
__date__ = 'May 17 2010'

#Basic imports
from ctypes import *
import sys
#Phidget specific imports
from Phidgets.Phidget import Phidget
from Phidgets.PhidgetException import PhidgetErrorCodes, PhidgetException
from Phidgets.Events.Events import SpatialDataEventArgs, AttachEventArgs, DetachEventArgs, ErrorEventArgs
from Phidgets.Devices.Spatial import Spatial, SpatialEventData, TimeSpan
from Phidgets.Phidget import PhidgetLogLevel
import time




class IMU(object):

    #global iPrint
    #iPrint = True

    def __init__(self, print_text_in):

        self.dataList = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        #########self.print_text_in = print_text_in
        if (print_text_in == "print"):
            self.iPrint = True
            self.iDebug = True
        elif (print_text_in == "no_print"):
            self.iPrint = False
            self.iDebug = False
        elif (print_text_in == "some"):
            self.iPrint = False
            self.iDebug = True
        else:
            print("Please say what kind of data you wish to display, call spatialC.IMU() with either 'print', 'no_print', or 'some'. e.g. imuObj = spatialC.IMU('some')")
            time.sleep(1)
            exit(1)
        #self.iPrint = iPrint
        #Create an accelerometer object
        try:
            self.spatial = Spatial()
        except RuntimeError as e:
            if (iDebug):
                print("Runtime Exception: %s" % e.details)
                print("Exiting....")
            time.sleep(2)
            exit(1)


            # Main Program Code
        try:
            # logging example, uncomment to generate a log file
            # spatial.enableLogging(PhidgetLogLevel.PHIDGET_LOG_VERBOSE, "phidgetlog.log")

            self.spatial.setOnAttachHandler(self.SpatialAttached)
            self.spatial.setOnDetachHandler(self.SpatialDetached)
            self.spatial.setOnErrorhandler(self.SpatialError)
            self.spatial.setOnSpatialDataHandler(self.SpatialData)
        except PhidgetException as e:
            if (iDebug):
                print("Phidget Exception %i: %s" % (e.code, e.details))
                print("Exiting....")
            time.sleep(2)
            exit(1)
        if (self.iDebug):
            print("Opening phidget object....")

        try:
            self.spatial.openPhidget()
        except PhidgetException as e:
            if (self.iDebug):
                print("Phidget Exception %i: %s" % (e.code, e.details))
                print("Exiting....")
            time.sleep(2)
            exit(1)

        if (self.iDebug):
            print("Waiting for attach....")

        try:
            self.spatial.waitForAttach(10000)
        except PhidgetException as e:
            if (self.iDebug):
                print("Phidget Exception %i: %s" % (e.code, e.details))
            try:
                self.spatial.closePhidget()
            except PhidgetException as e:
                if (self.iDebug):
                    print("Phidget Exception %i: %s" % (e.code, e.details))
                    print("Exiting....")
                exit(1)
            if (self.iDebug):
                print("Exiting....")
            time.sleep(2)
            exit(1)
        else:
            self.spatial.setDataRate(4)
            self.DisplayDeviceInfo()
        """
        if (self.iDebug):
            print("Press Enter to quit....")
        
        # This is how the phidget used to shut down when enter was pressed. Find a way to re-integrate this!
        chr = sys.stdin.read(1)

        if (self.iDebug):
            print("Closing...")

        try:
            self.spatial.closePhidget()
        except PhidgetException as e:
            if (self.iDebug):
                print("Phidget Exception %i: %s" % (e.code, e.details))
                print("Exiting....")
            time.sleep(2)
            exit(1)

        if (self.iDebug):
            print("Done.")
        """
    #Information Display Function
    def DisplayDeviceInfo(self):
        if (self.iDebug):
            print("|------------|----------------------------------|--------------|------------|")
            print("|- Attached -|-              Type              -|- Serial No. -|-  Version -|")
            print("|------------|----------------------------------|--------------|------------|")
            print("|- %8s -|- %30s -|- %10d -|- %8d -|" % (self.spatial.isAttached(), self.spatial.getDeviceName(), self.spatial.getSerialNum(), self.spatial.getDeviceVersion()))
            print("|------------|----------------------------------|--------------|------------|")
            print("Number of Acceleration Axes: %i" % (self.spatial.getAccelerationAxisCount()))
            print("Number of Gyro Axes: %i" % (self.spatial.getGyroAxisCount()))
            print("Number of Compass Axes: %i" % (self.spatial.getCompassAxisCount()))

    def stopIMU(self):

        if (self.iDebug):
            print("Closing...")

        try:
            self.spatial.closePhidget()
        except PhidgetException as e:
            if (self.iDebug):
                print("Phidget Exception %i: %s" % (e.code, e.details))
                print("Exiting....")
            time.sleep(2)
            exit(1)

        if (self.iDebug):
            print("Done.")

    #Event Handler Callback Functions
    def SpatialAttached(self, e):
        attached = e.device
        if (self.iDebug):
            print("Spatial %i Attached!" % (attached.getSerialNum()))

    def SpatialDetached(self, e):
        detached = e.device
        if (self.iDebug):
            print("Spatial %i Detached!" % (detached.getSerialNum()))

    def SpatialError(self, e):
        try:
            source = e.device
            if (self.iDebug):
                print("Spatial %i: Phidget Error %i: %s" % (source.getSerialNum(), e.eCode, e.description))
        except PhidgetException as e:
            if (self.iDebug):
                print("Phidget Exception %i: %s" % (e.code, e.details))

    def SpatialData(self, e):
        source = e.device
        if (self.iPrint):
            print("Spatial %i: Amount of data %i" % (source.getSerialNum(), len(e.spatialData)))
        for index, spatialData in enumerate(e.spatialData):
            if (self.iPrint):
                print("=== Data Set: %i ===" % (index))
            if len(spatialData.Acceleration) > 0:
                self.dataList[1] = spatialData.Acceleration[0]
                self.dataList[2] = spatialData.Acceleration[1]
                self.dataList[3] = spatialData.Acceleration[2]
                if (self.iPrint):
                    print("Acceleration> x: %6f  y: %6f  z: %6f" % (spatialData.Acceleration[0], spatialData.Acceleration[1], spatialData.Acceleration[2]))
            if len(spatialData.AngularRate) > 0:
                self.dataList[4] = spatialData.AngularRate[0]
                self.dataList[5] = spatialData.AngularRate[1]
                self.dataList[6] = spatialData.AngularRate[2]
                if (self.iPrint):
                    print("Angular Rate> x: %6f  y: %6f  z: %6f" % (spatialData.AngularRate[0], spatialData.AngularRate[1], spatialData.AngularRate[2]))
            if len(spatialData.MagneticField) > 0:
                if (self.iPrint):
                    print("Magnetic Field> x: %6f  y: %6f  z: %6f" % (spatialData.MagneticField[0], spatialData.MagneticField[1], spatialData.MagneticField[2]))
            if (self.iPrint):
                print("Time Span> Seconds Elapsed: %i  microseconds since last packet: %i" % (spatialData.Timestamp.seconds, spatialData.Timestamp.microSeconds))
            seconds = float(spatialData.Timestamp.seconds + spatialData.Timestamp.microSeconds/1000000.0)
            self.dataList[0] = float(self.truncate(seconds, 5))
        if (self.iPrint):
            print("------------------------------------------")
        self.dataReady = True

    def getData(self):
        if (self.dataReady):
            self.dataReady = False
            return self.dataList



    def truncate(self, f, n):
        '''Truncates/pads a float f to n decimal places without rounding'''
        s = '{}'.format(f)
        if 'e' in s or 'E' in s:
            return '{0:.{1}f}'.format(f, n)
        i, p, d = s.partition('.')
        return '.'.join([i, (d + '0' * n)[:n]])
