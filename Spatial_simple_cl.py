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
        #########self.print_text_in = print_text_in
        if (print_text_in == "print"):
            self.iPrint = True
        elif (print_text_in == "no_print"):
            self.iPrint = False
        else:
            self.iPrint = True
        #self.iPrint = iPrint
        #Create an accelerometer object
        try:
            self.spatial = Spatial()
        except RuntimeError as e:
            if (iPrint):
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
            if (iPrint):
                print("Phidget Exception %i: %s" % (e.code, e.details))
                print("Exiting....")
            time.sleep(2)
            exit(1)
        if (self.iPrint):
            print("Opening phidget object....")

        try:
            self.spatial.openPhidget()
        except PhidgetException as e:
            if (self.iPrint):
                print("Phidget Exception %i: %s" % (e.code, e.details))
                print("Exiting....")
            time.sleep(2)
            exit(1)

        if (self.iPrint):
            print("Waiting for attach....")

        try:
            self.spatial.waitForAttach(10000)
        except PhidgetException as e:
            if (self.iPrint):
                print("Phidget Exception %i: %s" % (e.code, e.details))
            try:
                self.spatial.closePhidget()
            except PhidgetException as e:
                if (self.iPrint):
                    print("Phidget Exception %i: %s" % (e.code, e.details))
                    print("Exiting....")
                exit(1)
            if (self.iPrint):
                print("Exiting....")
            time.sleep(2)
            exit(1)
        else:
            self.spatial.setDataRate(4)
            self.DisplayDeviceInfo()

        if (self.iPrint):
            print("Press Enter to quit....")

        chr = sys.stdin.read(1)

        if (self.iPrint):
            print("Closing...")

        try:
            self.spatial.closePhidget()
        except PhidgetException as e:
            if (self.iPrint):
                print("Phidget Exception %i: %s" % (e.code, e.details))
                print("Exiting....")
            time.sleep(2)
            exit(1)

        if (self.iPrint):
            print("Done.")

    #Information Display Function
    def DisplayDeviceInfo(self):
        if (self.iPrint):
            print("|------------|----------------------------------|--------------|------------|")
            print("|- Attached -|-              Type              -|- Serial No. -|-  Version -|")
            print("|------------|----------------------------------|--------------|------------|")
            print("|- %8s -|- %30s -|- %10d -|- %8d -|" % (self.spatial.isAttached(), self.spatial.getDeviceName(), self.spatial.getSerialNum(), self.spatial.getDeviceVersion()))
            print("|------------|----------------------------------|--------------|------------|")
            print("Number of Acceleration Axes: %i" % (self.spatial.getAccelerationAxisCount()))
            print("Number of Gyro Axes: %i" % (self.spatial.getGyroAxisCount()))
            print("Number of Compass Axes: %i" % (self.spatial.getCompassAxisCount()))

    #Event Handler Callback Functions
    def SpatialAttached(self, e):
        attached = e.device
        if (self.iPrint):
            print("Spatial %i Attached!" % (attached.getSerialNum()))

    def SpatialDetached(self, e):
        detached = e.device
        if (self.iPrint):
            print("Spatial %i Detached!" % (detached.getSerialNum()))

    def SpatialError(self, e):
        try:
            source = e.device
            if (self.iPrint):
                print("Spatial %i: Phidget Error %i: %s" % (source.getSerialNum(), e.eCode, e.description))
        except PhidgetException as e:
            if (self.iPrint):
                print("Phidget Exception %i: %s" % (e.code, e.details))

    def SpatialData(self, e):
        source = e.device
        if (self.iPrint):
            print("Spatial %i: Amount of data %i" % (source.getSerialNum(), len(e.spatialData)))
        for index, spatialData in enumerate(e.spatialData):
            if (self.iPrint):
                print("=== Data Set: %i ===" % (index))
            if len(spatialData.Acceleration) > 0:
                if (self.iPrint):
                    print("Acceleration> x: %6f  y: %6f  z: %6f" % (spatialData.Acceleration[0], spatialData.Acceleration[1], spatialData.Acceleration[2]))
            if len(spatialData.AngularRate) > 0:
                if (self.iPrint):
                    print("Angular Rate> x: %6f  y: %6f  z: %6f" % (spatialData.AngularRate[0], spatialData.AngularRate[1], spatialData.AngularRate[2]))
            if len(spatialData.MagneticField) > 0:
                if (self.iPrint):
                    print("Magnetic Field> x: %6f  y: %6f  z: %6f" % (spatialData.MagneticField[0], spatialData.MagneticField[1], spatialData.MagneticField[2]))
            if (self.iPrint):
                print("Time Span> Seconds Elapsed: %i  microseconds since last packet: %i" % (spatialData.Timestamp.seconds, spatialData.Timestamp.microSeconds))
        if (self.iPrint):
            print("------------------------------------------")

    # if (self.iPrint):
    #     print("end of object")
    #time.sleep(2)
    #exit(0)
