#@todo initialise the complete data set
#@todo create a main function? or whatever pythons version of it is
#@todo plot the pretties
#@todo filter the data...

####################################################
################### DEPENDENCIES ###################
####################################################
import numpy as np
import quaternion as qt


####################################################
################# GLOBAL CONSTANTS #################
####################################################
inputType = "file"
fileLocale = "C:/Users/Student/Desktop/Uni/IMU/ROCO503x/IMU_Stationary.txt"
numSamplesMax = 100


####################################################
################# GLOBAL VARIABLES #################
####################################################
dataFile = None
listRaw = []
listCrude = []
listFiltered = []


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
        # Try to read from the file. If at the end, then return None
        try:
            data = dataFile.readline()[:-1]  # Considered an array of chars, so [:-1] removes the last character
        except EOFError:
            data = None

    else:
        data = None
    data = [float(i) for i in data.split(",")]
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

    return


####################################################
################# MAIN FUNCTION(S) #################
####################################################
def main():
    # Variable declarations
    global listRaw, listCrude, listFiltered, numSamplesMax

    # Initialize the system
    init()

    # Perform the main code
    while True:

        listRaw.append(getNextData())
        if (listRaw[-1] == None):
            listRaw = listRaw[:-1]
        print(listRaw.__len__())

        # If enough samples have been collected, do more analysis
        if (listRaw.__len__() >= numSamplesMax):
            listRaw = listRaw[1-numSamplesMax:]


if __name__ == '__main__':
    main()









































































