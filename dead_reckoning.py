import quaternion as qt
import complementary_filter as cf
from helper_functions import *


class DeadReckon(object):

    accSmooth = 0.9
    prevAccActual = [0, 0, 0] #-0.0095
    prevAccSmooth = [0.0065, -0.007, 1.0198595]  # Filtered & Smoothed
    # prevAccSmooth = [0.005971, -0.0064823, 1.0198595] # Raw with no smoothing

    velSmooth = 0.995
    prevVelActual = [0, 0, 0]
    prevVelSmooth = [0, 0, 0]

    posSmooth = 1
    prevPosActual = [0, 0, 0]
    prevPosSmooth = [0, 0, 0]

    # Ensure angles remain within 0 - 2pi range
    def angleRange(self, angle):
        if angle < 0:
            angle += int(1 + abs(angle) / 360) * 360
        elif angle > 360:
            angle -= int(    abs(angle) / 360) * 360
        return angle

    # Generate a rotation matrix from the roll pitch & yaw values
    def getRotationMatrix(self, x, y, z):
        rotX = [[1, 0, 0], [0, cos(d2r(x)), -sin(d2r(x))], [0, sin(d2r(x)), cos(d2r(x))]]
        rotY = [[cos(d2r(y)), 0, sin(d2r(y))], [0, 1, 0], [-sin(d2r(y)), 0, cos(d2r(y))]]
        rotZ = [[cos(d2r(z)), -sin(d2r(z)), 0], [sin(d2r(z)), cos(d2r(z)), 0], [0, 0, 1]]
        rot = np.matrix(rotZ) * np.matrix(rotY) * np.matrix(rotX)
        return rot.tolist()

    # Perform dead reckoning on the provided raw data
    def doDeadReckoning(self, prevComplete, raw, useComplimentaryFilter=False):
        # Ensure that prevComplete isn't modified anywhere by copying the data over beforehand
        delTime = raw[0] - prevComplete[0]
        complete = prevComplete * 1  # Deep copy items over

        # Update the time
        complete[0] = raw[0]

        # Update the Euler orientation
        if useComplimentaryFilter is False:
            orientation = integrateGyro(prevComplete[13:16], raw[4:7], delTime)
            for i in range(0, 3):
                complete[i + 10] = raw[i + 4]
                complete[i + 13] = orientation[i]
        else:
            orientation = cf.doComplementaryFilter(prevComplete, raw)
            for i in range(0, 3):
                complete[i + 10] = orientation[i+3]
                complete[i + 13] = orientation[i]
        # orientation[2] = 0
        # orientation[0] *= pi/180
        # orientation[1] *= pi/180
        # orientation[2] *= pi/180
        complete[16:20] = qt.euler_to_quat(orientation[0:3])

        # UPDATE THE QUATERNION ORIENTATION
        # Small angle Euler angles do not work over time
        # Quaternions do work, and are safer as they don't suffer from gimbal lock
        # Choose between the 2 methods below:

        # dcm = np.array(getRotationMatrix(orientation[0], orientation[1], orientation[2]))
        dcm = np.array(qt.quat_to_dcm(complete[16:20]))

        # Re-orientate the accelerometer values based on the IMU orientation
        self.prevAccActual = np.array([raw[1], raw[2], raw[3]])
        # self.prevAccActual = dcm.dot(self.prevAccActual.transpose())

        # Update the acceleration, velocity & position info
        g = -9.81
        for i in range(0, 3):
            # Acceleration
            self.prevAccSmooth[i] = expAvg(self.prevAccSmooth[i], self.prevAccActual[i], self.accSmooth)
            self.prevAccActual[i] = self.prevAccActual[i]*g-self.prevAccSmooth[i]*g
            complete[i + 1] = self.prevAccActual[i]

            # Velocity += a*t
            self.prevVelActual[i] += self.prevAccActual[i] * delTime
            self.prevVelSmooth[i] = expAvg(self.prevVelSmooth[i], self.prevVelActual[i], self.velSmooth)
            complete[i + 4] = self.prevVelActual[i] - self.prevVelSmooth[i]

            # Position += v*t - 0.5*a*t^2
            self.prevPosActual[i] += (complete[i+4] - 0.5 * self.prevAccActual[i] * delTime) * delTime
            self.prevPosSmooth[i] = expAvg(self.prevPosSmooth[i], self.prevPosActual[i], self.posSmooth)
            complete[i + 7] = self.prevPosActual[i] - self.prevPosSmooth[i]

        return complete
