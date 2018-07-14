import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D #<-- Note the capitalization!

# Pose representation (simplified version)

################# Pose related constants #######################

N_JOINTS = 15

# Joint indexes
RIGHT_SHOULDER = 0
LEFT_SHOULDER = 1
NECK = 2
CENTRAL_TORSO = 3
RIGHT_HIP = 4
LEFT_HIP = 5
RIGHT_ELBOW = 6
LEFT_ELBOW = 7
RIGHT_HAND = 8
LEFT_HAND = 9
RIGHT_KNEE = 10
LEFT_KNEE = 11
RIGHT_FOOT = 12
LEFT_FOOT = 13
HEAD = 14

# switched to single reference system, better experimental results
# (see Pose_old.py for the previous system)
REFERENCE = CENTRAL_TORSO

############## Class definitions ################

class MetricPose:

    joints_x = []
    joints_y = []
    joints_z = []
    extended = []

    def __init__(self, vx, vy, vz):
        self.joints_x = vx
        self.joints_y = vy
        self.joints_z = vz

    def getJointsX(self):
        return self.joints_x

    def getJointsY(self):
        return self.joints_y

    def getJointsZ(self):
        return self.joints_z

    def getJoint(self, num):
        return [self.joints_x[num], self.joints_y[num],self.joints_z[num]]

    def getDataRow(self):
        return self.joints_x + self.joints_y + self.joints_z + self.extended

    def addExtension(self, value):
        self.extended.append(value)

    def metricToSpherical(self):
        pose_spherical = SphericalPose()
        for joint in range(0, N_JOINTS):
            ## convert joint's neighbor to local spherical coordinates
            ## base (torso) joints: use CENTRAL_TORSO as reference system
            reference = REFERENCE
            if joint != reference:  # dont add zeros for the central reference
                local_x = self.joints_x[joint] - self.joints_x[reference]
                local_y = self.joints_y[joint] - self.joints_y[reference]
                local_z = self.joints_z[joint] - self.joints_z[reference]
                r = math.sqrt(local_x ** 2 + local_y ** 2 + local_z ** 2)
                # theta:
                theta = math.atan2(local_y, local_x)
                pose_spherical.addTheta(theta)
                # phi:
                phi = math.atan2(local_z, r)
                pose_spherical.addPhi(phi)
        return pose_spherical

    def savePosePlot(self, name):
        fig = plt.figure()
        ax = Axes3D(fig)
        min_ax = min(min(self.joints_x), min(self.joints_y), min(self.joints_z))
        max_ax = max(max(self.joints_x), max(self.joints_y), max(self.joints_z))
        ax.set_xlim(min_ax, max_ax)
        ax.set_ylim(min_ax, max_ax)
        ax.set_zlim(min_ax, max_ax)
        ax.scatter(self.joints_x, self.joints_z, self.joints_y)
        plt.savefig(name)


class SphericalPose:
    pose_theta = []
    pose_phi = []
    pose_extended = []

    def __init__(self):
        self.pose_theta = []
        self.pose_phi = []
        self.pose_extended = []

    def addTheta(self, value):
        self.pose_theta.append(value)

    def addPhi(self, value):
        self.pose_phi.append(value)

    def addExtension(self, value):
        self.pose_extended.append(value)

    def getJoint(self, num):
        return [self.pose_theta[num], self.pose_phi[num]]

    def getDataRow(self):
        return self.pose_theta + self.pose_phi + self.pose_extended

    def setTheta(self,joint,value):
        self.pose_theta[joint] = value

    def setPhi(self,joint,value):
        self.pose_phi[joint] = value

    def getTheta(self,joint):
        return self.pose_theta[joint]

    def getPhi(self,joint):
        return self.pose_phi[joint]

    def addCosines(self):
        for value in self.pose_theta:
            self.addExtension(math.cos(value))
        for value in self.pose_phi:
            self.addExtension(math.cos(value))