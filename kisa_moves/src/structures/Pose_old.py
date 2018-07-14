import math
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d, Axes3D #<-- Note the capitalization!

# Pose representation (generic version, too much MSRA3D data set structure influence, unused)

################# Pose related constants ###########################################

# General constants
C_POSE_JOINTS = 20

# Joint indexes
RIGHT_SHOULDER = 0
LEFT_SHOULDER = 1
NECK = 2
CENTRAL_TORSO = 3
RIGHT_HIP = 4
LEFT_HIP = 5
RIGHT_ELBOW = 7
LEFT_ELBOW = 8
RIGHT_HAND = 9
LEFT_HAND = 10
RIGHT_KNEE = 13
LEFT_KNEE = 14
RIGHT_FOOT = 15
LEFT_FOOT = 16
HEAD = 19

USED_JOINTS = [RIGHT_SHOULDER, LEFT_SHOULDER, NECK, CENTRAL_TORSO, RIGHT_HIP,
               LEFT_HIP, RIGHT_ELBOW, LEFT_ELBOW, RIGHT_HAND, LEFT_HAND,
               RIGHT_KNEE, LEFT_KNEE, RIGHT_FOOT, LEFT_FOOT, HEAD]

# Used joint indexes
U_RIGHT_SHOULDER = 0
U_LEFT_SHOULDER = 1
U_NECK = 2
# U_CENTRAL_TORSO   = --- reference, not used in training/test
U_RIGHT_HIP = 3
U_LEFT_HIP = 4
U_RIGHT_ELBOW = 5
U_LEFT_ELBOW = 6
U_RIGHT_HAND = 7
U_LEFT_HAND = 8
U_RIGHT_KNEE = 9
U_LEFT_KNEE = 10
U_RIGHT_FOOT = 11
U_LEFT_FOOT = 12
U_HEAD = 13

# Data load constants
N_ACTIONS = 20
N_SUBJECTS = 10
N_EXPERIMENTS = 3

# %%%%%%%% Body joint adjacency model %%%%%%
ADJACENT = [RIGHT_ELBOW,
            LEFT_ELBOW,
            HEAD,
            0,
            RIGHT_KNEE,
            LEFT_KNEE,
            0,
            RIGHT_HAND,
            LEFT_HAND,
            0,
            0,
            0,
            0,
            RIGHT_FOOT,
            LEFT_FOOT,
            0,
            0,
            0,
            0,
            0
            ]
REFERENCE = [CENTRAL_TORSO,
             CENTRAL_TORSO,
             CENTRAL_TORSO,
             CENTRAL_TORSO,
             CENTRAL_TORSO,
             CENTRAL_TORSO,
             0,
             RIGHT_SHOULDER,
             LEFT_SHOULDER,
             RIGHT_ELBOW,
             LEFT_ELBOW,
             0,
             0,
             RIGHT_HIP,
             LEFT_HIP,
             RIGHT_KNEE,
             LEFT_KNEE,
             0,
             0,
             NECK
             ]

REFERENCE2 = [CENTRAL_TORSO,
              CENTRAL_TORSO,
              CENTRAL_TORSO,
              CENTRAL_TORSO,
              CENTRAL_TORSO,
              CENTRAL_TORSO,
              0,
              CENTRAL_TORSO,
              CENTRAL_TORSO,
              CENTRAL_TORSO,
              CENTRAL_TORSO,
              0,
              0,
              CENTRAL_TORSO,
              CENTRAL_TORSO,
              CENTRAL_TORSO,
              CENTRAL_TORSO,
              0,
              0,
              CENTRAL_TORSO
             ]


############## Class definitions ############

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
        for joint in USED_JOINTS:
            ## convert joint's neighbor to local spherical coordinates
            ## base (torso) joints: use CENTRAL_TORSO as reference system
            reference = REFERENCE2[joint]
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