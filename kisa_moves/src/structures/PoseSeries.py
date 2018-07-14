import subprocess
import spherical_metrics as sm
import math

MOVING_AVG_SIZE = 3
SIMILAR_LIMIT = math.pi / 16

class MetricPoseSeries:
    series = []

    def __init__(self):
        self.series = []

    def getPose(self, t):
        return self.series[t]

    def getSize(self):
        return len(self.series)

    def addPose(self, pose):
        self.series.append(pose)

    def removePose(self, t):
        self.series.pop(t)

    def createMp4(self):
        for i in range(0,len(self.series)):
            self.series[i].savePosePlot('frame'+`i`.zfill(2))
        # ffmpeg -framerate 15 -i frame%02d.png -r 15 output.avi -y
        subprocess.call(['sh', 'createmp4.sh'])

    def getSphericalSeries(self):
        sp_series = SphericalPoseSeries()
        for pose in self.series:
            sp_pose = pose.metricToSpherical()
            sp_series.addPose(sp_pose)
        return sp_series

class SphericalPoseSeries:
    series = []

    def __init__(self):
        self.series = []

    def getPose(self, t):
        return self.series[t]

    def getSize(self):
        return len(self.series)

    def addPose(self, pose):
        self.series.append(pose)

    def removePose(self, t):
        self.series.pop(t)

    def addCosines(self):
        for pose in self.series:
            pose.addCosines()

    def calculateMovingAverages(self):
        for joint in range(0, 14):
            values_0 = []
            values_1 = []
            for t,pose in enumerate(self.series):
                frame = pose.getJoint(joint)
                values_0.append(frame[0])
                values_1.append(frame[1])
                if len(values_0) > MOVING_AVG_SIZE:
                    values_0.pop(0)
                    values_1.pop(0)
                if len(values_0) == MOVING_AVG_SIZE:
                    avg0 = sum(values_0) / MOVING_AVG_SIZE
                    avg1 = sum(values_1) / MOVING_AVG_SIZE
                    self.series[t-MOVING_AVG_SIZE/2].setTheta(joint,avg0)
                    self.series[t - MOVING_AVG_SIZE / 2].setPhi(joint,avg1)

    def removeFirstSimilarPoses(self):
        size = len(self.series)
        i = 0
        while i < size - 1:
            distances = []
            for joint in range(0,14):
                distances.append(sm.geodesicDistance(self.series[0].getJoint(joint)[0],
                                                     self.series[0].getJoint(joint)[1],
                                                     self.series[1].getJoint(joint)[0],
                                                     self.series[1].getJoint(joint)[1]))
            if max(distances) < SIMILAR_LIMIT:
                self.series.pop(0)
                i += 1
            else:
                # print "FirstRemoved: ", i
                break

    def removeFinalSimilarPoses(self):
        size = len(self.series)
        i = size - 1
        while i > 0:
            distances = []
            for joint in range(0, 14):
                distances.append(sm.geodesicDistance(self.series[i].getJoint(joint)[0],
                                                     self.series[i].getJoint(joint)[1],
                                                     self.series[i-1].getJoint(joint)[0],
                                                     self.series[i-1].getJoint(joint)[1]))
            if max(distances) < SIMILAR_LIMIT:
                self.series.pop(i)
                i -= 1
            else:
                # print "LastRemoved: ", i
                break

    def compressData(self):
        # TODO function for representative motion  maps
        return


    def addSpeedInfo2(self):
        TIME_WINDOWS = [1, 2, 3, 4, 5]
        for joint in range(0, 14):
            enum = enumerate(self.series)
            for i,item in enum:
                acc_0 = 0
                acc_1 = 0
                for tw in TIME_WINDOWS:
                    if i > tw - 1:
                        item.addExtension(self.series[i - tw].getTheta(joint))
                        item.addExtension(self.series[i - tw].getPhi(joint))
                        # else:
                        # value.append(None)
                        # value.append(None)
                        # First rows will be removed (missing values strategy is discarded)
        for i in range(0, min(len(self.series), max(TIME_WINDOWS))):
            self.series.pop(0)

    def addSpeedInfo(self):
        TIME_WINDOWS = [4, 8]
        for joint in range(0, 14):
            enum = enumerate(self.series)
            for i,item in enum:
                for tw in TIME_WINDOWS:
                    if i > tw - 1:
                        speed_0 = sm.angularSpeed(item.getTheta(joint), self.series[i - tw].getTheta(joint), tw)
                        speed_1 = sm.angularSpeed(item.getPhi(joint), self.series[i - tw].getPhi(joint), tw)
                        item.addExtension(speed_0)
                        item.addExtension(speed_1)
                        # else:
                        # value.append(None)
                        # value.append(None)
                        # First rows will be removed (missing values strategy is discarded)
        for i in range(0, min(len(self.series), max(TIME_WINDOWS))):
            self.series.pop(0)

    def getSet(self):
        return self.series