class PoseSet:
    set = []

    def __init__(self):
        self.set = []

    def getPose(self, i):
        return self.series[i]

    def addPose(self, pose):
        self.series.append(pose)

    def addSubSet(self,subset):
        self.set += subset

    def removePose(self, i):
        self.series.pop(i)

    def getDataFrame(self):
        data_frame = []
        for pose in self.set:
            data_frame.append(pose.getDataRow())
        return data_frame