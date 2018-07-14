from structures import Pose, PoseSeries

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

def loadPoseXYZSeries(action, subject, experiment):
    dataDir = "/home/asier/robotak/kinect/datasets/MSRA3D/skeleton/"
    dataFile = "a"+`action`.zfill(2)+"_s"+`subject`.zfill(2)+"_e"+ \
        `experiment`.zfill(2)+"_skeleton3D.txt"
    joints_x = []
    joints_y = []
    joints_z = []
    f = open(dataDir+dataFile)
    for line in f:
        values = line.split()
        joints_x.append(float(values[0]))
        joints_y.append(float(values[1]))
        joints_z.append(float(values[2]))
    poses_x = []
    poses_y = []
    poses_z = []
    for i in xrange(0,len(joints_x),20):
        pose_x = []
        pose_y = []
        pose_z = []
        for joint in USED_JOINTS:
            pose_x.append(joints_x[i+joint])
            pose_y.append(joints_y[i+joint])
            pose_z.append(joints_z[i+joint])
        poses_x.append(pose_x)
        poses_y.append(pose_y)
        poses_z.append(pose_z)
    return [poses_x, poses_y, poses_z]

def loadPoseSeries(action, subject, experiment):
    posesXYZ = loadPoseXYZSeries(action, subject, experiment)
    poses_x = posesXYZ[0]
    poses_y = posesXYZ[1]
    poses_z = posesXYZ[2]
    poses = PoseSeries.MetricPoseSeries()
    for i in range(0,len(poses_x)):
        pose = Pose.MetricPose(poses_x[i], poses_y[i], poses_z[i])
        poses.addPose(pose)
    return poses