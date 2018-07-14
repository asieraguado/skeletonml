import tf
import rospy
from structures.Pose import MetricPose
import euclidean_metrics

class TFReader:

    _listener = None
    _sensor_frame_data = []
    _has_data = False

    def __init__(self):
        self._listener = tf.TransformListener()
        self._sensor_frame_data = []
        self._has_data = False

    def lookupJoints(self, joint_list, time):
        data = [0] * 15
        try:
            for i in range(0, len(joint_list)):
                data[i] = self._listener.lookupTransform("/openni_depth_frame", joint_list[i], time)[0]
        except tf.LookupException:
            print "No TF data. Waiting for calibration..."
            self._has_data = False
            return
        self._sensor_frame_data = data
        self._has_data = True

    def lookupData(self, time):
        self._sensor_frame_data = []
        # joint order is important!
        joints = ["/right_shoulder_1", "/left_shoulder_1", "/neck_1", "/torso_1", "/right_hip_1", "/left_hip_1",
                  "/right_elbow_1", "/left_elbow_1", "/right_hand_1", "/left_hand_1", "/right_knee_1", "/left_knee_1",
                  "/right_foot_1", "/left_foot_1", "/head_1"]
        self.lookupJoints(joints, time)

    def getCoordinateVectors(self):
        vx = []
        vy = []
        vz = []
        for joint in self._sensor_frame_data:
            vx.append(joint[1])
            vy.append(joint[2])
            vz.append(joint[0])
        return vx,vy,vz


    def getMetricPose(self):
        vx,vy,vz = self.getCoordinateVectors()
        pose = MetricPose(vx,vy,vz)
        return pose

    def getSphericalPose(self):
        metric = self.getMetricPose()
        spherical = metric.metricToSpherical()
        return spherical

    def hasData(self):
        return self._has_data

    def intervalSpeed(self, t1, t2):
        # TODO: test this function
        past = rospy.Time.now() - rospy.Duration(t1 + 0.05)
        now  = rospy.Time.now() - rospy.Duration(t2 + 0.05)
        self.lookupData(past)
        past_data = self._sensor_frame_data
        self.lookupData(now)
        now_data = self._sensor_frame_data
        distances = []
        for i in range(0,15):
            distances[i] = euclidean_metrics.euclideanDistance(past_data[i], now_data[i])
        max = max(distances)
        return max/(t1 - t2)

    def isMoving(self):
        interval = 2
        leftSpeed = self.intervalSpeed(interval, interval/2);
        rightSpeed = self.intervalSpeed(interval/2, 0);
        globalSpeed = self.intervalSpeed(interval, 0);
        weightedMean = (leftSpeed*0.25 + rightSpeed*0.75 + globalSpeed*1) / (0.25+0.75+1);
        return weightedMean