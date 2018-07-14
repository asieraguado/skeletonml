#!/usr/bin/env python

import rospy
import pickle
from Transformations import TFReader
from std_msgs.msg import String
from structures.PoseSeries import SphericalPoseSeries
from HistogramClassifier import HistogramClassifier

MODEL_DIR = "/home/asier/robotak/kinect/models/actions/msra3d_on5_rf50.pkl"

msra_actions = ["nothing", "(1)high arm wave", "(2)horizontal arm wave", "(3)hammer", "(4)hand catch",
                "(5)forward punch", "(6)high throw", "(7)draw x", "(8)draw tick", "(9)draw circle", "(10)hand clap",
                "(11)two hand wave", "(12)side - boxing", "(13)bend", "(14)forward kick", "(15)side kick", "(16)jogging",
                "(17)tennis swing", "(18)tennis serve", "(19)golf swing", "(20)pickup & throw"]

utka_actions = ["nothing", "walk", "sitDown", "standUp", "pickUp", "carry",
            "throw", "push", "pull", "waveHands", "clapHands"]

def monitor():
    pub = rospy.Publisher('human_moves', String, queue_size=10) # I'll use it later
    rospy.init_node('move_predictor', anonymous=True)
    rate = rospy.Rate(120) # 120hz listener
    pose_reader = TFReader()
    count = 0
    rospy.loginfo("Running...")
    series = SphericalPoseSeries()
    model_file = open(MODEL_DIR)
    instant_model = pickle.load(model_file)
    classifier = HistogramClassifier(instant_model, None)
    while not rospy.is_shutdown():
        pose_reader.lookupData(rospy.Time(0))
        if pose_reader.hasData():
            pose = pose_reader.getSphericalPose()
            series.addPose(pose)
            if series.getSize() == 15: # 15 frames (1 second): classify
                #series.calculateMovingAverages()
                series.addSpeedInfo()
                prediction = classifier.classifySeries(series)
                series = SphericalPoseSeries()
                print "Detected action: ", msra_actions[prediction]
        rate.sleep()

if __name__ == '__main__':
    try:
        monitor()
    except rospy.ROSInterruptException:
        pass