#include "ros/ros.h"
#include "tf/transform_listener.h"
#include <iostream>

using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_hands_detector");
  ros::NodeHandle node;
  tf::TransformListener listener;
  ros::Rate rate(10.0);

  struct ststruct {
    bool leftHandRised;
    bool rightHandRised;
  } status;

  while (node.ok()){
    tf::StampedTransform leftHand, rightHand;
    try{
      // left hand relative position (right hand from camera view)
      listener.lookupTransform("/torso_1", "/right_hand_1",
                               ros::Time(0), leftHand);
      // right hand relative position (left hand from camera view)
      listener.lookupTransform("/torso_1", "/left_hand_1",
                               ros::Time(0), rightHand);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    if (leftHand.getOrigin().y() > 0 && !status.leftHandRised) {
      status.leftHandRised = true;
      cout << "left hand UP" << endl;
    }
    if (leftHand.getOrigin().y() <= 0 && status.leftHandRised) {
      status.leftHandRised = false;
      cout << "left hand DOWN" << endl;
    }
    if (rightHand.getOrigin().y() > 0 && !status.rightHandRised) {
      status.rightHandRised = true;
      cout << "right hand UP" << endl;
    }
    if (rightHand.getOrigin().y() <= 0 && status.rightHandRised) {
      status.rightHandRised = false;
      cout << "right hand DOWN" << endl;
    }
    rate.sleep();
  }
  return 0;
};
