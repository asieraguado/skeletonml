#include "ros/ros.h"
#include "tf/transform_listener.h"
#include <iostream>
#include <fstream>
#include <string>

#define LIMIT 300

using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "save_raw");
  ros::NodeHandle node;
  tf::TransformListener listener;
  ros::Rate rate(10.0);
  string name;
  tf::StampedTransform transforms[14];
/*
  tf::StampedTransform 
0	head,
1	neck,
2	left_shoulder,
3	left_elbow,
4	left_hand,
5	left_hip,
6	left_knee,
7	left_foot,
8	right_shoulder,
9	right_elbow,
10	right_hand,
11	right_hip,
12	right_knee,
13	right_foot;
*/
    cout << "Please enter the transformation name" << endl;
    cin >> name;
    cout << "Name OK. " << endl;
    cout << "Starting to read data in 10 seconds..." << endl;
    ros::Duration(10.0).sleep();
  
  for (int k=0; k<LIMIT; k++) {
   if (node.ok()) {
    try{
      // Read all relative positions from torso
      listener.lookupTransform("/torso_1", "/head_1", ros::Time(0), transforms[0]);
      listener.lookupTransform("/torso_1", "/neck_1",  ros::Time(0), transforms[1]);
      listener.lookupTransform("/torso_1", "/left_shoulder_1",  ros::Time(0), transforms[2]);
      listener.lookupTransform("/torso_1", "/left_elbow_1",  ros::Time(0), transforms[3]);
      listener.lookupTransform("/torso_1", "/left_hand_1",  ros::Time(0), transforms[4]);
      listener.lookupTransform("/torso_1", "/left_hip_1",  ros::Time(0), transforms[5]);
      listener.lookupTransform("/torso_1", "/left_knee_1",  ros::Time(0), transforms[6]);
      listener.lookupTransform("/torso_1", "/left_foot_1",  ros::Time(0), transforms[7]);
      listener.lookupTransform("/torso_1", "/right_shoulder_1",  ros::Time(0), transforms[8]);
      listener.lookupTransform("/torso_1", "/right_elbow_1",  ros::Time(0), transforms[9]);
      listener.lookupTransform("/torso_1", "/right_hand_1",  ros::Time(0), transforms[10]);
      listener.lookupTransform("/torso_1", "/right_hip_1",  ros::Time(0), transforms[11]);
      listener.lookupTransform("/torso_1", "/right_knee_1",  ros::Time(0), transforms[12]);
      listener.lookupTransform("/torso_1", "/right_foot_1",  ros::Time(0), transforms[13]);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      exit(1);
    }

    ofstream datafile;
    datafile.open ("rawdata", ios::app);
    datafile << name;
    for (int i=0; i<14; i++)
         datafile << transforms[i].getOrigin().x() << " " << transforms[i].getOrigin().y() << " " << transforms[i].getOrigin().z() << endl;
    datafile.close();
    cout << "Read frame (" << k << ")." << endl;
    rate.sleep();
   } else exit(1);
  }
  cout << "Read complete." << endl;
  return 0;
}
