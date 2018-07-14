#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "transformations.h"
#include <iostream>
#include <fstream>
#include <string>

#define LIMIT 50

using namespace std;

int main(int argc, char** argv){
  ros::init(argc, argv, "save_angle");
  ros::NodeHandle node;
  transformations current_tf;
  ros::Rate rate(10.0);
  string name;
  float* angles;

  cout << "Please enter the transformation name" << endl;
  cin >> name;
  cout << "Name OK. " << endl;
  cout << "Starting to read data in 10 seconds..." << endl;
  ros::Duration(10.0).sleep();

  for (int k=0; k<LIMIT; k++) {
   if (node.ok()) {
	angles = current_tf.getAllAngles();
    ofstream datafile;
    datafile.open ("angledata", ios::app);
    datafile << name;
    for (int i=0; i<42; i++)
         datafile << " " << angles[i];
    datafile << endl;
    datafile.close();

    cout << "Read (" << k << ")." << endl;

    rate.sleep();
   } 
	else exit(1);
  }
  cout << "Read complete." << endl;
  return 0;
}
