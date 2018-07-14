#include "ros/ros.h"
#include "movement_tracker.h"
#include <iostream>

int main(int argc, char** argv){

  ros::init(argc, argv, "movement");
  ros::NodeHandle node;
  movement_tracker movement_machine(node);

  while (node.ok()) {
      movement_machine.run();
  }
}
