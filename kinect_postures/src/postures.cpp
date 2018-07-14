#include "ros/ros.h"
#include "posture_tracker.h"

int main(int argc, char** argv){

  ros::init(argc, argv, "postures");
  ros::NodeHandle node;
  posture_tracker postures_machine(node);

  while (node.ok()) {
    postures_machine.run();
  }

}
