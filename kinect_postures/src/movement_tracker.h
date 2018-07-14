#ifndef MOVEMENT_TRACKER_H
#define MOVEMENT_TRACKER_H

#define N_INTERMEDIATE 8
#define MODEL_POSTURES "models/posture_classifiers/arms"
#define MODEL_MOVEMENT "models/movements/right_arm"
#define MODEL_FINAL    "models/static_postures/right_arm_up"

#define STATE_WAIT 0
#define STATE_FIRSTPOS 1
#define STATE_MOVING 2
#define STATE_NOCAL -1

#include "ros/ros.h"
#include "Move.h"
#include "std_msgs/Int8.h"
#include "tf/transform_listener.h"
#include "transformations.h"
#include "classifier.h"
#include "clustermodel.h"
#include <iostream>

class movement_tracker
{
public:
	// constructor. Argument: ROS main node for the publisher.
    movement_tracker(ros::NodeHandle rosnode);

	// Run an iteration of posture_tracker. If OpenNI tracker data is available,
	//  it will generate the necessary messages.
    void run();

private:
	// State of previous iteration.
    int state;

	// ROS main node, posture information will be published here.
    ros::NodeHandle node;

	// 'transformations' object to save tf info and calculate joint angles.
    transformations current_tf;

	// current dataset of joint (string) - angle (float,rads) relationships.
    std::map<std::string, float> current_dataset;

	// Variable for the current posture, for the movement tracker.
    std::string current_posture;


    float current_speed;
    classifier myclassifier;
    clustermodel mymovemodel;
    centroid finalposmodel;
    float intermediates[N_INTERMEDIATE];
    bool was_calibrated;
//  ros::Publisher guide_publisher;
    ros::Publisher status_publisher;
    ros::Publisher moves_publisher;

    void update_instant();
    float intermediate_accuracy();
    float final_accuracy();
    void publish_move_info(bool correct, float interaccuracy, float finalaccuracy);
    void publish_status(int state);
    void wait_actions();
    void firstpos_actions();
    void moving_actions();
};

#endif // MOVEMENT_TRACKER_H
