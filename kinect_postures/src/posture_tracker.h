#ifndef POSTURE_TRACKER_H
#define POSTURE_TRACKER_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "transformations.h"
#include "classifier.h"
#include "clustermodel.h"
#include "randomforest.h"
#include <string>

#define MODEL_ONLY_POSTURES "models/posture_classifiers/random_trees/body"

// This class contains all methods and variables for recognizing user postures.
class posture_tracker
{
public:
	// constructor. Argument: ROS main node for the publisher.
    posture_tracker(ros::NodeHandle rosnode);

	// Run an iteration of posture_tracker. If OpenNI tracker data is available,
	// it will classify the current posture.
    void run();

private:
	// ROS main node, posture information will be published here.
    ros::NodeHandle node;

	// 'transformations' object to save tf info and calculate joint angles.
    transformations current_tf;

	// current dataset of joint (string) - angle (float,rads) relationships.
    std::map<std::string, float> current_dataset;

	// Previous posture is saved to detect when it changes.
    std::string previous_posture;

	// Postures publisher in topic "/kinect_postures/postures" (see constructor).
    ros::Publisher postures_publisher;

	// The classifier is a random forest (more accurate than a single tree).
    randomforest forest_classifier;

	// Publish posture info in postures_publisher topic.
    void publish_posture(std::string posture);
};

#endif // POSTURE_TRACKER_H
