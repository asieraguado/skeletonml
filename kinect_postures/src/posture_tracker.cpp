#include "posture_tracker.h"

posture_tracker::posture_tracker(ros::NodeHandle rosnode)
{
    node = rosnode;
	forest_classifier.createFromFile(MODEL_ONLY_POSTURES);
    postures_publisher = node.advertise<std_msgs::String>("/kinect_postures/postures", 1000);
    previous_posture = "Unknown";
}


void posture_tracker::publish_posture(std::string posture) {
    std_msgs::String msg;
    msg.data = posture.data();
    postures_publisher.publish(msg);
}


void posture_tracker::run() {
    try {
        current_dataset = current_tf.getAngleData();
        std::string current_posture = forest_classifier.classify(current_dataset);
        if (current_posture != previous_posture) {
            publish_posture(current_posture);
            previous_posture = current_posture;
        }
        ros::Duration(0.1).sleep();
    }
      catch (NoTrackerDataException &ex) {
          ROS_WARN(ex.what());
          ros::Duration(1.0).sleep();
    }
}
