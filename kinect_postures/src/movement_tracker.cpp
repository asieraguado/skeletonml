#include "movement_tracker.h"


movement_tracker::movement_tracker(ros::NodeHandle rosnode)
{
    node = rosnode;
    myclassifier.createFromFile(MODEL_POSTURES);
    mymovemodel.createFromFile(MODEL_MOVEMENT);
    finalposmodel.createFromFile(MODEL_FINAL);
    state = 0;
    for (int i=0; i<N_INTERMEDIATE; i++) intermediates[i] = 0;
    was_calibrated = false;
//  guide_publisher  = node.advertise<std_msgs::String>("/kinect_postures/guide", 1000);
    status_publisher = node.advertise<std_msgs::Int8>("/kinect_postures/status", 1000);
    moves_publisher  = node.advertise<kinect_postures::Move>("/kinect_postures/moves", 1000);
}


float movement_tracker::intermediate_accuracy() {
    int ipcount = 0;
    float interaccuracy = 0;
    for (int i=0; i<N_INTERMEDIATE; i++) {
        if (intermediates[i] > 0) {
            ipcount++;
            interaccuracy += intermediates[i];
        }
    }
    return ( interaccuracy / N_INTERMEDIATE );
}


float movement_tracker::final_accuracy() {
    return ( 1 - finalposmodel.getDistanceFrom(current_dataset) / ( 2*PI*sqrt(finalposmodel.getDimensions()) ) );
}


void movement_tracker::update_instant() {
    try {
        current_speed = current_tf.averageSpeed(2);
        current_dataset = current_tf.getAngleData();
        current_posture = myclassifier.classify(current_dataset);
    }
    catch (NoTrackerDataException &ex) {
        throw ex;
    }
}


void movement_tracker::publish_move_info(bool correct, float interaccuracy, float finalaccuracy) {
    kinect_postures::Move mmsg;
    mmsg.correct = correct;
    mmsg.inter   = interaccuracy;
    mmsg.final   = finalaccuracy;
    moves_publisher.publish(mmsg);
}


void movement_tracker::publish_status(int state) {
    std_msgs::Int8 status;
    status.data = state;
    status_publisher.publish(status);
}


void movement_tracker::wait_actions() {
    if (! was_calibrated) {
        publish_status(STATE_WAIT);
        was_calibrated = true;
    }
    if (current_speed < 0.2 && current_posture == "none") {
        state = STATE_FIRSTPOS;
        publish_status(STATE_FIRSTPOS);
        for (int i=0; i<N_INTERMEDIATE; i++)
          intermediates[i] = 0;
    }
}


void movement_tracker::firstpos_actions() {
    if (current_speed > 0.2) {
        state = STATE_MOVING;
        publish_status(STATE_MOVING);
    }
}


void movement_tracker::moving_actions() {
    if (current_speed < 0.2 && current_posture == "right-arm-up") {
        state = STATE_WAIT;
        publish_status(STATE_WAIT);
        publish_move_info(true, intermediate_accuracy(), final_accuracy());
    }
    else if (current_speed < 0.1 && current_posture != "right-arm-up") {
        state = STATE_WAIT;
        publish_status(STATE_WAIT);
        float inter_accy = intermediate_accuracy();
        float final_accy = final_accuracy();
        publish_move_info(false, inter_accy, final_accy);
    }
    else {
        int cluster  = mymovemodel.getCluster(current_dataset);
        float accuracy = mymovemodel.getAccuracy(current_dataset, cluster);
        if (intermediates[cluster] < accuracy) intermediates[cluster] = accuracy;
    }
}


void movement_tracker::run() {
    try {
        update_instant();
        switch (state) {
        case STATE_WAIT:
            wait_actions();
        break;
        case STATE_FIRSTPOS:
            firstpos_actions();
        break;
        case STATE_MOVING:
            moving_actions();
        break;
        }
        ros::Duration(0.05).sleep();
      }
      catch (NoTrackerDataException &ex) {
          ROS_WARN(ex.what());
          publish_status(STATE_NOCAL);
          was_calibrated = false;
          ros::Duration(1.0).sleep();
      }
}
