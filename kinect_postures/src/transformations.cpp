#include "transformations.h"
#include <iostream>

const std::string angle_attributes[42] = 
{
     "head_x",
	 "head_y",  
	 "head_z" , 
	 "neck_x" ,
	 "neck_y" ,
	 "neck_z" ,
	 "left_shoulder_x" ,
	 "left_shoulder_y" ,
	 "left_shoulder_z" ,
	 "left_elbow_x" ,
	 "left_elbow_y" ,
	 "left_elbow_z" ,
	 "left_hand_x" ,
	 "left_hand_y" ,
	 "left_hand_z" ,
	 "left_hip_x" ,
	 "left_hip_y" ,
	 "left_hip_z" ,
	 "left_knee_x" ,
	 "left_knee_y" ,
	 "left_knee_z" ,
	 "left_foot_x" ,
	 "left_foot_y" ,
	 "left_foot_z" ,
	 "right_shoulder_x" ,
	 "right_shoulder_y" ,
	 "right_shoulder_z" ,
	 "right_elbow_x" ,
	 "right_elbow_y" ,
	 "right_elbow_z" ,
	 "right_hand_x" ,
	 "right_hand_y" ,
	 "right_hand_z" ,
	 "right_hip_x" ,
	 "right_hip_y" ,
	 "right_hip_z" ,
	 "right_knee_x" ,
	 "right_knee_y" ,
	 "right_knee_z" ,
	 "right_foot_x" ,
	 "right_foot_y" ,
	 "right_foot_z" ,
};

transformations::transformations() {
}

void transformations::updateTransforms(ros::Time when) {
    try{
      // Read all relative positions from torso
/*
      listener.waitForTransform("/torso_1", "/head_1", when, ros::Duration(0.2));
      listener.waitForTransform("/torso_1", "/neck_1", when, ros::Duration(0.2));
      listener.waitForTransform("/torso_1", "/left_shoulder_1", when, ros::Duration(0.2));
      listener.waitForTransform("/torso_1", "/left_elbow_1", when, ros::Duration(0.2));
      listener.waitForTransform("/torso_1", "/left_hand_1", when, ros::Duration(0.2));
      listener.waitForTransform("/torso_1", "/left_hip_1", when, ros::Duration(0.2));
      listener.waitForTransform("/torso_1", "/left_knee_1", when, ros::Duration(0.2));
      listener.waitForTransform("/torso_1", "/left_foot_1", when, ros::Duration(0.2));
      listener.waitForTransform("/torso_1", "/right_shouder_1", when, ros::Duration(0.2));
      listener.waitForTransform("/torso_1", "/right_elbow_1", when, ros::Duration(0.2));
      listener.waitForTransform("/torso_1", "/right_hand_1", when, ros::Duration(0.2));
      listener.waitForTransform("/torso_1", "/right_hip_1", when, ros::Duration(0.2));
      listener.waitForTransform("/torso_1", "/right_knee_1", when, ros::Duration(0.2));
      listener.waitForTransform("/torso_1", "/right_foot_1", when, ros::Duration(0.2));
*/
      listener.lookupTransform("/torso_1", "/head_1", when, transforms[0]);
      listener.lookupTransform("/torso_1", "/neck_1",  when, transforms[1]);
      listener.lookupTransform("/torso_1", "/left_shoulder_1",  when, transforms[2]);
      listener.lookupTransform("/torso_1", "/left_elbow_1",  when, transforms[3]);
      listener.lookupTransform("/torso_1", "/left_hand_1",  when, transforms[4]);
      listener.lookupTransform("/torso_1", "/left_hip_1",  when, transforms[5]);
      listener.lookupTransform("/torso_1", "/left_knee_1",  when, transforms[6]);
      listener.lookupTransform("/torso_1", "/left_foot_1",  when, transforms[7]);
      listener.lookupTransform("/torso_1", "/right_shoulder_1",  when, transforms[8]);
      listener.lookupTransform("/torso_1", "/right_elbow_1",  when, transforms[9]);
      listener.lookupTransform("/torso_1", "/right_hand_1",  when, transforms[10]);
      listener.lookupTransform("/torso_1", "/right_hip_1",  when, transforms[11]);
      listener.lookupTransform("/torso_1", "/right_knee_1",  when, transforms[12]);
      listener.lookupTransform("/torso_1", "/right_foot_1",  when, transforms[13]);
    }
    catch (tf::TransformException &ex) {
      throw notfdataex;
    }
}

std::map<std::string, float> transformations::getAngleData() {
    try {
        updateTransforms(ros::Time(0));
        std::map<std::string, float> data;
        for (int i=0; i<14; i++) {
            data[angle_attributes[3*i]]   = atan2(transforms[i].getOrigin().y(), transforms[i].getOrigin().x());
            data[angle_attributes[3*i+1]] = atan2(transforms[i].getOrigin().x(), transforms[i].getOrigin().y());
            data[angle_attributes[3*i+2]] = atan2(transforms[i].getOrigin().x(), transforms[i].getOrigin().z());
        }
        return data;
    }
    catch (NoTrackerDataException &ex) {
        throw ex;
    }
}

float* transformations::getAllAngles() {
    try {
        updateTransforms(ros::Time(0));
        static float angles[42];
        for (int i=0; i<14; i++) {
            angles[3*i]   = atan2(transforms[i].getOrigin().y(), transforms[i].getOrigin().x());
            angles[3*i+1] = atan2(transforms[i].getOrigin().x(), transforms[i].getOrigin().y());
            angles[3*i+2] = atan2(transforms[i].getOrigin().x(), transforms[i].getOrigin().z());
        }
        return angles;
    }
    catch (NoTrackerDataException &ex) {
        throw ex;
    }
}

float transformations::intervalSpeed(float t1, float t2) {
    try {
        ros::Time past = ros::Time::now() - ros::Duration(t1 + 0.05);
        ros::Time now  = ros::Time::now() - ros::Duration(t2 + 0.05);
        tf::StampedTransform oldtransforms[14];
        updateTransforms(past);
        for (int i=0; i<14; i++)
            oldtransforms[i] = transforms[i];
        updateTransforms(now);
        // euclidean distance. Use only 3,4 for tracking movements of the right arm.
        float dist[14];
        int max = 3;
        for (int i=3; i<5; i++) {
            dist[i] = 0;
            dist[i] += (transforms[i].getOrigin().x()-oldtransforms[i].getOrigin().x()) * (transforms[i].getOrigin().x()-oldtransforms[i].getOrigin().x());
            dist[i] += (transforms[i].getOrigin().y()-oldtransforms[i].getOrigin().y()) * (transforms[i].getOrigin().y()-oldtransforms[i].getOrigin().y());
            dist[i] += (transforms[i].getOrigin().z()-oldtransforms[i].getOrigin().z()) * (transforms[i].getOrigin().z()-oldtransforms[i].getOrigin().z());
            dist[i] = sqrt(dist[i]);
            if ( dist[i] > dist[max] )   max = i;
        }
        return dist[max]/(t1-t2);
    }
    catch (NoTrackerDataException &ex) {
        throw ex;
    }
}

float transformations::averageSpeed(float t) {
    try {
        float leftSpeed = intervalSpeed(t, t/2);
        float rightSpeed = intervalSpeed(t/2, 0);
        float globalSpeed = intervalSpeed(t, 0);
        float weightedMean = (leftSpeed*0.25 + rightSpeed*0.75 + globalSpeed*1) / (0.25+0.75+1);
        return weightedMean;
    }
    catch (NoTrackerDataException &ex) {
        throw ex;
    }
}


