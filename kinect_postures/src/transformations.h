#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "exceptions.h"
#include <math.h>
#include <map>
#include <string>
#include <exception>

class transformations {
public:
	transformations();
	std::map<std::string,float> getAngleData();
	float* getAllAngles();
	float intervalSpeed(float t1, float t2);
	float averageSpeed(float t);
private:
	tf::TransformListener listener;
	tf::StampedTransform transforms[14];
    NoTrackerDataException notfdataex;
    void updateTransforms(ros::Time when);
};
