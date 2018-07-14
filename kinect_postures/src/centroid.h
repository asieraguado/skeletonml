#ifndef CENTROID_H
#define	CENTROID_H

#include <vector>
#include <string>
#include <map>
#include "exceptions.h"

#define PI 3.14159265358979323846
#define CENTROID_MAX_DIST 8.88577 /* 2*pi*sqrt(2) */

// Relationship between an attribute (joint name) and the angle value.
struct attribute {
    std::string name;
    float value;
};

// A centroid is a set of attribute-values (it can also be a single case),
//   mainly used for a cluster model.
class centroid {
public:
	// empty constructor
    centroid();
	// Constructor for a single case, with all attribute and values.
    centroid(std::vector<attribute> attrs);
	// Euclidean distance from the dataset to the centroid.
    float getDistanceFrom(std::map<std::string, float> dataset);
	// Add a new attribute and value.
    void addAttribute(std::string name, float value);
	// Create centroid from file.
	// 	File format: First line with an integer 'i', where 'i' is the number of attribute-values of the centroid.
	//               Rest of the lines: each line has a string and a float, where the string is the attribute and the float is the value. 
	void createFromFile(const char* file);
	// Get the centroid dimensions in the euclidean space.
	int getDimensions();
private:
    std::vector<attribute> attributes;
};

#endif	/* CENTROID_H */

