#ifndef CLUSTERMODEL_H
#define	CLUSTERMODEL_H

#include <vector>
#include "centroid.h"

// Cluster model. It uses centroids to classify data in clusters, according to the distance.
class clustermodel {
public:
    clustermodel();
    void createFromFile(const char* file);
    int getCluster(std::map<std::string, float> dataset);
	// Accuracy: a value between 0 and 1, where 0 is the most different 
	//          possible set of angles and 1 is the same set of angles.
    float getAccuracy(std::map<std::string, float> dataset, int clusterid);
	// Get the model dimensions in euclidean space
	int getDimensions();
private:
    std::vector<centroid> centroids;
	int dimensions;
};

#endif	/* CLUSTERMODEL_H */

