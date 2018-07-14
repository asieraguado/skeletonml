#include <fstream>
#include <iostream>
#include <math.h>
#include "clustermodel.h"

clustermodel::clustermodel() {
	dimensions = 0;
}

int clustermodel::getCluster(std::map<std::string, float> dataset) {
    int nearest_id = -1;
    float nearest_val = CENTROID_MAX_DIST;
    float dist;
    for (int i=0; i<centroids.size(); i++) {
        dist = centroids[i].getDistanceFrom(dataset);
        if (dist < nearest_val) {
            nearest_val = dist;
            nearest_id  = i;
        }
    }
    return nearest_id;
}

float clustermodel::getAccuracy(std::map<std::string,float> dataset, int clusterid) {
    float dist = centroids[clusterid].getDistanceFrom(dataset);
    // float imprecission = dist/CENTROID_MAX_DIST; this is wrong! Only for n=2 dimensions.
	float imprecission = dist / ( 2*PI*sqrt(dimensions) );
    if (imprecission > 1) {
        std::cerr << "[Warning] Intermediate imprecission greater than 100% (" << imprecission*100 << "%)";
    }
    return 1 - imprecission;
}

void clustermodel::createFromFile(const char* file) {
    std::ifstream filestream;
    filestream.open(file);
    if (!filestream) {
        LoadModelFileException ex;
        throw ex;
    }
    int n, a;
    filestream >> n >> a;
    for (int i=0; i<n; i++) {
        centroid nc;
        centroids.push_back(nc);
    }
    std::string name;
    float value;
    for (int i=0; i<a; i++) {
        filestream >> name;
        for (int j=0; j<n; j++) {
            filestream >> value;
            centroids[j].addAttribute(name, value);
        }
    }
	dimensions = centroids[0].getDimensions();
}

int clustermodel::getDimensions() {
	return dimensions;
}
