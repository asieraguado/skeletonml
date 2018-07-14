#include <map>
#include <fstream>
#include <iostream>
#include <math.h>
#include "centroid.h"

centroid::centroid(){
}

centroid::centroid(std::vector< attribute > attrs) {
    attributes = attrs;
}

float centroid::getDistanceFrom(std::map<std::string,float> dataset) {
    float d = 0;
    float diff;
    for (int i=0; i < attributes.size(); i++) {
        diff = dataset.at(attributes[i].name) - attributes[i].value;
        d = d + diff*diff;
    }
    return sqrt(d);
}

void centroid::addAttribute(std::string name, float value) {
    struct attribute attr;
    attr.name = name;
    attr.value = value;
    //std::cout << "adding: " << name << ", " << value << std::endl;
    attributes.push_back(attr);
}

void centroid::createFromFile(const char* file) {
    std::ifstream filestream;
    filestream.open(file);
    if (!filestream) {
        LoadModelFileException ex;
        throw ex;
    }
    int a;
    filestream >> a;
    std::string name;
    float value;
    for (int i=0; i<a; i++) {
        filestream >> name;
        filestream >> value;
        addAttribute(name, value);
    }
}

int centroid::getDimensions() {
	return attributes.size();
}
