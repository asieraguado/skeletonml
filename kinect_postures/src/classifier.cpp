#include <iostream>
#include <fstream>
#include <vector>
#include <stack>
#include <string>
#include <sstream>
#include <cstdlib>
#include "classifier.h"

classifier::classifier() {
}

// Create a posture classifier tree from the Weka output format
void classifier::createFromFile(const char*  file) {
    std::stack< std::pair<modeltree*, int> > nodes;
    std::ifstream filestream;
    filestream.open(file);
    if (!filestream) {
        LoadModelFileException ex;
        throw ex;
    }
    int nextside = -1;
    std::string line;
    while (std::getline(filestream, line)) {
        std::stringstream cline(line);
        std::string word;
        cline >> word;
        if (word == "") continue;

        int depth, side;
        float value;
        std::string attribute;
        depth = 0;
        while (word == "|") {
            depth++;
            cline >> word;
        }
        attribute = word;
        cline >> word;
        side = nextside;
        if (word == "<=")
            nextside = 0;
        else
            nextside = 1;
        cline >> value;
        if (side < 0) {
            root = new modeltree(0, attribute, value);
            nodes.push(std::pair<modeltree*, int>(root,0));
        } else {
            while (depth < nodes.top().second) {
                nodes.pop();
            }
            if (depth > nodes.top().second) {
                modeltree* child = new modeltree(0, attribute, value, nodes.top().first, side);
                nodes.push(std::pair<modeltree*, int>(child, depth));
            }
        }
        if (!cline.eof()) {
            cline >> word;
            if (word == ":") {
                cline >> attribute;
                modeltree* leaf = new modeltree(1, attribute, 0, nodes.top().first, nextside);
            }
        }
    }
    filestream.close();
}

// Create a posture classifier tree from file (old format)
void classifier::createFromFile_old(const char*  file) {
	std::vector<modeltree*> nodes;
	std::ifstream filestream;
	filestream.exceptions ( std::ifstream::failbit | std::ifstream::badbit );
	filestream.open(file);
    if (!filestream) {
        LoadModelFileException ex;
        throw ex;
    }
	int n;
	filestream >> n;
	for (int i=0; i<n ; i++) {
	    int type, parentid, side;
	    std::string attribute;
	    float value;
	    filestream >> type;
	    filestream >> attribute;
	    filestream >> value;
	    if (i==0) {
	        root = new modeltree(type, attribute, value);
	        nodes.push_back(root);
	    } else {
	        filestream >> parentid;
	        filestream >> side;
	        modeltree* next = new modeltree(type, attribute, value, nodes.at(parentid), side);
	        nodes.push_back(next);
	    }
	}
}

std::string classifier::classify(std::map<std::string,float> dataset) {
    return root->classify(dataset);
}
