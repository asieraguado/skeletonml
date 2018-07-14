#include <iostream>
#include <fstream>
#include <vector>
#include <stack>
#include <string>
#include <sstream>
#include <cstdlib>
#include "randomforest.h"

randomforest::randomforest() {
}

// Create a random forest classifier from the Weka output format
void randomforest::createFromFile(const char*  file) {
    std::stack< std::pair<modeltree*, int> > nodes;
    std::ifstream filestream;
    filestream.open(file);
    if (!filestream) {
        LoadModelFileException ex;
        throw ex;
    }
    std::string w1;
    while (filestream >> w1) {
        if (w1 == "==========") {
            int nextside = -1;
            std::string line;
            while (std::getline(filestream, line)) {
                std::stringstream cline(line);
                std::string word;
                cline >> word;
                if (word == "") continue;
                if (word == "Size") break;

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
                if (word == "<")
                    nextside = 0;
                else
                    nextside = 1;
                cline >> value;
                if (side < 0) {
                    modeltree* root = new modeltree(0, attribute, value);
                    nodes.push(std::pair<modeltree*, int>(root,0));
                    roots.push_back(root);
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
       }
    }
    filestream.close();
}

int find_posture_votes(std::string posture, std::vector< posture_votes > votes) {
    for (int i=0; i<votes.size(); i++) {
        if (votes[i].posture == posture)
            return i;
    }
    return -1;
}

std::string randomforest::classify(std::map<std::string,float> dataset) {
    std::vector< posture_votes > votes;
    for (int i=0; i<roots.size(); i++) {
        std::string result;
        result = roots[i]->classify(dataset);
        int pos = find_posture_votes(result, votes);
        if (pos >= 0) {
            votes[pos].votes++;
        } else {
            posture_votes new_posture;
            new_posture.posture = result;
            new_posture.votes = 1;
            votes.push_back(new_posture);
        }
    }
    int max = 0;
    int max_index = 0;
    for (int i=0; i<votes.size(); i++){
        if (votes[i].votes > max) {
            max_index = i;
            max = votes[i].votes;
        }
    }
    return votes[max_index].posture;
}
