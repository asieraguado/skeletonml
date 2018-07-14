#include <vector>
#include "modeltree.h"
#include "exceptions.h"

#ifndef RANDOMFOREST_H
#define	RANDOMFOREST_H

struct posture_votes {
    std::string posture;
    int votes;
};

class randomforest {
public:
    randomforest();
    void createFromFile(const char*  file);
    std::string classify(std::map<std::string,float> dataset);
private:
    std::vector<modeltree*> roots;
};

#endif	/* RANDOMFOREST_H */

