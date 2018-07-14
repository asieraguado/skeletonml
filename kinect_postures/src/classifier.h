#include <vector>
#include "modeltree.h"
#include "exceptions.h"

#ifndef TREECONTAINER_H
#define	TREECONTAINER_H

// Tree classifier (contains the model tree root object)
class classifier {
public:
    classifier();
    void createFromFile(const char*  file);
    void createFromFile_old(const char*  file);
    std::string classify(std::map<std::string,float> dataset);
private:
    modeltree* root;
};

#endif	/* TREECONTAINER_H */

