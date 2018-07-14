#include "modeltree.h"
#include <iostream>

modeltree::modeltree(int type, std::string attr, float val) {
    treetype = type;
    attribute = attr;
    value = val;
}

modeltree::modeltree(int type, std::string attr, float val, modeltree* parent, int pos) {
    treetype = type;
    attribute = attr;
    value = val;
    if (pos == TREE_POS_LEFT)
        parent->left = this;
    else if (pos == TREE_POS_RIGHT)
        parent->right = this;
}

std::string modeltree::classify(std::map<std::string,float> dataset) {
    if (treetype == TREE_TYPE_ATTRIBUTE) {
        float data_value = dataset.at(attribute);
        if (data_value <= value)
            return left->classify(dataset);
        else
            return right->classify(dataset);
    } 
    else {
        return attribute;
    }
}
