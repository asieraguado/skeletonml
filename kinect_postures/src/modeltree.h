#ifndef MODELTREE_H
#define	MODELTREE_H

#define TREE_TYPE_ATTRIBUTE 0
#define TREE_TYPE_CLASS 1
#define TREE_POS_LEFT 0
#define TREE_POS_RIGHT 1

#include <string>
#include <map>

// Recursive tree for classifying postures. An object of this class contains a tree node.
class modeltree {
public:
    // Root constructor
    modeltree(int type, std::string attr, float val);
    // Child constructor
    modeltree(int type, std::string attr, float val, modeltree* parent, int pos);
    // Classify: Recursive method to classify a dataset. Returns the posture of the leaf.
    std::string classify (std::map<std::string, float> dataset);
private:
	// type: 1=leaf, 0=rest of nodes.
    int treetype;
	// attribute, value: attribute and value to compare, to continue with left or right child.
    std::string attribute;
    float value;
	// left and right child nodes.
    modeltree* left;
    modeltree* right;
};

#endif	/* MODELTREE_H */
