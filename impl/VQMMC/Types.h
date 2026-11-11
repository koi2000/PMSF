#ifndef TYPES_H
#define TYPES_H

#include <vector>
#include "../../deps/MCGAL/include/Point.h"

typedef MCGAL::Point Vec3d;
typedef std::vector<double> VectorXd;

struct PTTreeNode {
    int nodeId;
    Vec3d codeword;
    int leftChild;
    int rightChild;
    int depth;

    PTTreeNode() : nodeId(-1), codeword(Vec3d(0.0, 0.0, 0.0)),
                   leftChild(-1), rightChild(-1), depth(0) {}
};

struct VQCodebook {
    std::vector<Vec3d> codewords;
    std::vector<PTTreeNode> progressiveTree;
    int dimension;
    int codebookSize;

    VQCodebook() : dimension(3), codebookSize(0) {}
};

#endif // TYPES_H
