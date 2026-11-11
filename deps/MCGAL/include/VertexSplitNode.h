#ifndef VERTEX_SPLIT_NODE
#define VERTEX_SPLIT_NODE
#include "Point.h"
#include <stdint.h>

namespace MCGAL {
struct VertexSplitNode {
    MCGAL::Point c;
    MCGAL::Point d;
    uint16_t bitmap;
    int order;

    VertexSplitNode* left = nullptr;
    VertexSplitNode* right = nullptr;
};
}  // namespace MCGAL
#endif
