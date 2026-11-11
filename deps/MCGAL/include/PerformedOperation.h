#pragma once

#include "Vertex.h"

namespace MCGAL {
struct PerformedOperation {
    Vertex* v1 = nullptr;
    Vertex* v2 = nullptr;
    Vertex* vr = nullptr;

    Vertex* vhPivotL = nullptr;
    Vertex* vhPivotR = nullptr;
};
}  // namespace MCGAL
