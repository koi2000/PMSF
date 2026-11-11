#pragma once
#include "Halfedge.h"
#include "Point.h"
#include "Vertex.h"
#include <cstdint>

struct HalfedgeErrorEntry {
    MCGAL::Halfedge* edge;
    MCGAL::Point p;
    float error;
    std::uint64_t version;

    // Extension point for custom data.
    // PHASE67: std::any -> raw pointer. std::any's copy ctor is NON-trivial, which makes
    // HalfedgeErrorEntry non-trivially-copyable => vector reallocation cannot memmove, and
    // every push_heap/pop_heap move takes the non-trivial path. With tens of millions of
    // push/pop that is O(N log N) non-trivial moves. customData is used ONLY by POMAR
    // (write: POMARCustomizer.h, read: PomarEliminateOperator.cpp) and only ever holds
    // MCGAL::Vertex* => a raw pointer makes the struct trivially copyable.
    // Byte-neutral: error, comparator and heap algorithm are untouched => pop order unchanged.
    MCGAL::Vertex* customData = nullptr;

    HalfedgeErrorEntry(MCGAL::Halfedge* e) : edge(e) {}
    HalfedgeErrorEntry(MCGAL::Halfedge* e, MCGAL::Point pos, float err, std::uint64_t ver) : edge(e), p(pos), error(err), version(ver) {}
};
