// SPDX-License-Identifier: Apache-2.0
//
// Shared helpers for encoding geometry residuals in a local Frenet frame.
// Algorithms such as POMAR and PPMC differ in how they select primitives and
// prediction neighborhoods, but they should not reimplement NaN-safe frame
// construction and forward/inverse residual rotation.

#ifndef PMSF_PREDICT_FRENET_RESIDUAL_CODEC_H
#define PMSF_PREDICT_FRENET_RESIDUAL_CODEC_H

#include "Facet.h"
#include "Halfedge.h"
#include "MeshUtils.h"
#include "Point.h"

#include <algorithm>
#include <cmath>

namespace pmsf {

struct FrenetFrame {
    MCGAL::Vector3 t1;
    MCGAL::Vector3 t2;
    MCGAL::Vector3 normal;
};

inline FrenetFrame safeEdgeFrame(MCGAL::Vector3 v1,
                                 MCGAL::Vector3 v2,
                                 MCGAL::Vector3 normal);

inline bool isFiniteVector3(MCGAL::Vector3 v) {
    return std::isfinite(v[0]) && std::isfinite(v[1]) && std::isfinite(v[2]);
}

inline FrenetFrame safeAdjacentFaceFrame(MCGAL::Halfedge* h) {
    MCGAL::Vector3 n1raw(0, 0, 0);
    MCGAL::Vector3 n2raw(0, 0, 0);
    if (h && h->face()) n1raw = h->face()->computeNormal();
    if (h && h->opposite() && h->opposite()->face()) {
        n2raw = h->opposite()->face()->computeNormal();
    }
    if (!isFiniteVector3(n1raw)) n1raw = MCGAL::Vector3(0, 0, 0);
    if (!isFiniteVector3(n2raw)) n2raw = MCGAL::Vector3(0, 0, 0);

    MCGAL::Vector3 rawNormal = n1raw + n2raw;
    MCGAL::Vector3 normal;
    if (rawNormal.length() > 1e-6f) {
        normal = rawNormal.normalize();
    } else if (isFiniteVector3(n1raw) && n1raw.length() > 1e-6f) {
        normal = n1raw.normalize();
    } else {
        normal = MCGAL::Vector3(0, 0, 1);
    }
    if (!isFiniteVector3(normal)) normal = MCGAL::Vector3(0, 0, 1);

    FrenetFrame frame;
    frame.normal = normal;
    if (h && h->vertex() && h->end_vertex()) {
        MCGAL::determineFrenetFrame(h->vertex()->point(),
                                    h->end_vertex()->point(),
                                    frame.normal,
                                    frame.t1,
                                    frame.t2);
    }
    if (!isFiniteVector3(frame.t1) || !isFiniteVector3(frame.t2)) {
        frame.t1 = MCGAL::Vector3(1, 0, 0);
        frame.t2 = MCGAL::Vector3(0, 1, 0);
        frame.normal = MCGAL::Vector3(0, 0, 1);
    }
    return frame;
}

inline MCGAL::Vector3 safeAdjacentNormal(MCGAL::Halfedge* h) {
    MCGAL::Vector3 n1raw(0, 0, 0);
    MCGAL::Vector3 n2raw(0, 0, 0);
    if (h && h->face()) n1raw = h->face()->computeNormal();
    if (h && h->opposite() && h->opposite()->face()) {
        n2raw = h->opposite()->face()->computeNormal();
    }
    if (!isFiniteVector3(n1raw)) n1raw = MCGAL::Vector3(0, 0, 0);
    if (!isFiniteVector3(n2raw)) n2raw = MCGAL::Vector3(0, 0, 0);

    MCGAL::Vector3 rawNormal = n1raw + n2raw;
    if (rawNormal.length() > 1e-6f) return rawNormal.normalize();
    if (isFiniteVector3(n1raw) && n1raw.length() > 1e-6f) return n1raw.normalize();
    return MCGAL::Vector3(0, 0, 1);
}

inline FrenetFrame safeAdjacentFaceFrame(MCGAL::Halfedge* h,
                                         MCGAL::Vector3 frameStart,
                                         MCGAL::Vector3 frameEnd) {
    return safeEdgeFrame(frameStart, frameEnd, safeAdjacentNormal(h));
}

inline FrenetFrame safeEdgeFrame(MCGAL::Vector3 v1,
                                 MCGAL::Vector3 v2,
                                 MCGAL::Vector3 normal) {
    if (!isFiniteVector3(normal) || normal.length() == 0) {
        normal = MCGAL::Vector3(0, 0, 1);
    }

    FrenetFrame frame;
    frame.normal = normal;
    MCGAL::determineFrenetFrame(v1, v2, frame.normal, frame.t1, frame.t2);
    if (!isFiniteVector3(frame.t1) || !isFiniteVector3(frame.t2)) {
        frame.t1 = MCGAL::Vector3(1, 0, 0);
        frame.t2 = MCGAL::Vector3(0, 1, 0);
        frame.normal = MCGAL::Vector3(0, 0, 1);
    }
    return frame;
}

inline FrenetFrame safeHalfedgeFrame(MCGAL::Halfedge* h) {
    MCGAL::Vector3 v1(0, 0, 0);
    MCGAL::Vector3 v2(1, 0, 0);
    if (h && h->vertex()) v1 = MCGAL::Vector3(h->vertex()->point());
    if (h && h->opposite() && h->opposite()->vertex()) {
        v2 = MCGAL::Vector3(h->opposite()->vertex()->point());
    } else if (h && h->end_vertex()) {
        v2 = MCGAL::Vector3(h->end_vertex()->point());
    }
    MCGAL::Vector3 normal = h ? MCGAL::computeNormalFromHeh(h) : MCGAL::Vector3(0, 0, 1);
    return safeEdgeFrame(v1, v2, normal);
}

inline MCGAL::Vector3i rotateResidualToFrenet(MCGAL::Vector3i residual,
                                              const FrenetFrame& frame,
                                              int clampAbs = 1 << 14) {
    MCGAL::Vector3 t1 = frame.t1;
    MCGAL::Vector3 t2 = frame.t2;
    MCGAL::Vector3 normal = frame.normal;
    MCGAL::Vector3i rotated = MCGAL::frenetRotation(residual, t1, t2, normal);
    for (int k = 0; k < 3; ++k) {
        rotated[k] = std::clamp(rotated[k], -clampAbs, clampAbs);
    }
    return rotated;
}

inline MCGAL::Vector3i rotateResidualFromFrenet(MCGAL::Vector3i encoded,
                                                const FrenetFrame& frame) {
    MCGAL::Vector3 t1 = frame.t1;
    MCGAL::Vector3 t2 = frame.t2;
    MCGAL::Vector3 normal = frame.normal;
    return MCGAL::invFrenetRotation(encoded, t1, t2, normal);
}

}  // namespace pmsf

#endif  // PMSF_PREDICT_FRENET_RESIDUAL_CODEC_H
