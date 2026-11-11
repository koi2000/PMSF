#pragma once

#include "Facet.h"
#include "common/BoundedRingTraversal.h"
#include "Halfedge.h"
#include "Vertex.h"
#include "common/HalfedgeErrorEntry.h"
#include "operator/IsRemovableOperator.h"
#include "operator/UnifiedManifoldIsRemovableOperator.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

class VisualTopologyIsRemovableOperator : public IsRemovableOperator {
  public:
    enum class Mode {
        VertexRemoval,
        EdgeCollapse,
    };

    explicit VisualTopologyIsRemovableOperator(Mode mode = Mode::VertexRemoval,
                                               double minNormalDot = 0.82)
        : mode_(mode), minNormalDot_(minNormalDot) {}

    bool isRemovable(HalfedgeErrorEntry& entry) override {
        if (mode_ == Mode::EdgeCollapse) {
            return edgeCollapseOk(entry.edge);
        }
        return vertexRemovalOk(entry.edge, minNormalDot_);
    }

  private:
    struct Vec2d {
        double x = 0.0;
        double y = 0.0;
    };

    struct Vec3d {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    static bool edgeCollapseOk(MCGAL::Halfedge* h) {
        if (!h || h->isRemoved() || !h->vertex() || !h->end_vertex()) return false;
        if (!h->next() || !h->opposite() || !h->opposite()->next()) return false;
        if (!h->face() || !h->opposite()->face()) return false;
        if (h->face()->isRemoved() || h->opposite()->face()->isRemoved()) return false;
        if (h->isBoundary() || h->opposite()->isBoundary()) return false;
        if (h->face()->facet_degree() != 3 || h->opposite()->face()->facet_degree() != 3) {
            return false;
        }

        MCGAL::Vertex* from = h->vertex();
        MCGAL::Vertex* to = h->end_vertex();
        MCGAL::Vertex* vl = h->next()->end_vertex();
        MCGAL::Vertex* vr = h->opposite()->next()->end_vertex();
        if (!from || !to || !vl || !vr || from->isRemoved() || to->isRemoved()) return false;
        if (vl == vr) return false;

        // OpenMesh/POMAR is_collapse_ok link condition: after merging from->to,
        // the two one-rings may intersect only at the two adjacent triangle tips.
        for (MCGAL::Halfedge* hit : pmsf::boundedRing(from)) {
            if (!hit || !hit->end_vertex()) return false;
            MCGAL::Vertex* n = hit->end_vertex();
            if (n != vl && n != vr && n->isConnected(to)) {
                return false;
            }
        }

        // Extra duplicate-diagonal guard: if the two opposite vertices are
        // already connected, removing the edge would create overlapping or
        // duplicate local triangles in common visualizers.
        if (vl->isConnected(vr) || vr->isConnected(vl)) return false;
        return true;
    }

    static bool vertexRemovalOk(MCGAL::Halfedge* h, double minNormalDot) {
        if (!h || h->isRemoved() || !h->end_vertex()) return false;
        MCGAL::Vertex* center = h->end_vertex();
        if (center->isRemoved() || center->isConquered()) return false;

        const int degree = center->is_degree_dirty()
                               ? center->vertex_degree()
                               : center->vertex_degree_cached();
        if (degree <= 2 || degree > 16) return false;

        std::vector<MCGAL::Vertex*> ring;
        std::vector<MCGAL::Facet*> affectedFaces;
        std::vector<Vec3d> boundaryNormals;
        if (!collectRemovalPatch(h, ring, affectedFaces, boundaryNormals)) return false;
        if (ring.size() <= 2 || ring.size() > 16) return false;
        applyDefaultFanOrder(ring, boundaryNormals);

        if (UnifiedManifoldIsRemovableOperator::checkManifoldViolation(center, ring, true)) {
            return false;
        }

        Vec3d patchNormal = averageIncidentNormal(affectedFaces);
        if (length2(patchNormal) <= 0.0) {
            patchNormal = newellNormal(ring);
        }
        if (length2(patchNormal) <= 0.0) return false;
        const Vec3d fanNormal = newellNormal(ring);
        if (length2(fanNormal) > 0.0 && dot(patchNormal, fanNormal) < 0.0) {
            patchNormal = negate(patchNormal);
        }

        return projectedSimpleConvex(ring, patchNormal) &&
               fanTriangulationIsClean(ring,
                                       patchNormal,
                                       minNormalDot,
                                       std::min(0.2, minNormalDot),
                                       boundaryNormals);
    }

    static bool collectRemovalPatch(MCGAL::Halfedge* h,
                                    std::vector<MCGAL::Vertex*>& ring,
                                    std::vector<MCGAL::Facet*>& affectedFaces,
                                    std::vector<Vec3d>& boundaryNormals) {
        if (!h || !h->next() || !h->vertex() || !h->end_vertex()) return false;
        MCGAL::Vertex* center = h->end_vertex();
        if (!center || center->isRemoved()) return false;

        ring.clear();
        affectedFaces.clear();
        boundaryNormals.clear();
        ring.reserve(16);
        affectedFaces.reserve(16);
        boundaryNormals.reserve(16);

        ring.push_back(h->vertex());
        MCGAL::Halfedge* cur = h;
        bool closed = false;
        for (int guard = 0; guard < 64; ++guard) {
            if (!cur || cur->isRemoved() || !cur->face() || cur->face()->isRemoved()) {
                return false;
            }
            if (cur->end_vertex() != center) return false;
            if (cur->face()->facet_degree() != 3) return false;
            if (std::find(affectedFaces.begin(), affectedFaces.end(), cur->face()) !=
                affectedFaces.end()) {
                return false;
            }
            affectedFaces.push_back(cur->face());

            MCGAL::Halfedge* next = cur->next();
            if (!next || !next->end_vertex()) return false;
            MCGAL::Vertex* boundaryVertex = next->end_vertex();
            if (!boundaryVertex || boundaryVertex->isRemoved() || boundaryVertex == center) {
                return false;
            }

            Vec3d outsideNormal;
            MCGAL::Halfedge* boundaryEdge = next->next();
            if (!boundaryEdge || boundaryEdge->end_vertex() != cur->vertex()) return false;
            MCGAL::Halfedge* outside = boundaryEdge->opposite();
            if (outside && !outside->isBoundary() && outside->face() &&
                !outside->face()->isRemoved()) {
                outsideNormal = faceNormalRaw(outside->face());
            }
            boundaryNormals.push_back(normalized(outsideNormal));

            if (boundaryVertex != ring.front()) {
                if (std::find(ring.begin(), ring.end(), boundaryVertex) != ring.end()) {
                    return false;
                }
                if (ring.size() >= 17) return false;
                ring.push_back(boundaryVertex);
            }

            MCGAL::Halfedge* opp = next->opposite();
            if (!opp || next->isBoundary() || opp->isBoundary()) return false;
            cur = opp;
            if (cur == h) {
                closed = true;
                break;
            }
        }
        return closed && ring.size() >= 3 && affectedFaces.size() == ring.size() &&
               boundaryNormals.size() == ring.size();
    }

    static void applyDefaultFanOrder(std::vector<MCGAL::Vertex*>& ring,
                                     std::vector<Vec3d>& boundaryNormals) {
        const int n = static_cast<int>(ring.size());
        if (n < 3 || static_cast<int>(boundaryNormals.size()) != n) return;

        std::vector<MCGAL::Vertex*> ordered;
        std::vector<Vec3d> orderedBoundaryNormals;
        ordered.reserve(ring.size());
        orderedBoundaryNormals.reserve(boundaryNormals.size());

        // VertexRemovalEliminateOperator::erase_center_vertex() resets the
        // polygon proxy to find_prev(h), so Default's v0-fan is built from:
        // r1, r0, rN, ..., r2. Validate exactly that fan, not the natural
        // one-ring order.
        ordered.push_back(ring[1]);
        ordered.push_back(ring[0]);
        orderedBoundaryNormals.push_back(boundaryNormals[0]);
        for (int i = n - 1; i >= 2; --i) {
            ordered.push_back(ring[i]);
            orderedBoundaryNormals.push_back(boundaryNormals[i]);
        }
        orderedBoundaryNormals.push_back(boundaryNormals[1]);

        ring.swap(ordered);
        boundaryNormals.swap(orderedBoundaryNormals);
    }

    static bool projectedSimpleConvex(const std::vector<MCGAL::Vertex*>& ring,
                                      const Vec3d& normal) {
        const int n = static_cast<int>(ring.size());
        if (n < 3) return false;
        if (length2(normal) <= 0.0) return false;

        std::vector<Vec2d> p(static_cast<std::size_t>(n));
        double sign = 0.0;
        for (int i = 0; i < n; ++i) {
            if (!ring[i] || ring[i]->isRemoved()) return false;
            p[i] = projectPoint(ring[i]->point(), normal);
        }

        for (int i = 0; i < n; ++i) {
            const int i2 = (i + 1) % n;
            for (int j = i + 1; j < n; ++j) {
                const int j2 = (j + 1) % n;
                if (i == j || i2 == j || j2 == i) continue;
                if (segmentsIntersect2(p[i], p[i2], p[j], p[j2])) return false;
            }
        }

        for (int i = 0; i < n; ++i) {
            const double cross = cross2(p[i], p[(i + 1) % n], p[(i + 2) % n]);
            if (std::fabs(cross) <= 1e-12) return false;
            if (sign == 0.0) {
                sign = cross;
            } else if (sign * cross < -1e-12) {
                return false;
            }
        }
        return sign != 0.0;
    }

    static bool fanTriangulationIsClean(const std::vector<MCGAL::Vertex*>& ring,
                                        const Vec3d& patchNormal,
                                        double minNormalDot,
                                        double minBoundaryNormalDot,
                                        const std::vector<Vec3d>& boundaryNormals) {
        const int n = static_cast<int>(ring.size());
        if (n < 3) return false;
        if (length2(patchNormal) <= 0.0) return false;

        double minX = std::numeric_limits<double>::max();
        double minY = std::numeric_limits<double>::max();
        double minZ = std::numeric_limits<double>::max();
        double maxX = std::numeric_limits<double>::lowest();
        double maxY = std::numeric_limits<double>::lowest();
        double maxZ = std::numeric_limits<double>::lowest();
        for (MCGAL::Vertex* v : ring) {
            if (!v || v->isRemoved()) return false;
            const MCGAL::Point& p = v->point();
            minX = std::min(minX, static_cast<double>(p.x()));
            minY = std::min(minY, static_cast<double>(p.y()));
            minZ = std::min(minZ, static_cast<double>(p.z()));
            maxX = std::max(maxX, static_cast<double>(p.x()));
            maxY = std::max(maxY, static_cast<double>(p.y()));
            maxZ = std::max(maxZ, static_cast<double>(p.z()));
        }
        const double dx = maxX - minX;
        const double dy = maxY - minY;
        const double dz = maxZ - minZ;
        const double diag2 = std::max(dx * dx + dy * dy + dz * dz, 1e-24);
        const double minEdge2 = std::max(1e-24, diag2 * 1e-12);
        const double minArea2 = std::max(1e-30, diag2 * diag2 * 1e-18);

        for (int i = 1; i + 1 < n; ++i) {
            const MCGAL::Point& a = ring[0]->point();
            const MCGAL::Point& b = ring[i]->point();
            const MCGAL::Point& c = ring[i + 1]->point();
            if (distance2(a, b) <= minEdge2 ||
                distance2(b, c) <= minEdge2 ||
                distance2(c, a) <= minEdge2 ||
                triangleArea2(a, b, c) <= minArea2) {
                return false;
            }
            const Vec3d raw = cross(subtract(b, a), subtract(c, a));
            const Vec3d nrm = normalized(raw);
            if (length2(nrm) <= 0.0 || dot(nrm, patchNormal) <= minNormalDot) {
                return false;
            }
            if (!boundaryNormalOk(nrm, patchNormal, boundaryNormals, i, minBoundaryNormalDot)) {
                return false;
            }
            if (i == 1 &&
                !boundaryNormalOk(nrm, patchNormal, boundaryNormals, 0, minBoundaryNormalDot)) {
                return false;
            }
            if (i == n - 2 &&
                !boundaryNormalOk(nrm, patchNormal, boundaryNormals, n - 1, minBoundaryNormalDot)) {
                return false;
            }
        }
        return true;
    }

    static bool boundaryNormalOk(const Vec3d& triangleNormal,
                                 const Vec3d& patchNormal,
                                 const std::vector<Vec3d>& boundaryNormals,
                                 int boundaryIndex,
                                 double minBoundaryNormalDot) {
        if (boundaryIndex < 0 ||
            boundaryIndex >= static_cast<int>(boundaryNormals.size())) {
            return false;
        }
        Vec3d outside = boundaryNormals[boundaryIndex];
        if (length2(outside) <= 0.0) return true;
        return dot(triangleNormal, outside) > minBoundaryNormalDot;
    }

    static Vec3d averageIncidentNormal(const std::vector<MCGAL::Facet*>& faces) {
        Vec3d normal;
        for (MCGAL::Facet* face : faces) {
            const Vec3d raw = faceNormalRaw(face);
            normal.x += raw.x;
            normal.y += raw.y;
            normal.z += raw.z;
        }
        return normalized(normal);
    }

    static Vec3d faceNormalRaw(MCGAL::Facet* face) {
        if (!face || face->isRemoved() || !face->proxyHalfedge()) return {};
        MCGAL::Halfedge* h = face->proxyHalfedge();
        if (!h || !h->next() || !h->next()->next()) return {};
        if (!h->vertex() || !h->next()->vertex() || !h->next()->next()->vertex()) return {};
        return cross(subtract(h->next()->vertex()->point(), h->vertex()->point()),
                     subtract(h->next()->next()->vertex()->point(), h->vertex()->point()));
    }

    static Vec3d newellNormal(const std::vector<MCGAL::Vertex*>& ring) {
        Vec3d n;
        const int count = static_cast<int>(ring.size());
        if (count < 3) return n;
        for (int i = 0; i < count; ++i) {
            const MCGAL::Point a = ring[i]->point();
            const MCGAL::Point b = ring[(i + 1) % count]->point();
            n.x += (static_cast<double>(a.y()) - b.y()) *
                   (static_cast<double>(a.z()) + b.z());
            n.y += (static_cast<double>(a.z()) - b.z()) *
                   (static_cast<double>(a.x()) + b.x());
            n.z += (static_cast<double>(a.x()) - b.x()) *
                   (static_cast<double>(a.y()) + b.y());
        }
        return normalized(n);
    }

    static Vec3d subtract(const MCGAL::Point& a, const MCGAL::Point& b) {
        return {static_cast<double>(a.x()) - b.x(),
                static_cast<double>(a.y()) - b.y(),
                static_cast<double>(a.z()) - b.z()};
    }

    static Vec3d cross(const Vec3d& a, const Vec3d& b) {
        return {a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x};
    }

    static double dot(const Vec3d& a, const Vec3d& b) {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    static double length2(const Vec3d& v) {
        return dot(v, v);
    }

    static Vec3d normalized(const Vec3d& v) {
        const double len2 = length2(v);
        if (len2 <= 1e-30) return {};
        const double invLen = 1.0 / std::sqrt(len2);
        return {v.x * invLen, v.y * invLen, v.z * invLen};
    }

    static Vec3d negate(const Vec3d& v) {
        return {-v.x, -v.y, -v.z};
    }

    static Vec2d projectPoint(const MCGAL::Point& p, const Vec3d& normal) {
        const double ax = std::fabs(normal.x);
        const double ay = std::fabs(normal.y);
        const double az = std::fabs(normal.z);
        if (ax >= ay && ax >= az) {
            return {static_cast<double>(p.y()), static_cast<double>(p.z())};
        }
        if (ay >= az) {
            return {static_cast<double>(p.x()), static_cast<double>(p.z())};
        }
        return {static_cast<double>(p.x()), static_cast<double>(p.y())};
    }

    static double cross2(const Vec2d& a, const Vec2d& b, const Vec2d& c) {
        return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
    }

    static bool onSegment2(const Vec2d& a, const Vec2d& b, const Vec2d& p) {
        constexpr double eps = 1e-12;
        return std::min(a.x, b.x) - eps <= p.x && p.x <= std::max(a.x, b.x) + eps &&
               std::min(a.y, b.y) - eps <= p.y && p.y <= std::max(a.y, b.y) + eps;
    }

    static bool segmentsIntersect2(const Vec2d& a,
                                   const Vec2d& b,
                                   const Vec2d& c,
                                   const Vec2d& d) {
        constexpr double eps = 1e-12;
        const double o1 = cross2(a, b, c);
        const double o2 = cross2(a, b, d);
        const double o3 = cross2(c, d, a);
        const double o4 = cross2(c, d, b);
        if (o1 * o2 < -eps && o3 * o4 < -eps) return true;
        if (std::fabs(o1) <= eps && onSegment2(a, b, c)) return true;
        if (std::fabs(o2) <= eps && onSegment2(a, b, d)) return true;
        if (std::fabs(o3) <= eps && onSegment2(c, d, a)) return true;
        if (std::fabs(o4) <= eps && onSegment2(c, d, b)) return true;
        return false;
    }

    static double distance2(const MCGAL::Point& a, const MCGAL::Point& b) {
        const double dx = static_cast<double>(a.x()) - b.x();
        const double dy = static_cast<double>(a.y()) - b.y();
        const double dz = static_cast<double>(a.z()) - b.z();
        return dx * dx + dy * dy + dz * dz;
    }

    static double triangleArea2(const MCGAL::Point& a,
                                const MCGAL::Point& b,
                                const MCGAL::Point& c) {
        const double ux = static_cast<double>(b.x()) - a.x();
        const double uy = static_cast<double>(b.y()) - a.y();
        const double uz = static_cast<double>(b.z()) - a.z();
        const double vx = static_cast<double>(c.x()) - a.x();
        const double vy = static_cast<double>(c.y()) - a.y();
        const double vz = static_cast<double>(c.z()) - a.z();
        const double cx = uy * vz - uz * vy;
        const double cy = uz * vx - ux * vz;
        const double cz = ux * vy - uy * vx;
        return cx * cx + cy * cy + cz * cz;
    }

    Mode mode_ = Mode::VertexRemoval;
    double minNormalDot_ = 0.82;
};
