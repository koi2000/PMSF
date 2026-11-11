// SPDX-License-Identifier: Apache-2.0
//
// PCATMPaperMode2.h
//
// Wire-visible recovery helper for the paper PCATM mode-2 patch.  The helper
// deliberately does not use poolId(), compId(), or any other allocation order:
// only live face/halfedge connectivity, the supplied face colors, and
// quantized vertex positions participate in the decision.

#ifndef PMSF_PCATM_PAPER_MODE2_H
#define PMSF_PCATM_PAPER_MODE2_H

#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Point.h"
#include "Vertex.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace pmsf {
namespace pcatm {
namespace detail {

using Face = MCGAL::Facet;
using Halfedge = MCGAL::Halfedge;
using Vertex = MCGAL::Vertex;

constexpr std::uint8_t kZero = 0;
constexpr std::uint8_t kBlue = 1;

struct TriangleView {
    std::array<Halfedge*, 3> h{{nullptr, nullptr, nullptr}};
    std::array<Vertex*, 3> v{{nullptr, nullptr, nullptr}};
};

inline std::array<Face*, 2> mode2Invalid() {
    return std::array<Face*, 2>{{nullptr, nullptr}};
}

inline bool colorOf(
    const std::unordered_map<Face*, std::uint8_t>& colors,
    Face* face,
    std::uint8_t& color) {
    const auto it = colors.find(face);
    if (it == colors.end()) return false;
    color = it->second;
    return true;
}

// A bounded face walk.  In particular, do not trust facet_degree() or an
// iterator whose termination depends on a malformed next() cycle.
inline bool readTriangle(Face* face, TriangleView& out) {
    if (face == nullptr || face->isRemoved()) return false;

    Halfedge* h0 = face->proxyHalfedge();
    if (h0 == nullptr || h0->face() != face || h0->isRemoved()) return false;
    Halfedge* h1 = h0->next();
    Halfedge* h2 = h1 ? h1->next() : nullptr;
    if (h1 == nullptr || h2 == nullptr ||
        h1->isRemoved() || h2->isRemoved() ||
        h1->face() != face || h2->face() != face ||
        h2->next() != h0) {
        return false;
    }

    out.h = {{h0, h1, h2}};
    for (int i = 0; i < 3; ++i) {
        Halfedge* h = out.h[static_cast<std::size_t>(i)];
        Halfedge* next = out.h[static_cast<std::size_t>((i + 1) % 3)];
        if (h == nullptr || next == nullptr ||
            h->face() != face || h->isRemoved() ||
            h->vertex() == nullptr || h->end_vertex() == nullptr ||
            h->vertex()->isRemoved() || h->end_vertex()->isRemoved() ||
            h->end_vertex() != next->vertex()) {
            return false;
        }

        // A present opposite must be a live, symmetric wire edge.  A null
        // opposite is allowed for a genuine mesh boundary.
        if (h->opposite() != nullptr) {
            Halfedge* opposite = h->opposite();
            if (opposite->isRemoved() ||
                opposite->opposite() != h ||
                opposite->face() == nullptr ||
                opposite->face()->isRemoved() ||
                opposite->vertex() != h->end_vertex() ||
                opposite->end_vertex() != h->vertex()) {
                return false;
            }
        }
        out.v[static_cast<std::size_t>(i)] = h->vertex();
    }

    if (out.v[0] == out.v[1] || out.v[1] == out.v[2] || out.v[2] == out.v[0]) {
        return false;
    }
    return true;
}

inline Halfedge* previousInTriangle(Halfedge* h) {
    if (h == nullptr || h->next() == nullptr || h->next()->next() == nullptr) {
        return nullptr;
    }
    return h->next()->next();
}

inline Face* oppositeFace(Halfedge* h) {
    if (h == nullptr || h->opposite() == nullptr) return nullptr;
    Halfedge* opposite = h->opposite();
    if (opposite->isRemoved() ||
        opposite->opposite() != h ||
        opposite->face() == nullptr ||
        opposite->face()->isRemoved() ||
        opposite->vertex() != h->end_vertex() ||
        opposite->end_vertex() != h->vertex()) {
        return nullptr;
    }
    return opposite->face();
}

inline bool containsFace(const std::vector<Face*>& faces, Face* needle) {
    for (Face* face : faces) {
        if (face == needle) return true;
    }
    return false;
}

inline void addUniqueNeighbor(
    std::unordered_map<Face*, std::vector<Face*>>& graph,
    Face* from,
    Face* to,
    bool& duplicate) {
    std::vector<Face*>& neighbors = graph[from];
    for (Face* neighbor : neighbors) {
        if (neighbor == to) {
            duplicate = true;
            return;
        }
    }
    neighbors.push_back(to);
}

// Build the blue adjacency graph and reject a blue face that has an
// unaccounted-for blue neighbor.  This makes "ones" the complete wire-visible
// blue component rather than an arbitrary subset of a branch.
inline bool buildBlueGraph(
    const std::vector<Face*>& ones,
    const std::unordered_set<Face*>& oneSet,
    const std::unordered_map<Face*, TriangleView>& views,
    const std::unordered_map<Face*, std::uint8_t>& colors,
    std::unordered_map<Face*, std::vector<Face*>>& graph) {
    std::unordered_set<Halfedge*> inspected;
    for (Face* face : ones) {
        const TriangleView& view = views.at(face);
        for (Halfedge* h : view.h) {
            Halfedge* opposite = h->opposite();
            if (opposite == nullptr) continue;
            if (inspected.find(h) != inspected.end() ||
                inspected.find(opposite) != inspected.end()) {
                continue;
            }
            inspected.insert(h);
            inspected.insert(opposite);

            Face* neighbor = oppositeFace(h);
            if (neighbor == nullptr) return false;
            std::uint8_t neighborColor = 0;
            if (!colorOf(colors, neighbor, neighborColor)) return false;
            if (neighborColor != kBlue) continue;

            if (oneSet.find(neighbor) == oneSet.end()) return false;
            bool duplicate = false;
            addUniqueNeighbor(graph, face, neighbor, duplicate);
            if (duplicate) return false;
            addUniqueNeighbor(graph, neighbor, face, duplicate);
            if (duplicate) return false;
        }
    }
    for (Face* face : ones) {
        (void)graph[face];
    }
    return true;
}

inline bool zeroFaceAcross(
    Halfedge* h,
    const std::unordered_set<Face*>& oneSet,
    const std::unordered_map<Face*, std::uint8_t>& colors,
    Face*& zero) {
    zero = nullptr;
    Face* candidate = oppositeFace(h);
    if (candidate == nullptr || oneSet.find(candidate) != oneSet.end()) return false;

    std::uint8_t color = 0;
    if (!colorOf(colors, candidate, color) || color != kZero) return false;

    TriangleView ignored;
    if (!readTriangle(candidate, ignored)) return false;
    zero = candidate;
    return true;
}

struct EdgeOccurrence {
    Face* face = nullptr;
    Halfedge* h = nullptr;
};

struct PatchEdge {
    Vertex* a = nullptr;
    Vertex* b = nullptr;
    std::vector<EdgeOccurrence> occurrences;
};

inline bool sameUndirectedEdge(Vertex* a, Vertex* b, Vertex* c, Vertex* d) {
    return (a == c && b == d) || (a == d && b == c);
}

inline int findPatchEdge(
    const std::vector<PatchEdge>& edges,
    Vertex* a,
    Vertex* b) {
    for (std::size_t i = 0; i < edges.size(); ++i) {
        if (sameUndirectedEdge(edges[i].a, edges[i].b, a, b)) {
            return static_cast<int>(i);
        }
    }
    return -1;
}

// Validate the candidate as a single triangulated disk.  This is intentionally
// local to the candidate patch; cross-patch equality is the encoder's job.
inline bool isSimpleTriangulatedDisk(
    const std::vector<Face*>& patch,
    const std::unordered_map<Face*, std::uint8_t>& colors) {
    if (patch.empty()) return false;

    std::unordered_set<Face*> patchFaces;
    std::unordered_set<Vertex*> patchVertices;
    std::vector<PatchEdge> edges;
    std::unordered_map<Face*, std::vector<Face*>> faceGraph;

    for (Face* face : patch) {
        if (face == nullptr || !patchFaces.insert(face).second) return false;
        std::uint8_t ignoredColor = 0;
        if (!colorOf(colors, face, ignoredColor)) return false;

        TriangleView view;
        if (!readTriangle(face, view)) return false;
        for (Vertex* vertex : view.v) {
            if (vertex == nullptr) return false;
            patchVertices.insert(vertex);
        }
        for (Halfedge* h : view.h) {
            Vertex* a = h->vertex();
            Vertex* b = h->end_vertex();
            const int edgeIndex = findPatchEdge(edges, a, b);
            if (edgeIndex < 0) {
                PatchEdge edge;
                edge.a = a;
                edge.b = b;
                edge.occurrences.push_back(EdgeOccurrence{face, h});
                edges.push_back(edge);
            } else {
                edges[static_cast<std::size_t>(edgeIndex)].occurrences.push_back(
                    EdgeOccurrence{face, h});
            }
        }
    }

    std::unordered_map<Vertex*, std::vector<Vertex*>> boundaryGraph;
    std::unordered_map<Vertex*, std::vector<Vertex*>> directedBoundary;
    std::unordered_map<Vertex*, std::vector<Vertex*>> incomingBoundary;
    std::unordered_set<Vertex*> boundaryVertices;
    int internalEdgeCount = 0;
    int boundaryEdgeCount = 0;

    for (const PatchEdge& edge : edges) {
        if (edge.occurrences.empty() || edge.occurrences.size() > 2) return false;
        if (edge.occurrences.size() == 2) {
            const EdgeOccurrence& first = edge.occurrences[0];
            const EdgeOccurrence& second = edge.occurrences[1];
            if (first.face == second.face ||
                first.h->opposite() != second.h ||
                second.h->opposite() != first.h) {
                return false;
            }
            faceGraph[first.face].push_back(second.face);
            faceGraph[second.face].push_back(first.face);
            ++internalEdgeCount;
        } else {
            boundaryGraph[edge.a].push_back(edge.b);
            boundaryGraph[edge.b].push_back(edge.a);
            Halfedge* boundaryHalfedge = edge.occurrences[0].h;
            directedBoundary[boundaryHalfedge->vertex()].push_back(
                boundaryHalfedge->end_vertex());
            incomingBoundary[boundaryHalfedge->end_vertex()].push_back(
                boundaryHalfedge->vertex());
            boundaryVertices.insert(edge.a);
            boundaryVertices.insert(edge.b);
            ++boundaryEdgeCount;
        }
    }

    // Every patch vertex must be on one boundary cycle, and every boundary
    // vertex must have exactly two boundary neighbors.
    if (boundaryVertices.size() != patchVertices.size()) return false;
    for (Vertex* vertex : patchVertices) {
        const auto it = boundaryGraph.find(vertex);
        if (it == boundaryGraph.end() || it->second.size() != 2) return false;
        const auto outgoing = directedBoundary.find(vertex);
        const auto incoming = incomingBoundary.find(vertex);
        if (outgoing == directedBoundary.end() ||
            incoming == incomingBoundary.end() ||
            outgoing->second.size() != 1 ||
            incoming->second.size() != 1) {
            return false;
        }
    }

    Vertex* boundarySeed = *boundaryVertices.begin();
    std::unordered_set<Vertex*> seenBoundary;
    std::queue<Vertex*> boundaryQueue;
    boundaryQueue.push(boundarySeed);
    seenBoundary.insert(boundarySeed);
    while (!boundaryQueue.empty()) {
        Vertex* vertex = boundaryQueue.front();
        boundaryQueue.pop();
        for (Vertex* next : boundaryGraph[vertex]) {
            if (seenBoundary.insert(next).second) boundaryQueue.push(next);
        }
    }
    if (seenBoundary.size() != boundaryVertices.size()) return false;

    // Also require a single *directed* boundary cycle.  The undirected degree
    // check above is not enough: oppositely oriented components can otherwise
    // look like a valid degree-two boundary.
    std::unordered_set<Vertex*> seenDirectedBoundary;
    Vertex* directedSeed = *boundaryVertices.begin();
    Vertex* directedCurrent = directedSeed;
    for (int step = 0; step < static_cast<int>(boundaryVertices.size()); ++step) {
        if (!seenDirectedBoundary.insert(directedCurrent).second) return false;
        const auto& outgoing = directedBoundary[directedCurrent];
        if (outgoing.size() != 1) return false;
        directedCurrent = outgoing.front();
    }
    if (directedCurrent != directedSeed ||
        seenDirectedBoundary.size() != boundaryVertices.size()) {
        return false;
    }

    // The internal-edge graph must also be connected; otherwise two disks
    // could be glued only at a vertex, which is not a single patch disk.
    Face* faceSeed = patch.front();
    std::unordered_set<Face*> seenFaces;
    std::queue<Face*> faceQueue;
    faceQueue.push(faceSeed);
    seenFaces.insert(faceSeed);
    while (!faceQueue.empty()) {
        Face* face = faceQueue.front();
        faceQueue.pop();
        for (Face* next : faceGraph[face]) {
            if (seenFaces.insert(next).second) faceQueue.push(next);
        }
    }
    if (seenFaces.size() != patchFaces.size()) return false;

    const int faceCount = static_cast<int>(patchFaces.size());
    const int vertexCount = static_cast<int>(patchVertices.size());
    if (vertexCount != faceCount + 2 ||
        internalEdgeCount != faceCount - 1 ||
        boundaryEdgeCount != vertexCount) {
        return false;
    }
    return true;
}

struct QuantizedPoint {
    std::array<int, 3> q{{0, 0, 0}};
};

inline bool quantizePoint(
    Vertex* vertex,
    const MCGAL::Point& bboxMin,
    float quantStep,
    QuantizedPoint& out) {
    if (vertex == nullptr || !std::isfinite(quantStep) || quantStep <= 0.0f) {
        return false;
    }
    const MCGAL::Point point = vertex->point();
    const float coordinate[3] = {point.x(), point.y(), point.z()};
    const float minimum[3] = {bboxMin.x(), bboxMin.y(), bboxMin.z()};
    for (int axis = 0; axis < 3; ++axis) {
        const float raw = (coordinate[axis] - minimum[axis]) / quantStep;
        if (!std::isfinite(raw)) return false;
        // Match PointInt(floatPosToInt(...)) truncation, followed by the
        // paper-mode 12-bit clamp used by the base-mesh wire format.
        if (raw <= 0.0f) {
            out.q[static_cast<std::size_t>(axis)] = 0;
        } else if (raw >= 4095.0f) {
            out.q[static_cast<std::size_t>(axis)] = 4095;
        } else {
            out.q[static_cast<std::size_t>(axis)] =
                static_cast<int>(std::trunc(raw));
        }
    }
    return true;
}

inline std::array<std::int64_t, 3> subtract(
    const QuantizedPoint& a,
    const QuantizedPoint& b) {
    return {{
        static_cast<std::int64_t>(a.q[0]) - b.q[0],
        static_cast<std::int64_t>(a.q[1]) - b.q[1],
        static_cast<std::int64_t>(a.q[2]) - b.q[2]}};
}

inline std::int64_t squaredLength(const std::array<std::int64_t, 3>& v) {
    return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
}

inline std::array<std::int64_t, 3> cross(
    const std::array<std::int64_t, 3>& a,
    const std::array<std::int64_t, 3>& b) {
    return {{
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0]}};
}

inline bool recoverSingleton(
    Face* blue,
    const std::unordered_set<Face*>& oneSet,
    const std::unordered_map<Face*, TriangleView>& views,
    const std::unordered_map<Face*, std::uint8_t>& colors,
    const MCGAL::Point& bboxMin,
    float quantStep,
    std::array<Face*, 2>& result) {
    const TriangleView& view = views.at(blue);
    std::array<QuantizedPoint, 3> q;
    for (int i = 0; i < 3; ++i) {
        if (!quantizePoint(view.v[static_cast<std::size_t>(i)],
                           bboxMin, quantStep, q[static_cast<std::size_t>(i)])) {
            return false;
        }
    }

    const auto e0 = subtract(q[1], q[0]);
    const auto e1 = subtract(q[2], q[1]);
    const auto e2 = subtract(q[0], q[2]);
    const auto normal = cross(subtract(q[1], q[0]), subtract(q[2], q[0]));
    if (normal[0] == 0 && normal[1] == 0 && normal[2] == 0) return false;

    const std::int64_t lengths[3] = {
        squaredLength(e0), squaredLength(e1), squaredLength(e2)};
    int shortest = 0;
    if (lengths[1] < lengths[shortest]) shortest = 1;
    if (lengths[2] < lengths[shortest]) shortest = 2;
    for (int i = 0; i < 3; ++i) {
        if (i != shortest && lengths[i] <= lengths[shortest]) return false;
    }

    const int edgeA = (shortest + 1) % 3;
    const int edgeB = (shortest + 2) % 3;
    Face* zeroA = nullptr;
    Face* zeroB = nullptr;
    if (!zeroFaceAcross(view.h[static_cast<std::size_t>(edgeA)],
                        oneSet, colors, zeroA) ||
        !zeroFaceAcross(view.h[static_cast<std::size_t>(edgeB)],
                        oneSet, colors, zeroB) ||
        zeroA == zeroB) {
        return false;
    }

    // In a nondegenerate triangle, the unique shortest edge is opposite the
    // unique smallest angle. Integer squared lengths suffice; no floating
    // trigonometry (and no allocation-dependent tie break) belongs on wire.
    result = {{zeroA, zeroB}};
    return isSimpleTriangulatedDisk({blue, zeroA, zeroB}, colors);
}

struct DirectedRecovery {
    bool ok = false;
    Face* firstZero = nullptr;
    Face* lastZero = nullptr;
};

inline Halfedge* uniqueEdgeToFace(
    Face* from,
    Face* to,
    const TriangleView& view) {
    Halfedge* found = nullptr;
    for (Halfedge* h : view.h) {
        Face* opposite = oppositeFace(h);
        if (opposite != to) continue;
        if (found != nullptr) return nullptr;
        found = h;
    }
    return found;
}

inline DirectedRecovery recoverDirectedPath(
    const std::vector<Face*>& path,
    const std::unordered_map<Face*, TriangleView>& views,
    const std::unordered_set<Face*>& oneSet,
    const std::unordered_map<Face*, std::uint8_t>& colors) {
    DirectedRecovery out;
    const std::size_t count = path.size();
    if (count < 2) return out;

    Halfedge* out0 = uniqueEdgeToFace(path[0], path[1], views.at(path[0]));
    if (out0 == nullptr) return out;
    if (!zeroFaceAcross(
            previousInTriangle(out0), oneSet, colors, out.firstZero)) {
        return DirectedRecovery{};
    }

    std::unordered_set<Face*> visited;
    visited.insert(path[0]);
    for (std::size_t j = 1; j < count; ++j) {
        Face* current = path[j];
        if (!visited.insert(current).second) return DirectedRecovery{};

        Halfedge* in = out0->opposite();
        if (in == nullptr || in->face() != current) return DirectedRecovery{};
        Halfedge* diagonal =
            (j & 1u) ? previousInTriangle(in) : in->next();
        if (diagonal == nullptr || diagonal->face() != current) {
            return DirectedRecovery{};
        }

        Face* next = oppositeFace(diagonal);
        if (next == nullptr) return DirectedRecovery{};
        std::uint8_t nextColor = 0;
        if (!colorOf(colors, next, nextColor)) return DirectedRecovery{};

        if (j + 1 < count) {
            if (next != path[j + 1] ||
                nextColor != kBlue ||
                oneSet.find(next) == oneSet.end() ||
                visited.find(next) != visited.end()) {
                return DirectedRecovery{};
            }
            out0 = diagonal;
        } else {
            if (nextColor != kZero ||
                oneSet.find(next) != oneSet.end()) {
                return DirectedRecovery{};
            }
            TriangleView zeroView;
            if (!readTriangle(next, zeroView)) return DirectedRecovery{};
            out.lastZero = next;
        }
    }

    if (out.firstZero == nullptr || out.lastZero == nullptr ||
        out.firstZero == out.lastZero) {
        return DirectedRecovery{};
    }
    out.ok = true;
    return out;
}

}  // namespace detail

// Recover the unique pair of external color-0 faces for a paper mode-2 patch.
//
// Return value:
//   {zero0, zero1}  recoverable and locally disk-valid;
//   {nullptr,nullptr} otherwise.
//
// `ones` is the complete blue face set for this candidate patch.  The helper
// intentionally treats face order as irrelevant and never sorts by pool/comp
// identifiers.
inline std::array<MCGAL::Facet*, 2> paperMode2Zeros(
    const std::vector<MCGAL::Facet*>& ones,
    const std::unordered_map<MCGAL::Facet*, std::uint8_t>& colors,
    const MCGAL::Point& bboxMin,
    float quantStep) {
    using namespace detail;

    if (ones.empty()) return mode2Invalid();

    std::unordered_set<Face*> oneSet;
    std::unordered_map<Face*, TriangleView> views;
    for (Face* face : ones) {
        if (face == nullptr || !oneSet.insert(face).second) return mode2Invalid();
        std::uint8_t color = 0;
        if (!colorOf(colors, face, color) || color != kBlue) {
            return mode2Invalid();
        }
        TriangleView view;
        if (!readTriangle(face, view)) return mode2Invalid();
        views.emplace(face, view);
    }

    std::unordered_map<Face*, std::vector<Face*>> blueGraph;
    if (!buildBlueGraph(ones, oneSet, views, colors, blueGraph)) {
        return mode2Invalid();
    }

    if (ones.size() == 1) {
        std::array<Face*, 2> result = {{nullptr, nullptr}};
        if (!recoverSingleton(
                ones.front(), oneSet, views, colors,
                bboxMin, quantStep, result)) {
            return mode2Invalid();
        }
        return result;
    }

    std::vector<Face*> endpoints;
    for (Face* face : ones) {
        const std::vector<Face*>& neighbors = blueGraph[face];
        if (neighbors.size() == 1) {
            endpoints.push_back(face);
        } else if (neighbors.size() != 2) {
            return mode2Invalid();
        }
    }
    if (endpoints.size() != 2) return mode2Invalid();

    auto makePath = [&](Face* start) {
        std::vector<Face*> path;
        Face* previous = nullptr;
        Face* current = start;
        while (current != nullptr) {
            if (containsFace(path, current)) {
                path.clear();
                return path;
            }
            path.push_back(current);
            Face* next = nullptr;
            for (Face* neighbor : blueGraph[current]) {
                if (neighbor != previous) {
                    if (next != nullptr) {
                        path.clear();
                        return path;
                    }
                    next = neighbor;
                }
            }
            previous = current;
            current = next;
        }
        if (path.size() != ones.size()) path.clear();
        return path;
    };

    const std::vector<Face*> path0 = makePath(endpoints[0]);
    if (path0.size() != ones.size()) return mode2Invalid();
    std::vector<Face*> path1 = path0;
    std::reverse(path1.begin(), path1.end());

    const DirectedRecovery forward =
        recoverDirectedPath(path0, views, oneSet, colors);
    const DirectedRecovery reverse =
        recoverDirectedPath(path1, views, oneSet, colors);
    if (!forward.ok && !reverse.ok) return mode2Invalid();

    const bool sameZeros =
        (forward.firstZero == reverse.firstZero &&
         forward.lastZero == reverse.lastZero) ||
        (forward.firstZero == reverse.lastZero &&
         forward.lastZero == reverse.firstZero);
    if (forward.ok && reverse.ok && !sameZeros) return mode2Invalid();

    const auto& selected = forward.ok ? forward : reverse;
    Face* zero0 = selected.firstZero;
    Face* zero1 = selected.lastZero;
    if (zero0 == nullptr || zero1 == nullptr ||
        zero0 == zero1 ||
        oneSet.find(zero0) != oneSet.end() ||
        oneSet.find(zero1) != oneSet.end()) {
        return mode2Invalid();
    }

    std::vector<Face*> patch = ones;
    patch.push_back(zero0);
    patch.push_back(zero1);
    if (!isSimpleTriangulatedDisk(patch, colors)) {
        return mode2Invalid();
    }
    return std::array<Face*, 2>{{zero0, zero1}};
}


// ---------------------------------------------------------------------------
// WS-B task-18: explicit side-info disambiguation of the mode-2 external zero
// pair.  The paper's 2-color Z-shape scheme relies on a geometric matcher to
// pick the two external zero triangles; that matcher is only unique when the
// patch triangulation is the paper's exact Z shape.  Instead, the encoder can
// enumerate the same deterministic candidate list the decoder will enumerate
// (only quantized vertex positions, face colors and connectivity are used) and
// transmit a small per-patch index.
//
// Candidate determinism: a face's sort key is the integer centroid of its
// quantized vertices (sum/count truncation, order-invariant); pairs are sorted
// lexicographically by their normalized (smaller, larger) key pair; pairs whose
// key pair collides with another pair are dropped (ambiguous).
// ---------------------------------------------------------------------------

using Face = MCGAL::Facet;
using Halfedge = MCGAL::Halfedge;
using Vertex = MCGAL::Vertex;
constexpr std::uint8_t kZero = 0;
constexpr std::uint8_t kBlue = 1;

struct Mode2ZeroPair {
    Face* a = nullptr;
    Face* b = nullptr;
};

inline bool mode2FaceCentroidQ(Face* face,
                               const std::shared_ptr<MCGAL::Mesh>& mesh,
                               std::array<std::int64_t, 3>& out) {
    if (face == nullptr || mesh == nullptr) return false;
    Halfedge* start = face->proxyHalfedge();
    if (start == nullptr) return false;
    std::int64_t sx = 0, sy = 0, sz = 0;
    int n = 0;
    Halfedge* it = start;
    for (int guard = 0; guard < 64; ++guard) {
        Vertex* v = it->vertex();
        if (v != nullptr) {
            const MCGAL::PointInt q = mesh->floatPosToInt(v->point());
            sx += q.x();
            sy += q.y();
            sz += q.z();
            ++n;
        }
        Halfedge* nx = it->next();
        if (nx == nullptr || nx == start) break;
        it = nx;
    }
    if (n == 0) return false;
    out = {{sx / n, sy / n, sz / n}};
    return true;
}

inline bool mode2KeyLess(const std::array<std::int64_t, 3>& a,
                         const std::array<std::int64_t, 3>& b) {
    if (a[0] != b[0]) return a[0] < b[0];
    if (a[1] != b[1]) return a[1] < b[1];
    return a[2] < b[2];
}

inline bool mode2KeyEqual(const std::array<std::int64_t, 3>& a,
                          const std::array<std::int64_t, 3>& b) {
    return a[0] == b[0] && a[1] == b[1] && a[2] == b[2];
}

// Path-shape validity of a candidate patch: the induced face-adjacency graph of
// ones+{zeroA,zeroB} must be a path (all degrees 1..2, exactly two leaves and
// they are the two zeros), i.e. the colored middle run is a path whose two
// external zero triangles attach at its endpoints.
inline bool mode2PatchIsZPath(
    const std::vector<Face*>& ones,
    Face* zeroA,
    Face* zeroB,
    const std::unordered_map<Face*, std::uint8_t>& colors) {
    if (ones.empty() || zeroA == nullptr || zeroB == nullptr || zeroA == zeroB)
        return false;
    auto colorOfFace = [&](Face* f, std::uint8_t& c) {
        auto it = colors.find(f);
        if (it == colors.end()) return false;
        c = it->second;
        return true;
    };
    std::uint8_t za = 0, zb = 0;
    if (!colorOfFace(zeroA, za) || za != kZero) return false;
    if (!colorOfFace(zeroB, zb) || zb != kZero) return false;
    std::unordered_set<Face*> oneSet(ones.begin(), ones.end());
    if (oneSet.size() != ones.size()) return false;
    if (oneSet.count(zeroA) != 0 || oneSet.count(zeroB) != 0) return false;
    for (Face* f : ones) {
        std::uint8_t c = 0;
        if (!colorOfFace(f, c) || c != kBlue) return false;
    }
    std::unordered_set<Face*> faceSet = oneSet;
    faceSet.insert(zeroA);
    faceSet.insert(zeroB);
    std::unordered_map<Face*, int> degree;
    int edgeTwice = 0;
    for (Face* f : faceSet) {
        Halfedge* start = f->proxyHalfedge();
        if (start == nullptr) return false;
        Halfedge* it = start;
        for (int guard = 0; guard < 64; ++guard) {
            Halfedge* opp = it->opposite();
            Face* nf = (opp != nullptr) ? opp->face() : nullptr;
            if (nf != nullptr && faceSet.count(nf) != 0) {
                ++degree[f];
                ++edgeTwice;
            }
            Halfedge* nx = it->next();
            if (nx == nullptr || nx == start) break;
            it = nx;
        }
    }
    if (edgeTwice / 2 != static_cast<int>(faceSet.size()) - 1) return false;
    int leaves = 0;
    for (Face* f : faceSet) {
        const int d = degree[f];
        if (d == 1) ++leaves;
        else if (d != 2) return false;
    }
    return leaves == 2 && degree[zeroA] == 1 && degree[zeroB] == 1;
}

// Tree-shape validity: ones+{zeroA,zeroB} must be a connected face set with
// exactly (n-1) shared edges (a triangulated disk), which is what the decoder's
// collectPatchTreeDiagonals() reconstruction requires. Unlike mode2PatchIsZPath
// this does NOT require the patch adjacency to be a path, so it accepts every
// triangulation of the patch polygon.
inline bool mode2PatchIsTree(
    const std::vector<Face*>& ones,
    Face* zeroA,
    Face* zeroB,
    const std::unordered_map<Face*, std::uint8_t>& colors) {
    if (ones.empty() || zeroA == nullptr || zeroB == nullptr || zeroA == zeroB)
        return false;
    auto colorOfFace = [&](Face* f, std::uint8_t& c) {
        auto it = colors.find(f);
        if (it == colors.end()) return false;
        c = it->second;
        return true;
    };
    std::uint8_t c;
    if (!colorOfFace(zeroA, c) || c != kZero) return false;
    if (!colorOfFace(zeroB, c) || c != kZero) return false;
    std::unordered_set<Face*> oneSet(ones.begin(), ones.end());
    if (oneSet.size() != ones.size()) return false;
    if (oneSet.count(zeroA) != 0 || oneSet.count(zeroB) != 0) return false;
    std::unordered_set<Face*> faceSet = oneSet;
    faceSet.insert(zeroA);
    faceSet.insert(zeroB);
    std::unordered_map<Face*, std::vector<Face*>> nb;
    for (Face* f : faceSet) {
        Halfedge* start = f->proxyHalfedge();
        if (start == nullptr) return false;
        Halfedge* it = start;
        for (int guard = 0; guard < 64; ++guard) {
            Halfedge* opp = it->opposite();
            Face* nf = (opp != nullptr) ? opp->face() : nullptr;
            if (nf != nullptr && faceSet.count(nf) != 0) nb[f].push_back(nf);
            Halfedge* nx = it->next();
            if (nx == nullptr || nx == start) break;
            it = nx;
        }
    }
    int edgesTwice = 0;
    for (const auto& kv : nb) edgesTwice += static_cast<int>(kv.second.size());
    if (edgesTwice / 2 != static_cast<int>(faceSet.size()) - 1) return false;
    std::unordered_set<Face*> seen;
    std::queue<Face*> q;
    q.push(zeroA);
    seen.insert(zeroA);
    while (!q.empty()) {
        Face* cur = q.front();
        q.pop();
        for (Face* n : nb[cur]) {
            if (seen.insert(n).second) q.push(n);
        }
    }
    return seen.size() == faceSet.size();
}

// Enumerate all deterministic candidate zero pairs for one blue component.
// requirePath=true selects the paper Z-path shape (mode-2 geometric matcher);
// requirePath=false accepts any triangulated-disk completion (used by the
// explicit side-info mode, where the index disambiguates the zero pair).
inline std::vector<Mode2ZeroPair> enumerateMode2ZeroPairs(
    const std::vector<Face*>& ones,
    const std::unordered_map<Face*, std::uint8_t>& colors,
    const std::shared_ptr<MCGAL::Mesh>& mesh,
    bool requirePath = true) {
    std::vector<Mode2ZeroPair> result;
    if (ones.empty() || mesh == nullptr) return result;
    std::unordered_set<Face*> oneSet(ones.begin(), ones.end());
    std::vector<Face*> zeros;
    std::unordered_set<Face*> seenZero;
    // Side-info mode additionally rejects zero candidates that touch a FOREIGN
    // blue face (color 1 outside this run): those zeros belong to an adjacent
    // patch, and including them makes the candidate list needlessly large. The
    // filter uses only connectivity + colors, so encoder and decoder agree.
    auto touchesForeignBlue = [&](Face* z) {
        Halfedge* start = z->proxyHalfedge();
        if (start == nullptr) return true;
        Halfedge* it = start;
        for (int guard = 0; guard < 64; ++guard) {
            Halfedge* opp = it->opposite();
            Face* nf = (opp != nullptr) ? opp->face() : nullptr;
            if (nf != nullptr && oneSet.count(nf) == 0) {
                auto cit = colors.find(nf);
                if (cit != colors.end() && cit->second == kBlue) return true;
            }
            Halfedge* nx = it->next();
            if (nx == nullptr || nx == start) break;
            it = nx;
        }
        return false;
    };
    for (Face* f : ones) {
        Halfedge* start = f->proxyHalfedge();
        if (start == nullptr) continue;
        Halfedge* it = start;
        for (int guard = 0; guard < 64; ++guard) {
            Halfedge* opp = it->opposite();
            Face* nf = (opp != nullptr) ? opp->face() : nullptr;
            if (nf != nullptr && oneSet.count(nf) == 0 &&
                seenZero.insert(nf).second) {
                auto cit = colors.find(nf);
                if (cit != colors.end() && cit->second == kZero) {
                    if (requirePath || !touchesForeignBlue(nf)) zeros.push_back(nf);
                }
            }
            Halfedge* nx = it->next();
            if (nx == nullptr || nx == start) break;
            it = nx;
        }
    }
    struct Cand {
        std::array<std::int64_t, 3> ka;
        std::array<std::int64_t, 3> kb;
        Face* a;
        Face* b;
    };
    std::vector<Cand> cands;
    for (std::size_t i = 0; i < zeros.size(); ++i) {
        for (std::size_t j = i + 1; j < zeros.size(); ++j) {
            Face* a = zeros[i];
            Face* b = zeros[j];
            const bool shapeOk = requirePath
                                      ? mode2PatchIsZPath(ones, a, b, colors)
                                      : mode2PatchIsTree(ones, a, b, colors);
            if (!shapeOk) continue;
            std::array<std::int64_t, 3> ca{}, cb{};
            if (!mode2FaceCentroidQ(a, mesh, ca)) continue;
            if (!mode2FaceCentroidQ(b, mesh, cb)) continue;
            if (mode2KeyLess(cb, ca)) std::swap(ca, cb);
            cands.push_back(Cand{ca, cb, a, b});
        }
    }
    std::sort(cands.begin(), cands.end(), [](const Cand& x, const Cand& y) {
        if (!mode2KeyEqual(x.ka, y.ka)) return mode2KeyLess(x.ka, y.ka);
        return mode2KeyLess(x.kb, y.kb);
    });
    // v4: candidate COUNT must be position-independent (it is the number of
    // tree-valid completions), so key-collision groups are NOT dropped anymore.
    // Only the ORDER uses quantized centroids; exact key ties are ordered by the
    // inserted-faces order, which is itself deterministic for a given mesh.
    for (std::size_t i = 0; i < cands.size(); ++i) {
        result.push_back(Mode2ZeroPair{cands[i].a, cands[i].b});
    }
    return result;
}

// ---------------------------------------------------------------------------
// v7 (task-36): direct zero-index side stream.
//
// Instead of an index into a *pair* list (v4-v6; the pair list depends on the
// decoded colour map and was therefore not patch-exact), the encoder transmits
// the two zero faces as indices into a geometry-ordered candidate list:
//
//   candidates = faces adjacent (across one edge) to any face of `ones`,
//                excluding faces of `ones`, restricted to the current round's
//                face set (`allowed`), ordered by quantized face-centroid key.
//
// The quantized centroid is available bit-identically on both sides (decoder
// vertex positions are the encoder's reconstructed positions), so the list is
// deterministic without depending on half-edge iteration order or on the
// decoded colour map. Ambiguous centroid keys (two candidates with the same key)
// make the round invalid -> mode-4 fallback, so an index can never silently
// select a different face on the two sides. The decoder additionally accepts a
// patch only after the existing tree check (n faces, n-1 diagonals, connected).
inline std::vector<Face*> enumerateMode2ZeroCandidates(
    const std::vector<Face*>& ones,
    const std::unordered_set<Face*>& allowed,
    const std::shared_ptr<MCGAL::Mesh>& mesh,
    bool& keysUnique) {
    keysUnique = true;
    std::vector<Face*> out;
    if (ones.empty() || allowed.empty() || mesh == nullptr) return out;
    std::unordered_set<Face*> oneSet(ones.begin(), ones.end());
    std::unordered_set<Face*> seen;
    for (Face* f : ones) {
        Halfedge* start = f ? f->proxyHalfedge() : nullptr;
        if (start == nullptr) continue;
        Halfedge* it = start;
        for (int guard = 0; guard < 64; ++guard) {
            Halfedge* opp = it ? it->opposite() : nullptr;
            Face* nf = opp ? opp->face() : nullptr;
            if (nf != nullptr && oneSet.count(nf) == 0 &&
                allowed.count(nf) != 0 && seen.insert(nf).second) {
                out.push_back(nf);
            }
            Halfedge* nx = it ? it->next() : nullptr;
            if (nx == nullptr || nx == start) break;
            it = nx;
        }
    }
    if (out.size() < 2) return out;
    struct Cand {
        std::array<std::int64_t, 3> key;
        Face* face;
    };
    std::vector<Cand> cands;
    cands.reserve(out.size());
    for (Face* f : out) {
        std::array<std::int64_t, 3> key{};
        if (!mode2FaceCentroidQ(f, mesh, key)) {
            keysUnique = false;
            return {};
        }
        cands.push_back(Cand{key, f});
    }
    std::sort(cands.begin(), cands.end(), [](const Cand& a, const Cand& b) {
        if (!mode2KeyEqual(a.key, b.key)) return mode2KeyLess(a.key, b.key);
        return false;
    });
    for (std::size_t i = 1; i < cands.size(); ++i) {
        if (mode2KeyEqual(cands[i - 1].key, cands[i].key)) {
            keysUnique = false;
            return {};
        }
    }
    std::vector<Face*> result;
    result.reserve(cands.size());
    for (const auto& c : cands) result.push_back(c.face);
    return result;
}

inline int mode2FindZeroCandidateIndex(const std::vector<Face*>& cands,
                                       Face* face) {
    if (face == nullptr) return -1;
    for (std::size_t i = 0; i < cands.size(); ++i) {
        if (cands[i] == face) return static_cast<int>(i);
    }
    return -1;
}

inline int mode2FindPairIndex(const std::vector<Mode2ZeroPair>& cands,
                              Face* a,
                              Face* b) {
    for (std::size_t i = 0; i < cands.size(); ++i) {
        if ((cands[i].a == a && cands[i].b == b) ||
            (cands[i].a == b && cands[i].b == a))
            return static_cast<int>(i);
    }
    return -1;
}

}  // namespace pcatm
}  // namespace pmsf

#endif  // PMSF_PCATM_PAPER_MODE2_H
