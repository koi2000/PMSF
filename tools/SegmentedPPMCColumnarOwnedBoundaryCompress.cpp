// HEAD-port (2026-10): PMSF.h facade removed in Phase-A/B refactor; owned-boundary
// operators moved from src/ to impl/<ALGO>/ subtrees:
//   compress/PPMCSegmentationSymbolCollectOperator.h -> PPMC/compress/...
//   elimination/PPMCEliminateOperator.h              -> PPMC/elimination/...
//   elimination/SegVertexRemovalEliminateOperator.h  -> Segmentation/elimination/...
#include "common/TimeRecorder.h"
#include "core/MeshRuntime.h"
#include "PPMC/compress/PPMCSegmentationSymbolCollectOperator.h"
#include "compress/ColumnarSerializeOperator.h"
#include "PPMC/elimination/PPMCEliminateOperator.h"
#include "Segmentation/elimination/SegVertexRemovalEliminateOperator.h"
#include "header/PMSFHeader.h"
#include "operator/IsRemovableOperator.h"
#include "operator/SeedIsRemovableOperator.h"
#include "options/SelectOptions.h"
#include "segmentation/PreDefinedFacetSegmentOperator.h"
#include "segmentation/PreDefinedVertexSegmentOperator.h"
#include "selection/ErrorSource.h"
#include "selection/GreedySelector.h"
#include "PPMC/PPMCIsRemovableOperator.h"
#include "Mesh.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <numeric>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace {

int parseIntArg(const std::string& arg, const std::string& prefix, int fallback) {
    if (arg.rfind(prefix, 0) != 0) return fallback;
    return std::atoi(arg.substr(prefix.size()).c_str());
}

bool parseBoolValue(const std::string& v) {
    return !(v == "0" || v == "false" || v == "FALSE" || v == "off" || v == "OFF");
}

bool envFlag(const char* name) {
    const char* v = std::getenv(name);
    return v && *v && parseBoolValue(v);
}

int envInt(const char* name, int fallback) {
    const char* v = std::getenv(name);
    return (v && *v) ? std::atoi(v) : fallback;
}

struct CoordKey {
    int x = 0;
    int y = 0;
    int z = 0;

    bool operator==(const CoordKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct CoordKeyHash {
    std::size_t operator()(const CoordKey& k) const {
        std::size_t h = std::hash<int>{}(k.x);
        h ^= std::hash<int>{}(k.y) + 0x9e3779b9u + (h << 6) + (h >> 2);
        h ^= std::hash<int>{}(k.z) + 0x9e3779b9u + (h << 6) + (h >> 2);
        return h;
    }
};

struct BoundaryCompressionStats {
    int freeProtectedBoundaryEliminated = 0;
    int freeTopologyBoundaryEliminated = 0;
    int freeInteriorEliminated = 0;
    int atomicCandidateCoords = 0;
    int atomicAcceptedCoords = 0;
    int atomicRejectedCoords = 0;
    int atomicRejectedByCopies = 0;
    int atomicRejectedByGain = 0;
    int atomicEstimatedSavedBytes = 0;
    int atomicEstimatedCostBytes = 0;
};

CoordKey coordKeyForVertex(const std::shared_ptr<MCGAL::Mesh>& mesh, MCGAL::Vertex* v) {
    if (!mesh || !v) return {};
    MCGAL::PointInt q = mesh->floatPosToInt(v->point());
    return CoordKey{q.x(), q.y(), q.z()};
}

bool touchesTopologyBoundary(MCGAL::Vertex* v) {
    if (!v) return false;
    for (MCGAL::Halfedge* h : v->halfedges()) {
        if (!h) continue;
        if (!h->opposite()) return true;
    }
    return false;
}

std::unordered_set<MCGAL::Facet*> collectReachableGroupFaces(MCGAL::Halfedge* seed,
                                                              int group) {
    std::unordered_set<MCGAL::Facet*> seen;
    if (!seed) return seen;
    std::vector<MCGAL::Halfedge*> stack;
    stack.push_back(seed);
    while (!stack.empty()) {
        MCGAL::Halfedge* h = stack.back();
        stack.pop_back();
        if (!h || !h->face() || h->face()->isRemoved() ||
            h->face()->groupId() != group) {
            continue;
        }
        MCGAL::Facet* f = h->face();
        if (!seen.insert(f).second) continue;
        MCGAL::Halfedge* e = f->proxyHalfedge();
        MCGAL::Halfedge* start = e;
        int guard = 0;
        while (e && ++guard <= 8192) {
            MCGAL::Halfedge* opp = e->opposite();
            if (opp && opp->face() && !opp->face()->isRemoved() &&
                opp->face()->groupId() == group && !seen.count(opp->face())) {
                stack.push_back(opp);
            }
            if (!e->next()) break;
            e = e->next();
            if (e == start) break;
        }
    }
    return seen;
}

bool inReachableFaces(MCGAL::Halfedge* h,
                      const std::unordered_set<MCGAL::Facet*>* reachable) {
    if (!reachable) return true;
    return h && h->face() && reachable->count(h->face()) > 0;
}

std::vector<MCGAL::Vertex*> liveFaceVertices(MCGAL::Facet* f) {
    std::vector<MCGAL::Vertex*> vertices;
    if (!f || f->isRemoved() || !f->proxyHalfedge()) return vertices;
    MCGAL::Halfedge* h = f->proxyHalfedge();
    MCGAL::Halfedge* start = h;
    int guard = 0;
    while (h && ++guard <= 8192) {
        if (h->vertex() && !h->vertex()->isRemoved()) {
            vertices.push_back(h->vertex());
        }
        if (!h->next()) {
            if (h->end_vertex() && !h->end_vertex()->isRemoved()) {
                vertices.push_back(h->end_vertex());
            }
            break;
        }
        h = h->next();
        if (h == start) break;
    }
    return vertices;
}

std::unordered_set<MCGAL::Vertex*> collectPartitionBoundaryVertices(
    const std::shared_ptr<MCGAL::Mesh>& mesh) {
    struct CoordGroups {
        int firstGroup = -1;
        bool multiGroup = false;
    };
    std::unordered_map<CoordKey, CoordGroups, CoordKeyHash> coordGroups;
    std::unordered_set<MCGAL::Vertex*> boundary;
    if (!mesh) return boundary;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        const int group = f->groupId();
        for (MCGAL::Vertex* v : liveFaceVertices(f)) {
            CoordGroups& info = coordGroups[coordKeyForVertex(mesh, v)];
            if (info.firstGroup < 0) {
                info.firstGroup = group;
            } else if (info.firstGroup != group) {
                info.multiGroup = true;
            }
        }
    }
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved()) continue;
        for (MCGAL::Vertex* v : liveFaceVertices(f)) {
            auto it = coordGroups.find(coordKeyForVertex(mesh, v));
            if (it != coordGroups.end() && it->second.multiGroup) {
                boundary.insert(v);
            }
        }
    }
    return boundary;
}

bool removesProtectedPartitionBoundary(
    MCGAL::Halfedge* h,
    const std::unordered_set<MCGAL::Vertex*>* protectedVertices) {
    if (!protectedVertices || !h || !h->end_vertex()) return false;
    return protectedVertices->count(h->end_vertex()) > 0 ||
           touchesTopologyBoundary(h->end_vertex());
}

bool touchesProtectedPartitionBoundaryOneRing(
    MCGAL::Halfedge* h,
    const std::unordered_set<MCGAL::Vertex*>* protectedVertices) {
    if (removesProtectedPartitionBoundary(h, protectedVertices)) return true;
    if (!protectedVertices || !h || !h->end_vertex()) return false;
    MCGAL::Vertex* removed = h->end_vertex();
    for (MCGAL::Halfedge* e : removed->halfedges()) {
        if (!e) continue;
        if ((e->vertex() && protectedVertices->count(e->vertex()) > 0) ||
            (e->end_vertex() && protectedVertices->count(e->end_vertex()) > 0)) {
            return true;
        }
    }
    return false;
}

bool violatesProtectedPartitionBoundaryPolicy(
    MCGAL::Halfedge* h,
    const std::unordered_set<MCGAL::Vertex*>* protectedVertices,
    bool protectOneRing) {
    return protectOneRing
               ? touchesProtectedPartitionBoundaryOneRing(h, protectedVertices)
               : removesProtectedPartitionBoundary(h, protectedVertices);
}

void recordOpenBoundaryElimination(
    MCGAL::Vertex* removed,
    const std::unordered_set<MCGAL::Vertex*>* protectedVertices,
    BoundaryCompressionStats* stats,
    bool wasTopologyBoundary) {
    if (!removed || !stats) return;
    if (protectedVertices && protectedVertices->count(removed) > 0) {
        ++stats->freeProtectedBoundaryEliminated;
        return;
    }
    if (wasTopologyBoundary) {
        ++stats->freeTopologyBoundaryEliminated;
        return;
    }
    ++stats->freeInteriorEliminated;
}

MCGAL::Halfedge* findLiveGroupHalfedge(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                       MCGAL::Vertex* from,
                                       MCGAL::Vertex* to,
                                       int group) {
    if (!mesh) return nullptr;
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || f->groupId() != group || !f->proxyHalfedge()) continue;
        MCGAL::Halfedge* h = f->proxyHalfedge();
        MCGAL::Halfedge* start = h;
        int guard = 0;
        do {
            if (h && !h->isRemoved() && (from == nullptr || h->vertex() == from) &&
                (to == nullptr || h->end_vertex() == to)) {
                return h;
            }
            h = h ? h->next() : nullptr;
        } while (h && h != start && ++guard <= 8192);
    }
    return nullptr;
}

MCGAL::Halfedge* refreshGroupSeed(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                  MCGAL::Vertex* v0,
                                  MCGAL::Vertex* v1,
                                  int group,
                                  MCGAL::Halfedge* fallback) {
    if (MCGAL::Halfedge* h = findLiveGroupHalfedge(mesh, v0, v1, group)) return h;
    if (MCGAL::Halfedge* h = findLiveGroupHalfedge(mesh, v0, nullptr, group)) return h;
    if (MCGAL::Halfedge* h = findLiveGroupHalfedge(mesh, v1, nullptr, group)) return h;
    if (fallback && !fallback->isRemoved() && fallback->face() &&
        !fallback->face()->isRemoved() && fallback->face()->groupId() == group) {
        return fallback;
    }
    return findLiveGroupHalfedge(mesh, nullptr, nullptr, group);
}

MCGAL::Halfedge* safePrevInFace(MCGAL::Halfedge* h) {
    if (!h || !h->face() || !h->face()->proxyHalfedge()) return nullptr;
    MCGAL::Halfedge* start = h->face()->proxyHalfedge();
    MCGAL::Halfedge* it = start;
    int guard = 0;
    while (it && it->next() && it->next() != h) {
        it = it->next();
        if (it == start || ++guard > 4096) return nullptr;
    }
    return (it && it->next() == h) ? it : nullptr;
}

void assignFaceGroupsFromVertices(const std::shared_ptr<MCGAL::Mesh>& mesh, int groupCount) {
    if (!mesh || groupCount <= 0) return;
    std::vector<int> counts(static_cast<std::size_t>(groupCount), 0);
    for (MCGAL::Facet* f : mesh->faces()) {
        if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
        std::fill(counts.begin(), counts.end(), 0);
        MCGAL::Halfedge* start = f->proxyHalfedge();
        MCGAL::Halfedge* h = start;
        int guard = 0;
        while (h && ++guard <= 8192) {
            if (h->vertex()) {
                const int gid = h->vertex()->groupId();
                if (gid >= 0 && gid < groupCount) {
                    counts[static_cast<std::size_t>(gid)]++;
                }
            }
            if (!h->next()) {
                if (h->end_vertex()) {
                    const int gid = h->end_vertex()->groupId();
                    if (gid >= 0 && gid < groupCount) {
                        counts[static_cast<std::size_t>(gid)]++;
                    }
                }
                break;
            }
            h = h->next();
            if (h == start) break;
        }
        int bestGroup = f->groupId();
        int bestCount = -1;
        for (int gid = 0; gid < groupCount; ++gid) {
            if (counts[static_cast<std::size_t>(gid)] > bestCount) {
                bestCount = counts[static_cast<std::size_t>(gid)];
                bestGroup = gid;
            }
        }
        if (bestGroup >= 0) {
            f->setGroupId(bestGroup);
        }
    }
}

class OpenBoundaryPPMCIsRemovableOperator : public IsRemovableOperator {
  public:
    void init(std::vector<MCGAL::Vertex*>,
              std::vector<MCGAL::Halfedge*>,
              std::vector<MCGAL::Facet*>) override {}

    bool isRemovable(HalfedgeErrorEntry& entry) override {
        MCGAL::Halfedge* h = entry.edge;
        if (!h || !h->vertex() || !h->end_vertex() ||
            !h->face() || h->face()->isRemoved()) {
            return false;
        }
        if (h->vertex()->isRemoved() || h->end_vertex()->isRemoved() ||
            h->vertex()->isCollapsed() || h->end_vertex()->isCollapsed()) {
            return false;
        }

        MCGAL::Vertex* removed = h->end_vertex();
        if (!removed->isBoundary() && !touchesTopologyBoundary(removed)) {
            if (!incidentFacesAvailable(removed)) return false;
            return ppmc_.isRemovable(entry);
        }
        removed->setBoundary();
        return isBoundaryRemovable(h, removed);
    }

  private:
    static bool incidentFacesAvailable(MCGAL::Vertex* v) {
        if (!v) return false;
        for (MCGAL::Halfedge* e : v->halfedges()) {
            if (!e) continue;
            MCGAL::Facet* f = e->face();
            if (f && (f->isConquered() || f->isRemoved() || faceIsOpen(f))) return false;
            MCGAL::Halfedge* opp = e->opposite();
            MCGAL::Facet* of = opp ? opp->face() : nullptr;
            if (of && (of->isConquered() || of->isRemoved() || faceIsOpen(of))) return false;
        }
        return true;
    }

    static bool faceIsOpen(MCGAL::Facet* f) {
        if (!f || !f->proxyHalfedge()) return true;
        MCGAL::Halfedge* start = f->proxyHalfedge();
        MCGAL::Halfedge* h = start;
        int guard = 0;
        while (h) {
            if (!h->next()) return true;
            h = h->next();
            if (h == start) return false;
            if (++guard > 4096) return true;
        }
        return true;
    }

    static bool isBoundaryRemovable(MCGAL::Halfedge* h, MCGAL::Vertex* removed) {
        int degree = static_cast<int>(removed->halfedges().size());
        if (degree < 3 || degree > 16 || removed->isConquered()) {
            return false;
        }
        if (!canEraseBoundaryPlus(h)) {
            return false;
        }
        for (MCGAL::Halfedge* e : removed->halfedges()) {
            if (!e || !e->vertex() || !e->end_vertex()) return false;
            if (e->isRemoved() || e->vertex()->isRemoved() || e->end_vertex()->isRemoved() ||
                e->vertex()->isCollapsed() || e->end_vertex()->isCollapsed()) {
                return false;
            }
            MCGAL::Facet* f = e->face();
            if (f && (f->isRemoved() || f->isConquered())) return false;
            MCGAL::Halfedge* opp = e->opposite();
            MCGAL::Facet* of = opp ? opp->face() : nullptr;
            if (of && (of->isRemoved() || of->isConquered())) return false;
        }
        return true;
    }

    static bool canEraseBoundaryPlus(MCGAL::Halfedge* h) {
        if (!h || !h->end_vertex()) return false;
        MCGAL::Halfedge* start = h;
        int guard = 0;
        while (start && start->opposite()) {
            if (!start->opposite()->next() || !start->opposite()->next()->next()) {
                return false;
            }
            start = start->opposite()->next()->next();
            if (++guard > 64) return false;
        }
        if (!start || !start->next() || !start->next()->opposite()) return false;
        if (!safePrevInFace(start)) return false;

        MCGAL::Halfedge* g = start->next()->opposite();
        guard = 0;
        while (g && g != start) {
            if (!g->opposite() || !g->next()) return false;
            if (!safePrevInFace(g)) return false;
            MCGAL::Halfedge* gnext = g->next()->opposite();
            if (gnext == nullptr) return true;
            g = gnext;
            if (++guard > 64) return false;
        }
        return g == start;
    }

    PPMCIsRemovableOperator ppmc_;
};

class PartitionBoundaryIsRemovableOperator : public IsRemovableOperator {
  public:
    explicit PartitionBoundaryIsRemovableOperator(
        const std::unordered_set<MCGAL::Vertex*>* protectedVertices,
        bool protectOneRing = false)
        : protectedVertices_(protectedVertices), protectOneRing_(protectOneRing) {}

    void init(std::vector<MCGAL::Vertex*>,
              std::vector<MCGAL::Halfedge*>,
              std::vector<MCGAL::Facet*>) override {}

    bool isRemovable(HalfedgeErrorEntry& entry) override {
        return !violatesProtectedPartitionBoundaryPolicy(
            entry.edge, protectedVertices_, protectOneRing_);
    }

  private:
    const std::unordered_set<MCGAL::Vertex*>* protectedVertices_ = nullptr;
    bool protectOneRing_ = false;
};

class OpenBoundaryPPMCEliminateOperator {
  public:
    void init(std::shared_ptr<MCGAL::Mesh> mesh, PMSFHeader header) {
        mesh_ = mesh;
        interior_.init(mesh, header);
        boundary_.init(mesh, header, true);
    }

    bool eliminate(HalfedgeErrorEntry& entry) {
        MCGAL::Halfedge* h = entry.edge;
        if (!h || !h->end_vertex()) return false;
        MCGAL::Vertex* removed = h->end_vertex();
        if (!removed->isBoundary() && !touchesTopologyBoundary(removed)) {
            return interior_.eliminate(entry);
        }

        removed->setBoundary();
        return boundary_.eliminate(entry);
    }

  private:
    std::shared_ptr<MCGAL::Mesh> mesh_;
    PPMCEliminateOperator interior_;
    SegVertexRemovalEliminateOperator boundary_;
};

int runExhaustiveOpenBoundaryRound(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                   int group,
		                                   OpenBoundaryPPMCIsRemovableOperator& removable,
	                                   SeedIsRemovableOperator& seedGuard,
	                                   OpenBoundaryPPMCEliminateOperator& eliminate,
	                                   int maxPerGroup,
	                                   bool boundaryOnly = false,
	                                   const std::unordered_set<MCGAL::Facet*>* reachable = nullptr,
	                                   const std::unordered_set<MCGAL::Vertex*>* protectedVertices = nullptr,
	                                   BoundaryCompressionStats* stats = nullptr) {
    int eliminated = 0;
    bool progressed = true;
    while (progressed) {
        progressed = false;
        for (MCGAL::Facet* face : mesh->faces()) {
	            if (!face || face->isRemoved() || face->groupId() != group ||
	                !face->proxyHalfedge() || face->isConquered()) {
	                continue;
	            }
	            if (reachable && !reachable->count(face)) continue;
            MCGAL::Halfedge* start = face->proxyHalfedge();
            MCGAL::Halfedge* h = start;
            int guard = 0;
            while (h && ++guard <= 8192) {
                if (boundaryOnly) {
                    MCGAL::Vertex* removed = h->end_vertex();
                    if (!removed ||
                        (!removed->isBoundary() && !touchesTopologyBoundary(removed))) {
                        if (!h->next()) break;
                        h = h->next();
                        if (h == start) break;
                        continue;
                    }
                }
                MCGAL::Point p;
                HalfedgeErrorEntry entry(h, p, 0, 0);
                MCGAL::Vertex* removed = h->end_vertex();
                const bool wasTopologyBoundary =
                    removed && (removed->isBoundary() || touchesTopologyBoundary(removed));
	                if (inReachableFaces(h, reachable) &&
	                    removable.isRemovable(entry) && seedGuard.isRemovable(entry) &&
	                    eliminate.eliminate(entry)) {
                    recordOpenBoundaryElimination(
                        removed, protectedVertices, stats, wasTopologyBoundary);
                    ++eliminated;
                    progressed = true;
                    break;
                }
                if (!h->next()) break;
                h = h->next();
                if (h == start) break;
            }
            if (progressed) break;
        }
        if (maxPerGroup > 0 && eliminated >= maxPerGroup) break;
    }
    return eliminated;
}

int runSynchronizedBoundaryCoordClosure(
    const std::shared_ptr<MCGAL::Mesh>& mesh,
    const CoordKey& key,
    int sourceGroup,
    std::vector<MCGAL::Halfedge*>& seeds,
    const std::vector<std::pair<MCGAL::Vertex*, MCGAL::Vertex*>>& seedEndpoints,
    OpenBoundaryPPMCIsRemovableOperator& removable,
    SeedIsRemovableOperator& seedGuard,
    OpenBoundaryPPMCEliminateOperator& eliminate,
    const std::unordered_set<MCGAL::Vertex*>* protectedVertices,
    BoundaryCompressionStats* stats,
    int maxCopies) {
    if (!mesh) return 0;
    int eliminated = 0;
    for (std::size_t group = 0; group < seeds.size(); ++group) {
        if (maxCopies > 0 && eliminated >= maxCopies) break;
        if (static_cast<int>(group) == sourceGroup) continue;
        if (group < seedEndpoints.size()) {
            seeds[group] = refreshGroupSeed(mesh,
                                            seedEndpoints[group].first,
                                            seedEndpoints[group].second,
                                            static_cast<int>(group),
                                            seeds[group]);
        }
        auto reachable = collectReachableGroupFaces(seeds[group], static_cast<int>(group));
        bool progressedInGroup = false;
        for (MCGAL::Facet* face : mesh->faces()) {
            if (!face || face->isRemoved() ||
                face->groupId() != static_cast<int>(group) ||
                !face->proxyHalfedge() || face->isConquered() ||
                !reachable.count(face)) {
                continue;
            }
            MCGAL::Halfedge* start = face->proxyHalfedge();
            MCGAL::Halfedge* h = start;
            int guard = 0;
            while (h && ++guard <= 8192) {
                MCGAL::Vertex* removed = h->end_vertex();
                if (removed && !removed->isRemoved() &&
                    coordKeyForVertex(mesh, removed) == key &&
                    (removed->isBoundary() || touchesTopologyBoundary(removed) ||
                     (protectedVertices && protectedVertices->count(removed) > 0))) {
                    MCGAL::Point p;
                    HalfedgeErrorEntry entry(h, p, 0, 0);
                    const bool wasTopologyBoundary =
                        removed->isBoundary() || touchesTopologyBoundary(removed);
                    if (removable.isRemovable(entry) &&
                        seedGuard.isRemovable(entry) &&
                        eliminate.eliminate(entry)) {
                        recordOpenBoundaryElimination(
                            removed, protectedVertices, stats, wasTopologyBoundary);
                        ++eliminated;
                        progressedInGroup = true;
                        break;
                    }
                }
                if (!h->next()) break;
                h = h->next();
                if (h == start) break;
            }
            if (progressedInGroup) break;
        }
    }
    return eliminated;
}

int runSynchronizedBoundaryCoordSweep(
    const std::shared_ptr<MCGAL::Mesh>& mesh,
    std::vector<MCGAL::Halfedge*>& seeds,
    const std::vector<std::pair<MCGAL::Vertex*, MCGAL::Vertex*>>& seedEndpoints,
    OpenBoundaryPPMCIsRemovableOperator& removable,
    SeedIsRemovableOperator& seedGuard,
    OpenBoundaryPPMCEliminateOperator& eliminate,
    const std::unordered_set<MCGAL::Vertex*>* protectedVertices,
    BoundaryCompressionStats* stats,
    int maxCoords,
    int maxCopiesPerCoord) {
    if (!mesh || !protectedVertices) return 0;
    std::vector<CoordKey> keys;
    keys.reserve(protectedVertices->size());
    std::unordered_set<CoordKey, CoordKeyHash> seen;
    for (MCGAL::Vertex* v : *protectedVertices) {
        if (!v || v->isRemoved()) continue;
        CoordKey key = coordKeyForVertex(mesh, v);
        if (seen.insert(key).second) {
            keys.push_back(key);
            if (maxCoords > 0 && static_cast<int>(keys.size()) >= maxCoords) break;
        }
    }

    int eliminated = 0;
    for (const CoordKey& key : keys) {
        eliminated += runSynchronizedBoundaryCoordClosure(
            mesh, key, -1, seeds, seedEndpoints, removable, seedGuard,
            eliminate, protectedVertices, stats, maxCopiesPerCoord);
    }
    return eliminated;
}

struct AtomicBoundaryCoordCandidate {
    MCGAL::Vertex* vertex = nullptr;
    MCGAL::Halfedge* edge = nullptr;
    bool wasTopologyBoundary = false;
};

struct BoundaryZeroOverheadGate {
    bool enabled = false;
    int minCopies = 4;
    int maxCopies = 16;
    int coordSaveBytes = 9;
    int copySaveBytes = 16;
    int eventCostBytes = 12;
    double minGainRatio = 2.0;
};

BoundaryZeroOverheadGate readBoundaryZeroOverheadGate() {
    BoundaryZeroOverheadGate gate;
    gate.enabled = envFlag("PMSF_OPEN_BOUNDARY_ZERO_OVERHEAD");
    gate.minCopies = std::max(1, envInt("PMSF_OPEN_BOUNDARY_MIN_COPIES", gate.minCopies));
    gate.maxCopies = std::max(0, envInt("PMSF_OPEN_BOUNDARY_MAX_COPIES", gate.maxCopies));
    gate.coordSaveBytes = std::max(
        1, envInt("PMSF_OPEN_BOUNDARY_COORD_SAVE_BYTES", gate.coordSaveBytes));
    gate.copySaveBytes = std::max(
        0, envInt("PMSF_OPEN_BOUNDARY_COPY_SAVE_BYTES", gate.copySaveBytes));
    gate.eventCostBytes = std::max(
        1, envInt("PMSF_OPEN_BOUNDARY_EVENT_COST_BYTES", gate.eventCostBytes));
    if (const char* v = std::getenv("PMSF_OPEN_BOUNDARY_MIN_GAIN_RATIO")) {
        if (*v) gate.minGainRatio = std::max(0.0, std::atof(v));
    }
    return gate;
}

bool boundaryCoordPassesZeroOverheadGate(const BoundaryZeroOverheadGate& gate,
                                         int copyCount,
                                         int& estimatedSavedBytes,
                                         int& estimatedCostBytes) {
    estimatedSavedBytes = gate.coordSaveBytes + copyCount * gate.copySaveBytes;
    estimatedCostBytes = copyCount * gate.eventCostBytes;
    if (!gate.enabled) return true;
    if (copyCount < gate.minCopies) return false;
    if (gate.maxCopies > 0 && copyCount > gate.maxCopies) return false;
    return static_cast<double>(estimatedSavedBytes) >=
           gate.minGainRatio * static_cast<double>(estimatedCostBytes);
}

bool collectAtomicBoundaryCoordCandidates(
    const std::shared_ptr<MCGAL::Mesh>& mesh,
    const CoordKey& key,
    std::vector<MCGAL::Halfedge*>& seeds,
    const std::vector<std::pair<MCGAL::Vertex*, MCGAL::Vertex*>>& seedEndpoints,
    OpenBoundaryPPMCIsRemovableOperator& removable,
    SeedIsRemovableOperator& seedGuard,
    const std::unordered_set<MCGAL::Vertex*>* protectedVertices,
    int maxCopies,
    std::vector<AtomicBoundaryCoordCandidate>& candidates) {
    candidates.clear();
    if (!mesh) return false;
    std::unordered_set<MCGAL::Vertex*> liveVertices;
    std::unordered_map<MCGAL::Vertex*, AtomicBoundaryCoordCandidate> candidateByVertex;

    for (std::size_t group = 0; group < seeds.size(); ++group) {
        if (group < seedEndpoints.size()) {
            seeds[group] = refreshGroupSeed(mesh,
                                            seedEndpoints[group].first,
                                            seedEndpoints[group].second,
                                            static_cast<int>(group),
                                            seeds[group]);
        }
        auto reachable = collectReachableGroupFaces(seeds[group], static_cast<int>(group));
        for (MCGAL::Facet* face : mesh->faces()) {
            if (!face || face->isRemoved() ||
                face->groupId() != static_cast<int>(group) ||
                !face->proxyHalfedge() || face->isConquered() ||
                !reachable.count(face)) {
                continue;
            }
            MCGAL::Halfedge* start = face->proxyHalfedge();
            MCGAL::Halfedge* h = start;
            int guard = 0;
            while (h && ++guard <= 8192) {
                if (h->vertex() && !h->vertex()->isRemoved() &&
                    coordKeyForVertex(mesh, h->vertex()) == key) {
                    liveVertices.insert(h->vertex());
                }
                MCGAL::Vertex* removed = h->end_vertex();
                if (removed && !removed->isRemoved() &&
                    coordKeyForVertex(mesh, removed) == key) {
                    liveVertices.insert(removed);
                    if (removed->isBoundary() || touchesTopologyBoundary(removed) ||
                        (protectedVertices && protectedVertices->count(removed) > 0)) {
                        MCGAL::Point p;
                        HalfedgeErrorEntry entry(h, p, 0, 0);
                        if (removable.isRemovable(entry) && seedGuard.isRemovable(entry)) {
                            AtomicBoundaryCoordCandidate candidate;
                            candidate.vertex = removed;
                            candidate.edge = h;
                            candidate.wasTopologyBoundary =
                                removed->isBoundary() || touchesTopologyBoundary(removed);
                            candidateByVertex.emplace(removed, candidate);
                        }
                    }
                }
                if (!h->next()) break;
                h = h->next();
                if (h == start) break;
            }
        }
    }

    if (liveVertices.empty()) return false;
    if (maxCopies > 0 && static_cast<int>(liveVertices.size()) > maxCopies) return false;
    for (MCGAL::Vertex* v : liveVertices) {
        auto it = candidateByVertex.find(v);
        if (it == candidateByVertex.end()) return false;
        candidates.push_back(it->second);
    }
    return !candidates.empty();
}

int runAtomicBoundaryCoordSweep(
    const std::shared_ptr<MCGAL::Mesh>& mesh,
    std::vector<MCGAL::Halfedge*>& seeds,
    const std::vector<std::pair<MCGAL::Vertex*, MCGAL::Vertex*>>& seedEndpoints,
    OpenBoundaryPPMCIsRemovableOperator& removable,
    SeedIsRemovableOperator& seedGuard,
    OpenBoundaryPPMCEliminateOperator& eliminate,
    const std::unordered_set<MCGAL::Vertex*>* protectedVertices,
    BoundaryCompressionStats* stats,
    int maxCoords,
    int maxCopiesPerCoord,
    const BoundaryZeroOverheadGate& zeroGate = BoundaryZeroOverheadGate{}) {
    if (!mesh || !protectedVertices) return 0;
    std::vector<CoordKey> keys;
    keys.reserve(protectedVertices->size());
    std::unordered_set<CoordKey, CoordKeyHash> seen;
    for (MCGAL::Vertex* v : *protectedVertices) {
        if (!v || v->isRemoved()) continue;
        CoordKey key = coordKeyForVertex(mesh, v);
        if (seen.insert(key).second) keys.push_back(key);
    }
    std::sort(keys.begin(), keys.end(), [](const CoordKey& a, const CoordKey& b) {
        if (a.x != b.x) return a.x < b.x;
        if (a.y != b.y) return a.y < b.y;
        return a.z < b.z;
    });
    if (maxCoords > 0 && static_cast<int>(keys.size()) > maxCoords) {
        keys.resize(static_cast<std::size_t>(maxCoords));
    }

    int eliminated = 0;
    int completedCoords = 0;
    std::vector<AtomicBoundaryCoordCandidate> candidates;
    for (const CoordKey& key : keys) {
        if (!collectAtomicBoundaryCoordCandidates(
                mesh, key, seeds, seedEndpoints, removable, seedGuard,
                protectedVertices, maxCopiesPerCoord, candidates)) {
            continue;
        }
        if (stats) ++stats->atomicCandidateCoords;
        int estimatedSavedBytes = 0;
        int estimatedCostBytes = 0;
        const int copyCount = static_cast<int>(candidates.size());
        if (!boundaryCoordPassesZeroOverheadGate(zeroGate, copyCount,
                                                 estimatedSavedBytes,
                                                 estimatedCostBytes)) {
            if (stats) {
                ++stats->atomicRejectedCoords;
                if (copyCount < zeroGate.minCopies ||
                    (zeroGate.maxCopies > 0 && copyCount > zeroGate.maxCopies)) {
                    ++stats->atomicRejectedByCopies;
                } else {
                    ++stats->atomicRejectedByGain;
                }
                stats->atomicEstimatedSavedBytes += estimatedSavedBytes;
                stats->atomicEstimatedCostBytes += estimatedCostBytes;
            }
            continue;
        }

        bool stillRemovable = true;
        for (const auto& candidate : candidates) {
            if (!candidate.vertex || candidate.vertex->isRemoved() ||
                !candidate.edge || candidate.edge->isRemoved() ||
                !(coordKeyForVertex(mesh, candidate.vertex) == key)) {
                stillRemovable = false;
                break;
            }
            MCGAL::Point p;
            HalfedgeErrorEntry entry(candidate.edge, p, 0, 0);
            if (!removable.isRemovable(entry) || !seedGuard.isRemovable(entry)) {
                stillRemovable = false;
                break;
            }
        }
        if (!stillRemovable) continue;
        if (stats) {
            ++stats->atomicAcceptedCoords;
            stats->atomicEstimatedSavedBytes += estimatedSavedBytes;
            stats->atomicEstimatedCostBytes += estimatedCostBytes;
        }

        int removedForCoord = 0;
        for (const auto& candidate : candidates) {
            if (!candidate.vertex || candidate.vertex->isRemoved()) continue;
            MCGAL::Point p;
            HalfedgeErrorEntry entry(candidate.edge, p, 0, 0);
            if (!removable.isRemovable(entry) || !seedGuard.isRemovable(entry) ||
                !eliminate.eliminate(entry)) {
                break;
            }
            recordOpenBoundaryElimination(candidate.vertex, protectedVertices, stats,
                                          candidate.wasTopologyBoundary);
            ++removedForCoord;
        }
        if (removedForCoord == static_cast<int>(candidates.size())) {
            ++completedCoords;
        }
        eliminated += removedForCoord;
    }

    if (completedCoords > 0) {
        std::cerr << "[PMSF_BOS1_ATOMIC_BOUNDARY_COORDS]"
                  << " completed_coords=" << completedCoords
                  << " eliminated=" << eliminated << "\n";
    }
    return eliminated;
}

int runExhaustiveInteriorRound(const std::shared_ptr<MCGAL::Mesh>& mesh,
                               int group,
	                               PPMCIsRemovableOperator& removable,
	                               SeedIsRemovableOperator& seedGuard,
	                               PPMCEliminateOperator& eliminate,
	                               int maxPerGroup,
	                               const std::unordered_set<MCGAL::Facet*>* reachable = nullptr,
	                               const std::unordered_set<MCGAL::Vertex*>* protectedVertices = nullptr,
                               bool protectOneRing = false) {
    int eliminated = 0;
    bool progressed = true;
    while (progressed) {
        progressed = false;
        for (MCGAL::Facet* face : mesh->faces()) {
	            if (!face || face->isRemoved() || face->groupId() != group ||
	                !face->proxyHalfedge() || face->isConquered()) {
	                continue;
	            }
	            if (reachable && !reachable->count(face)) continue;
            MCGAL::Halfedge* start = face->proxyHalfedge();
            MCGAL::Halfedge* h = start;
            int guard = 0;
            while (h && ++guard <= 8192) {
                MCGAL::Point p;
                HalfedgeErrorEntry entry(h, p, 0, 0);
	                if (inReachableFaces(h, reachable) &&
	                    !violatesProtectedPartitionBoundaryPolicy(
                            h, protectedVertices, protectOneRing) &&
	                    removable.isRemovable(entry) && seedGuard.isRemovable(entry) &&
	                    eliminate.eliminate(entry)) {
                    ++eliminated;
                    progressed = true;
                    break;
                }
                if (!h->next()) break;
                h = h->next();
                if (h == start) break;
            }
            if (progressed) break;
        }
        if (maxPerGroup > 0 && eliminated >= maxPerGroup) break;
    }
    return eliminated;
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "用法: " << argv[0]
                  << " <input.off> <output-base-or.parquet> "
	                     "[--target-faces=150 | --segments=N] [--rounds=5] "
	                     "[--partitioner=face|vertex] [--max-per-group=1] "
	                     "[--compact-partitions=0|1] [--boundary-rounds=all|last] "
	                     "[--protected-rounds=N --free-boundary-rounds=M] "
	                     "[--protected-selector=greedy|exhaustive|hybrid] "
	                     "[--free-selector=greedy|exhaustive|hybrid] "
	                     "[--quant-bits=N] [--prediction=0|1]\n";
        return 1;
    }

    std::string inputPath = argv[1];
    std::string outputPath = argv[2];
    int segments = 0;
    int targetFaces = 150;
    int rounds = 5;
    int maxPerGroup = 1;
    int protectedRounds = -1;
    int freeBoundaryRounds = -1;
    int quantBits = 12;
    bool compactPartitions = true;
    bool boundaryAllRounds = true;
    bool enablePrediction = false;
    std::string partitioner = "vertex";
    std::string protectedSelector = "hybrid";
    std::string freeSelector = "exhaustive";

    for (int i = 3; i < argc; ++i) {
        std::string arg = argv[i];
        segments = parseIntArg(arg, "--segments=", segments);
        targetFaces = parseIntArg(arg, "--target-faces=", targetFaces);
        rounds = parseIntArg(arg, "--rounds=", rounds);
        maxPerGroup = parseIntArg(arg, "--max-per-group=", maxPerGroup);
        protectedRounds = parseIntArg(arg, "--protected-rounds=", protectedRounds);
        freeBoundaryRounds = parseIntArg(arg, "--free-boundary-rounds=", freeBoundaryRounds);
        quantBits = parseIntArg(arg, "--quant-bits=", quantBits);
        if (arg.rfind("--partitioner=", 0) == 0) {
            partitioner = arg.substr(std::string("--partitioner=").size());
        } else if (arg.rfind("--protected-selector=", 0) == 0) {
            protectedSelector = arg.substr(std::string("--protected-selector=").size());
        } else if (arg.rfind("--free-selector=", 0) == 0) {
            freeSelector = arg.substr(std::string("--free-selector=").size());
        } else if (arg == "--compact-partitions") {
            compactPartitions = true;
        } else if (arg.rfind("--compact-partitions=", 0) == 0) {
            compactPartitions = parseBoolValue(arg.substr(std::string("--compact-partitions=").size()));
        } else if (arg.rfind("--prediction=", 0) == 0) {
            enablePrediction = parseBoolValue(arg.substr(std::string("--prediction=").size()));
        } else if (arg == "--boundary-rounds=last") {
            boundaryAllRounds = false;
        } else if (arg == "--boundary-rounds=all") {
            boundaryAllRounds = true;
        }
    }
    const bool explicitBoundarySchedule = protectedRounds >= 0 || freeBoundaryRounds >= 0;
    if (explicitBoundarySchedule) {
        if (protectedRounds < 0) protectedRounds = std::max(0, rounds - std::max(0, freeBoundaryRounds));
        if (freeBoundaryRounds < 0) freeBoundaryRounds = std::max(0, rounds - protectedRounds);
        protectedRounds = std::max(0, protectedRounds);
        freeBoundaryRounds = std::max(0, freeBoundaryRounds);
        rounds = std::max(0, protectedRounds + freeBoundaryRounds);
    }
    if (partitioner != "vertex" && partitioner != "face") {
        std::cerr << "unknown partitioner: " << partitioner << "\n";
        return 1;
    }
    if (protectedSelector != "greedy" && protectedSelector != "exhaustive" &&
        protectedSelector != "hybrid") {
        std::cerr << "unknown protected selector: " << protectedSelector << "\n";
        return 1;
    }
    if (freeSelector != "greedy" && freeSelector != "exhaustive" &&
        freeSelector != "hybrid") {
        std::cerr << "unknown free selector: " << freeSelector << "\n";
        return 1;
    }
    quantBits = std::max(4, std::min(16, quantBits));

    TimeRecorder recorder("segmented_ppmc_columnar_owned_boundary_compress");

    // HEAD-port: the removed `PMSF pmsf;` ctor used to size + register the MCGAL
    // context pool. On current HEAD that bootstrap lives in MeshRuntime; without
    // it loadOFF() dereferences a null pool and segfaults.
    pmsf::ensureMeshRuntimeInitialized();
    auto mesh = std::make_shared<MCGAL::Mesh>(MCGAL::Mesh(inputPath));
    if (segments <= 0) {
        targetFaces = std::max(1, targetFaces);
        segments = std::max(
            1, static_cast<int>(std::ceil(static_cast<double>(mesh->size_of_facets()) /
                                          static_cast<double>(targetFaces))));
    }
    segments = std::max(1, std::min(segments, static_cast<int>(mesh->size_of_facets())));

    mesh->i_nbQuantBits = static_cast<unsigned>(quantBits);
    mesh->quantizeVertexPositions();

    std::vector<MCGAL::Halfedge*> seeds;
    if (partitioner == "vertex") {
        PreDefinedVertexSegmentOperator segmentOperator(segments);
        segmentOperator.segment(mesh);
        seeds = segmentOperator.exportSeeds();
        assignFaceGroupsFromVertices(mesh, segments);
    } else {
        PreDefinedFacetSegmentOperator segmentOperator(segments);
        segmentOperator.segment(mesh);
        seeds = segmentOperator.exportSeeds();
    }
    if (seeds.empty()) {
        std::cerr << "segmentation produced no seeds\n";
        return 1;
    }
    std::vector<std::pair<MCGAL::Vertex*, MCGAL::Vertex*>> seedEndpoints;
    seedEndpoints.reserve(seeds.size());
    for (MCGAL::Halfedge* seed : seeds) {
        seedEndpoints.emplace_back(seed ? seed->vertex() : nullptr,
                                   seed ? seed->end_vertex() : nullptr);
    }
    const auto partitionBoundaryVertices = collectPartitionBoundaryVertices(mesh);
    const bool protectBoundaryOneRing =
        envFlag("PMSF_PROTECT_BOUNDARY_ONE_RING");

    std::cerr << "[SegmentedPPMCColumnarOwnedBoundaryCompress] partitioner=" << partitioner
              << " groups=" << seeds.size()
              << " target_faces=" << targetFaces
              << " input_faces=" << mesh->size_of_facets()
              << " rounds=" << rounds
              << " max_per_group=" << maxPerGroup
              << " global_quantization=1"
              << " quant_bits=" << quantBits
              << " shared_coords=1"
              << " face_coord_ids=1"
              << " compact_partitions=" << (compactPartitions ? 1 : 0)
              << " prediction=" << (enablePrediction ? 1 : 0)
              << " boundary_compression=owned_open_boundary"
              << " boundary_rounds=" << (boundaryAllRounds ? "all" : "last")
              << " protected_rounds=" << (explicitBoundarySchedule ? protectedRounds : -1)
              << " free_boundary_rounds=" << (explicitBoundarySchedule ? freeBoundaryRounds : -1)
              << " protected_selector=" << protectedSelector
              << " free_selector=" << freeSelector
              << " protected_partition_boundary_vertices="
              << partitionBoundaryVertices.size()
              << " protect_boundary_one_ring=" << (protectBoundaryOneRing ? 1 : 0)
              << "\n";

    PMSFHeader header = PMSFHeader::Builder()
                            .setRound(rounds)
                            .setEnableCompress(false)
                            .setEnableQuantization(true)
                            .setEnableSegmentation(true)
                            .setEnablePrediction(enablePrediction)
                            .setEnableConnectivityPredictionFaces(enablePrediction)
                            .setEnableConnectivityPredictionEdges(enablePrediction)
                            .setEnableTriangulation(false)
                            .build();
    header.setGroupNumber(static_cast<int>(seeds.size()));

    auto removable = std::make_shared<OpenBoundaryPPMCIsRemovableOperator>();
    auto interiorRemovable = std::make_shared<PPMCIsRemovableOperator>();
    auto partitionBoundaryGuard =
        std::make_shared<PartitionBoundaryIsRemovableOperator>(
            &partitionBoundaryVertices, protectBoundaryOneRing);
    auto seedGuard = std::make_shared<SeedIsRemovableOperator>();
    seedGuard->init({}, seeds, {});
    auto selectOptions = SelectOptions::Builder()
                             .withErrorSource(ErrorSource::Halfedge)
                             .withIsRemovableOperator(removable)
                             .build();
    auto interiorSelectOptions = SelectOptions::Builder()
                                     .withErrorSource(ErrorSource::Halfedge)
                                     .withIsRemovableOperator(interiorRemovable)
                                     .build();

    OpenBoundaryPPMCEliminateOperator eliminate;
    eliminate.init(mesh, header);
    PPMCEliminateOperator interiorEliminate;
    interiorEliminate.init(mesh, header);

    auto collector = std::make_shared<PPMCSegmentationSymbolCollectOperator>();
    collector->setEnablePrediction(enablePrediction);
    collector->setConnectivityPredictionFaces(enablePrediction);
    collector->setConnectivityPredictionEdges(enablePrediction);
    collector->setUseTriangleMeshConnectivityPrediction(false);
    collector->init(mesh, seeds, header);
    bool boundaryAfterProtectedInterior =
        envFlag("PMSF_OPEN_BOUNDARY_AFTER_PROTECTED_INTERIOR");
    bool syncBoundaryCoords = envFlag("PMSF_OPEN_BOUNDARY_SYNC_COORDS");
    bool atomicBoundaryCoords = envFlag("PMSF_OPEN_BOUNDARY_ATOMIC_COORDS");
    const BoundaryZeroOverheadGate zeroOverheadGate = readBoundaryZeroOverheadGate();
    if (zeroOverheadGate.enabled) {
        atomicBoundaryCoords = true;
        syncBoundaryCoords = false;
        boundaryAfterProtectedInterior = false;
        unsetenv("PMSF_EMBED_BOUNDARY_OWNER_IN_PARTITIONED_BASE");
        unsetenv("PMSF_BOS1_OWNER_MANIFEST");
        std::cerr << "[PMSF_ZERO_OVERHEAD_BOUNDARY]"
                  << " enabled=1"
                  << " min_copies=" << zeroOverheadGate.minCopies
                  << " max_copies=" << zeroOverheadGate.maxCopies
                  << " coord_save_bytes=" << zeroOverheadGate.coordSaveBytes
                  << " copy_save_bytes=" << zeroOverheadGate.copySaveBytes
                  << " event_cost_bytes=" << zeroOverheadGate.eventCostBytes
                  << " min_gain_ratio=" << zeroOverheadGate.minGainRatio
                  << " storage=implicit"
                  << "\n";
    }
    const int syncBoundaryMaxCoords =
        std::max(0, envInt("PMSF_OPEN_BOUNDARY_SYNC_MAX_COORDS", 128));
    const int syncBoundaryMaxCopies =
        std::max(0, envInt("PMSF_OPEN_BOUNDARY_SYNC_MAX_COPIES", 64));
    if (syncBoundaryCoords) {
        std::cerr << "[PMSF_BOS1_SYNC_BOUNDARY_COORDS]"
                  << " enabled=1"
                  << " max_coords=" << syncBoundaryMaxCoords
                  << " max_copies=" << syncBoundaryMaxCopies << "\n";
    }
    if (atomicBoundaryCoords) {
        std::cerr << "[PMSF_BOS1_ATOMIC_BOUNDARY_COORDS]"
                  << " enabled=1"
                  << " max_coords=" << syncBoundaryMaxCoords
                  << " max_copies=" << syncBoundaryMaxCopies << "\n";
    }
    BoundaryCompressionStats boundaryStats;
    std::unordered_set<CoordKey, CoordKeyHash> initialPartitionBoundaryCoords;
    for (MCGAL::Vertex* v : partitionBoundaryVertices) {
        if (v && !v->isRemoved()) {
            initialPartitionBoundaryCoords.insert(coordKeyForVertex(mesh, v));
        }
    }

    for (int round = 0; round < rounds; ++round) {
	        const bool boundaryEnabledThisRound = explicitBoundarySchedule
	                                                  ? round >= protectedRounds
	                                                  : (boundaryAllRounds || round == rounds - 1);
	        for (std::size_t group = 0; group < seeds.size(); ++group) {
	            if (group < seedEndpoints.size()) {
	                seeds[group] = refreshGroupSeed(mesh,
	                                                seedEndpoints[group].first,
	                                                seedEndpoints[group].second,
	                                                static_cast<int>(group),
	                                                seeds[group]);
	            }
	            auto reachable = collectReachableGroupFaces(seeds[group], static_cast<int>(group));
	            const auto* reachablePtr = &reachable;
            if (boundaryEnabledThisRound && zeroOverheadGate.enabled) {
                std::cerr << "[SegmentedPPMCColumnarOwnedBoundaryCompress] round=" << round
                          << " group=" << group
                          << " boundary_enabled=1"
                          << " zero_overhead_atomic_only=1"
                          << " eliminated=0\n";
                continue;
            }
            if (boundaryEnabledThisRound) {
                int eliminated = 0;
                if (freeSelector == "greedy" || freeSelector == "hybrid") {
                    GreedySelector selector(*selectOptions);
                    selector.init(mesh, true, static_cast<int>(group));
                    selector.addIsRemovableOperator(seedGuard);
                    MCGAL::Halfedge* seed = nullptr;
                    selector.selectSeed(seed, [&, group](std::shared_ptr<MCGAL::Mesh>) {
                        return seeds[group];
                    });

	                    MCGAL::Point p;
	                    HalfedgeErrorEntry entry(nullptr, p, 0, 0);
	                    while (selector.select(entry)) {
	                        if (!inReachableFaces(entry.edge, reachablePtr)) continue;
	                        MCGAL::Vertex* removed = entry.edge ? entry.edge->end_vertex() : nullptr;
	                        const bool wasTopologyBoundary =
	                            removed &&
	                            (removed->isBoundary() || touchesTopologyBoundary(removed));
	                        bool ok = eliminate.eliminate(entry);
                        if (ok) {
                            recordOpenBoundaryElimination(
                                removed, &partitionBoundaryVertices, &boundaryStats,
                                wasTopologyBoundary);
                            selector.update(entry);
                            ++eliminated;
                            if (maxPerGroup > 0 && eliminated >= maxPerGroup) break;
                        }
                    }
                }
                if ((maxPerGroup <= 0 || eliminated < maxPerGroup) &&
                    (freeSelector == "exhaustive" || freeSelector == "hybrid")) {
	                    const int remaining = maxPerGroup > 0 ? maxPerGroup - eliminated : 0;
	                    eliminated += runExhaustiveOpenBoundaryRound(
	                        mesh, static_cast<int>(group), *removable, *seedGuard,
	                        eliminate, remaining, false, reachablePtr,
	                        &partitionBoundaryVertices, &boundaryStats);
                }
                std::cerr << "[SegmentedPPMCColumnarOwnedBoundaryCompress] round=" << round
                          << " group=" << group
                          << " boundary_enabled=1"
                          << " eliminated=" << eliminated << "\n";
                continue;
            }
            int eliminated = 0;
            if (protectedSelector == "greedy" || protectedSelector == "hybrid") {
                GreedySelector selector(boundaryEnabledThisRound ? *selectOptions : *interiorSelectOptions);
                selector.init(mesh, true, static_cast<int>(group));
                selector.addIsRemovableOperator(partitionBoundaryGuard);
                selector.addIsRemovableOperator(seedGuard);
                MCGAL::Halfedge* seed = nullptr;
                selector.selectSeed(seed, [&, group](std::shared_ptr<MCGAL::Mesh>) {
                    return seeds[group];
                });

	                MCGAL::Point p;
	                HalfedgeErrorEntry entry(nullptr, p, 0, 0);
	                while (selector.select(entry)) {
	                    if (!inReachableFaces(entry.edge, reachablePtr)) continue;
	                    if (violatesProtectedPartitionBoundaryPolicy(
                                entry.edge, &partitionBoundaryVertices,
                                protectBoundaryOneRing)) {
	                        continue;
	                    }
	                    bool ok = interiorEliminate.eliminate(entry);
                    if (ok) {
                        selector.update(entry);
                        ++eliminated;
                        if (maxPerGroup > 0 && eliminated >= maxPerGroup) break;
                    }
                }
            }
            if ((maxPerGroup <= 0 || eliminated < maxPerGroup) &&
                (protectedSelector == "exhaustive" || protectedSelector == "hybrid")) {
	                const int remaining = maxPerGroup > 0 ? maxPerGroup - eliminated : 0;
	                eliminated += runExhaustiveInteriorRound(
	                    mesh, static_cast<int>(group), *interiorRemovable, *seedGuard,
	                    interiorEliminate, remaining, reachablePtr,
	                    &partitionBoundaryVertices, protectBoundaryOneRing);
            }
            std::cerr << "[SegmentedPPMCColumnarOwnedBoundaryCompress] round=" << round
                      << " group=" << group
                      << " boundary_enabled=0"
                      << " eliminated=" << eliminated << "\n";
            if (boundaryAfterProtectedInterior) {
		                const int boundaryEliminated = runExhaustiveOpenBoundaryRound(
		                    mesh, static_cast<int>(group), *removable, *seedGuard,
	                    eliminate, maxPerGroup, true, reachablePtr,
	                    &partitionBoundaryVertices, &boundaryStats);
                std::cerr << "[SegmentedPPMCColumnarOwnedBoundaryCompress] round=" << round
                          << " group=" << group
                          << " boundary_after_interior=1"
	                          << " eliminated=" << boundaryEliminated << "\n";
            }
        }
        if (boundaryEnabledThisRound && atomicBoundaryCoords) {
            const int atomicEliminated = runAtomicBoundaryCoordSweep(
                mesh, seeds, seedEndpoints, *removable, *seedGuard, eliminate,
                &partitionBoundaryVertices, &boundaryStats, syncBoundaryMaxCoords,
                syncBoundaryMaxCopies, zeroOverheadGate);
            std::cerr << "[PMSF_BOS1_ATOMIC_BOUNDARY_COORDS]"
                      << " round=" << round
                      << " eliminated=" << atomicEliminated << "\n";
        } else if (boundaryEnabledThisRound && syncBoundaryCoords) {
            const int syncEliminated = runSynchronizedBoundaryCoordSweep(
                mesh, seeds, seedEndpoints, *removable, *seedGuard, eliminate,
                &partitionBoundaryVertices, &boundaryStats, syncBoundaryMaxCoords,
                syncBoundaryMaxCopies);
            std::cerr << "[PMSF_BOS1_SYNC_BOUNDARY_COORDS]"
                      << " round=" << round
                      << " eliminated=" << syncEliminated << "\n";
        }
		        for (std::size_t group = 0; group < seeds.size() && group < seedEndpoints.size(); ++group) {
		            seeds[group] = refreshGroupSeed(mesh,
		                                            seedEndpoints[group].first,
	                                            seedEndpoints[group].second,
	                                            static_cast<int>(group),
	                                            seeds[group]);
	        }
	        collector->collect(seeds);
        mesh->resetState();
    }

    int finalLiveVertices = 0;
    int finalLiveFaces = 0;
    int finalRetainedBoundaryVertices = 0;
    std::unordered_set<CoordKey, CoordKeyHash> finalRetainedBoundaryCoords;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        ++finalLiveVertices;
        CoordKey key = coordKeyForVertex(mesh, v);
        if (initialPartitionBoundaryCoords.count(key) > 0) {
            ++finalRetainedBoundaryVertices;
            finalRetainedBoundaryCoords.insert(key);
        }
    }
    for (MCGAL::Facet* f : mesh->faces()) {
        if (f && !f->isRemoved()) ++finalLiveFaces;
    }
    const int freeBoundaryEliminated =
        boundaryStats.freeProtectedBoundaryEliminated +
        boundaryStats.freeTopologyBoundaryEliminated;
    const int finalRemovedBoundaryCoords =
        static_cast<int>(initialPartitionBoundaryCoords.size()) -
        static_cast<int>(finalRetainedBoundaryCoords.size());
    std::cerr << "[PMSF_BOS1_BOUNDARY_STATS]"
              << " initial_boundary_vertices=" << partitionBoundaryVertices.size()
              << " initial_boundary_coords=" << initialPartitionBoundaryCoords.size()
              << " free_boundary_eliminated=" << freeBoundaryEliminated
              << " free_protected_boundary_eliminated="
              << boundaryStats.freeProtectedBoundaryEliminated
              << " free_topology_boundary_eliminated="
              << boundaryStats.freeTopologyBoundaryEliminated
              << " free_interior_eliminated=" << boundaryStats.freeInteriorEliminated
              << " final_retained_boundary_vertices=" << finalRetainedBoundaryVertices
              << " final_retained_boundary_coords=" << finalRetainedBoundaryCoords.size()
              << " final_removed_boundary_coords=" << std::max(0, finalRemovedBoundaryCoords)
              << " final_live_vertices=" << finalLiveVertices
              << " final_live_faces=" << finalLiveFaces
              << "\n";
    std::cerr << "[PMSF_ZERO_OVERHEAD_BOUNDARY_STATS]"
              << " enabled=" << (zeroOverheadGate.enabled ? 1 : 0)
              << " atomic_candidate_coords=" << boundaryStats.atomicCandidateCoords
              << " atomic_accepted_coords=" << boundaryStats.atomicAcceptedCoords
              << " atomic_rejected_coords=" << boundaryStats.atomicRejectedCoords
              << " atomic_rejected_by_copies=" << boundaryStats.atomicRejectedByCopies
              << " atomic_rejected_by_gain=" << boundaryStats.atomicRejectedByGain
              << " estimated_saved_bytes=" << boundaryStats.atomicEstimatedSavedBytes
              << " estimated_cost_bytes=" << boundaryStats.atomicEstimatedCostBytes
              << "\n";

    ColumnarSerializeOperator serializer(collector, enablePrediction, true, false);
    serializer.init();
    serializer.setHeader(std::make_shared<PMSFHeader>(header));
    serializer.setSharedBaseMeshFaceCoordIds(true);
    serializer.setPartitionedCompactBaseMesh(compactPartitions);
    serializer.serializeBaseMeshWithSeed(mesh, seeds);
    serializer.serialize(outputPath);
    return 0;
}
