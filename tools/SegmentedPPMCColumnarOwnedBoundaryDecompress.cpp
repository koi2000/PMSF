// HEAD-port (2026-10): PMSF.h facade removed; PPMC decompress operators moved
//   decompress/PPMCSymbolReadOperator.h   -> PPMC/decompress/...
//   reconstruct/PPMCReconstructOperator.h -> PPMC/reconstruct/...
#include "common/TimeRecorder.h"
#include "core/MeshRuntime.h"
#include "PPMC/decompress/PPMCSymbolReadOperator.h"
#include "decompress/ColumnarDeserializeOperator.h"
#include "header/PMSFHeader.h"
#include "options/DecompressOptions.h"
#include "PPMC/reconstruct/PPMCReconstructOperator.h"
#include "reconstruct/ReconstructOpertator.h"
#include "Global.h"
#include "Mesh.h"
#include "MeshUtils.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <fstream>
#include <initializer_list>
#include <iomanip>
#include <iostream>
#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {


bool openBoundaryJoinBeforeInsert() {
    static const bool enabled = [] {
        const char* env = std::getenv("PMSF_OPEN_BOUNDARY_JOIN_BEFORE_INSERT");
        return env && env[0] != '\0' && std::string(env) != "0";
    }();
    return enabled;
}

bool joinCrossGroupAddedEdges() {
    static const bool enabled = [] {
        const char* env = std::getenv("PMSF_JOIN_CROSS_GROUP_ADDED_EDGES");
        return env && env[0] != '\0' && std::string(env) != "0";
    }();
    return enabled;
}

bool ownerLocalSeamReplay() {
    static const bool enabled = [] {
        const char* env = std::getenv("PMSF_OWNER_LOCAL_SEAM_REPLAY");
        return env && env[0] != '\0' && std::string(env) != "0";
    }();
    return enabled;
}

bool ownerLocalSeamReplaySplitOpposite() {
    static const bool enabled = [] {
        const char* env = std::getenv("PMSF_OWNER_LOCAL_SEAM_REPLAY_SPLIT_OPP");
        return env && env[0] != '\0' && std::string(env) != "0";
    }();
    return enabled;
}

struct EdgeKey {
    std::uintptr_t a = 0;
    std::uintptr_t b = 0;
    bool operator==(const EdgeKey& other) const {
        return a == other.a && b == other.b;
    }
};

struct EdgeKeyHash {
    std::size_t operator()(const EdgeKey& k) const {
        return std::hash<std::uintptr_t>{}(k.a) ^ (std::hash<std::uintptr_t>{}(k.b) << 1);
    }
};

struct DumpStats {
    std::size_t vertices = 0;
    std::size_t faces = 0;
    std::size_t skippedMalformed = 0;
    std::size_t skippedRepeatedVertices = 0;
    std::size_t skippedDeadVertices = 0;
};

struct CoordBitsKey {
    std::uint32_t x = 0;
    std::uint32_t y = 0;
    std::uint32_t z = 0;
    bool operator==(const CoordBitsKey& other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct CoordBitsKeyHash {
    std::size_t operator()(const CoordBitsKey& k) const {
        std::size_t h = std::hash<std::uint32_t>{}(k.x);
        h ^= std::hash<std::uint32_t>{}(k.y) + 0x9e3779b9u + (h << 6) + (h >> 2);
        h ^= std::hash<std::uint32_t>{}(k.z) + 0x9e3779b9u + (h << 6) + (h >> 2);
        return h;
    }
};

CoordBitsKey coordBitsKey(MCGAL::Vertex* v) {
    CoordBitsKey key;
    if (!v) return key;
    static_assert(sizeof(float) == sizeof(std::uint32_t),
                  "float coordinates must be 32-bit");
    const float x = v->x();
    const float y = v->y();
    const float z = v->z();
    std::memcpy(&key.x, &x, sizeof(float));
    std::memcpy(&key.y, &y, sizeof(float));
    std::memcpy(&key.z, &z, sizeof(float));
    return key;
}

struct TriKey {
    int a = 0;
    int b = 0;
    int c = 0;
    bool operator==(const TriKey& other) const {
        return a == other.a && b == other.b && c == other.c;
    }
};

struct TriKeyHash {
    std::size_t operator()(const TriKey& k) const {
        std::size_t h = std::hash<int>{}(k.a);
        h ^= std::hash<int>{}(k.b) + 0x9e3779b9u + (h << 6) + (h >> 2);
        h ^= std::hash<int>{}(k.c) + 0x9e3779b9u + (h << 6) + (h >> 2);
        return h;
    }
};

TriKey makeTriKey(int a, int b, int c) {
    std::array<int, 3> ids{a, b, c};
    std::sort(ids.begin(), ids.end());
    return TriKey{ids[0], ids[1], ids[2]};
}

EdgeKey makeEdgeKey(MCGAL::Vertex* a, MCGAL::Vertex* b) {
    std::uintptr_t pa = reinterpret_cast<std::uintptr_t>(a);
    std::uintptr_t pb = reinterpret_cast<std::uintptr_t>(b);
    if (pb < pa) std::swap(pa, pb);
    return EdgeKey{pa, pb};
}

std::vector<MCGAL::Vertex*> faceVerticesBounded(MCGAL::Facet* face) {
    std::vector<MCGAL::Vertex*> verts;
    if (!face || face->isRemoved() || !face->proxyHalfedge()) return verts;
    MCGAL::Halfedge* start = face->proxyHalfedge();
    MCGAL::Halfedge* h = start;
    int guard = 0;
    while (h && ++guard <= 8192) {
        if (!h->vertex() || h->vertex()->isRemoved()) {
            verts.clear();
            return verts;
        }
        verts.push_back(h->vertex());
        if (!h->next()) {
            if (!h->end_vertex() || h->end_vertex()->isRemoved()) {
                verts.clear();
                return verts;
            }
            verts.push_back(h->end_vertex());
            break;
        }
        h = h->next();
        if (h == start) break;
    }
    if (guard > 8192) verts.clear();
    return verts;
}

DumpStats dumpRepairedOff(const std::shared_ptr<MCGAL::Mesh>& mesh, const std::string& path) {
    std::vector<std::vector<MCGAL::Vertex*>> keptFaces;
    std::unordered_map<EdgeKey, int, EdgeKeyHash> edgeUse;
    std::unordered_set<MCGAL::Vertex*> usedVertices;
    std::unordered_set<std::string> seenFaces;

    for (MCGAL::Facet* face : mesh->faces()) {
        std::vector<MCGAL::Vertex*> verts = faceVerticesBounded(face);
        if (verts.size() < 3) continue;
        std::unordered_set<MCGAL::Vertex*> unique(verts.begin(), verts.end());
        if (unique.size() != verts.size()) continue;
        MCGAL::Point origin = verts[0]->point();
        bool areaOk = true;
        for (std::size_t i = 1; i + 1 < verts.size(); ++i) {
            const double ax = static_cast<double>(verts[i]->x()) - origin.x();
            const double ay = static_cast<double>(verts[i]->y()) - origin.y();
            const double az = static_cast<double>(verts[i]->z()) - origin.z();
            const double bx = static_cast<double>(verts[i + 1]->x()) - origin.x();
            const double by = static_cast<double>(verts[i + 1]->y()) - origin.y();
            const double bz = static_cast<double>(verts[i + 1]->z()) - origin.z();
            const double cx = ay * bz - az * by;
            const double cy = az * bx - ax * bz;
            const double cz = ax * by - ay * bx;
            const double area2 = cx * cx + cy * cy + cz * cz;
            if (area2 < 1e-30) {
                areaOk = false;
                break;
            }
        }
        if (!areaOk) continue;

        std::vector<std::uintptr_t> sortedPtrs;
        sortedPtrs.reserve(verts.size());
        for (MCGAL::Vertex* v : verts) {
            sortedPtrs.push_back(reinterpret_cast<std::uintptr_t>(v));
        }
        std::sort(sortedPtrs.begin(), sortedPtrs.end());
        std::string faceKey;
        for (std::uintptr_t p : sortedPtrs) {
            faceKey.append(reinterpret_cast<const char*>(&p), sizeof(p));
        }
        if (!seenFaces.insert(faceKey).second) continue;

        bool manifoldOk = true;
        std::vector<EdgeKey> edges;
        edges.reserve(verts.size());
        for (std::size_t i = 0; i < verts.size(); ++i) {
            EdgeKey key = makeEdgeKey(verts[i], verts[(i + 1) % verts.size()]);
            if (key.a == key.b || edgeUse[key] >= 2) {
                manifoldOk = false;
                break;
            }
            edges.push_back(key);
        }
        if (!manifoldOk) continue;
        for (const EdgeKey& key : edges) {
            edgeUse[key] += 1;
        }
        for (MCGAL::Vertex* v : verts) {
            usedVertices.insert(v);
        }
        keptFaces.push_back(std::move(verts));
    }

    std::unordered_map<MCGAL::Vertex*, int> vid;
    vid.reserve(usedVertices.size());
    std::vector<MCGAL::Vertex*> orderedVertices;
    orderedVertices.reserve(usedVertices.size());
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved() || !usedVertices.count(v)) continue;
        int id = static_cast<int>(orderedVertices.size());
        vid[v] = id;
        orderedVertices.push_back(v);
    }

    std::ofstream out(path);
    out << std::setprecision(17);
    out << "OFF\n";
    out << orderedVertices.size() << " " << keptFaces.size() << " 0\n";
    for (MCGAL::Vertex* v : orderedVertices) {
        out << v->x() << " " << v->y() << " " << v->z() << "\n";
    }
    for (const auto& face : keptFaces) {
        out << face.size();
        for (MCGAL::Vertex* v : face) {
            out << " " << vid[v];
        }
        out << "\n";
    }
    return DumpStats{orderedVertices.size(), keptFaces.size()};
}

DumpStats dumpCompleteOff(const std::shared_ptr<MCGAL::Mesh>& mesh, const std::string& path) {
    DumpStats stats;
    std::unordered_map<MCGAL::Vertex*, int> vid;
    std::vector<MCGAL::Vertex*> orderedVertices;
    orderedVertices.reserve(mesh->vertices().size());
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        int id = static_cast<int>(orderedVertices.size());
        vid[v] = id;
        orderedVertices.push_back(v);
    }

    std::vector<std::vector<MCGAL::Vertex*>> keptFaces;
    keptFaces.reserve(mesh->faces().size());
    for (MCGAL::Facet* face : mesh->faces()) {
        std::vector<MCGAL::Vertex*> verts = faceVerticesBounded(face);
        if (verts.size() < 3) {
            ++stats.skippedMalformed;
            continue;
        }
        std::unordered_set<MCGAL::Vertex*> unique(verts.begin(), verts.end());
        if (unique.size() != verts.size()) {
            ++stats.skippedRepeatedVertices;
            continue;
        }
        bool allVerticesLive = true;
        for (MCGAL::Vertex* v : verts) {
            if (!vid.count(v)) {
                allVerticesLive = false;
                break;
            }
        }
        if (!allVerticesLive) {
            ++stats.skippedDeadVertices;
            continue;
        }
        keptFaces.push_back(std::move(verts));
    }

    std::size_t outputFaceCount = 0;
    for (const auto& face : keptFaces) {
        outputFaceCount += face.size() <= 3 ? 1 : face.size() - 2;
    }

    std::ofstream out(path);
    out << std::setprecision(17);
    out << "OFF\n";
    out << orderedVertices.size() << " " << outputFaceCount << " 0\n";
    for (MCGAL::Vertex* v : orderedVertices) {
        out << v->x() << " " << v->y() << " " << v->z() << "\n";
    }
    for (const auto& face : keptFaces) {
        if (face.size() == 3) {
            out << "3";
            for (MCGAL::Vertex* v : face) {
                out << " " << vid[v];
            }
            out << "\n";
            continue;
        }
        for (std::size_t i = 1; i + 1 < face.size(); ++i) {
            out << "3 " << vid[face[0]]
                << " " << vid[face[i]]
                << " " << vid[face[i + 1]]
                << "\n";
        }
    }
    stats.vertices = orderedVertices.size();
    stats.faces = outputFaceCount;
    return stats;
}

DumpStats dumpWeldedOff(const std::shared_ptr<MCGAL::Mesh>& mesh, const std::string& path) {
    DumpStats stats;
    std::unordered_map<CoordBitsKey, int, CoordBitsKeyHash> idByCoord;
    std::unordered_map<MCGAL::Vertex*, int> canonicalId;
    std::vector<MCGAL::Vertex*> orderedVertices;
    idByCoord.reserve(mesh->vertices().size());
    canonicalId.reserve(mesh->vertices().size());

    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        const CoordBitsKey key = coordBitsKey(v);
        auto [it, inserted] = idByCoord.emplace(key, static_cast<int>(orderedVertices.size()));
        if (inserted) {
            orderedVertices.push_back(v);
        }
        canonicalId[v] = it->second;
    }

    std::vector<std::array<int, 3>> triangles;
    triangles.reserve(mesh->faces().size());
    std::unordered_set<TriKey, TriKeyHash> seenTriangles;
    seenTriangles.reserve(mesh->faces().size() * 2 + 1);

    auto addTriangle = [&](int a, int b, int c) {
        if (a == b || b == c || a == c) {
            ++stats.skippedRepeatedVertices;
            return;
        }
        const TriKey key = makeTriKey(a, b, c);
        if (!seenTriangles.insert(key).second) return;
        triangles.push_back({a, b, c});
    };

    for (MCGAL::Facet* face : mesh->faces()) {
        std::vector<MCGAL::Vertex*> verts = faceVerticesBounded(face);
        if (verts.size() < 3) {
            ++stats.skippedMalformed;
            continue;
        }
        std::vector<int> ids;
        ids.reserve(verts.size());
        bool allVerticesLive = true;
        for (MCGAL::Vertex* v : verts) {
            auto it = canonicalId.find(v);
            if (it == canonicalId.end()) {
                allVerticesLive = false;
                break;
            }
            ids.push_back(it->second);
        }
        if (!allVerticesLive) {
            ++stats.skippedDeadVertices;
            continue;
        }
        std::unordered_set<int> unique(ids.begin(), ids.end());
        if (unique.size() != ids.size()) {
            ++stats.skippedRepeatedVertices;
            continue;
        }
        for (std::size_t i = 1; i + 1 < ids.size(); ++i) {
            addTriangle(ids[0], ids[i], ids[i + 1]);
        }
    }

    std::vector<char> used(orderedVertices.size(), 0);
    for (const auto& tri : triangles) {
        used[static_cast<std::size_t>(tri[0])] = 1;
        used[static_cast<std::size_t>(tri[1])] = 1;
        used[static_cast<std::size_t>(tri[2])] = 1;
    }
    std::vector<int> remap(orderedVertices.size(), -1);
    std::vector<MCGAL::Vertex*> compactVertices;
    compactVertices.reserve(orderedVertices.size());
    for (std::size_t i = 0; i < orderedVertices.size(); ++i) {
        if (!used[i]) continue;
        remap[i] = static_cast<int>(compactVertices.size());
        compactVertices.push_back(orderedVertices[i]);
    }

    std::ofstream out(path);
    out << std::setprecision(17);
    out << "OFF\n";
    out << compactVertices.size() << " " << triangles.size() << " 0\n";
    for (MCGAL::Vertex* v : compactVertices) {
        out << v->x() << " " << v->y() << " " << v->z() << "\n";
    }
    for (const auto& tri : triangles) {
        out << "3 " << remap[static_cast<std::size_t>(tri[0])]
            << " " << remap[static_cast<std::size_t>(tri[1])]
            << " " << remap[static_cast<std::size_t>(tri[2])] << "\n";
    }
    stats.vertices = compactVertices.size();
    stats.faces = triangles.size();
    return stats;
}

std::vector<std::string> splitCsvLine(const std::string& line) {
    std::vector<std::string> out;
    std::string cur;
    std::stringstream ss(line);
    while (std::getline(ss, cur, ',')) {
        if (!cur.empty() && static_cast<unsigned char>(cur[0]) == 0xEF) cur.erase(0, 3);
        out.push_back(cur);
    }
    return out;
}

int parseIntArg(const std::string& arg, const std::string& prefix, int fallback) {
    if (arg.rfind(prefix, 0) != 0) return fallback;
    return std::atoi(arg.substr(prefix.size()).c_str());
}

std::string parseStringArg(const std::string& arg,
                           const std::string& prefix,
                           const std::string& fallback) {
    if (arg.rfind(prefix, 0) != 0) return fallback;
    return arg.substr(prefix.size());
}

int voxelsFromCsv(const std::string& path, int rowIndex, int lod) {
    std::ifstream in(path);
    if (!in) {
        std::cerr << "cannot open csv: " << path << "\n";
        return -1;
    }
    std::string line;
    if (!std::getline(in, line)) return -1;
    auto header = splitCsvLine(line);
    const std::string target = "lod " + std::to_string(lod) + " #voxels";
    int col = -1;
    for (std::size_t i = 0; i < header.size(); ++i) {
        if (header[i] == target) {
            col = static_cast<int>(i);
            break;
        }
    }
    if (col < 0) {
        std::cerr << "missing csv column: " << target << "\n";
        return -1;
    }
    int current = 0;
    while (std::getline(in, line)) {
        if (current++ != rowIndex) continue;
        auto cols = splitCsvLine(line);
        if (col >= static_cast<int>(cols.size()) || cols[col].empty()) return 0;
        return std::atoi(cols[col].c_str());
    }
    return -1;
}

class OpenBoundaryPPMCReconstructOperator : public ReconstructOperator {
  public:
    void reconstruct(MCGAL::Halfedge*) override {}

    void reconstruct(std::vector<MCGAL::Vertex*>& vertices,
                     std::vector<MCGAL::Halfedge*>& halfedges,
                     std::vector<MCGAL::Facet*>& facets) override {
        ownerLocalSeamReplays_ = 0;
                for (MCGAL::Vertex* v : vertices) {
            if (v) v->setPoint(v->vertexPoint());
        }

        for (MCGAL::Facet* f : facets) {
            if (f && !f->isRemoved() && f->proxyHalfedge()) {
                resolveResidualPoint(f);
            }
        }

        const int ownerGroup = inferOwnerGroup(facets, halfedges);
        std::size_t boundaryPreJoined = 0;
        std::size_t boundaryCleaned = 0;
        std::size_t addedPreJoined = 0;
        std::size_t addedPreFailed = 0;
        if (openBoundaryJoinBeforeInsert()) {
            for (MCGAL::Halfedge* h : halfedges) {
                if (!h || h->isRemoved()) continue;
                if (mergeAddedFace(h, ownerGroup)) ++addedPreJoined;
                else ++addedPreFailed;
            }
        }
        for (MCGAL::Halfedge* h : halfedges) {
            if (!h || h->isRemoved()) continue;
            if (!edgeTouchesOpenBoundaryPatch(h)) continue;
            if (hasSameGroupOpposite(h)) {
                if (canUseNativeJoinFace(h) && mergeSameGroupFace(h)) {
                    ++boundaryPreJoined;
                }
            } else {
                if (cleanupBoundaryAddedEdge(h)) ++boundaryCleaned;
            }
        }
        
        std::size_t insertedOpen = 0;
        std::size_t insertedClosed = 0;
        for (MCGAL::Facet* f : facets) {
            if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
            const bool open = faceHasOpenBoundary(f);
            if (insertPoint(f, open)) {
                if (open) ++insertedOpen;
                else ++insertedClosed;
            }
        }
        
	        std::size_t joined = 0;
	        std::size_t joinFailed = 0;
	        if (!openBoundaryJoinBeforeInsert()) {
	            for (MCGAL::Halfedge* h : halfedges) {
	                if (!h || h->isRemoved()) continue;
	                if (mergeAddedFace(h, ownerGroup)) {
	                    ++joined;
	                } else {
	                    ++joinFailed;
	                    	                }
	            }
	        }
        
        // Boundary halfedges without an in-group opposite represent artificial
        // partition borders.  They are cleaned before open fan insertion when
        // possible; after insertion they must stay as explicit local boundary
        // edges or valid triangles can be turned into dangling paths.
            }

  private:
    static bool faceHasOpenBoundary(MCGAL::Facet* f) {
        if (!f || !f->proxyHalfedge()) return false;
        MCGAL::Halfedge* start = f->proxyHalfedge();
        MCGAL::Halfedge* e = start;
        int guard = 0;
        do {
            if (!e) return true;
            if (!e->opposite() || !e->next()) return true;
            e = e->next();
        } while (e && e != start && ++guard <= 4096);
        return false;
    }

    static bool hasSameGroupOpposite(MCGAL::Halfedge* h) {
        return h && h->opposite() && h->face() && h->opposite()->face() &&
               h->face()->groupId() == h->opposite()->face()->groupId();
    }

    static bool edgeTouchesOpenBoundaryPatch(MCGAL::Halfedge* h) {
        if (!h) return false;
        if (h->face() && faceHasOpenBoundary(h->face())) return true;
        if (h->opposite() && h->opposite()->face() &&
            faceHasOpenBoundary(h->opposite()->face())) {
            return true;
        }
        return !h->opposite() || !h->next();
    }

    static bool cleanupBoundaryAddedEdge(MCGAL::Halfedge* h) {
        if (!h || hasSameGroupOpposite(h)) return false;
        MCGAL::Facet* f = h->face();
        if (!f) return false;
        if (h->opposite()) {
            return false;
        }
        if (!f->isSplittable()) {
            f->setRemoved();
        }
        MCGAL::Halfedge* hstart = h->next();
        MCGAL::Halfedge* hend = safePrevInPath(h);
        if (hstart && hstart->face()) {
            hstart->face()->setProxyHalfedge(hstart);
        }
        if (hend) {
            hend->setNext(nullptr);
        }
        h->setRemoved();
        return true;
    }

    static MCGAL::Halfedge* safePrevInPath(MCGAL::Halfedge* h) {
        if (!h || !h->face() || !h->face()->proxyHalfedge()) return nullptr;
        MCGAL::Halfedge* start = h->face()->proxyHalfedge();
        MCGAL::Halfedge* it = start;
        MCGAL::Halfedge* prev = nullptr;
        int guard = 0;
        while (it && ++guard <= 8192) {
            if (it == h) {
                if (prev) return prev;
                MCGAL::Halfedge* tail = h;
                int tailGuard = 0;
                while (tail && tail->next() && tail->next() != h && ++tailGuard <= 8192) {
                    tail = tail->next();
                }
                return (tail && tail->next() == h) ? tail : nullptr;
            }
            prev = it;
            it = it->next();
            if (it == start) break;
        }
        return nullptr;
    }

    static MCGAL::Halfedge* firstLive(std::initializer_list<MCGAL::Halfedge*> edges) {
        for (MCGAL::Halfedge* e : edges) {
            if (e && !e->isRemoved()) return e;
        }
        return nullptr;
    }

    static void replaceVertexProxy(MCGAL::Vertex* v,
                                   MCGAL::Halfedge* old,
                                   std::initializer_list<MCGAL::Halfedge*> candidates) {
        if (!v || v->proxyHalfedge() != old) return;
        if (MCGAL::Halfedge* repl = firstLive(candidates)) {
            v->setProxyHalfedge(repl);
        }
    }

    static bool mergeSameGroupFace(MCGAL::Halfedge* h) {
        if (!h || h->isRemoved() || !hasSameGroupOpposite(h)) return false;
        if (canUseNativeJoinFace(h)) {
            MCGAL::join_face(h);
            return true;
        }
        return joinAddedEdge(h, h->face() ? h->face()->groupId() : -1);
    }

    bool mergeAddedFace(MCGAL::Halfedge* h, int ownerGroup) {
        if (!h || h->isRemoved() || !h->isAdded()) return false;
        if (hasSameGroupOpposite(h)) return mergeSameGroupFace(h);
        if (ownerLocalSeamReplay() && mergeOwnerLocalSeam(h, ownerGroup)) {
            ++ownerLocalSeamReplays_;
            return true;
        }
	        if (!joinCrossGroupAddedEdges()) return false;
	        if (!h->opposite() || !h->face() || !h->opposite()->face()) return false;
	        if (ownerGroup >= 0 &&
	            h->face()->groupId() != ownerGroup &&
	            h->opposite()->face()->groupId() != ownerGroup) {
	            return false;
	        }
	        return joinAddedEdge(h, ownerGroup);
	    }

    bool mergeOwnerLocalSeam(MCGAL::Halfedge* h, int ownerGroup) {
        if (!mesh_ || ownerGroup < 0 || !h || h->isRemoved() ||
            !h->opposite() || h->opposite()->isRemoved()) {
            return false;
        }

        MCGAL::Halfedge* owner = h;
        MCGAL::Halfedge* other = h->opposite();
        if (!owner->face() || owner->face()->groupId() != ownerGroup) {
            if (other && other->face() && other->face()->groupId() == ownerGroup) {
                std::swap(owner, other);
            } else {
                return false;
            }
        }
        if (!owner->isAdded() || !owner->face() || !other || !other->face()) {
            return false;
        }
        MCGAL::Facet* ownerFace = owner->face();
        MCGAL::Facet* otherFace = other->face();
        if (!ownerFace || !otherFace || ownerFace->isRemoved() ||
            otherFace->isRemoved() || ownerFace->groupId() != ownerGroup ||
            otherFace->groupId() == ownerGroup) {
            return false;
        }

        // Keep the default narrow: the first proven drift is an unsplittable
        // owner seam. Split-opposite seams can be enabled separately for the
        // next diagnostic step without changing default behaviour.
        if ((ownerFace->isSplittable() || otherFace->isSplittable()) &&
            !ownerLocalSeamReplaySplitOpposite()) {
            return false;
        }

        MCGAL::Halfedge* ownerPrev = safePrevInPath(owner);
        MCGAL::Halfedge* ownerNext = owner->next();
        MCGAL::Halfedge* otherPrev = safePrevInPath(other);
        MCGAL::Halfedge* otherNext = other->next();
        if (!ownerPrev || !ownerNext || !otherPrev || !otherNext) {
            return false;
        }

        std::vector<MCGAL::Halfedge*> sourceChain;
        MCGAL::Halfedge* it = otherNext;
        int guard = 0;
        while (it && !it->isRemoved() && ++guard <= 8192) {
            if (!it->vertex() || !it->end_vertex()) return false;
            sourceChain.push_back(it);
            if (it == otherPrev) break;
            it = it->next();
            if (it == other || it == otherNext) return false;
        }
        if (sourceChain.empty() || sourceChain.back() != otherPrev) {
            return false;
        }

        
        const int meshId = mesh_->meshId() >= 0 ? mesh_->meshId() : DEFAULT_MESH_ID;
        std::vector<MCGAL::Halfedge*> copies;
        copies.reserve(sourceChain.size());
        for (MCGAL::Halfedge* src : sourceChain) {
            MCGAL::Halfedge* copy =
                MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(
                    meshId, src->vertex(), src->end_vertex());
            copy->setFace(ownerFace);
            // Mirror the encoder-side traversal relation without stealing the
            // neighbour face: the reverse pointer remains owned by its original
            // halfedge, so this is a read-side local seam view.
            copy->setOpposite(src->opposite());
            copies.push_back(copy);
        }

        ownerPrev->setNext(copies.front());
        for (std::size_t i = 1; i < copies.size(); ++i) {
            copies[i - 1]->setNext(copies[i]);
        }
        copies.back()->setNext(ownerNext);

        replaceVertexProxy(owner->vertex(), owner, {ownerPrev, ownerNext});
        owner->setRemoved();
        ownerFace->setNotRemoved();
        ownerFace->setGroupId(ownerGroup);
        ownerFace->reset_without_init(ownerPrev);
        return true;
    }

    static bool canUseNativeJoinFace(MCGAL::Halfedge* h) {
        if (!h || h->isRemoved() || !hasSameGroupOpposite(h)) return false;
        MCGAL::Halfedge* o = h->opposite();
        MCGAL::Halfedge* hp = livePrevInOwnCycle(h);
        MCGAL::Halfedge* op = livePrevInOwnCycle(o);
        return hp && op && hp != h && hp != o && op != h && op != o &&
               !hp->isRemoved() && !op->isRemoved();
    }

    static MCGAL::Halfedge* livePrevInOwnCycle(MCGAL::Halfedge* h) {
        if (!h) return nullptr;
        MCGAL::Halfedge* it = h;
        int guard = 0;
        while (it && ++guard <= 8192) {
            if (it->isRemoved()) return nullptr;
            MCGAL::Halfedge* next = it->next();
            if (!next) return nullptr;
            if (next == h) return it;
            it = next;
        }
        return nullptr;
    }

	    static bool joinAddedEdge(MCGAL::Halfedge* h, int ownerGroup = -1) {
	        if (!h || h->isRemoved() || !h->opposite() || h->opposite()->isRemoved()) {
	            return false;
	        }
	        MCGAL::Halfedge* o = h->opposite();
	        MCGAL::Facet* f = h->face();
	        MCGAL::Facet* g = o->face();
	        if (!f || !g || f->isRemoved() || g->isRemoved()) return false;

	        MCGAL::Halfedge* hp = safePrevInPath(h);
	        MCGAL::Halfedge* op = safePrevInPath(o);
        MCGAL::Halfedge* hn = h->next();
        MCGAL::Halfedge* on = o->next();
        if (!hp && !op) return false;

        if (hp) hp->setNext(on);
        if (op) op->setNext(hn);

        replaceVertexProxy(h->vertex(), h, {hp, hn, op, on});
        replaceVertexProxy(o->vertex(), o, {op, on, hp, hn});
        h->setRemoved();
        o->setRemoved();

        const bool keepOppositeFace = g->isSplittable();
        MCGAL::Facet* kept = keepOppositeFace ? g : f;
        MCGAL::Facet* discarded = keepOppositeFace ? f : g;
        MCGAL::Halfedge* proxy = keepOppositeFace ? op : hp;
        if (!proxy || proxy->isRemoved()) {
            proxy = firstLive({keepOppositeFace ? on : hn,
                               keepOppositeFace ? hn : on,
                               hp, op});
        }
        if (!proxy) {
            kept->setRemoved();
            if (discarded) discarded->setRemoved();
            return false;
        }

	        if (discarded && discarded != kept) discarded->setRemoved();
	        kept->setNotRemoved();
	        kept->reset_without_init(proxy);
	        if (ownerGroup >= 0) kept->setGroupId(ownerGroup);
	        return true;
	    }

    int inferOwnerGroup(const std::vector<MCGAL::Facet*>& facets,
                        const std::vector<MCGAL::Halfedge*>& halfedges) const {
        for (MCGAL::Facet* f : facets) {
            if (f && f->groupId() >= 0) return f->groupId();
        }
        for (MCGAL::Halfedge* h : halfedges) {
            if (h && h->face() && h->face()->groupId() >= 0) {
                return h->face()->groupId();
            }
        }
        return -1;
    }

    void markNewFan(MCGAL::Halfedge* hnew) {
        if (!hnew || !hnew->vertex()) return;
        if (hnew->vertex()->proxyHalfedge() == nullptr) {
            hnew->vertex()->setProxyHalfedge(hnew);
        }
        const bool fillCenterHalfedgeList = hnew->vertex()->halfedges().empty();
        MCGAL::Halfedge* h = hnew;
        int guard = 0;
        while (h && ++guard <= 4096) {
            h->setNew();
            if (h->opposite()) {
                h->opposite()->setNew();
            }
            if (fillCenterHalfedgeList) {
                hnew->vertex()->halfedges().push_back(h);
            }
            h = h->opposite() ? h->opposite()->next() : nullptr;
            if (h == hnew) break;
        }
    }

    void resolveResidualPoint(MCGAL::Facet* f) {
        MCGAL::Halfedge* gate = f->proxyHalfedge();
        if (!gate) return;
        MCGAL::PointInt baryQ = mesh_->floatPosToInt(MCGAL::barycenter(gate));
        f->setRemovedVertexPos(mesh_->intPosToFloat(baryQ + f->getRemovedVertexPosInt()));
    }

    bool insertPoint(MCGAL::Facet* f, bool open) {
        MCGAL::Halfedge* gate = f->proxyHalfedge();
        if (!gate) return false;
        const std::size_t degree = boundedFacePathDegree(gate, 4096);
        if (degree < 3 || degree > 4096) {
                        return false;
        }
        MCGAL::Point p = f->getRemovedVertexPos();
        MCGAL::Halfedge* hnew = open
                                    ? mesh_->create_center_vertex_plus(gate, p)
                                    : mesh_->create_center_vertex_without_init(gate, p);
        markNewFan(hnew);
        return hnew != nullptr;
    }

    static std::size_t boundedFacePathDegree(MCGAL::Halfedge* gate, std::size_t cap) {
        if (!gate) return 0;
        std::size_t n = 0;
        MCGAL::Halfedge* e = gate;
        while (e && !e->isRemoved()) {
            ++n;
            if (n > cap) return n;
            if (!e->next()) {
                return n + 1;
            }
            e = e->next();
            if (e == gate) return n;
        }
        return n;
    }

    MCGAL::Halfedge* createOpenFan(MCGAL::Halfedge* gate, const MCGAL::Point& point) {
        if (!gate || !gate->face()) return nullptr;
        MCGAL::Facet* originalFace = gate->face();
        std::vector<MCGAL::Halfedge*> outerEdges;
        MCGAL::Halfedge* e = gate;
        int guard = 0;
        while (e && !e->isRemoved() && ++guard <= 8192) {
            outerEdges.push_back(e);
            e = e->next();
            if (e == gate) break;
        }
        if (outerEdges.size() < 2) return nullptr;
        if (guard > 8192 || outerEdges.size() > 16) {
                        return nullptr;
        }

        const bool closed = outerEdges.back()->next() == gate;
        std::vector<MCGAL::Vertex*> verts;
        verts.reserve(outerEdges.size() + (closed ? 0 : 1));
        for (MCGAL::Halfedge* edge : outerEdges) {
            if (!edge || !edge->vertex() || !edge->end_vertex()) return nullptr;
            verts.push_back(edge->vertex());
        }
        if (!closed) {
            verts.push_back(outerEdges.back()->end_vertex());
        }
        if (verts.size() < 3) return nullptr;

        int meshId = mesh_->meshId();
        if (meshId < 0) meshId = DEFAULT_MESH_ID;
        MCGAL::Vertex* center = MCGAL::contextPool.allocateVertexFromPool(meshId, point);
        center->setGroupId(gate->end_vertex() ? gate->end_vertex()->groupId() : gate->vertex()->groupId());
        mesh_->vertices().push_back(center);

        const std::size_t vertexCount = verts.size();
        const std::size_t edgeCount = closed ? vertexCount : vertexCount - 1;
        std::vector<MCGAL::Halfedge*> toCenter(vertexCount, nullptr);
        std::vector<MCGAL::Halfedge*> fromCenter(vertexCount, nullptr);

        auto allocHalfedge = [&](MCGAL::Vertex* a, MCGAL::Vertex* b) {
            MCGAL::Halfedge* h = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(meshId, a, b);
            h->setNew();
            return h;
        };

        for (std::size_t i = 0; i < vertexCount; ++i) {
            const bool needToCenter = closed || i > 0;
            const bool needFromCenter = closed || i + 1 < vertexCount;
            if (needToCenter) toCenter[i] = allocHalfedge(verts[i], center);
            if (needFromCenter) {
                fromCenter[i] = allocHalfedge(center, verts[i]);
                center->halfedges().push_back(fromCenter[i]);
            }
        }
        for (std::size_t i = 0; i < vertexCount; ++i) {
            if (toCenter[i] && fromCenter[i]) {
                toCenter[i]->setOpposite(fromCenter[i]);
                fromCenter[i]->setOpposite(toCenter[i]);
            }
        }

        for (std::size_t i = 0; i < edgeCount; ++i) {
            const std::size_t next = (i + 1) % vertexCount;
            MCGAL::Halfedge* outer = outerEdges[i];
            MCGAL::Halfedge* radialIn = toCenter[next];
            MCGAL::Halfedge* radialOut = fromCenter[i];
            if (!outer || !radialIn || !radialOut) return nullptr;

            MCGAL::Facet* face = nullptr;
            if (i == 0) {
                face = originalFace;
                face->setNotRemoved();
            } else {
                face = MCGAL::contextPool.allocateFaceFromPool(meshId);
                face->setGroupId(originalFace->groupId());
                mesh_->faces().push_back(face);
            }
            outer->setNext(radialIn);
            radialIn->setNext(radialOut);
            radialOut->setNext(outer);
            face->reset_without_init(outer);
        }

        if (center->proxyHalfedge() == nullptr && !center->halfedges().empty()) {
            center->setProxyHalfedge(center->halfedges().front());
        }
        return center->proxyHalfedge();
    }

    std::size_t ownerLocalSeamReplays_ = 0;
};

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 3) {
	        std::cerr << "用法: " << argv[0]
	                  << " <input-base-or.parquet> <output.off> "
	                     "[--voxels=N | --csv=docs/decoding_simple.csv --row=0 --lod=80] "
	                     "[--target-round=N] [--ppmc-reconstruct] "
	                     "[--repair-output|--weld-output] "
                         "[--expect-vertices=N --expect-faces=N]\n";
	        return 1;
	    }

    std::string inputPath = argv[1];
    std::string outputPath = argv[2];
    int requestedVoxels = -1;
    int row = 0;
    int lod = 100;
    int targetRound = -1;
    int expectedVertices = -1;
    int expectedFaces = -1;
	    std::string csvPath;
	    bool openBoundaryReconstruct = true;
	    bool repairOutput = false;
	    bool weldOutput = false;

    for (int i = 3; i < argc; ++i) {
        std::string arg = argv[i];
        requestedVoxels = parseIntArg(arg, "--voxels=", requestedVoxels);
        row = parseIntArg(arg, "--row=", row);
        lod = parseIntArg(arg, "--lod=", lod);
        targetRound = parseIntArg(arg, "--target-round=", targetRound);
        expectedVertices = parseIntArg(arg, "--expect-vertices=", expectedVertices);
        expectedFaces = parseIntArg(arg, "--expect-faces=", expectedFaces);
        csvPath = parseStringArg(arg, "--csv=", csvPath);
	        if (arg == "--ppmc-reconstruct") {
	            openBoundaryReconstruct = false;
	        } else if (arg == "--repair-output") {
	            repairOutput = true;
	        } else if (arg == "--weld-output") {
	            weldOutput = true;
	        }
	    }
    if (const char* env = std::getenv("PMSF_WELD_OUTPUT")) {
        if (*env && std::string(env) != "0") weldOutput = true;
    }
    if (!csvPath.empty()) {
        requestedVoxels = voxelsFromCsv(csvPath, row, lod);
    }

    TimeRecorder recorder("segmented_ppmc_columnar_owned_boundary_decompress");

    // HEAD-port: size + register the MCGAL context pool (was the `PMSF pmsf;`
    // ctor's side effect at ac9a973).
    pmsf::ensureMeshRuntimeInitialized();
    auto reader = std::make_shared<PPMCSymbolReadOperator>();
    reader->setEnablePrediction(false);
    std::shared_ptr<ReconstructOperator> reconstructor =
        openBoundaryReconstruct
            ? std::static_pointer_cast<ReconstructOperator>(
                  std::make_shared<OpenBoundaryPPMCReconstructOperator>())
            : std::static_pointer_cast<ReconstructOperator>(
                  std::make_shared<PPMCReconstructOperator>());
    auto deserializer = std::make_shared<ColumnarDeserializeOperator>(reader, reconstructor);
    deserializer->init(inputPath);
    if (requestedVoxels >= 0) deserializer->setMaxGroups(requestedVoxels);

    PMSFHeader header = deserializer->header();
    auto mesh = deserializer->deserializeBaseMesh();

    const int rounds = targetRound < 0 ? header.getRound()
                                       : std::min(targetRound, header.getRound());
    for (int round = 0; round < rounds; ++round) {
        mesh->resetState();
        deserializer->deserializeOneRound();
    }

	    DumpStats dumpStats = weldOutput ? dumpWeldedOff(mesh, outputPath)
	                         : (repairOutput ? dumpRepairedOff(mesh, outputPath)
	                                         : dumpCompleteOff(mesh, outputPath));
	    std::cerr << "[SegmentedPPMCColumnarOwnedBoundary] decoded_groups="
	              << (requestedVoxels < 0 ? header.getGroupNumber()
	                                      : std::min(requestedVoxels, header.getGroupNumber()))
	              << " target_round=" << rounds
	              << " total_groups=" << header.getGroupNumber()
	              << " open_boundary_reconstruct=" << (openBoundaryReconstruct ? 1 : 0)
	              << " repair_output=" << (repairOutput ? 1 : 0)
	              << " weld_output=" << (weldOutput ? 1 : 0)
	              << " output_vertices=" << dumpStats.vertices
	              << " output_faces=" << dumpStats.faces;
	    	    std::cerr << "\n";
    if ((expectedVertices >= 0 &&
         dumpStats.vertices != static_cast<std::size_t>(expectedVertices)) ||
        (expectedFaces >= 0 &&
         dumpStats.faces != static_cast<std::size_t>(expectedFaces))) {
        std::cerr << "[SegmentedPPMCColumnarOwnedBoundary] strict count mismatch"
                  << " expected_vertices=" << expectedVertices
                  << " actual_vertices=" << dumpStats.vertices
                  << " expected_faces=" << expectedFaces
                  << " actual_faces=" << dumpStats.faces
                  << "\n";
        return 4;
    }
    return 0;
}
