// QEMFaceBudget: standalone QEM edge-collapse face-budget snapshot.
// Usage: QEMFaceBudget input.off output.off targetFaces
// Build: cmake --build build --target QEMFaceBudget
//
// Three collapse policies:
//   * default (legacy): MCGAL half-edge path with persistent one-ring blocking
//     + incremental quadrics. Kept byte-identical for the historical
//     9.1427e-06 E250 row.
//   * PMSF_QEM_REF_STYLE=1: MCGAL half-edge path with reference mesh_simp.cpp
//     *policy* (round deferral, exact quadrics, flip penalty) but still bound
//     by `Mesh::new_is_collapse_ok`; this is the historical 2.526e-05 path.
//   * PMSF_QEM_REF_ENGINE=1: framework-owned union-find/face-list engine
//     (impl/QEM/QEMRefStyleEngine.h) — independent vertex/face/edge storage,
//     no MCGAL link condition, reference topology policy. Task-16 path; the
//     default legacy bytes are untouched.
//
// The framework-owned engine reads/writes OFF directly and never touches
// reference/OpenMesh_QEM (the in-tree reference build stays calibration only).
//
// Progressive artifact (task-20, env-gated; own format until PaperStream M4):
//   * encode snapshot + replayable split stream:
//       PMSF_QEM_REF_ENGINE=1 PMSF_QEM_PROG_ARTIFACT=<artifact.bin> \
//       [PMSF_QEM_PROG_GEOM64=1] QEMFaceBudget input.off base.off 250
//   * decode base -> any LOD (maxLod = split rounds to apply; -1 = all):
//       PMSF_QEM_PROG_DECODE=1 QEMFaceBudget <artifact.bin> out.off <maxLod>
//   Both modes are additive; the snapshot path above is byte-identical.
//
// M5-3 QEM progressive PaperStream fusion (env-gated; default paths unchanged):
//   * encode fused `_paper.parquet` (PSM2 four-stream bundle + P2BL blob):
//       PMSF_QEM_REF_ENGINE=1 PMSF_QEM_PROG_PAPER_ARTIFACT=<out.parquet> \
//       [PMSF_QEM_PROG_GEOM64=1] QEMFaceBudget input.off base.off 250
//   * decode fused artifact -> OFF at any LOD:
//       PMSF_QEM_PROG_PAPER_DECODE=1 QEMFaceBudget <artifact.parquet> out.off <maxLod>
//   The four streams are the verbatim buildLogicalStreams() bytes; decode
//   rebuilds the own-format buffer in memory and reuses the phase-1 decoder.
#include "core/MeshRuntime.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "QEMProgressiveArtifact.h"
#include "QEMProgressivePaperStream.h"
#include "QEMRefStyleEngine.h"
#include "Vertex.h"
#include "symetric_matrix.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace {
struct Entry {
    double cost;
    MCGAL::Halfedge* h;
    MCGAL::Point target;
    int timer = 0;
    bool operator<(const Entry& o) const { return cost > o.cost; }
};

double vertexError(SymetricMatrix q, double x, double y, double z) {
    return q[0] * x * x + 2 * q[1] * x * y + 2 * q[2] * x * z + 2 * q[3] * x
         + q[4] * y * y + 2 * q[5] * y * z + 2 * q[6] * y
         + q[7] * z * z + 2 * q[8] * z + q[9];
}

int liveFaces(MCGAL::Mesh& mesh) {
    int n = 0;
    for (MCGAL::Facet* f : mesh.faces()) {
        if (f && !f->isRemoved()) ++n;
    }
    return n;
}

// Unit-normal plane quadric of a live triangular facet.
bool faceQuadric(MCGAL::Facet* f, SymetricMatrix* out) {
    if (!f || f->isRemoved()) return false;
    auto hs = f->getHalfedges();
    if (hs.size() < 3) return false;
    MCGAL::Point p0 = hs[0]->vertex()->point();
    MCGAL::Point p1 = hs[1]->vertex()->point();
    MCGAL::Point p2 = hs[2]->vertex()->point();
    double ux = p1[0] - p0[0], uy = p1[1] - p0[1], uz = p1[2] - p0[2];
    double vx = p2[0] - p0[0], vy = p2[1] - p0[1], vz = p2[2] - p0[2];
    double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
    double len = std::sqrt(nx * nx + ny * ny + nz * nz);
    if (len < 1e-20) return false;
    nx /= len; ny /= len; nz /= len;
    double d = -(nx * p0[0] + ny * p0[1] + nz * p0[2]);
    *out = SymetricMatrix(nx, ny, nz, d);
    return true;
}

// Exact quadric from the incident surviving faces (reference recomputes from
// face_list after every collapse).
SymetricMatrix vertexQuadric(MCGAL::Vertex* v) {
    SymetricMatrix q(0.0);
    if (!v) return q;
    std::unordered_set<MCGAL::Facet*> seen;
    for (MCGAL::Halfedge* hh : v->halfedges()) {
        if (!hh) continue;
        MCGAL::Facet* f = hh->face();
        if (!f || f->isRemoved()) continue;
        if (!seen.insert(f).second) continue;
        SymetricMatrix pq;
        if (faceQuadric(f, &pq)) q += pq;
    }
    return q;
}

struct Vec3 {
    double x = 0, y = 0, z = 0;
};
Vec3 crossV(MCGAL::Point a, MCGAL::Point b, MCGAL::Point c) {
    Vec3 u{b[0] - a[0], b[1] - a[1], b[2] - a[2]};
    Vec3 v{c[0] - a[0], c[1] - a[1], c[2] - a[2]};
    return Vec3{u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z,
                u.x * v.y - u.y * v.x};
}
double normedDot(const Vec3& a, const Vec3& b) {
    double la = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    double lb = std::sqrt(b.x * b.x + b.y * b.y + b.z * b.z);
    if (la < 1e-20 || lb < 1e-20) return 1.0;  // degenerate: do not veto
    return (a.x * b.x + a.y * b.y + a.z * b.z) / (la * lb);
}

// Reference flip penalty: for every surviving face incident to v0 or v1 the
// old/new normal dot product must stay > 0.25, otherwise cost := 1e5.
bool collapseWouldFlip(MCGAL::Vertex* v0, MCGAL::Vertex* v1,
                       const MCGAL::Point& target) {
    for (MCGAL::Vertex* vp : {v0, v1}) {
        if (!vp) continue;
        std::unordered_set<MCGAL::Facet*> seen;
        for (MCGAL::Halfedge* hh : vp->halfedges()) {
            if (!hh) continue;
            MCGAL::Facet* f = hh->face();
            if (!f || f->isRemoved()) continue;
            if (!seen.insert(f).second) continue;
            auto hs = f->getHalfedges();
            if (hs.size() < 3) continue;
            bool has0 = false, has1 = false;
            for (MCGAL::Halfedge* h2 : hs) {
                MCGAL::Vertex* vv = h2->vertex();
                if (vv == v0) has0 = true;
                if (vv == v1) has1 = true;
            }
            if (has0 && has1) continue;  // face disappears in the collapse
            MCGAL::Point oldP[3], newP[3];
            for (int i = 0; i < 3; ++i) {
                MCGAL::Vertex* vv = hs[static_cast<size_t>(i)]->vertex();
                oldP[i] = vv->point();
                if (vv == v0 || vv == v1) newP[i] = target;
                else newP[i] = vv->point();
            }
            Vec3 nOld = crossV(oldP[0], oldP[1], oldP[2]);
            Vec3 nNew = crossV(newP[0], newP[1], newP[2]);
            if (normedDot(nOld, nNew) <= 0.25) return true;
        }
    }
    return false;
}

struct PairHash {
    size_t operator()(const std::pair<MCGAL::Vertex*, MCGAL::Vertex*>& p) const {
        uintptr_t a = reinterpret_cast<uintptr_t>(p.first);
        uintptr_t b = reinterpret_cast<uintptr_t>(p.second);
        return std::hash<uintptr_t>()(a * 1000003ull ^ b);
    }
};
using TimerMap = std::unordered_map<std::pair<MCGAL::Vertex*, MCGAL::Vertex*>,
                                   int, PairHash>;
std::pair<MCGAL::Vertex*, MCGAL::Vertex*> edgeKey(MCGAL::Vertex* a,
                                                  MCGAL::Vertex* b) {
    return a < b ? std::make_pair(a, b) : std::make_pair(b, a);
}

bool envFlagOn(const char* name) {
    const char* v = std::getenv(name);
    return v && *v && v[0] != '0';
}

/// Collects the engine's collapse records for the progressive encoder.
class VectorCollapseRecorder final
    : public pmsf::qem_refstyle::CollapseRecorder {
  public:
    std::vector<pmsf::qem_refstyle::CollapseRecord> records;

    void onCollapse(
        const pmsf::qem_refstyle::CollapseRecord& record) override {
        records.push_back(record);
    }
};

bool readBinaryFile(const char* path, std::vector<std::uint8_t>* data,
                    std::string* error) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        if (error) *error = std::string("cannot open file: ") + path;
        return false;
    }
    in.seekg(0, std::ios::end);
    const std::streamoff size = in.tellg();
    if (size < 0) {
        if (error) *error = std::string("cannot stat file: ") + path;
        return false;
    }
    in.seekg(0, std::ios::beg);
    data->resize(static_cast<std::size_t>(size));
    if (size > 0) {
        in.read(reinterpret_cast<char*>(data->data()),
                static_cast<std::streamsize>(size));
        if (!in) {
            if (error) *error = std::string("cannot read file: ") + path;
            return false;
        }
    }
    return true;
}

bool writeBinaryFile(const char* path, const std::vector<std::uint8_t>& data,
                     std::string* error) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) {
        if (error) *error = std::string("cannot create file: ") + path;
        return false;
    }
    if (!data.empty()) {
        out.write(reinterpret_cast<const char*>(data.data()),
                  static_cast<std::streamsize>(data.size()));
    }
    if (!out) {
        if (error) *error = std::string("cannot write file: ") + path;
        return false;
    }
    return true;
}

// Progressive decoder entry (task-20): artifact -> OFF at requested LOD.
int runProgressiveDecoder(const char* artifactPath, const char* outPath,
                          const char* maxLodArg) {
    std::vector<std::uint8_t> bytes;
    std::string error;
    if (!readBinaryFile(artifactPath, &bytes, &error)) {
        std::fprintf(stderr, "QEMFaceBudget(prog-decode): %s\n", error.c_str());
        return 2;
    }
    pmsf::qem_prog::DecodeOptions opts;
    if (std::strcmp(maxLodArg, "all") == 0 || std::strcmp(maxLodArg, "-1") == 0) {
        opts.maxAppliedRounds = -1;
    } else {
        opts.maxAppliedRounds = std::atoi(maxLodArg);
        if (opts.maxAppliedRounds < 0) opts.maxAppliedRounds = -1;
    }
    opts.verbose = envFlagOn("PMSF_QEM_VERBOSE");
    pmsf::qem_prog::DecodeStats stats;
    pmsf::qem_refstyle::TriangleMesh mesh;
    if (!pmsf::qem_prog::decodeProgressiveArtifact(bytes.data(), bytes.size(),
                                                   opts, &mesh, &stats,
                                                   &error)) {
        std::fprintf(stderr, "QEMFaceBudget(prog-decode): %s\n", error.c_str());
        return 3;
    }
    if (!pmsf::qem_refstyle::writeOffMesh(outPath, mesh, &error, 0)) {
        std::fprintf(stderr, "QEMFaceBudget(prog-decode): %s\n", error.c_str());
        return 2;
    }
    std::fprintf(stderr, "%s\n", stats.summary().c_str());
    std::fprintf(stderr, "QEMProg lod_faces:");
    for (int f : stats.lodFaceCounts) std::fprintf(stderr, " %d", f);
    std::fprintf(stderr, "\nQEMProg lod_vertices:");
    for (int v : stats.lodVertexCounts) std::fprintf(stderr, " %d", v);
    std::fprintf(stderr, "\n");
    return 0;
}

// M5-3 fused PaperStream v2 decoder: P2BL blob artifact -> PSM2 four streams ->
// rebuilt own-format buffer -> the same progressive decoder as the own-format path.
int runPaperProgressiveDecoder(const char* artifactPath, const char* outPath,
                               const char* maxLodArg) {
    pmsf::qem_prog::DecodeOptions opts;
    if (std::strcmp(maxLodArg, "all") == 0 || std::strcmp(maxLodArg, "-1") == 0) {
        opts.maxAppliedRounds = -1;
    } else {
        opts.maxAppliedRounds = std::atoi(maxLodArg);
        if (opts.maxAppliedRounds < 0) opts.maxAppliedRounds = -1;
    }
    opts.verbose = envFlagOn("PMSF_QEM_VERBOSE");
    pmsf::qem_prog::TriangleMesh mesh;
    pmsf::qem_prog::DecodeStats stats;
    std::string error;
    if (!pmsf::qem_paper::decodeQemPaperArtifact(artifactPath, opts, &mesh,
                                                 &stats, &error)) {
        std::fprintf(stderr, "QEMFaceBudget(paper-decode): %s\n", error.c_str());
        return 3;
    }
    if (!pmsf::qem_refstyle::writeOffMesh(outPath, mesh, &error, 0)) {
        std::fprintf(stderr, "QEMFaceBudget(paper-decode): %s\n", error.c_str());
        return 2;
    }
    std::fprintf(stderr, "%s\n", stats.summary().c_str());
    std::fprintf(stderr, "QEMProg lod_faces:");
    for (int f : stats.lodFaceCounts) std::fprintf(stderr, " %d", f);
    std::fprintf(stderr, "\nQEMProg lod_vertices:");
    for (int v : stats.lodVertexCounts) std::fprintf(stderr, " %d", v);
    std::fprintf(stderr, "\n");
    return 0;
}

// Framework-owned reference-style engine path (task-16). Reads/writes OFF
// directly: no MCGAL mesh, no in-tree reference build, no link condition.
int runRefStyleEngine(const char* inputPath, const char* outputPath,
                      int targetFaces) {
    pmsf::qem_refstyle::SimplifyOptions opts;
    opts.targetFaces = targetFaces;
    if (const char* t = std::getenv("PMSF_QEM_REF_T")) {
        opts.pairThreshold = std::atof(t);
    }
    if (const char* tb = std::getenv("PMSF_QEM_REF_TIEBREAK")) {
        const std::string mode(tb);
        if (mode == "random") {
            opts.tieBreak = pmsf::qem_refstyle::TieBreakMode::Random;
        } else if (mode == "fifo") {
            opts.tieBreak = pmsf::qem_refstyle::TieBreakMode::Fifo;
        } else if (mode == "lifo") {
            opts.tieBreak = pmsf::qem_refstyle::TieBreakMode::Lifo;
        } else {
            opts.tieBreak = pmsf::qem_refstyle::TieBreakMode::Seeded;
        }
    }
    if (const char* s = std::getenv("PMSF_QEM_REF_SEED")) {
        opts.tieBreakSeed = static_cast<std::uint64_t>(std::strtoull(s, nullptr, 10));
    }
    opts.verbose = envFlagOn("PMSF_QEM_VERBOSE");
    const char* artifactPath = std::getenv("PMSF_QEM_PROG_ARTIFACT");
    const char* paperArtifactPath = std::getenv("PMSF_QEM_PROG_PAPER_ARTIFACT");
    const bool encodeOwnArtifact = artifactPath && *artifactPath;
    const bool encodePaperArtifact = paperArtifactPath && *paperArtifactPath;
    const bool encodeArtifact = encodeOwnArtifact || encodePaperArtifact;
    VectorCollapseRecorder recorder;
    if (encodeArtifact) opts.recorder = &recorder;

    pmsf::qem_refstyle::TriangleMesh input;
    pmsf::qem_refstyle::TriangleMesh output;
    std::string error;
    if (!pmsf::qem_refstyle::loadOffMesh(inputPath, &input, &error)) {
        std::fprintf(stderr, "QEMFaceBudget(ref-engine): %s\n", error.c_str());
        return 2;
    }
    // Random tie-break mirrors the reference engine's `rand() & 1`; the
    // reference process never calls srand(), so its sequence starts at seed 1.
    if (opts.tieBreak == pmsf::qem_refstyle::TieBreakMode::Random) {
        std::srand(1);
    }
    const auto stats =
        pmsf::qem_refstyle::simplifyMesh(input, &output, opts, &error);
    if (!error.empty()) {
        std::fprintf(stderr, "QEMFaceBudget(ref-engine): %s\n", error.c_str());
        return 2;
    }
    int decimals = 0;
    if (const char* d = std::getenv("PMSF_QEM_REF_OUT_DECIMALS")) {
        decimals = std::atoi(d);
    }
    if (!pmsf::qem_refstyle::writeOffMesh(outputPath, output, &error, decimals)) {
        std::fprintf(stderr, "QEMFaceBudget(ref-engine): %s\n", error.c_str());
        return 2;
    }
    std::fprintf(stderr,
                 "QEMFaceBudget: engine=ref-style(union-find) tiebreak=%s seed=%llu "
                 "%s\n",
                 opts.tieBreak == pmsf::qem_refstyle::TieBreakMode::Random
                     ? "random"
                     : (opts.tieBreak == pmsf::qem_refstyle::TieBreakMode::Fifo
                            ? "fifo"
                            : (opts.tieBreak == pmsf::qem_refstyle::TieBreakMode::Lifo
                                   ? "lifo"
                                   : "seeded")),
                 static_cast<unsigned long long>(opts.tieBreakSeed),
                 stats.summary().c_str());
    pmsf::qem_prog::EncodeOptions encodeOpts;
    encodeOpts.geomF64 = envFlagOn("PMSF_QEM_PROG_GEOM64");
    if (encodeOwnArtifact) {
        std::vector<std::uint8_t> artifact;
        pmsf::qem_prog::EncodeStats encodeStats;
        if (!pmsf::qem_prog::encodeProgressiveArtifact(
                output, stats.outputVertexOriginalIds, recorder.records,
                stats.inputVertices, stats.inputFaces, encodeOpts, &artifact,
                &encodeStats, &error)) {
            std::fprintf(stderr, "QEMFaceBudget(prog-encode): %s\n",
                         error.c_str());
            return 2;
        }
        if (!writeBinaryFile(artifactPath, artifact, &error)) {
            std::fprintf(stderr, "QEMFaceBudget(prog-encode): %s\n",
                         error.c_str());
            return 2;
        }
        std::fprintf(stderr, "%s artifact=%s\n", encodeStats.summary().c_str(),
                     artifactPath);
        for (std::size_t i = 0; i < encodeStats.roundDescriptors.size(); ++i) {
            const auto& d = encodeStats.roundDescriptors[i];
            std::fprintf(stderr,
                         "QEMProg chunk: index=%zu offset=%zu payload_bytes=%zu "
                         "collapses=%d resulting_faces=%d round_id=%lld\n",
                         i, d.artifactOffset,
                         d.topologyBytes + d.geometryBytes, d.collapseCount,
                         d.resultingFaceCount, d.roundIndex);
        }
        std::fprintf(stderr, "QEMProg encode_total: bytes=%zu bpv=%.6f\n",
                     encodeStats.totalBytes, encodeStats.bpv);
    }
    if (encodePaperArtifact) {
        pmsf::qem_prog::ArtifactStreams streams;
        pmsf::qem_prog::EncodeStats encodeStats;
        std::string encodeError;
        if (!pmsf::qem_prog::buildLogicalStreams(
                output, stats.outputVertexOriginalIds, recorder.records,
                stats.inputVertices, stats.inputFaces, encodeOpts, &streams,
                &encodeStats, &encodeError)) {
            std::fprintf(stderr, "QEMFaceBudget(paper-encode): %s\n",
                         encodeError.c_str());
            return 2;
        }
        const pmsf::qem_prog::SeamStreams seam =
            pmsf::qem_paper::seamStreamsFromArtifactStreams(streams,
                                                            encodeStats);
        pmsf::paper_multistream::QemPaperOptions paperOptions;
        const char* codecEnv = std::getenv("PMSF_QEM_PROG_PAPER_CODEC");
        paperOptions.payloadCodec =
            (codecEnv && std::string(codecEnv) == "none")
                ? pmsf::paper_multistream::PayloadCodec::None
                : pmsf::paper_multistream::PayloadCodec::ZlibRaw;
        if (!pmsf::qem_paper::writeQemPaperArtifact(paperArtifactPath, seam,
                                                    paperOptions, &encodeError)) {
            std::fprintf(stderr, "QEMFaceBudget(paper-encode): %s\n",
                         encodeError.c_str());
            return 2;
        }
        // Ledger: raw four-stream bytes, PSM2 bundle raw bytes, coded payload.
        const std::vector<pmsf::paper_multistream::SeamStream> seamStreams =
            pmsf::qem_paper::makeSeamStreams(seam);
        const std::vector<std::uint8_t> bundle = pmsf::paper_multistream::serializeBundle(
            seamStreams, pmsf::qem_paper::bundleMetaFromSeamMeta(seam.meta),
            &encodeError);
        std::vector<std::uint8_t> codedPayload;
        pmsf::paper_multistream::encodePayload(bundle, paperOptions.payloadCodec, codedPayload,
                            &encodeError);
        std::ifstream paperIn(paperArtifactPath, std::ios::binary);
        paperIn.seekg(0, std::ios::end);
        const std::streamoff fusedBytes = paperIn.tellg();
        std::fprintf(
            stderr,
            "QEMProg paper encode: fused_bytes=%lld streams=(base=%zu round=%zu "
            "topology=%zu geometry=%zu) raw_four_stream_bytes=%zu "
            "psm2_bundle_raw_bytes=%zu payload_coded_bytes=%zu codec=%s\n",
            static_cast<long long>(fusedBytes), seam.baseMesh.size(),
            seam.round.size(), seam.topology.size(), seam.geometry.size(),
            seam.baseMesh.size() + seam.round.size() + seam.topology.size() +
                seam.geometry.size(),
            bundle.size(), codedPayload.size(),
            paperOptions.payloadCodec == pmsf::paper_multistream::PayloadCodec::None ? "none"
                                                                  : "zlib-raw");
        std::fprintf(stderr, "QEMProg paper artifact=%s\n", paperArtifactPath);
    }
    if (!stats.reachedTarget || stats.outputFaces != targetFaces) {
        std::fprintf(stderr,
                     "QEMFaceBudget(ref-engine): target not reached exactly "
                     "(targetFaces=%d outputFaces=%d)\n",
                     targetFaces, stats.outputFaces);
        return 3;
    }
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    if (envFlagOn("PMSF_QEM_PROG_PAPER_DECODE")) {
        if (argc < 3) {
            std::fprintf(stderr,
                         "usage: PMSF_QEM_PROG_PAPER_DECODE=1 QEMFaceBudget "
                         "<artifact.parquet> <out.off> [maxLod|-1]\n");
            return 2;
        }
        const char* maxLod = argc >= 4 ? argv[3] : "-1";
        return runPaperProgressiveDecoder(argv[1], argv[2], maxLod);
    }
    if (envFlagOn("PMSF_QEM_PROG_DECODE")) {
        if (argc < 3) {
            std::fprintf(stderr,
                         "usage: PMSF_QEM_PROG_DECODE=1 QEMFaceBudget "
                         "<artifact.bin> <out.off> [maxLod|-1]\n");
            return 2;
        }
        const char* maxLod = argc >= 4 ? argv[3] : "-1";
        return runProgressiveDecoder(argv[1], argv[2], maxLod);
    }
    if (argc < 4) return 2;
    if (envFlagOn("PMSF_QEM_REF_ENGINE")) {
        return runRefStyleEngine(argv[1], argv[2], std::atoi(argv[3]));
    }
    pmsf::ensureMeshRuntimeInitializedForInputFile(argv[1]);
    MCGAL::Mesh mesh(argv[1]);
    const bool refStyle = [] {
        const char* v = std::getenv("PMSF_QEM_REF_STYLE");
        return v && *v && v[0] != '0';
    }();

    std::unordered_map<MCGAL::Vertex*, SymetricMatrix> Q;
    auto quadric = [&](MCGAL::Vertex* v) -> SymetricMatrix& {
        auto it = Q.find(v);
        if (it == Q.end()) it = Q.emplace(v, SymetricMatrix(0.0)).first;
        return it->second;
    };

    for (MCGAL::Facet* f : mesh.faces()) {
        if (!f || f->isRemoved()) continue;
        auto hs = f->getHalfedges();
        if (hs.size() < 3) continue;
        MCGAL::Point p0 = hs[0]->vertex()->point();
        MCGAL::Point p1 = hs[1]->vertex()->point();
        MCGAL::Point p2 = hs[2]->vertex()->point();
        double ux = p1[0] - p0[0], uy = p1[1] - p0[1], uz = p1[2] - p0[2];
        double vx = p2[0] - p0[0], vy = p2[1] - p0[1], vz = p2[2] - p0[2];
        double nx = uy * vz - uz * vy, ny = uz * vx - ux * vz, nz = ux * vy - uy * vx;
        double len = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (len < 1e-20) continue;
        nx /= len; ny /= len; nz /= len;
        double d = -(nx * p0[0] + ny * p0[1] + nz * p0[2]);
        SymetricMatrix plane(nx, ny, nz, d);
        for (MCGAL::Halfedge* h : hs) quadric(h->vertex()) += plane;
    }

    auto computeTarget = [&](MCGAL::Vertex* v0, MCGAL::Vertex* v1,
                             MCGAL::Point* t, double* cost) {
        SymetricMatrix q = quadric(v0) + quadric(v1);
        MCGAL::Point p0 = v0->point();
        MCGAL::Point p1 = v1->point();
        double det = q.Det(0, 1, 2, 1, 4, 5, 2, 5, 7);
        if (std::fabs(det) > 1e-6) {
            (*t)[0] = -1.0 / det * q.Det(1, 2, 3, 4, 5, 6, 5, 7, 8);
            (*t)[1] = 1.0 / det * q.Det(0, 2, 3, 1, 5, 6, 2, 7, 8);
            (*t)[2] = -1.0 / det * q.Det(0, 1, 3, 1, 4, 6, 2, 5, 8);
            *cost = vertexError(q, (*t)[0], (*t)[1], (*t)[2]);
        } else {
            MCGAL::Point mid = (p0 + p1) / 2;
            MCGAL::Point cand[3] = {p0, p1, mid};
            double err[3] = {vertexError(q, p0[0], p0[1], p0[2]),
                             vertexError(q, p1[0], p1[1], p1[2]),
                             vertexError(q, mid[0], mid[1], mid[2])};
            int best = 0;
            for (int i = 1; i < 3; ++i)
                if (err[i] < err[best]) best = i;
            *t = cand[best];
            *cost = err[best];
        }
    };

    auto makeEntry = [&](MCGAL::Halfedge* h, MCGAL::Vertex* v0,
                         MCGAL::Vertex* v1) {
        double cost = 0.0;
        MCGAL::Point t;
        computeTarget(v0, v1, &t, &cost);
        if (refStyle && collapseWouldFlip(v0, v1, t)) cost = 1e5;
        Entry e;
        e.cost = cost;
        e.h = h;
        e.target = t;
        return e;
    };

    std::priority_queue<Entry> pq;
    TimerMap timer;
    int nextTimer = 1;
    auto pushEdge = [&](MCGAL::Halfedge* h) {
        if (!h || h->isRemoved()) return;
        auto* o = h->opposite();
        if (!o) return;
        auto* v0 = h->vertex();
        auto* v1 = o->vertex();
        if (!v0 || !v1 || v0->isRemoved() || v1->isRemoved()) return;
        Entry e = makeEntry(h, v0, v1);
        e.timer = ++nextTimer;
        timer[edgeKey(v0, v1)] = e.timer;
        pq.push(e);
    };
    auto refill = [&](std::priority_queue<Entry>& q) {
        q = std::priority_queue<Entry>();
        for (MCGAL::Facet* f : mesh.faces()) {
            if (!f || f->isRemoved()) continue;
            for (MCGAL::Halfedge* h : f->getHalfedges()) pushEdge(h);
        }
    };

    const int target = std::atoi(argv[3]);
    int live = liveFaces(mesh);
    int done = 0;

    if (!refStyle) {
        // ---------------- legacy policy (unchanged) ----------------
        refill(pq);
        int sinceCount = 0;
        std::unordered_set<MCGAL::Vertex*> blocked;
        const char* protectEnv = std::getenv("PMSF_QEM_PROTECT_MODE");
        const bool protectSurvivor = !(protectEnv && protectEnv[0] == '1');
        const bool protectNeighbors = !(protectEnv && protectEnv[0] == '2');
        while (live > target) {
            if (pq.empty()) {
                blocked.clear();
                refill(pq);
                if (pq.empty()) break;
            }
            Entry e = pq.top();
            pq.pop();
            MCGAL::Halfedge* h = e.h;
            if (!h || h->isRemoved()) continue;
            MCGAL::Vertex* v0 = h->vertex();
            MCGAL::Vertex* v1 = h->end_vertex();
            if (!v0 || !v1 || v0->isRemoved() || v1->isRemoved()) continue;
            if (blocked.count(v0) || blocked.count(v1)) continue;
            if (!mesh.new_is_collapse_ok(h)) continue;
            std::vector<MCGAL::Vertex*> protectedNeighbors;
            if (protectSurvivor) protectedNeighbors.push_back(v0);
            protectedNeighbors.push_back(v1);
            if (protectNeighbors) {
                std::vector<MCGAL::Vertex*> firstRing;
                for (MCGAL::Halfedge* hh : v1->halfedges()) {
                    if (!hh) continue;
                    MCGAL::Vertex* vv = hh->vertex();
                    if (vv && vv != v1) {
                        protectedNeighbors.push_back(vv);
                        firstRing.push_back(vv);
                    }
                }
                const char* radiusEnv = std::getenv("PMSF_QEM_PROTECT_RADIUS");
                if (radiusEnv && std::atoi(radiusEnv) >= 2) {
                    for (MCGAL::Vertex* vv : firstRing) {
                        for (MCGAL::Halfedge* hh : vv->halfedges()) {
                            if (!hh) continue;
                            MCGAL::Vertex* v2 = hh->vertex();
                            if (v2 && v2 != vv && v2 != v0 && v2 != v1)
                                protectedNeighbors.push_back(v2);
                        }
                    }
                }
            }
            v0->setPoint(e.target);
            mesh.new_collapse(h);
            if (!(protectEnv && protectEnv[0] == '3')) {
                for (MCGAL::Vertex* vv : protectedNeighbors) {
                    if (vv && !vv->isRemoved()) blocked.insert(vv);
                }
            }
            quadric(v0) += quadric(v1);
            Q.erase(v1);
            for (MCGAL::Halfedge* hh : v0->halfedges()) {
                if (!hh || hh->isRemoved() || !hh->face() ||
                    hh->face()->isRemoved())
                    continue;
                if (hh->vertex() != v0) continue;
                pushEdge(hh);
            }
            live -= 2;
            ++done;
            if (std::getenv("PMSF_QEM_FULL_REBUILD") && live <= 1000) {
                refill(pq);
            }
            if (++sinceCount >= 256) {
                live = liveFaces(mesh);
                sinceCount = 0;
            }
        }
    } else {
        // ---------------- reference mesh_simp.cpp-style policy ----------------
        refill(pq);
        bool stalledOnce = false;
        while (live > target) {
            std::unordered_set<MCGAL::Vertex*> cannotDelete;
            std::vector<Entry> deferred;
            bool collapsedThisRound = false;
            int guard = 0;
            while (live > target && ++guard < 4000000) {
                Entry e;
                bool found = false;
                while (!pq.empty()) {
                    Entry cand = pq.top();
                    pq.pop();
                    MCGAL::Halfedge* h = cand.h;
                    if (!h || h->isRemoved()) continue;
                    MCGAL::Vertex* v0 = h->vertex();
                    MCGAL::Vertex* v1 = h->end_vertex();
                    if (!v0 || !v1 || v0->isRemoved() || v1->isRemoved())
                        continue;
                    auto it = timer.find(edgeKey(v0, v1));
                    if (it == timer.end() || it->second != cand.timer) continue;
                    if (!mesh.new_is_collapse_ok(h) || cannotDelete.count(v1)) {
                        // Keep the candidate for a later round instead of
                        // dropping it: the link condition can flip back once
                        // neighbours change (reference-style deferral).
                        deferred.push_back(cand);
                        continue;
                    }
                    e = cand;
                    found = true;
                    break;
                }
                if (!found) break;

                MCGAL::Halfedge* h = e.h;
                MCGAL::Vertex* v0 = h->vertex();
                MCGAL::Vertex* v1 = h->end_vertex();
                // reference protect_deleted_neighborhood(v): v and its one-ring
                cannotDelete.insert(v1);
                for (MCGAL::Halfedge* hh : v1->halfedges()) {
                    if (!hh) continue;
                    MCGAL::Vertex* vv = hh->vertex();
                    if (vv && vv != v1) cannotDelete.insert(vv);
                }
                v0->setPoint(e.target);
                mesh.new_collapse(h);
                live -= 2;
                ++done;
                collapsedThisRound = true;

                // Exact quadric refresh for survivor + its one-ring.
                std::unordered_set<MCGAL::Vertex*> refresh{v0};
                for (MCGAL::Halfedge* hh : v0->halfedges()) {
                    if (!hh) continue;
                    MCGAL::Vertex* vv = hh->vertex();
                    if (vv && !vv->isRemoved()) refresh.insert(vv);
                }
                for (MCGAL::Vertex* vv : refresh) {
                    if (vv && !vv->isRemoved()) quadric(vv) = vertexQuadric(vv);
                }
                // Local candidate re-push (survivor + ring).
                for (MCGAL::Vertex* vv : refresh) {
                    if (!vv || vv->isRemoved()) continue;
                    for (MCGAL::Halfedge* hh : vv->halfedges()) {
                        if (!hh || hh->isRemoved()) continue;
                        pushEdge(hh);
                    }
                }
            }
            // Next round: protected vertices become eligible again.
            for (const Entry& d : deferred) pq.push(d);
            if (!collapsedThisRound) {
                // The reference heap never runs dry because every collapse
                // re-pushes the affected neighbourhood and stale costs remain
                // usable. Our link-condition deferral can consume the heap, so
                // fall back to a full rebuild; only give up if that also stalls.
                if (stalledOnce) break;
                stalledOnce = true;
                refill(pq);
                continue;
            }
            stalledOnce = false;
        }
    }

    mesh.dumpto_oldtype(argv[2]);
    std::fprintf(stderr, "QEMFaceBudget: collapses=%d live_faces=%d ref_style=%d\n",
                 done, liveFaces(mesh), refStyle ? 1 : 0);
    return 0;
}
