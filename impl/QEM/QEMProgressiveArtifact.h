/*
 * @file QEMProgressiveArtifact.h
 * @brief Paired progressive encoder/decoder for the framework-owned
 *        reference-style QEM engine (task-20, WS-B5).
 *
 * Context
 * -------
 * `QEMRefStyleEngine.h` produces a 250-face snapshot of bunny for the phase-1
 * E250 gate.  The QEM paper has no bitstream, so that row is snapshot-only.
 * Phase 2 (PaperStream) still needs a *progressive* QEM artifact: base mesh +
 * a replayable stream of vertex splits so a decoder can reconstruct any LOD
 * from the 250-face base back to the original 69,664-face mesh.
 *
 * This header implements that paired codec on top of the independent engine
 * (no MCGAL half-edge / link condition, no in-tree reference binary):
 *
 *   encode: run the engine with a `CollapseRecorder` attached, getting per
 *           collapse (survivor id, victim id, both pre-collapse positions, and
 *           the oriented `(victim, a, b)` fan of all incident faces).  Then
 *           serialize the base mesh + per-round split records in an own-format
 *           binary container (this header), ordered so that a byte prefix is a
 *           valid coarser LOD.
 *
 *   decode: load the base mesh (exact float64 positions => E250 unchanged),
 *           then walk the split chunks in file order (= reverse simplification
 *           round order) and undo collapses.  `maxAppliedRounds` and byte
 *           prefixes are supported; a truncated artifact decodes the longest
 *           complete prefix.
 *
 * Own format v1 (little-endian)
 * ----------------------------
 *   header (80 B): magic "QEMPROG1", version, flags, idWidth, geomBytes,
 *                  originalV/F, baseV/F, roundCount, totalCollapses,
 *                  headerBytes
 *   base mesh:    baseV x (id + 3 x float64 exact) + baseF x 3 ids
 *   chunks:       one per simplification round, LAST round first
 *                 u32 payloadBytes, u32 collapseCount,
 *                 topology stream: collapseCount x (survivor, victim,
 *                     u16 fanCount, fanCount x (a, b))
 *                 geometry stream: collapseCount x (oldU[3], oldV[3]) as
 *                     float32 (default, "quantized equivalent") or float64
 *                     (`geomF64`, bit-exact reconstruction)
 *
 * Why the reverse record is sufficient (correctness note)
 * -------------------------------------------------------
 * Forward edge collapse (u survivor, v victim):
 *   - faces containing both u and v are killed (the two edge triangles);
 *   - every other live face of v, originally `(v, a, b)` cyclically, is
 *     re-attributed to `(u, a, b)`;
 *   - u moves to the optimal placement, v dies.
 * Reverse: restore u's pre-collapse position, re-create v at its pre-collapse
 * position, then for each recorded `(v, a, b)` fan face: if `a`/`b` is u the
 * face was killed => create `(v, a, b)`; otherwise find the live `(u, a, b)`
 * face (orientation-preserving canonical rotation) and replace u by v.
 * Applying chunks in reverse order therefore inverts the exact forward
 * sequence, including position history (a vertex that is never killed ends at
 * its original position once its own survivor collapses are undone; a killed
 * vertex is restored at its pre-death position and then further undone by its
 * own earlier survivor records).  No face ids / adjacency are stored: fan ids
 * are original vertex ids, which are stable because union-find
 * representatives are always original ids.
 *
 * PaperStream seam (M4, task-22)
 * ------------------------------
 * `buildLogicalStreams()` produces the four logical streams
 *   baseMeshStream / roundStream / topologyStream / geometryStream
 * in the same order the own-format container uses.  The own format only adds
 * chunk framing + section headers on top.  A PaperStream/Parquet writer can
 * map them directly to tables/columns (base mesh table, per-round metadata
 * table, topology symbol table, geometry residual table) without touching the
 * engine or the decoder logic; only `serializeOwnFormat()` /
 * `parseOwnFormatHeader()` need replacing.  A future geometry predictor can
 * consume `geometryStream` records unchanged.
 *
 * Env wiring lives in tools/QEMFaceBudget.cpp (all env-gated; the snapshot and
 * legacy paths are untouched):
 *   encode: PMSF_QEM_REF_ENGINE=1 PMSF_QEM_PROG_ARTIFACT=<out.bin> \
 *           [PMSF_QEM_PROG_GEOM64=1] QEMFaceBudget input.off base.off 250
 *   decode: PMSF_QEM_PROG_DECODE=1 QEMFaceBudget <artifact.bin> out.off <maxLod>
 *           (maxLod = number of split rounds to apply; -1/absent = all)
 */
#ifndef PMSF_IMPL_QEM_QEM_PROGRESSIVE_ARTIFACT_H
#define PMSF_IMPL_QEM_QEM_PROGRESSIVE_ARTIFACT_H

#include "QEMRefStyleEngine.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstring>
#include <limits>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace pmsf {
namespace qem_prog {

using qem_refstyle::CollapseRecord;
using qem_refstyle::TriangleMesh;

constexpr std::uint32_t kArtifactVersion = 1;
constexpr std::size_t kHeaderBytes = 64;
// "QEMP" / "ROG1" as little-endian uint32.
constexpr std::uint32_t kMagic0 = 0x504D4551u;
constexpr std::uint32_t kMagic1 = 0x31474F52u;
// Header flag bits.
constexpr std::uint32_t kFlagGeomF64 = 1u << 0;
// Face-lookup key packing supports up to 2^21 original vertices (2.1M).
constexpr int kMaxKeyedVertices = 1 << 21;

// ===========================================================================
// Little-endian buffer helpers
// ===========================================================================
inline void putU16(std::vector<std::uint8_t>* out, std::uint16_t v) {
    out->push_back(static_cast<std::uint8_t>(v & 0xffu));
    out->push_back(static_cast<std::uint8_t>((v >> 8) & 0xffu));
}

inline void putU32(std::vector<std::uint8_t>* out, std::uint32_t v) {
    for (int i = 0; i < 4; ++i) {
        out->push_back(static_cast<std::uint8_t>((v >> (8 * i)) & 0xffu));
    }
}

inline void putU64(std::vector<std::uint8_t>* out, std::uint64_t v) {
    for (int i = 0; i < 8; ++i) {
        out->push_back(static_cast<std::uint8_t>((v >> (8 * i)) & 0xffu));
    }
}

inline void putF32(std::vector<std::uint8_t>* out, float v) {
    std::uint32_t bits = 0;
    std::memcpy(&bits, &v, sizeof(bits));
    putU32(out, bits);
}

inline void putF64(std::vector<std::uint8_t>* out, double v) {
    std::uint64_t bits = 0;
    std::memcpy(&bits, &v, sizeof(bits));
    putU64(out, bits);
}

inline void putId(std::vector<std::uint8_t>* out, int id, int idWidth) {
    if (idWidth == 2) {
        putU16(out, static_cast<std::uint16_t>(id));
    } else {
        putU32(out, static_cast<std::uint32_t>(id));
    }
}

class Reader {
  public:
    Reader(const std::uint8_t* data, std::size_t size)
        : data_(data), size_(size) {}

    bool ok() const { return ok_; }
    std::size_t pos() const { return pos_; }
    std::size_t remaining() const { return ok_ ? size_ - pos_ : 0; }

    bool skip(std::size_t n) {
        if (!ok_ || remaining() < n) return fail();
        pos_ += n;
        return true;
    }
    bool u16(std::uint16_t* v) {
        if (!ok_ || remaining() < 2) return fail();
        *v = static_cast<std::uint16_t>(data_[pos_]) |
             static_cast<std::uint16_t>(data_[pos_ + 1] << 8);
        pos_ += 2;
        return true;
    }
    bool u32(std::uint32_t* v) {
        if (!ok_ || remaining() < 4) return fail();
        std::uint32_t x = 0;
        for (int i = 0; i < 4; ++i) {
            x |= static_cast<std::uint32_t>(data_[pos_ + static_cast<std::size_t>(i)])
                 << (8 * i);
        }
        *v = x;
        pos_ += 4;
        return true;
    }
    bool u64(std::uint64_t* v) {
        if (!ok_ || remaining() < 8) return fail();
        std::uint64_t x = 0;
        for (int i = 0; i < 8; ++i) {
            x |= static_cast<std::uint64_t>(data_[pos_ + static_cast<std::size_t>(i)])
                 << (8 * i);
        }
        *v = x;
        pos_ += 8;
        return true;
    }
    bool f32(float* v) {
        std::uint32_t bits = 0;
        if (!u32(&bits)) return false;
        std::memcpy(v, &bits, sizeof(bits));
        return true;
    }
    bool f64(double* v) {
        std::uint64_t bits = 0;
        if (!u64(&bits)) return false;
        std::memcpy(v, &bits, sizeof(bits));
        return true;
    }
    bool id(int idWidth, int* v) {
        std::uint32_t x = 0;
        if (idWidth == 2) {
            std::uint16_t y = 0;
            if (!u16(&y)) return false;
            x = y;
        } else if (!u32(&x)) {
            return false;
        }
        *v = static_cast<int>(x);
        return true;
    }

  private:
    bool fail() {
        ok_ = false;
        return false;
    }

    const std::uint8_t* data_ = nullptr;
    std::size_t size_ = 0;
    std::size_t pos_ = 0;
    bool ok_ = true;
};

// ===========================================================================
// Encoder
// ===========================================================================
struct EncodeOptions {
    bool geomF64 = false;  // false = float32 pre-collapse positions (default)
};

/// One round of the simplification, for LOD bookkeeping (reverse round order:
/// index 0 is the last simplification round = first split chunk).
struct RoundDescriptor {
    long long roundIndex = 0;        // engine round id (1-based)
    int collapseCount = 0;
    int resultingFaceCount = 0;      // face count after undoing this round
    std::size_t topologyBytes = 0;   // per-round topology stream size
    std::size_t geometryBytes = 0;   // per-round geometry stream size
    std::size_t artifactOffset = 0;  // chunk header byte offset in artifact
};

struct EncodeStats {
    std::size_t totalBytes = 0;
    std::size_t headerBytes = 0;
    std::size_t baseMeshBytes = 0;
    std::size_t roundHeaderBytes = 0;  // chunk framing (8 bytes per round)
    std::size_t topologyBytes = 0;     // split symbols (ids + fan counts)
    std::size_t geometryBytes = 0;     // pre-collapse positions
    int idWidthBytes = 2;
    int geomBytesPerComp = 4;
    int originalVertices = 0;
    int originalFaces = 0;
    int baseVertices = 0;
    int baseFaces = 0;
    int rounds = 0;
    long long collapses = 0;
    double bpv = 0.0;  // totalBytes * 8 / originalVertices (storage ledger only)
    std::vector<RoundDescriptor> roundDescriptors;

    std::string summary() const;
};

/// Logical streams before own-format containerization (PaperStream seam).
struct ArtifactStreams {
    std::vector<std::uint8_t> baseMeshStream;  // ids + f64 base positions + faces
    std::vector<std::uint8_t> roundStream;     // per round: u32 collapseCount
    std::vector<std::uint8_t> topologyStream;  // per collapse: u,v,fanCount,fans
    std::vector<std::uint8_t> geometryStream;  // per collapse: pre-collapse pos
    std::vector<RoundDescriptor> roundDescriptors;  // reverse round order
};

/// Group `records` by engine round id, preserving forward order inside a round,
/// then order the groups as the artifact needs them: last simplification round
/// first (so a byte prefix is a coarser LOD).  Also verifies that replaying the
/// fans reconstructs the declared original face count.
inline bool groupRounds(const std::vector<CollapseRecord>& records,
                        int originalVertices, int originalFaces, int baseFaces,
                        std::vector<std::vector<CollapseRecord>>* rounds,
                        std::string* error) {
    rounds->clear();
    if (records.empty()) {
        if (error) *error = "no collapse records to encode";
        return false;
    }
    for (const CollapseRecord& r : records) {
        if (r.survivor < 0 || r.survivor >= originalVertices || r.victim < 0 ||
            r.victim >= originalVertices || r.survivor == r.victim) {
            if (error) *error = "collapse record has invalid vertex ids";
            return false;
        }
        if (r.fanFaces.size() > std::numeric_limits<std::uint16_t>::max()) {
            if (error) *error = "collapse fan exceeds the v1 stream count width";
            return false;
        }
        // A general pair contraction can absorb an isolated vertex. Its
        // inverse restores that point with an empty fan; this is valid in the
        // paper's non-manifold aggregation path and removes no faces.
        for (const auto& fan : r.fanFaces) {
            if (fan.a < 0 || fan.a >= originalVertices || fan.b < 0 ||
                fan.b >= originalVertices) {
                if (error) *error = "collapse fan has out-of-range vertex id";
                return false;
            }
        }
        if (rounds->empty() ||
            rounds->back().front().roundIndex != r.roundIndex) {
            rounds->emplace_back();
        }
        rounds->back().push_back(r);
    }
    for (std::size_t i = 1; i < rounds->size(); ++i) {
        if (rounds->at(i).front().roundIndex <=
            rounds->at(i - 1).front().roundIndex) {
            if (error) *error = "collapse records are not grouped by round";
            return false;
        }
    }
    int faces = baseFaces;
    for (auto it = rounds->rbegin(); it != rounds->rend(); ++it) {
        for (const CollapseRecord& r : *it) {
            for (const auto& fan : r.fanFaces) {
                if (fan.a == r.survivor || fan.b == r.survivor) ++faces;
            }
        }
    }
    if (faces != originalFaces) {
        if (error) {
            *error = "round fan streams do not reconstruct the original face count";
        }
        return false;
    }
    std::reverse(rounds->begin(), rounds->end());
    return true;
}

inline bool buildLogicalStreams(const TriangleMesh& baseMesh,
                                const std::vector<int>& baseVertexOriginalIds,
                                const std::vector<CollapseRecord>& records,
                                int originalVertices, int originalFaces,
                                const EncodeOptions& options,
                                ArtifactStreams* streams, EncodeStats* stats,
                                std::string* error) {
    const int baseVertices = static_cast<int>(baseMesh.vertices.size());
    const int baseFaces = static_cast<int>(baseMesh.faces.size());
    if (baseVertices <= 0 || baseFaces <= 0 || originalVertices <= 0 ||
        originalFaces <= 0) {
        if (error) *error = "invalid mesh counts";
        return false;
    }
    if (baseVertices != static_cast<int>(baseVertexOriginalIds.size())) {
        if (error) *error = "base vertex original-id table size mismatch";
        return false;
    }
    if (originalVertices > kMaxKeyedVertices) {
        if (error) *error = "artifact v1 supports up to 2^21 original vertices";
        return false;
    }
    {
        std::vector<char> seen(static_cast<std::size_t>(originalVertices), 0);
        for (int id : baseVertexOriginalIds) {
            if (id < 0 || id >= originalVertices ||
                seen[static_cast<std::size_t>(id)]) {
                if (error) *error = "base vertex original ids invalid/duplicated";
                return false;
            }
            seen[static_cast<std::size_t>(id)] = 1;
        }
        for (const auto& f : baseMesh.faces) {
            for (int k = 0; k < 3; ++k) {
                if (f[static_cast<std::size_t>(k)] < 0 ||
                    f[static_cast<std::size_t>(k)] >= baseVertices) {
                    if (error) *error = "base face has out-of-range corner";
                    return false;
                }
            }
        }
    }
    const int idWidth = originalVertices <= 65536 ? 2 : 4;

    std::vector<std::vector<CollapseRecord>> rounds;
    if (!groupRounds(records, originalVertices, originalFaces, baseFaces,
                     &rounds, error)) {
        return false;
    }

    // ---- base mesh stream (exact f64 positions: E250 snapshot unchanged) ----
    std::vector<std::uint8_t> base;
    base.reserve(static_cast<std::size_t>(baseVertices) *
                     (24 + static_cast<std::size_t>(idWidth)) +
                 static_cast<std::size_t>(baseFaces) * 3 *
                     static_cast<std::size_t>(idWidth));
    for (int i = 0; i < baseVertices; ++i) {
        putId(&base, baseVertexOriginalIds[static_cast<std::size_t>(i)], idWidth);
        putF64(&base, baseMesh.vertices[static_cast<std::size_t>(i)].x);
        putF64(&base, baseMesh.vertices[static_cast<std::size_t>(i)].y);
        putF64(&base, baseMesh.vertices[static_cast<std::size_t>(i)].z);
    }
    for (int i = 0; i < baseFaces; ++i) {
        for (int k = 0; k < 3; ++k) {
            // Base mesh faces use snapshot-local indices; the artifact keys
            // everything by stable original vertex id.
            const int snapshotIndex =
                baseMesh.faces[static_cast<std::size_t>(i)]
                              [static_cast<std::size_t>(k)];
            putId(&base,
                  baseVertexOriginalIds[static_cast<std::size_t>(snapshotIndex)],
                  idWidth);
        }
    }

    // ---- round / topology / geometry streams (reverse round order) --------
    std::vector<std::uint8_t> roundStream;
    std::vector<std::uint8_t> topologyStream;
    std::vector<std::uint8_t> geometryStream;
    std::vector<RoundDescriptor> descriptors;
    roundStream.reserve(rounds.size() * 4);
    for (const std::vector<CollapseRecord>& round : rounds) {
        RoundDescriptor desc;
        desc.roundIndex = round.front().roundIndex;
        desc.collapseCount = static_cast<int>(round.size());
        putU32(&roundStream, static_cast<std::uint32_t>(round.size()));
        const std::size_t topoStart = topologyStream.size();
        const std::size_t geoStart = geometryStream.size();
        for (const CollapseRecord& r : round) {
            putId(&topologyStream, r.survivor, idWidth);
            putId(&topologyStream, r.victim, idWidth);
            putU16(&topologyStream, static_cast<std::uint16_t>(r.fanFaces.size()));
            for (const auto& fan : r.fanFaces) {
                putId(&topologyStream, fan.a, idWidth);
                putId(&topologyStream, fan.b, idWidth);
            }
            if (options.geomF64) {
                putF64(&geometryStream, r.survivorPosBefore[0]);
                putF64(&geometryStream, r.survivorPosBefore[1]);
                putF64(&geometryStream, r.survivorPosBefore[2]);
                putF64(&geometryStream, r.victimPosBefore[0]);
                putF64(&geometryStream, r.victimPosBefore[1]);
                putF64(&geometryStream, r.victimPosBefore[2]);
            } else {
                putF32(&geometryStream, static_cast<float>(r.survivorPosBefore[0]));
                putF32(&geometryStream, static_cast<float>(r.survivorPosBefore[1]));
                putF32(&geometryStream, static_cast<float>(r.survivorPosBefore[2]));
                putF32(&geometryStream, static_cast<float>(r.victimPosBefore[0]));
                putF32(&geometryStream, static_cast<float>(r.victimPosBefore[1]));
                putF32(&geometryStream, static_cast<float>(r.victimPosBefore[2]));
            }
        }
        desc.topologyBytes = topologyStream.size() - topoStart;
        desc.geometryBytes = geometryStream.size() - geoStart;
        descriptors.push_back(desc);
    }

    stats->idWidthBytes = idWidth;
    stats->geomBytesPerComp = options.geomF64 ? 8 : 4;
    stats->originalVertices = originalVertices;
    stats->originalFaces = originalFaces;
    stats->baseVertices = baseVertices;
    stats->baseFaces = baseFaces;
    stats->rounds = static_cast<int>(rounds.size());
    stats->collapses = static_cast<long long>(records.size());
    stats->headerBytes = kHeaderBytes;
    stats->baseMeshBytes = base.size();
    stats->roundHeaderBytes = static_cast<std::size_t>(rounds.size()) * 8;
    stats->topologyBytes = topologyStream.size();
    stats->geometryBytes = geometryStream.size();
    stats->totalBytes = stats->headerBytes + stats->baseMeshBytes +
                        stats->roundHeaderBytes + stats->topologyBytes +
                        stats->geometryBytes;
    stats->bpv = originalVertices > 0
                     ? static_cast<double>(stats->totalBytes) * 8.0 /
                           originalVertices
                     : 0.0;
    stats->roundDescriptors = descriptors;

    streams->baseMeshStream = std::move(base);
    streams->roundStream = std::move(roundStream);
    streams->topologyStream = std::move(topologyStream);
    streams->geometryStream = std::move(geometryStream);
    streams->roundDescriptors = std::move(descriptors);
    return true;
}

/// Serialize logical streams into the own-format v1 container.  This is the
/// only function the PaperStream M4 writer needs to replace: chunk framing +
/// fixed header are container concerns, the streams above are semantic.
inline bool serializeOwnFormat(const ArtifactStreams& streams,
                               const EncodeOptions& options,
                               std::vector<std::uint8_t>* artifact,
                               EncodeStats* stats, std::string* error) {
    const int idWidth = stats->idWidthBytes;
    const std::size_t chunkCount = streams.roundDescriptors.size();
    if (chunkCount == 0) {
        if (error) *error = "no rounds to serialize";
        return false;
    }
    artifact->clear();
    artifact->reserve(stats->totalBytes);
    // ---- header ----
    putU32(artifact, kMagic0);
    putU32(artifact, kMagic1);
    putU32(artifact, kArtifactVersion);
    putU32(artifact, options.geomF64 ? kFlagGeomF64 : 0u);
    putU32(artifact, static_cast<std::uint32_t>(idWidth));
    putU32(artifact, static_cast<std::uint32_t>(stats->geomBytesPerComp));
    putU32(artifact, static_cast<std::uint32_t>(stats->originalVertices));
    putU32(artifact, static_cast<std::uint32_t>(stats->originalFaces));
    putU32(artifact, static_cast<std::uint32_t>(stats->baseVertices));
    putU32(artifact, static_cast<std::uint32_t>(stats->baseFaces));
    putU32(artifact, static_cast<std::uint32_t>(chunkCount));
    putU32(artifact, 0u);  // reserved
    putU64(artifact, static_cast<std::uint64_t>(stats->collapses));
    putU64(artifact, static_cast<std::uint64_t>(kHeaderBytes));
    if (artifact->size() != kHeaderBytes) {
        if (error) *error = "internal header size mismatch";
        return false;
    }
    // ---- base mesh ----
    artifact->insert(artifact->end(), streams.baseMeshStream.begin(),
                     streams.baseMeshStream.end());
    // ---- chunks (reverse simplification round order) ----
    std::size_t topoPos = 0;
    std::size_t geoPos = 0;
    for (std::size_t c = 0; c < chunkCount; ++c) {
        const RoundDescriptor& desc = streams.roundDescriptors[c];
        if (topoPos + desc.topologyBytes > streams.topologyStream.size() ||
            geoPos + desc.geometryBytes > streams.geometryStream.size()) {
            if (error) *error = "internal stream slicing mismatch";
            return false;
        }
        stats->roundDescriptors[c].artifactOffset = artifact->size();
        putU32(artifact,
               static_cast<std::uint32_t>(desc.topologyBytes + desc.geometryBytes));
        putU32(artifact, static_cast<std::uint32_t>(desc.collapseCount));
        artifact->insert(artifact->end(),
                         streams.topologyStream.begin() +
                             static_cast<std::ptrdiff_t>(topoPos),
                         streams.topologyStream.begin() +
                             static_cast<std::ptrdiff_t>(topoPos +
                                                         desc.topologyBytes));
        artifact->insert(artifact->end(),
                         streams.geometryStream.begin() +
                             static_cast<std::ptrdiff_t>(geoPos),
                         streams.geometryStream.begin() +
                             static_cast<std::ptrdiff_t>(geoPos +
                                                         desc.geometryBytes));
        topoPos += desc.topologyBytes;
        geoPos += desc.geometryBytes;
    }
    if (topoPos != streams.topologyStream.size() ||
        geoPos != streams.geometryStream.size()) {
        if (error) *error = "internal stream length mismatch after serialization";
        return false;
    }
    stats->totalBytes = artifact->size();
    stats->bpv = stats->originalVertices > 0
                     ? static_cast<double>(stats->totalBytes) * 8.0 /
                           stats->originalVertices
                     : 0.0;
    return true;
}

inline bool encodeProgressiveArtifact(
    const TriangleMesh& baseMesh, const std::vector<int>& baseVertexOriginalIds,
    const std::vector<CollapseRecord>& records, int originalVertices,
    int originalFaces, const EncodeOptions& options,
    std::vector<std::uint8_t>* artifact, EncodeStats* stats,
    std::string* error) {
    ArtifactStreams streams;
    if (!buildLogicalStreams(baseMesh, baseVertexOriginalIds, records,
                             originalVertices, originalFaces, options, &streams,
                             stats, error)) {
        return false;
    }
    if (!serializeOwnFormat(streams, options, artifact, stats, error)) {
        return false;
    }
    // Resulting face count per round (chunks are in reverse round order, so
    // base first).  Every split re-creates exactly as many faces as the
    // forward collapse killed; on bunny's closed manifold that is 2 per
    // collapse, and groupRounds() has already validated the total.
    int faces = stats->baseFaces;
    for (RoundDescriptor& desc : stats->roundDescriptors) {
        faces += 2 * desc.collapseCount;
        desc.resultingFaceCount = faces;
    }
    return true;
}

inline std::string EncodeStats::summary() const {
    std::ostringstream os;
    os << "QEMProg encode: bytes=" << totalBytes << " bpv=" << bpv
       << " header_bytes=" << headerBytes
       << " base_mesh_bytes=" << baseMeshBytes
       << " round_header_bytes=" << roundHeaderBytes
       << " topology_bytes=" << topologyBytes
       << " geometry_bytes=" << geometryBytes << " rounds=" << rounds
       << " collapses=" << collapses << " base=(v=" << baseVertices
       << ",f=" << baseFaces << ") original=(v=" << originalVertices
       << ",f=" << originalFaces << ") id_width=" << idWidthBytes
       << " geom_bits=" << geomBytesPerComp * 8;
    return os.str();
}

// ===========================================================================
// Decoder
// ===========================================================================
struct DecodeOptions {
    /// Number of split rounds to apply, counted from the base mesh: 0 = base
    /// LOD, -1 = apply all complete rounds present in the artifact.
    int maxAppliedRounds = -1;
    bool verbose = false;
};

struct DecodeStats {
    int originalVertices = 0;
    int originalFaces = 0;
    int baseVertices = 0;
    int baseFaces = 0;
    int availableRounds = 0;
    int appliedRounds = 0;
    int appliedCollapses = 0;
    int meshVertices = 0;
    int meshFaces = 0;
    bool truncated = false;  // artifact ended before all declared rounds
    bool stoppedByLod = false;
    bool geomF64 = false;
    std::vector<int> lodFaceCounts;    // [0] = base, then after each round
    std::vector<int> lodVertexCounts;

    std::string summary() const;
};

/// Canonical orientation-preserving key of a triangle: rotate so the smallest
/// id is first, then pack three 21-bit ids.
inline std::uint64_t orientedKey(int a, int b, int c) {
    int arr[3] = {a, b, c};
    int minAt = 0;
    if (arr[1] < arr[minAt]) minAt = 1;
    if (arr[2] < arr[minAt]) minAt = 2;
    const std::uint64_t x = static_cast<std::uint64_t>(arr[minAt]);
    const std::uint64_t y = static_cast<std::uint64_t>(arr[(minAt + 1) % 3]);
    const std::uint64_t z = static_cast<std::uint64_t>(arr[(minAt + 2) % 3]);
    return (x << 42) | (y << 21) | z;
}

/// Decoder state: position/alive tables keyed by *original* vertex id plus a
/// live face vector with an orientation-preserving lookup table.
struct DecoderState {
    int idWidth = 2;
    bool geomF64 = false;
    int originalVertices = 0;
    int originalFaces = 0;
    int baseVertices = 0;
    int baseFaces = 0;
    int declaredRounds = 0;
    long long declaredCollapses = 0;

    std::vector<double> posX, posY, posZ;
    std::vector<char> alive;
    int aliveCount = 0;

    struct DFace {
        int v[3] = {0, 0, 0};
    };
    std::vector<DFace> faces;
    std::unordered_map<std::uint64_t, std::vector<int>> faceIndex;

    void reset(int nv) {
        originalVertices = nv;
        posX.assign(static_cast<std::size_t>(nv), 0.0);
        posY.assign(static_cast<std::size_t>(nv), 0.0);
        posZ.assign(static_cast<std::size_t>(nv), 0.0);
        alive.assign(static_cast<std::size_t>(nv), 0);
        aliveCount = 0;
        faces.clear();
        faceIndex.clear();
    }

    bool addFace(int a, int b, int c, std::string* error) {
        if (a < 0 || a >= originalVertices || b < 0 || b >= originalVertices ||
            c < 0 || c >= originalVertices || a == b || b == c || a == c) {
            if (error) *error = "artifact creates a degenerate face";
            return false;
        }
        if (!alive[static_cast<std::size_t>(a)] ||
            !alive[static_cast<std::size_t>(b)] ||
            !alive[static_cast<std::size_t>(c)]) {
            if (error) *error = "artifact creates a face with a dead vertex";
            return false;
        }
        DFace f;
        f.v[0] = a;
        f.v[1] = b;
        f.v[2] = c;
        const int idx = static_cast<int>(faces.size());
        faces.push_back(f);
        faceIndex[orientedKey(a, b, c)].push_back(idx);
        return true;
    }

    int findFace(int a, int b, int c) const {
        const auto it = faceIndex.find(orientedKey(a, b, c));
        if (it == faceIndex.end() || it->second.empty()) return -1;
        return it->second.back();
    }

    void replaceFaceVertex(int faceIndexPos, int from, int to) {
        DFace& f = faces[static_cast<std::size_t>(faceIndexPos)];
        const std::uint64_t oldKey = orientedKey(f.v[0], f.v[1], f.v[2]);
        auto oldIt = faceIndex.find(oldKey);
        if (oldIt != faceIndex.end()) {
            auto& vec = oldIt->second;
            vec.erase(std::remove(vec.begin(), vec.end(), faceIndexPos),
                      vec.end());
            if (vec.empty()) faceIndex.erase(oldIt);
        }
        for (int i = 0; i < 3; ++i) {
            if (f.v[i] == from) {
                f.v[i] = to;
                break;
            }
        }
        faceIndex[orientedKey(f.v[0], f.v[1], f.v[2])].push_back(faceIndexPos);
    }

    /// Undo one collapse: restore both positions, re-create the victim, then
    /// re-create the killed edge faces and re-attribute the remapped fan.
    bool applyRecord(const CollapseRecord& record, std::string* error) {
        const int u = record.survivor;
        const int v = record.victim;
        if (u < 0 || u >= originalVertices || v < 0 || v >= originalVertices) {
            if (error) *error = "artifact split has out-of-range vertex id";
            return false;
        }
        if (!alive[static_cast<std::size_t>(u)]) {
            if (error) *error = "artifact split references a dead survivor";
            return false;
        }
        if (alive[static_cast<std::size_t>(v)]) {
            if (error) *error = "artifact split re-creates a live vertex";
            return false;
        }
        posX[static_cast<std::size_t>(u)] = record.survivorPosBefore[0];
        posY[static_cast<std::size_t>(u)] = record.survivorPosBefore[1];
        posZ[static_cast<std::size_t>(u)] = record.survivorPosBefore[2];
        posX[static_cast<std::size_t>(v)] = record.victimPosBefore[0];
        posY[static_cast<std::size_t>(v)] = record.victimPosBefore[1];
        posZ[static_cast<std::size_t>(v)] = record.victimPosBefore[2];
        alive[static_cast<std::size_t>(v)] = 1;
        ++aliveCount;
        for (const auto& fan : record.fanFaces) {
            if (fan.a == u || fan.b == u) {
                if (!addFace(v, fan.a, fan.b, error)) return false;
            } else {
                const int idx = findFace(u, fan.a, fan.b);
                if (idx < 0) {
                    if (error) {
                        *error = "artifact split cannot locate a remapped face "
                                 "(corrupt fan stream)";
                    }
                    return false;
                }
                replaceFaceVertex(idx, u, v);
            }
        }
        return true;
    }
};

namespace detail_own_format {

struct Header {
    std::uint32_t version = 0;
    std::uint32_t flags = 0;
    std::uint32_t idWidth = 0;
    std::uint32_t geomBytes = 0;
    std::uint32_t originalVertices = 0;
    std::uint32_t originalFaces = 0;
    std::uint32_t baseVertices = 0;
    std::uint32_t baseFaces = 0;
    std::uint32_t rounds = 0;
    std::uint64_t collapses = 0;
    std::uint64_t headerBytes = 0;
};

/// Own-format container header parser (replace with PaperStream metadata reader
/// when M4 schema lands; the semantic streams are unchanged).
inline bool parseOwnFormatHeader(Reader* rd, Header* h, std::string* error) {
    std::uint32_t m0 = 0, m1 = 0;
    if (!rd->u32(&m0) || !rd->u32(&m1)) {
        if (error) *error = "artifact shorter than magic";
        return false;
    }
    if (m0 != kMagic0 || m1 != kMagic1) {
        if (error) *error = "bad artifact magic (expected QEMPROG1)";
        return false;
    }
    rd->u32(&h->version);
    rd->u32(&h->flags);
    rd->u32(&h->idWidth);
    rd->u32(&h->geomBytes);
    rd->u32(&h->originalVertices);
    rd->u32(&h->originalFaces);
    rd->u32(&h->baseVertices);
    rd->u32(&h->baseFaces);
    rd->u32(&h->rounds);
    std::uint32_t reserved = 0;
    rd->u32(&reserved);
    rd->u64(&h->collapses);
    rd->u64(&h->headerBytes);
    if (!rd->ok()) {
        if (error) *error = "truncated artifact header";
        return false;
    }
    if (h->version != kArtifactVersion) {
        if (error) *error = "unsupported artifact version";
        return false;
    }
    if (h->idWidth != 2 && h->idWidth != 4) {
        if (error) *error = "unsupported id width";
        return false;
    }
    if (h->geomBytes != 4 && h->geomBytes != 8) {
        if (error) *error = "unsupported geometry width";
        return false;
    }
    if ((h->flags & kFlagGeomF64) != 0 && h->geomBytes != 8) {
        if (error) *error = "inconsistent geometry flag";
        return false;
    }
    if (h->headerBytes != kHeaderBytes || rd->pos() > h->headerBytes) {
        if (error) *error = "unsupported header size";
        return false;
    }
    return rd->skip(static_cast<std::size_t>(h->headerBytes) - rd->pos());
}

}  // namespace detail_own_format

inline bool decodeProgressiveArtifact(const std::uint8_t* data, std::size_t size,
                                      const DecodeOptions& options,
                                      TriangleMesh* mesh, DecodeStats* stats,
                                      std::string* error) {
    if (stats == nullptr || mesh == nullptr || error == nullptr) return false;
    error->clear();
    Reader rd(data, size);
    detail_own_format::Header h;
    if (!detail_own_format::parseOwnFormatHeader(&rd, &h, error)) return false;

    const int idWidth = static_cast<int>(h.idWidth);
    const bool geomF64 = h.geomBytes == 8;
    const int nv = static_cast<int>(h.originalVertices);
    if (h.baseVertices == 0 || h.baseFaces == 0 ||
        h.baseVertices > h.originalVertices) {
        *error = "invalid artifact base mesh counts";
        return false;
    }

    DecoderState st;
    st.idWidth = idWidth;
    st.geomF64 = geomF64;
    st.originalFaces = static_cast<int>(h.originalFaces);
    st.baseVertices = static_cast<int>(h.baseVertices);
    st.baseFaces = static_cast<int>(h.baseFaces);
    st.declaredRounds = static_cast<int>(h.rounds);
    st.declaredCollapses = static_cast<long long>(h.collapses);
    st.reset(nv);

    *stats = DecodeStats{};
    stats->originalVertices = nv;
    stats->originalFaces = st.originalFaces;
    stats->baseVertices = st.baseVertices;
    stats->baseFaces = st.baseFaces;
    stats->availableRounds = st.declaredRounds;
    stats->geomF64 = geomF64;

    // ---- base mesh ----
    const std::size_t baseVertexBytes =
        static_cast<std::size_t>(h.baseVertices) *
        (24 + static_cast<std::size_t>(idWidth));
    const std::size_t baseFaceBytes =
        static_cast<std::size_t>(h.baseFaces) * 3 *
        static_cast<std::size_t>(idWidth);
    if (rd.remaining() < baseVertexBytes + baseFaceBytes) {
        *error = "truncated artifact base mesh";
        return false;
    }
    for (std::uint32_t i = 0; i < h.baseVertices; ++i) {
        int id = 0;
        double x = 0.0, y = 0.0, z = 0.0;
        if (!rd.id(idWidth, &id) || !rd.f64(&x) || !rd.f64(&y) ||
            !rd.f64(&z)) {
            *error = "truncated artifact base vertices";
            return false;
        }
        if (id < 0 || id >= nv || st.alive[static_cast<std::size_t>(id)]) {
            *error = "base mesh has invalid/duplicate vertex id";
            return false;
        }
        st.posX[static_cast<std::size_t>(id)] = x;
        st.posY[static_cast<std::size_t>(id)] = y;
        st.posZ[static_cast<std::size_t>(id)] = z;
        st.alive[static_cast<std::size_t>(id)] = 1;
        ++st.aliveCount;
    }
    for (std::uint32_t i = 0; i < h.baseFaces; ++i) {
        int a = 0, b = 0, c = 0;
        if (!rd.id(idWidth, &a) || !rd.id(idWidth, &b) ||
            !rd.id(idWidth, &c)) {
            *error = "truncated artifact base faces";
            return false;
        }
        if (!st.addFace(a, b, c, error)) return false;
    }
    stats->meshVertices = st.aliveCount;
    stats->meshFaces = static_cast<int>(st.faces.size());
    stats->lodFaceCounts.push_back(stats->meshFaces);
    stats->lodVertexCounts.push_back(stats->meshVertices);

    // ---- split chunks (reverse simplification round order) ----
    for (int round = 0; round < st.declaredRounds; ++round) {
        if (options.maxAppliedRounds >= 0 &&
            stats->appliedRounds >= options.maxAppliedRounds) {
            stats->stoppedByLod = true;
            break;
        }
        if (rd.remaining() < 8) {
            stats->truncated = true;
            break;
        }
        std::uint32_t payloadBytes = 0, count = 0;
        if (!rd.u32(&payloadBytes) || !rd.u32(&count)) {
            stats->truncated = true;
            break;
        }
        if (rd.remaining() < payloadBytes) {
            stats->truncated = true;  // byte-prefix artifact: drop partial chunk
            break;
        }
        const std::size_t chunkStart = rd.pos();
        std::vector<CollapseRecord> records;
        records.reserve(count);
        bool parseOk = true;
        for (std::uint32_t i = 0; i < count && parseOk; ++i) {
            CollapseRecord rec;
            int fanCount = 0;
            if (!rd.id(idWidth, &rec.survivor) ||
                !rd.id(idWidth, &rec.victim)) {
                parseOk = false;
                break;
            }
            std::uint16_t fc = 0;
            if (!rd.u16(&fc)) {
                parseOk = false;
                break;
            }
            fanCount = fc;
            rec.fanFaces.reserve(static_cast<std::size_t>(fanCount));
            for (int f = 0; f < fanCount && parseOk; ++f) {
                int a = 0, b = 0;
                if (!rd.id(idWidth, &a) || !rd.id(idWidth, &b)) {
                    parseOk = false;
                    break;
                }
                rec.fanFaces.push_back({a, b});
            }
            if (parseOk) records.push_back(std::move(rec));
        }
        std::vector<std::array<double, 6>> geom;
        geom.reserve(count);
        for (std::uint32_t i = 0; parseOk && i < count; ++i) {
            std::array<double, 6> g{};
            if (geomF64) {
                for (int k = 0; k < 6 && parseOk; ++k) {
                    if (!rd.f64(&g[static_cast<std::size_t>(k)])) parseOk = false;
                }
            } else {
                for (int k = 0; k < 6 && parseOk; ++k) {
                    float v = 0.0f;
                    if (!rd.f32(&v)) {
                        parseOk = false;
                        break;
                    }
                    g[static_cast<std::size_t>(k)] = static_cast<double>(v);
                }
            }
            if (parseOk) geom.push_back(g);
        }
        if (!parseOk) {
            stats->truncated = true;
            break;
        }
        for (int i = static_cast<int>(count) - 1; i >= 0; --i) {
            CollapseRecord& rec = records[static_cast<std::size_t>(i)];
            const std::array<double, 6>& g =
                geom[static_cast<std::size_t>(i)];
            rec.survivorPosBefore[0] = g[0];
            rec.survivorPosBefore[1] = g[1];
            rec.survivorPosBefore[2] = g[2];
            rec.victimPosBefore[0] = g[3];
            rec.victimPosBefore[1] = g[4];
            rec.victimPosBefore[2] = g[5];
            if (!st.applyRecord(rec, error)) return false;
        }
        const std::size_t consumed = rd.pos() - chunkStart;
        if (consumed > payloadBytes) {
            *error = "artifact chunk overran its declared size";
            return false;
        }
        if (!rd.skip(payloadBytes - consumed)) {
            stats->truncated = true;
            break;
        }
        ++stats->appliedRounds;
        stats->appliedCollapses += static_cast<int>(count);
        stats->meshVertices = st.aliveCount;
        stats->meshFaces = static_cast<int>(st.faces.size());
        stats->lodFaceCounts.push_back(stats->meshFaces);
        stats->lodVertexCounts.push_back(stats->meshVertices);
        if (options.verbose) {
            std::fprintf(stderr,
                         "[QEMProg] round=%d collapses=%u mesh=(v=%d,f=%d)\n",
                         round, count, st.aliveCount,
                         static_cast<int>(st.faces.size()));
        }
    }

    const bool fullRoundSet =
        stats->appliedRounds == st.declaredRounds && !stats->truncated;
    if (fullRoundSet) {
        if (static_cast<long long>(stats->appliedCollapses) !=
                st.declaredCollapses ||
            stats->meshFaces != st.originalFaces) {
            *error = "full-artifact decode is inconsistent with header counts";
            return false;
        }
    }

    // ---- export (vertices in original-id order) ----
    std::vector<int> outIndex(static_cast<std::size_t>(nv), -1);
    int outVertices = 0;
    for (int id = 0; id < nv; ++id) {
        if (st.alive[static_cast<std::size_t>(id)]) {
            outIndex[static_cast<std::size_t>(id)] = outVertices++;
        }
    }
    mesh->vertices.clear();
    mesh->vertices.reserve(static_cast<std::size_t>(outVertices));
    for (int id = 0; id < nv; ++id) {
        if (outIndex[static_cast<std::size_t>(id)] < 0) continue;
        mesh->vertices.push_back(qem_refstyle::Vec3d{
            st.posX[static_cast<std::size_t>(id)],
            st.posY[static_cast<std::size_t>(id)],
            st.posZ[static_cast<std::size_t>(id)]});
    }
    mesh->faces.clear();
    mesh->faces.reserve(st.faces.size());
    for (const auto& f : st.faces) {
        const int a = outIndex[static_cast<std::size_t>(f.v[0])];
        const int b = outIndex[static_cast<std::size_t>(f.v[1])];
        const int c = outIndex[static_cast<std::size_t>(f.v[2])];
        if (a < 0 || b < 0 || c < 0) {
            *error = "decoded face references a dead vertex";
            return false;
        }
        mesh->faces.push_back(std::array<int, 3>{a, b, c});
    }
    stats->meshVertices = outVertices;
    stats->meshFaces = static_cast<int>(mesh->faces.size());
    return true;
}

inline std::string DecodeStats::summary() const {
    std::ostringstream os;
    os << "QEMProg decode: applied_rounds=" << appliedRounds << "/"
       << availableRounds << " applied_collapses=" << appliedCollapses
       << " mesh=(v=" << meshVertices << ",f=" << meshFaces
       << ") base=(v=" << baseVertices << ",f=" << baseFaces
       << ") original=(v=" << originalVertices << ",f=" << originalFaces
       << ") geom_bits=" << (geomF64 ? 64 : 32)
       << " truncated=" << (truncated ? "true" : "false")
       << " stopped_by_lod=" << (stoppedByLod ? "true" : "false");
    return os.str();
}

}  // namespace qem_prog
}  // namespace pmsf

#endif  // PMSF_IMPL_QEM_QEM_PROGRESSIVE_ARTIFACT_H
