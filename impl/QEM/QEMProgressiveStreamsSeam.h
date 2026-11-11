/*
 * @file QEMProgressiveStreamsSeam.h   (DRAFT, /tmp only; becomes part of
 *        impl/QEM/QEMProgressivePaperStream.h in the M5-3 integration patch)
 * @brief Container-agnostic seam between the QEM progressive four semantic
 *        streams and the existing own-format decoder.
 *
 * Key design choice (de-risks M5-3): do NOT refactor decodeProgressiveArtifact.
 * Instead, rebuild an in-memory own-format v1 buffer from
 * (baseMeshStream, roundStream, topologyStream, geometryStream, meta) and call
 * the existing decoder on it.  Consequences:
 *   - byte-identical decoder behavior: stats, LOD tables, maxLod/prefix
 *     semantics, truncated/integrity checks all reuse the phase-1-tested path;
 *   - own-format writer/reader (`serializeOwnFormat`/`parseOwnFormatHeader`)
 *     stay untouched -> own-format artifacts and the legacy paths are
 *     byte-unchanged;
 *   - the only new logic here is a topology-record walk (to recover per-round
 *     byte lengths) + the 64-byte header rebuild, both cheap and pure.
 *
 * The PaperStream writer/reader (PSM2 bundle) sits on top of this: it stores
 * the four streams verbatim + the meta block; decode wraps them back into this
 * seam and calls decodeProgressiveArtifact().
 */
#ifndef PMSF_IMPL_QEM_QEM_PROGRESSIVE_STREAMS_SEAM_H
#define PMSF_IMPL_QEM_QEM_PROGRESSIVE_STREAMS_SEAM_H

#include "QEMProgressiveArtifact.h"

#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace pmsf {
namespace qem_prog {

/// Container-independent description of the progressive payload.  Mirrors the
/// own-format v1 header's semantic fields (magic/version/headerBytes/reserved
/// are container concerns and are rebuilt here).
struct StreamMeta {
    int idWidth = 2;        ///< 2 or 4 bytes per vertex id
    int geomBytes = 4;      ///< 4 (f32, default) or 8 (f64 bit-exact mode)
    bool geomF64 = false;   ///< == (geomBytes == 8)
    int originalVertices = 0;
    int originalFaces = 0;
    int baseVertices = 0;
    int baseFaces = 0;
    int roundCount = 0;
    long long totalCollapses = 0;
    int qemVersion = 1;     ///< seam semantic version (independent of container)
};

/// The four verbatim logical streams produced by buildLogicalStreams().
struct SeamStreams {
    std::vector<std::uint8_t> baseMesh;
    std::vector<std::uint8_t> round;
    std::vector<std::uint8_t> topology;
    std::vector<std::uint8_t> geometry;
    StreamMeta meta;
};

namespace seam_detail {

inline std::uint16_t readU16(const std::vector<std::uint8_t>& in,
                             std::size_t* off) {
    const std::uint16_t v = static_cast<std::uint16_t>(in[*off]) |
                            static_cast<std::uint16_t>(in[*off + 1] << 8);
    *off += 2;
    return v;
}

inline std::uint32_t readU32(const std::vector<std::uint8_t>& in,
                             std::size_t* off) {
    std::uint32_t v = 0;
    for (int i = 0; i < 4; ++i) {
        v |= static_cast<std::uint32_t>(in[*off + static_cast<std::size_t>(i)])
             << (8 * i);
    }
    *off += 4;
    return v;
}

inline bool readId(const std::vector<std::uint8_t>& in, std::size_t* off,
                   int idWidth, int* id) {
    if (idWidth == 2) {
        if (*off + 2 > in.size()) return false;
        *id = static_cast<int>(readU16(in, off));
    } else {
        if (*off + 4 > in.size()) return false;
        *id = static_cast<int>(readU32(in, off));
    }
    return true;
}

}  // namespace seam_detail

/// Validate the four streams/meta and rebuild an own-format v1 artifact buffer.
/// Pure function; used by the PaperStream decode seam and by unit tests.
inline bool buildOwnFormatFromStreams(const SeamStreams& streams,
                                      std::vector<std::uint8_t>* artifact,
                                      std::string* error) {
    using namespace seam_detail;
    const StreamMeta& m = streams.meta;
    if (error) error->clear();
    auto fail = [&](const std::string& message) {
        if (error) *error = message;
        return false;
    };
    if (m.qemVersion != 1) return fail("seam: unsupported qemVersion");
    if (m.idWidth != 2 && m.idWidth != 4) return fail("seam: bad idWidth");
    if (m.geomBytes != 4 && m.geomBytes != 8) return fail("seam: bad geomBytes");
    if (m.geomF64 != (m.geomBytes == 8)) {
        return fail("seam: geomF64/geomBytes mismatch");
    }
    if (m.originalVertices <= 0 || m.originalFaces <= 0 ||
        m.baseVertices <= 0 || m.baseFaces <= 0 || m.roundCount < 0 ||
        m.totalCollapses < 0) {
        return fail("seam: invalid meta counts");
    }
    const std::size_t idWidth = static_cast<std::size_t>(m.idWidth);
    const std::size_t expectedBaseBytes =
        static_cast<std::size_t>(m.baseVertices) * (24 + idWidth) +
        static_cast<std::size_t>(m.baseFaces) * 3 * idWidth;
    if (streams.baseMesh.size() != expectedBaseBytes) {
        return fail("seam: baseMeshStream length mismatch");
    }
    const std::size_t expectedRoundBytes =
        static_cast<std::size_t>(m.roundCount) * 4;
    if (streams.round.size() != expectedRoundBytes) {
        return fail("seam: roundStream length mismatch");
    }
    const std::size_t expectedGeometryBytes =
        static_cast<std::size_t>(m.totalCollapses) * 6 *
        static_cast<std::size_t>(m.geomBytes);
    if (streams.geometry.size() != expectedGeometryBytes) {
        return fail("seam: geometryStream length mismatch");
    }

    // ---- round counts + per-round topology byte lengths ----
    std::vector<std::uint32_t> roundCounts(static_cast<std::size_t>(m.roundCount));
    std::vector<std::size_t> topoBytes(static_cast<std::size_t>(m.roundCount));
    std::size_t roundOff = 0;
    for (int r = 0; r < m.roundCount; ++r) {
        roundCounts[static_cast<std::size_t>(r)] = readU32(streams.round, &roundOff);
    }
    long long countedCollapses = 0;
    std::size_t topoOff = 0;
    for (int r = 0; r < m.roundCount; ++r) {
        const std::size_t roundStart = topoOff;
        for (std::uint32_t i = 0; i < roundCounts[static_cast<std::size_t>(r)];
             ++i) {
            int survivor = 0;
            int victim = 0;
            if (!readId(streams.topology, &topoOff, m.idWidth, &survivor) ||
                !readId(streams.topology, &topoOff, m.idWidth, &victim) ||
                topoOff + 2 > streams.topology.size()) {
                return fail("seam: truncated topology record");
            }
            const std::uint16_t fanCount = readU16(streams.topology, &topoOff);
            for (std::uint16_t f = 0; f < fanCount; ++f) {
                int a = 0;
                int b = 0;
                if (!readId(streams.topology, &topoOff, m.idWidth, &a) ||
                    !readId(streams.topology, &topoOff, m.idWidth, &b)) {
                    return fail("seam: truncated topology fan");
                }
            }
            ++countedCollapses;
        }
        topoBytes[static_cast<std::size_t>(r)] = topoOff - roundStart;
    }
    if (topoOff != streams.topology.size()) {
        return fail("seam: topologyStream has trailing bytes");
    }
    if (countedCollapses != m.totalCollapses) {
        return fail("seam: round counts != totalCollapses");
    }

    // ---- rebuild own-format v1 buffer (little-endian, 64 B header) ----
    std::vector<std::uint8_t>* out = artifact;
    out->clear();
    out->reserve(64 + streams.baseMesh.size() +
                 static_cast<std::size_t>(m.roundCount) * 8 +
                 streams.topology.size() + streams.geometry.size());
    // own-format magic "QEMPROG1" + version 1
    out->insert(out->end(), {'Q', 'E', 'M', 'P', 'R', 'O', 'G', '1'});
    putU32(out, 1u);
    putU32(out, m.geomF64 ? 1u : 0u);
    putU32(out, static_cast<std::uint32_t>(m.idWidth));
    putU32(out, static_cast<std::uint32_t>(m.geomBytes));
    putU32(out, static_cast<std::uint32_t>(m.originalVertices));
    putU32(out, static_cast<std::uint32_t>(m.originalFaces));
    putU32(out, static_cast<std::uint32_t>(m.baseVertices));
    putU32(out, static_cast<std::uint32_t>(m.baseFaces));
    putU32(out, static_cast<std::uint32_t>(m.roundCount));
    putU32(out, 0u);  // reserved
    putU64(out, static_cast<std::uint64_t>(m.totalCollapses));
    constexpr std::size_t kOwnHeaderBytes = 64;
    putU64(out, static_cast<std::uint64_t>(kOwnHeaderBytes));
    if (out->size() != kOwnHeaderBytes) {
        return fail("seam: internal header size mismatch");
    }
    out->insert(out->end(), streams.baseMesh.begin(), streams.baseMesh.end());
    std::size_t geoOff = 0;
    for (int r = 0; r < m.roundCount; ++r) {
        const std::uint32_t count = roundCounts[static_cast<std::size_t>(r)];
        const std::size_t geometryBytes =
            static_cast<std::size_t>(count) * 6 *
            static_cast<std::size_t>(m.geomBytes);
        // per-round topology slice offset
        std::size_t topoRoundOff = 0;
        for (int prev = 0; prev < r; ++prev) {
            topoRoundOff += topoBytes[static_cast<std::size_t>(prev)];
        }
        putU32(out, static_cast<std::uint32_t>(
                        topoBytes[static_cast<std::size_t>(r)] + geometryBytes));
        putU32(out, count);
        out->insert(out->end(),
                    streams.topology.begin() +
                        static_cast<std::ptrdiff_t>(topoRoundOff),
                    streams.topology.begin() + static_cast<std::ptrdiff_t>(
                        topoRoundOff + topoBytes[static_cast<std::size_t>(r)]));
        out->insert(out->end(),
                    streams.geometry.begin() + static_cast<std::ptrdiff_t>(geoOff),
                    streams.geometry.begin() + static_cast<std::ptrdiff_t>(
                        geoOff + geometryBytes));
        geoOff += geometryBytes;
    }
    if (out->size() !=
        64 + streams.baseMesh.size() +
            static_cast<std::size_t>(m.roundCount) * 8 +
            streams.topology.size() + streams.geometry.size()) {
        return fail("seam: internal own-format size mismatch");
    }
    return true;
}

/// Decode the four semantic streams (PaperStream container path) by rebuilding
/// the own-format buffer and reusing the phase-1-tested decoder verbatim.
inline bool decodeProgressiveStreams(const SeamStreams& streams,
                                     const DecodeOptions& options,
                                     TriangleMesh* mesh, DecodeStats* stats,
                                     std::string* error) {
    std::vector<std::uint8_t> ownFormat;
    if (!buildOwnFormatFromStreams(streams, &ownFormat, error)) return false;
    return decodeProgressiveArtifact(ownFormat.data(), ownFormat.size(),
                                     options, mesh, stats, error);
}

}  // namespace qem_prog
}  // namespace pmsf

#endif  // PMSF_IMPL_QEM_QEM_PROGRESSIVE_STREAMS_SEAM_H
