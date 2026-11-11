/*
 * @file QEMProgressivePaperStream.h
 * @brief QEM progressive four-stream <-> PaperStream v2 multi-stream artifact
 *        seam (M5-3). Thin adapter; no decode logic lives here.
 *
 * Storage side (storage-architect, src/storage):
 *   PaperStreamMultiStreamV2.h            PSM2 bundle + zlib codecs (portable)
 *   PaperStreamMultiStreamArtifactV2.h    P2BL blob artifact writer/reader
 *     writePaperStreamV2MultiStreamArtifact(path, streams, meta, options, err)
 *     readPaperStreamV2MultiStreamArtifact(path, &streams, &meta, err)
 *
 * QEM side (this header + QEMProgressiveStreamsSeam.h):
 *   engine -> buildLogicalStreams() -> four semantic streams                            (encode)
 *   four streams -> buildOwnFormatFromStreams() -> decodeProgressiveArtifact()          (decode)
 *
 * Container identity: the PSM2 streams are the *verbatim* logical streams
 * (baseMesh/round/topology/geometry); the own-format writer/reader and all
 * default paths stay untouched.  Env gates (see tools/QEMFaceBudget.cpp):
 *   encode: PMSF_QEM_REF_ENGINE=1 PMSF_QEM_PROG_PAPER_ARTIFACT=<out.parquet>
 *   decode: PMSF_QEM_PROG_PAPER_DECODE=1 QEMFaceBudget <artifact.parquet> <out.off> <maxLod>
 */
#ifndef PMSF_IMPL_QEM_QEM_PROGRESSIVE_PAPER_STREAM_H
#define PMSF_IMPL_QEM_QEM_PROGRESSIVE_PAPER_STREAM_H

#include "QEMProgressiveStreamsSeam.h"

#include "storage/PaperStreamMultiStreamArtifactV2.h"

#include <string>
#include <vector>

namespace pmsf {
namespace qem_paper {

namespace psm2 = pmsf::paper_multistream;

inline constexpr std::uint8_t kStreamBaseMesh = psm2::kStreamBaseMesh;
inline constexpr std::uint8_t kStreamRound = psm2::kStreamRound;
inline constexpr std::uint8_t kStreamTopology = psm2::kStreamTopology;
inline constexpr std::uint8_t kStreamGeometry = psm2::kStreamGeometry;

/// ArtifactStreams + EncodeStats (engine side) -> seam streams/meta.
inline qem_prog::SeamStreams seamStreamsFromArtifactStreams(
    const qem_prog::ArtifactStreams& streams,
    const qem_prog::EncodeStats& stats) {
    qem_prog::SeamStreams out;
    out.baseMesh = streams.baseMeshStream;   // verbatim buildLogicalStreams() bytes
    out.round = streams.roundStream;
    out.topology = streams.topologyStream;
    out.geometry = streams.geometryStream;
    out.meta.qemVersion = 1;
    out.meta.idWidth = stats.idWidthBytes;
    out.meta.geomBytes = stats.geomBytesPerComp;
    out.meta.geomF64 = stats.geomBytesPerComp == 8;
    out.meta.originalVertices = stats.originalVertices;
    out.meta.originalFaces = stats.originalFaces;
    out.meta.baseVertices = stats.baseVertices;
    out.meta.baseFaces = stats.baseFaces;
    out.meta.roundCount = stats.rounds;
    out.meta.totalCollapses = stats.collapses;
    return out;
}

inline psm2::QemBundleMeta bundleMetaFromSeamMeta(const qem_prog::StreamMeta& meta) {
    psm2::QemBundleMeta out;
    out.qemVersion = static_cast<std::uint16_t>(meta.qemVersion);
    out.idWidth = static_cast<std::uint16_t>(meta.idWidth);
    out.geomBytes = static_cast<std::uint16_t>(meta.geomBytes);
    out.reserved = 0;
    out.flags = (meta.geomF64 ? psm2::kQemFlagGeomF64 : 0u);
    out.originalVertices = static_cast<std::uint32_t>(meta.originalVertices);
    out.originalFaces = static_cast<std::uint32_t>(meta.originalFaces);
    out.baseVertices = static_cast<std::uint32_t>(meta.baseVertices);
    out.baseFaces = static_cast<std::uint32_t>(meta.baseFaces);
    out.roundCount = static_cast<std::uint32_t>(meta.roundCount);
    out.totalCollapses = static_cast<std::uint32_t>(meta.totalCollapses);
    return out;
}

inline qem_prog::StreamMeta seamMetaFromBundleMeta(const psm2::QemBundleMeta& meta) {
    qem_prog::StreamMeta out;
    out.qemVersion = meta.qemVersion;
    out.idWidth = static_cast<int>(meta.idWidth);
    out.geomBytes = static_cast<int>(meta.geomBytes);
    out.geomF64 = (meta.flags & psm2::kQemFlagGeomF64) != 0 ||
                  meta.geomBytes == 8;
    out.originalVertices = static_cast<int>(meta.originalVertices);
    out.originalFaces = static_cast<int>(meta.originalFaces);
    out.baseVertices = static_cast<int>(meta.baseVertices);
    out.baseFaces = static_cast<int>(meta.baseFaces);
    out.roundCount = static_cast<int>(meta.roundCount);
    out.totalCollapses = static_cast<long long>(meta.totalCollapses);
    return out;
}

inline std::vector<psm2::SeamStream> makeSeamStreams(
    const qem_prog::SeamStreams& streams) {
    return {
        psm2::SeamStream{kStreamBaseMesh, "base_mesh", streams.baseMesh},
        psm2::SeamStream{kStreamRound, "round", streams.round},
        psm2::SeamStream{kStreamTopology, "topology", streams.topology},
        psm2::SeamStream{kStreamGeometry, "geometry", streams.geometry},
    };
}

/// Encode: four verbatim streams -> coded PSM2 bundle -> P2BL blob artifact.
inline bool writeQemPaperArtifact(const std::string& path,
                                  const qem_prog::SeamStreams& streams,
                                  const psm2::QemPaperOptions& options,
                                  std::string* error) {
    return psm2::writePaperStreamV2MultiStreamArtifact(
        path, makeSeamStreams(streams),
        bundleMetaFromSeamMeta(streams.meta), options, error);
}

/// Read a fused artifact back into seam streams/meta (validated by storage).
inline bool readQemPaperArtifact(const std::string& path,
                                 qem_prog::SeamStreams* streams,
                                 psm2::QemBundleMeta* meta,
                                 std::string* error) {
    std::vector<psm2::SeamStream> rawStreams;
    if (!psm2::readPaperStreamV2MultiStreamArtifact(path, &rawStreams, meta,
                                                    error)) {
        return false;
    }
    qem_prog::SeamStreams out;
    for (const auto& stream : rawStreams) {
        switch (stream.id) {
            case kStreamBaseMesh: out.baseMesh = stream.bytes; break;
            case kStreamRound: out.round = stream.bytes; break;
            case kStreamTopology: out.topology = stream.bytes; break;
            case kStreamGeometry: out.geometry = stream.bytes; break;
            default: break;  // forward-compatible: skip unknown stream ids
        }
    }
    if (out.baseMesh.empty() || out.round.empty() || out.topology.empty() ||
        out.geometry.empty()) {
        if (error) *error = "QEM paper artifact: missing one of the four streams";
        return false;
    }
    out.meta = seamMetaFromBundleMeta(*meta);
    *streams = std::move(out);
    return true;
}

/// Decode a fused artifact: four streams -> rebuilt own-format buffer -> the
/// phase-1-tested progressive decoder (identical stats/LOD/prefix semantics).
inline bool decodeQemPaperArtifact(const std::string& path,
                                   const qem_prog::DecodeOptions& options,
                                   qem_prog::TriangleMesh* mesh,
                                   qem_prog::DecodeStats* stats,
                                   std::string* error) {
    qem_prog::SeamStreams streams;
    psm2::QemBundleMeta meta;
    if (!readQemPaperArtifact(path, &streams, &meta, error)) return false;
    return qem_prog::decodeProgressiveStreams(streams, options, mesh, stats,
                                              error);
}

}  // namespace qem_paper
}  // namespace pmsf

#endif  // PMSF_IMPL_QEM_QEM_PROGRESSIVE_PAPER_STREAM_H
