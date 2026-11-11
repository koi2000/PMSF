/*
 * @file PaperStreamOpaquePayloadV2.h
 * @brief PaperStream v2 opaque-payload artifact helper (M5-5).
 *
 * Some codecs already own a byte-exact container payload (VQMMC binary container:
 * 777 B binary header/base + 63,733 B CONN layout-30 container). M5-5 needs to
 * carry that payload verbatim inside a PaperStream v2 `_paper.parquet` without
 * re-encoding the protocol. This helper provides the minimal container glue:
 *
 *   opaque payload -> payload codec (raw/zlib/SBC1) -> P2BL blob container
 *                  -> single `_paper.parquet`
 *
 * Decode returns the exact payload bytes + codec id; the algorithm's own
 * deserializer (e.g. VQMMC strict binary path) consumes them. This does NOT go
 * through PaperDeserializeOperator / initV2 and never touches default paths.
 */
#ifndef PMSF_STORAGE_PAPER_STREAM_OPAQUE_PAYLOAD_V2_H
#define PMSF_STORAGE_PAPER_STREAM_OPAQUE_PAYLOAD_V2_H

#include "storage/PaperStreamMultiStreamV2.h"

#include <cstdint>
#include <string>
#include <vector>

namespace pmsf {
namespace paper_opaque {

struct OpaquePaperArtifactSpec {
    std::string algorithm;       // e.g. "vqmmc"
    std::int32_t blobLayout = 0; // provenance only (VQMMC canonical: 30)
    std::int32_t codecVersion = 0;
    std::int32_t schemeVersion = 0;
    std::int32_t rounds = 0;
    std::int32_t totalVertices = 0;
    std::int32_t totalFaces = 0;
    std::int32_t baseVertices = 0;
    std::int32_t baseFaces = 0;
    std::string boundaryCodec;   // empty for VQMMC
};

/// @brief Write `payload` (verbatim after the selected codec) into a PaperStream
///        v2 blob artifact with baseCodec=None.
bool writePaperStreamV2OpaquePayloadArtifact(
    const std::string& path, const std::vector<std::uint8_t>& payload,
    const OpaquePaperArtifactSpec& spec,
    paper_multistream::PayloadCodec codec, std::string* error = nullptr);

/// @brief Read the artifact back; returns verbatim payload bytes + codec id.
bool readPaperStreamV2OpaquePayloadArtifact(
    const std::string& path, std::vector<std::uint8_t>* payload,
    paper_multistream::PayloadCodec* codec, std::string* error = nullptr);

}  // namespace paper_opaque
}  // namespace pmsf

#endif  // PMSF_STORAGE_PAPER_STREAM_OPAQUE_PAYLOAD_V2_H
