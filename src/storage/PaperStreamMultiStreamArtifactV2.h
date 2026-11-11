/*
 * @file PaperStreamMultiStreamArtifactV2.h
 * @brief PaperStream v2 artifact writer/reader for PSM2 multi-stream bundles
 *        (M5-3 QEM progressive fusion).
 *
 * The portable bundle/codec helpers live in `PaperStreamMultiStreamV2.h`; this
 * header only declares the Arrow/Parquet container glue:
 *   PSM2 bundle -> payload codec -> P2BL blob container -> single `_paper.parquet`
 *   `_paper.parquet` -> P2BL blob container -> payload codec -> PSM2 bundle
 *
 * Decode does NOT go through PaperDeserializeOperator / initV2: progressive
 * codecs consume the four streams directly (their own decoder).
 * Implemented in `PaperStreamMultiStreamArtifactV2.cpp` so QEM tools can link
 * the TU without pulling Arrow headers into their own translation units.
 */
#ifndef PMSF_STORAGE_PAPER_STREAM_MULTI_STREAM_ARTIFACT_V2_H
#define PMSF_STORAGE_PAPER_STREAM_MULTI_STREAM_ARTIFACT_V2_H

#include "storage/PaperStreamMultiStreamV2.h"

#include <string>
#include <vector>

namespace pmsf {
namespace paper_multistream {

/// @brief Write a P2BL artifact whose native payload is the coded PSM2 bundle.
/// @param meta QEM progressive metadata (validated against stream lengths).
/// @throws nothing; returns false with `error` on contract/IO failure.
bool writePaperStreamV2MultiStreamArtifact(
    const std::string& path, const std::vector<SeamStream>& streams,
    const QemBundleMeta& meta, const QemPaperOptions& options,
    std::string* error = nullptr);

/// @brief Read a P2BL artifact back into the four streams + meta.
bool readPaperStreamV2MultiStreamArtifact(
    const std::string& path, std::vector<SeamStream>* streams,
    QemBundleMeta* meta, std::string* error = nullptr);

}  // namespace paper_multistream
}  // namespace pmsf

#endif  // PMSF_STORAGE_PAPER_STREAM_MULTI_STREAM_ARTIFACT_V2_H
