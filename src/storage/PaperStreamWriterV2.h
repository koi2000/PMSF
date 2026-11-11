/*
 * @file PaperStreamWriterV2.h
 * @brief PaperStream v2 writer (M4-2, opt-in).
 *
 * ADDITIVE: the v1 bridge writer (PaperStreamWriter.{h,cpp}) is untouched. This
 * writer only accepts PaperObjectRow::formatVersion == kPaperStreamFormatVersionV2.
 * Compact mode retains the original single-file v2 schema:
 *
 *   record_type | object_id | partition_id | lod_id
 *   header | base_mesh | connectivity | geometry | extensions
 *   native_payload | boundary
 *
 * Typed physical mode (object.schemaMode=Typed) replaces the connectivity /
 * geometry bundles with F/Et/Ef/V encoded stream columns and Cf/Cv float32 lists,
 * CfQ/CvQ int32 lists, six per-LOD counters and payload offset/length columns.
 * Float components use BYTE_STREAM_SPLIT + ZSTD; integers DELTA_BINARY_PACKED +
 * ZSTD. Native/extension streams remain self-described in the same artifact.
 *
 * Properties:
 *   - row group 0 = object row; then partition row groups (RG per
 *     `partitionsPerRowGroup` partitions; LOD rows ascending by decode-order
 *     lod_id); optional boundary RG last;
 *   - `lodsPerRowGroup > 0` splits a partition into LOD blocks (prefix profile)
 *     and records the first RG index per partition in the v2 dictionary;
 *   - native payload bytes are written exactly once (NULL for typed rows);
 *   - deterministic: same input + options => byte-identical file;
 *   - fail-closed: validates the v2 table contract before writing anything.
 */
#ifndef PMSF_STORAGE_PAPER_STREAM_WRITER_V2_H
#define PMSF_STORAGE_PAPER_STREAM_WRITER_V2_H

#include "storage/PaperStreamTable.h"

#include <cstdint>
#include <string>
#include <vector>

namespace pmsf {

/// @brief Physical container profile for a v2 artifact (M5-1 zero-overhead).
///        Columns = regular 11-column schema (default, M4 contract).
///        Blob    = single `paper_blob` BINARY column for whole-frame native
///                  artifacts (content-identical payload, minimal framing).
enum class PaperV2Container { Columns, Blob };

struct PaperWriterV2Options {
    /// Partitions per row group (1 = one partition per RG, the default).
    std::int32_t partitionsPerRowGroup = 1;
    /// LODs per row group: 0 = all LODs of a partition in its RG. >0 splits the
    /// partition into LOD blocks for prefix random access.
    std::int32_t lodsPerRowGroup = 0;
    /// Parquet data page size for the bundle columns.
    std::int64_t dataPageSize = 64 * 1024;
    /// Physical container profile; Blob requires payloadMode=Native with a
    /// whole-frame payload (otherwise write() fails closed).
    PaperV2Container container = PaperV2Container::Columns;
    /// Blob container provenance (M5-2): 0=raw payload,1=SBC1,2=SBC1-zstd;
    /// base: 0=compact-bin envelope bytes,1=raw V2CP payload (smaller).
    std::uint8_t blobPayloadCodec = 0;
    std::uint8_t blobBaseCodec = 0;
};

class PaperStreamWriterV2 {
  public:
    /// @brief Write a v2 PaperStreamTable to a single `<...>.parquet` artifact.
    /// @throws std::runtime_error on schema/contract/serialization/IO failure.
    void write(const PaperStreamTable& table, const std::string& path,
               const PaperWriterV2Options& options = {}) const;

    /// @brief Convert encoder-order LOD rows (row i = encoder round i,
    ///        fine->coarse) into the paper v2 decode order
    ///        (lod_id = numRounds-1-i), then sort ascending.
    static void assignDecodeOrderLodIds(std::vector<PaperLodRow>& rows,
                                        std::int32_t numRounds);
};

}  // namespace pmsf

#endif  // PMSF_STORAGE_PAPER_STREAM_WRITER_V2_H
