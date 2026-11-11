/*
 * @file PaperStreamReaderV2.h
 * @brief PaperStream v2 reader with v1/v2 dispatch (M4-2, opt-in).
 *
 * ADDITIVE: the v1 bridge reader (PaperStreamReader.{h,cpp}) is untouched.
 *   - artifacts whose object-row dictionary says version 2 are decoded by the
 *     v2 path (compact v2 schema, PPTH v2 partition headers, native payload
 *     directory, RG-level random access);
 *   - artifacts whose dictionary says version 1 fall back to the untouched v1
 *     reader, so v1 read behavior is byte-identical by construction.
 *
 * Random access (M4-2 scope):
 *   readPrefix(partitionId(s), maxLod) reads only the object row group plus the
 *   target partitions' row groups (directory from the v2 dictionary; LOD blocks
 *   when the artifact was written with lodsPerRowGroup > 0), filters rows by
 *   lod_id < maxLod and reports bytesRead/rowGroupsRead/footerBytes. Page-level
 *   skipping is M6; granularity() honestly reports "partition" or "lod_block".
 *   "lod" behavior on decoded rows (prefix table == full table truncated) is
 *   equivalent in both profiles.
 */
#ifndef PMSF_STORAGE_PAPER_STREAM_READER_V2_H
#define PMSF_STORAGE_PAPER_STREAM_READER_V2_H

#include "storage/PaperStreamTable.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace pmsf {

class PaperStreamReaderV2 {
  public:
    explicit PaperStreamReaderV2(const std::string& path);
    ~PaperStreamReaderV2();

    PaperStreamReaderV2(const PaperStreamReaderV2&) = delete;
    PaperStreamReaderV2& operator=(const PaperStreamReaderV2&) = delete;

    /// @brief Read every logical relation.
    PaperStreamTable readAll();

    /// @brief Read object + partition p + LOD rows with lod_id < maxLod.
    PaperStreamTable readPrefix(int partitionId, int maxLod);

    /// @brief ROI variant: union of the selected partitions (partition_id is
    ///        matched through the v2 dictionary directory; ids need not be
    ///        contiguous).
    PaperStreamTable readPrefix(const std::vector<int>& partitionIds, int maxLod);

    int numRowGroups() const;
    bool usedV1Fallback() const;
    const PaperObjectRow* v2Object() const;  ///< null for a v1 artifact
    /// @brief M5-3: true when the artifact uses the single-blob container profile.
    bool isBlobContainer() const;
    /// @brief P2BL payload codec id (0 raw,1 SBC1-zlib,2 SBC1-zstd,3 zlib-raw).
    std::uint8_t blobPayloadCodec() const;
    /// @brief P2BL base codec id (0 envelope,1 raw V2CP,2 none).
    std::uint8_t blobBaseCodec() const;
    /// @brief Honest random-access granularity: "lod" (row-level filtering of a
    ///        fully read partition/RG), "lod_block" or "partition" for the v2
    ///        RG profiles, "v1" for the v1 fallback reader.
    std::string granularity() const;

    std::uint64_t bytesRead() const { return bytesRead_; }
    std::uint64_t rowGroupsRead() const { return rowGroupsRead_; }
    std::uint64_t pagesRead() const { return pagesRead_; }
    std::uint64_t footerBytes() const { return footerBytes_; }

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    std::uint64_t bytesRead_ = 0;
    std::uint64_t rowGroupsRead_ = 0;
    std::uint64_t pagesRead_ = 0;
    std::uint64_t footerBytes_ = 0;
    PaperStreamTable readPrefixV2(const std::vector<int>& partitionIds, int maxLod);
};

}  // namespace pmsf

#endif  // PMSF_STORAGE_PAPER_STREAM_READER_V2_H
