/*
 * @file PaperStreamReader.h
 * @brief PaperStream v1 Parquet reader (M1).
 *
 * Layout contract (written by PaperStreamWriter):
 *   row group 0        : object row
 *   row group 1..P     : partition row + its LOD rows (lod_id ascending)
 *   optional last RG   : boundary rows
 */
#ifndef PMSF_STORAGE_PAPER_STREAM_READER_H
#define PMSF_STORAGE_PAPER_STREAM_READER_H

#include "storage/PaperStreamTable.h"

#include <cstdint>
#include <memory>
#include <string>

namespace parquet {
class ParquetFileReader;
}
namespace parquet {
namespace arrow {
    class FileReader;
}
}  // namespace parquet

namespace pmsf {

class PaperStreamReader {
  public:
    explicit PaperStreamReader(const std::string& path);
    ~PaperStreamReader();

    PaperStreamReader(const PaperStreamReader&) = delete;
    PaperStreamReader& operator=(const PaperStreamReader&) = delete;

    /// @brief Read every logical relation.
    PaperStreamTable readAll();

    /// @brief Read object + partition p + LOD rows with lod_id < maxLod.
    /// @param partitionId partition id as written (M1 assumes vector index)
    /// @param maxLod exclusive upper bound; < 0 means all
    PaperStreamTable readPrefix(int partitionId, int maxLod);

    int numRowGroups() const;
    std::uint64_t bytesRead() const {
        return bytesRead_;
    }
    std::uint64_t rowGroupsRead() const {
        return rowGroupsRead_;
    }

  private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    std::uint64_t bytesRead_ = 0;
    std::uint64_t rowGroupsRead_ = 0;
};

}  // namespace pmsf

#endif  // PMSF_STORAGE_PAPER_STREAM_READER_H
