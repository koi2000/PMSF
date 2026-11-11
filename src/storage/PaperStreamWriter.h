/*
 * @file PaperStreamWriter.h
 * @brief PaperStream v1 Parquet writer (M1).
 *
 * Produces one <base>_paper.parquet with one row group per logical relation
 * instance: RG0 = object, RG1..P = partition + its LOD rows, optional boundary
 * RG.  The reader relies on this layout for prefix access.
 */
#ifndef PMSF_STORAGE_PAPER_STREAM_WRITER_H
#define PMSF_STORAGE_PAPER_STREAM_WRITER_H

#include "storage/PaperStreamTable.h"

#include <string>

namespace pmsf {

class PaperStreamWriter {
  public:
    /// @brief Write the table to a single Parquet file.
    /// @throws std::runtime_error on schema/serialization/IO failure.
    void write(const PaperStreamTable& table, const std::string& path) const;
};

}  // namespace pmsf

#endif  // PMSF_STORAGE_PAPER_STREAM_WRITER_H
