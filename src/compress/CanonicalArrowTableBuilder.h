#ifndef PMSF_COMPRESS_CANONICAL_ARROW_TABLE_BUILDER_H
#define PMSF_COMPRESS_CANONICAL_ARROW_TABLE_BUILDER_H

#include "core/CanonicalSymbolEncoder.h"

#include <arrow/api.h>
#include <memory>
#include <vector>

namespace pmsf {

/// Wraps a CanonicalSymbolPayload into the four codec=4 canonical Arrow tables:
///   _bit, _residual, _lod, _side.
class CanonicalArrowTableBuilder {
public:
    explicit CanonicalArrowTableBuilder(const CanonicalSymbolPayload& payload);

    std::shared_ptr<arrow::Table> buildBitTable() const;
    std::shared_ptr<arrow::Table> buildResidualTable() const;
    std::shared_ptr<arrow::Table> buildLodTable() const;
    std::shared_ptr<arrow::Table> buildSideTable() const;

private:
    const CanonicalSymbolPayload& payload_;
};

}  // namespace pmsf

#endif  // PMSF_COMPRESS_CANONICAL_ARROW_TABLE_BUILDER_H
