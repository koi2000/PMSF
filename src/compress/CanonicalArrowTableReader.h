#ifndef PMSF_COMPRESS_CANONICAL_ARROW_TABLE_READER_H
#define PMSF_COMPRESS_CANONICAL_ARROW_TABLE_READER_H

#include "core/CanonicalSymbolEncoder.h"
#include "core/SymbolStreamSpec.h"

#include <arrow/api.h>
#include <memory>
#include <vector>

namespace pmsf {

/// Reconstructs a `CanonicalSymbolPayload` from the four codec=4 canonical
/// Arrow tables written by `CanonicalArrowTableBuilder`.
class CanonicalArrowTableReader {
public:
    /// Read the canonical tables back into a payload. Any of the table pointers
    /// may be null; missing side channels are treated as empty.
    static CanonicalSymbolPayload readPayload(
        const arrow::Table& bitTable,
        const arrow::Table& residualTable,
        const arrow::Table* lodTable,
        const arrow::Table* sideTable,
        const std::vector<SymbolStreamSpec>& specs);
};

}  // namespace pmsf

#endif  // PMSF_COMPRESS_CANONICAL_ARROW_TABLE_READER_H
