#ifndef PMSF_CORE_SYMBOL_STREAM_SPEC_H
#define PMSF_CORE_SYMBOL_STREAM_SPEC_H

#include "core/Symbol.h"
#include "storage/PaperBlobLayoutRegistry.h"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace pmsf {

/// Semantic role of a symbol stream in the canonical codec=4 storage.
/// The first six values are exactly the paper core channels; values >= 6 are
/// side-channels whose concrete interpretation is given by the stream name.
enum class SymbolStreamRole : std::uint8_t {
    Facet = 0,              ///< F_r  (per-visited-facet split flag / code)
    TriangulationEdge = 1,  ///< E_r^t (per-edge inserted/original flag)
    FlipEdge = 2,           ///< E_r^f (edge-flip adjustment flags)
    Vertex = 3,             ///< V_r  (per-visited-vertex adjustment flag / code)
    RemovedVertexCoord = 4, ///< C_r^f (geometry of removed vertices)
    ShiftedVertexCoord = 5, ///< C_r^v (geometry of adjusted vertices)
    SideChannel = 6,        ///< algorithm-specific side channel
    _Count
};

/// Optional entropy-coding context model for a scalar stream.
enum class SymbolContextModel : std::uint8_t {
    None = 0,    ///< single adaptive qsmodel (default)
    Valence4 = 1 ///< 4-context valence model (alphabet must be 8)
};

/// Static description of one canonical stream.
struct SymbolStreamSpec {
    std::string name;                         ///< e.g. "facet", "edge", "valence_code"
    SymbolStreamRole role = SymbolStreamRole::SideChannel;
    SideChannelSpec::Type cellType = SideChannelSpec::Type::Int32;
    SideChannelSpec::Shape shape = SideChannelSpec::Shape::Scalar;
    int alphabetBits = 8;                     ///< for scalar entropy coder; 0 = opaque bytes
    bool encodeWithRangeCoder = true;         ///< false = store raw bytes / bit-packed
    SymbolContextModel contextModel = SymbolContextModel::None;
};

/// Registry that maps each BlobLayout to its canonical stream layout.
class SymbolStreamRegistry {
public:
    static SymbolStreamRegistry& instance();

    /// Register the canonical streams for a BlobLayout.
    void registerLayout(std::int32_t blobLayout, std::vector<SymbolStreamSpec> specs);

    /// Return the canonical streams for a BlobLayout.
    /// Falls back to a single opaque residual stream if not registered.
    const std::vector<SymbolStreamSpec>& specsFor(std::int32_t blobLayout) const;

    /// Convenience: register the default Class-I layout (facet/edge/vertex/point).
    void registerClassI(std::int32_t blobLayout);

    /// Convenience: register a residual-only fallback layout.
    void registerResidualOnly(std::int32_t blobLayout);

private:
    SymbolStreamRegistry();
    std::unordered_map<std::int32_t, std::vector<SymbolStreamSpec>> map_;
    std::vector<SymbolStreamSpec> residualOnly_;
};

}  // namespace pmsf

#endif  // PMSF_CORE_SYMBOL_STREAM_SPEC_H
