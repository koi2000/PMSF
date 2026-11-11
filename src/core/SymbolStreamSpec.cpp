#include "core/SymbolStreamSpec.h"

namespace pmsf {

SymbolStreamRegistry& SymbolStreamRegistry::instance() {
    static SymbolStreamRegistry inst;
    return inst;
}

SymbolStreamRegistry::SymbolStreamRegistry() {
    residualOnly_.push_back({"residual", SymbolStreamRole::RemovedVertexCoord,
                             SideChannelSpec::Type::Bytes,
                             SideChannelSpec::Shape::Scalar, 0, false});
}

void SymbolStreamRegistry::registerLayout(std::int32_t blobLayout,
                                          std::vector<SymbolStreamSpec> specs) {
    map_[blobLayout] = std::move(specs);
}

const std::vector<SymbolStreamSpec>& SymbolStreamRegistry::specsFor(
    std::int32_t blobLayout) const {
    auto it = map_.find(blobLayout);
    if (it != map_.end()) return it->second;
    return residualOnly_;
}

void SymbolStreamRegistry::registerClassI(std::int32_t blobLayout) {
    std::vector<SymbolStreamSpec> specs;
    specs.push_back({"facet", SymbolStreamRole::Facet,
                     SideChannelSpec::Type::Int32,
                     SideChannelSpec::Shape::Scalar, 1, true});
    specs.push_back({"edge", SymbolStreamRole::TriangulationEdge,
                     SideChannelSpec::Type::Int32,
                     SideChannelSpec::Shape::Scalar, 1, true});
    specs.push_back({"vertex", SymbolStreamRole::Vertex,
                     SideChannelSpec::Type::Int32,
                     SideChannelSpec::Shape::Scalar, 1, true});
    specs.push_back({"point", SymbolStreamRole::RemovedVertexCoord,
                     SideChannelSpec::Type::Vec3f,
                     SideChannelSpec::Shape::Vec3, 0, false});
    specs.push_back({"vertex_point", SymbolStreamRole::ShiftedVertexCoord,
                     SideChannelSpec::Type::Vec3f,
                     SideChannelSpec::Shape::Vec3, 0, false});
    registerLayout(blobLayout, std::move(specs));
}

void SymbolStreamRegistry::registerResidualOnly(std::int32_t blobLayout) {
    registerLayout(blobLayout, residualOnly_);
}

}  // namespace pmsf
