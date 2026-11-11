#include "core/SymbolStreamSpec.h"
#include "common/PaperLayoutAliases.h"

namespace pmsf {

namespace {

struct CanonicalRegistrar {
    CanonicalRegistrar() {
        auto& reg = SymbolStreamRegistry::instance();

        // Class-I layouts: the blob already splits into facet/edge/vertex/point.
        reg.registerClassI(kPaperBlobLayoutRangeCodedPred);
        reg.registerClassI(kPaperBlobLayoutOptimizedPlain);
        reg.registerClassI(kPaperBlobLayoutDefaultPPMCLite);

        // Canonical codec=4 wrapper layout carries the same facet/edge/vertex/point
        // decomposition; the original algorithm-specific layout is preserved in
        // the metadata table but the symbol payload itself uses the Class-I spec.
        reg.registerClassI(kPaperBlobLayoutCanonical);

        // Migrated to canonical facet/edge/vertex/point + side-channel specs.
        {
            std::vector<SymbolStreamSpec> rdTagSpecs;
            rdTagSpecs.push_back({"facet", SymbolStreamRole::Facet,
                                  SideChannelSpec::Type::Int32,
                                  SideChannelSpec::Shape::Scalar, 3, true,
                                  SymbolContextModel::Valence4});
            rdTagSpecs.push_back({"edge", SymbolStreamRole::TriangulationEdge,
                                  SideChannelSpec::Type::Int32,
                                  SideChannelSpec::Shape::Scalar, 1, true});
            rdTagSpecs.push_back({"point", SymbolStreamRole::RemovedVertexCoord,
                                  SideChannelSpec::Type::Vec3f,
                                  SideChannelSpec::Shape::Vec3, 0, false});
            rdTagSpecs.push_back({"rd_tag/valence_other",
                                  SymbolStreamRole::SideChannel,
                                  SideChannelSpec::Type::Int8,
                                  SideChannelSpec::Shape::Scalar, 8, true});
            reg.registerLayout(kPaperBlobLayoutRDTag, std::move(rdTagSpecs));
        }
        {
            std::vector<SymbolStreamSpec> pclttmSpecs;
            pclttmSpecs.push_back({"facet", SymbolStreamRole::Facet,
                                   SideChannelSpec::Type::Int32,
                                   SideChannelSpec::Shape::Scalar, 2, true});
            pclttmSpecs.push_back({"point", SymbolStreamRole::RemovedVertexCoord,
                                   SideChannelSpec::Type::Vec3f,
                                   SideChannelSpec::Shape::Vec3, 0, false});
            pclttmSpecs.push_back({"pclttm_tag/valence",
                                   SymbolStreamRole::SideChannel,
                                   SideChannelSpec::Type::Int8,
                                   SideChannelSpec::Shape::Scalar, 3, true,
                                   SymbolContextModel::Valence4});
            pclttmSpecs.push_back({"pclttm_tag/valence_other",
                                   SymbolStreamRole::SideChannel,
                                   SideChannelSpec::Type::Int8,
                                   SideChannelSpec::Shape::Scalar, 8, true});
            reg.registerLayout(kPaperBlobLayoutPCLTTMTag,
                               std::move(pclttmSpecs));
        }
        {
            std::vector<SymbolStreamSpec> pcSpecs;
            pcSpecs.push_back({"facet", SymbolStreamRole::Facet,
                               SideChannelSpec::Type::Int32,
                               SideChannelSpec::Shape::Scalar, 3, true,
                               SymbolContextModel::Valence4});
            pcSpecs.push_back({"edge", SymbolStreamRole::TriangulationEdge,
                               SideChannelSpec::Type::Int32,
                               SideChannelSpec::Shape::Scalar, 1, true});
            pcSpecs.push_back({"vertex", SymbolStreamRole::Vertex,
                               SideChannelSpec::Type::Int32,
                               SideChannelSpec::Shape::Scalar, 1, true});
            pcSpecs.push_back({"flip", SymbolStreamRole::FlipEdge,
                               SideChannelSpec::Type::Int32,
                               SideChannelSpec::Shape::Scalar, 1, true});
            pcSpecs.push_back({"point", SymbolStreamRole::RemovedVertexCoord,
                               SideChannelSpec::Type::Vec3f,
                               SideChannelSpec::Shape::Vec3, 0, false});
            pcSpecs.push_back({"pcatm_color/valence_other",
                               SymbolStreamRole::SideChannel,
                               SideChannelSpec::Type::Int8,
                               SideChannelSpec::Shape::Scalar, 8, true});
            reg.registerLayout(kPaperBlobLayoutPCATMColor, std::move(pcSpecs));
        }
        {
            std::vector<SymbolStreamSpec> zcSpecs;
            zcSpecs.push_back({"facet", SymbolStreamRole::Facet,
                               SideChannelSpec::Type::Int32,
                               SideChannelSpec::Shape::Scalar, 3, true,
                               SymbolContextModel::Valence4});
            zcSpecs.push_back({"edge", SymbolStreamRole::TriangulationEdge,
                               SideChannelSpec::Type::Int32,
                               SideChannelSpec::Shape::Scalar, 1, true});
            zcSpecs.push_back({"flip", SymbolStreamRole::FlipEdge,
                               SideChannelSpec::Type::Int32,
                               SideChannelSpec::Shape::Scalar, 1, true});
            zcSpecs.push_back({"point", SymbolStreamRole::RemovedVertexCoord,
                               SideChannelSpec::Type::Vec3f,
                               SideChannelSpec::Shape::Vec3, 0, false});
            zcSpecs.push_back({"patch_color/valence_other",
                               SymbolStreamRole::SideChannel,
                               SideChannelSpec::Type::Int8,
                               SideChannelSpec::Shape::Scalar, 8, true});
            reg.registerLayout(kPaperBlobLayoutPCATMZColor, zcSpecs);
            reg.registerLayout(kPaperBlobLayoutPCATMPaper, std::move(zcSpecs));  // identical channels
        }
        {
            std::vector<SymbolStreamSpec> pureSpecs;
            pureSpecs.push_back({"facet", SymbolStreamRole::Facet,
                                 SideChannelSpec::Type::Int32,
                                 SideChannelSpec::Shape::Scalar, 3, true,
                                 SymbolContextModel::Valence4});
            pureSpecs.push_back({"edge", SymbolStreamRole::TriangulationEdge,
                                 SideChannelSpec::Type::Int32,
                                 SideChannelSpec::Shape::Scalar, 1, true});
            pureSpecs.push_back({"point", SymbolStreamRole::RemovedVertexCoord,
                                 SideChannelSpec::Type::Vec3f,
                                 SideChannelSpec::Shape::Vec3, 0, false});
            pureSpecs.push_back({"pcatm_pure/valence_other",
                                 SymbolStreamRole::SideChannel,
                                 SideChannelSpec::Type::Int8,
                                 SideChannelSpec::Shape::Scalar, 8, true});
            reg.registerLayout(kPaperBlobLayoutPCATMPure, std::move(pureSpecs));
        }

        // Container / algorithm-specific layouts: not yet migrated.
        reg.registerResidualOnly(kPaperBlobLayoutBasic);
        reg.registerResidualOnly(kPaperBlobLayoutPPMCRC);
        reg.registerResidualOnly(kPaperBlobLayoutValenceRC);
        reg.registerResidualOnly(kPaperBlobLayoutRDGate);
        reg.registerResidualOnly(kPaperBlobLayoutRDGateOnly);
        reg.registerResidualOnly(kPaperBlobLayoutValencePatchOnly);
        reg.registerResidualOnly(kPaperBlobLayoutCPMEdgeSplit);
        reg.registerResidualOnly(kPaperBlobLayoutPomar4Stream);
        reg.registerResidualOnly(kPaperBlobLayoutRDBitplane);
        reg.registerResidualOnly(kPaperBlobLayoutMMSEDualRing);
        reg.registerResidualOnly(kPaperBlobLayoutMMSEStrict);
        reg.registerResidualOnly(kPaperBlobLayoutVQMMCCodebook);
        reg.registerResidualOnly(kPaperBlobLayoutPCATMStrictPaper);
        reg.registerResidualOnly(kPaperBlobLayoutAPTMConquest);
        reg.registerResidualOnly(kPaperBlobLayoutVQMMCStrict);
        reg.registerResidualOnly(kPaperBlobLayoutPPMCSegmentedRC);
        reg.registerResidualOnly(kPaperBlobLayoutSegmentedRangeCodedPred);
    }
} g_canonicalRegistrar;

}  // namespace

}  // namespace pmsf
