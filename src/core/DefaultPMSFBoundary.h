#ifndef PMSF_CORE_DEFAULT_PAPER_BOUNDARY_H
#define PMSF_CORE_DEFAULT_PAPER_BOUNDARY_H

#include "Mesh.h"
#include "compress/SerializeOperator.h"
#include "compress/SymbolCollectOperator.h"
#include "core/CompressPipeline.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace pmsf {

inline bool defaultPMSFBoundaryEnvOn(const char* name) {
    return pmsfStorageEnvEnabled(name);
}

inline void addDefaultPaperSidecarValues(
    CompressPipelineConfig& cfg,
    std::vector<std::pair<std::string, int>> values) {
    auto previous = cfg.beforeSerialize;
    auto valuesPtr =
        std::make_shared<std::vector<std::pair<std::string, int>>>(std::move(values));
    cfg.beforeSerialize =
        [previous, valuesPtr](std::shared_ptr<SymbolCollectOperator> collector,
                              std::shared_ptr<SerializeOperator> serializer,
                              std::shared_ptr<MCGAL::Mesh> mesh,
                              MCGAL::Halfedge* seed) {
            if (previous) previous(collector, serializer, mesh, seed);
            (void)collector;
            (void)mesh;
            (void)seed;
            if (!serializer) return;
            for (const auto& entry : *valuesPtr) {
                serializer->addPaperSidecarValue(entry.first, entry.second);
            }
        };
}

}  // namespace pmsf

#endif  // PMSF_CORE_DEFAULT_PAPER_BOUNDARY_H
