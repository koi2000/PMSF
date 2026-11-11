/*
 * @file PPMCBindings.cpp
 * @brief PPMC 特有的几处 dynamic_pointer_cast 胶水，集中在此避免 PMSF.cpp 再走回头路。
 */
#include "PPMC/bindings/PPMCBindings.h"

#include <memory>

#include "Mesh.h"
#include "PPMC/compress/PPMCSymbolCollectOperator.h"
#include "PPMC/decompress/PPMCSymbolReadOperator.h"
#include "header/PMSFHeader.h"
#include "options/CompressOptions.h"

namespace pmsf {

void bindPPMCPredictionOnCompress(
    std::shared_ptr<SymbolCollectOperator> collector,
    const CompressOptions& options) {
    auto ppmcColl = std::dynamic_pointer_cast<PPMCSymbolCollectOperator>(collector);
    if (!ppmcColl) return;

    ppmcColl->setEnablePrediction(options.isEnablePrediction());
    // ⚠️ P2.a 对齐（原 PMSF::ppmcCompress）：triangle-mesh 判据改为局部 face_degree==3，
    //   因此全局 pureTriangle 标志已不再需要，直接按 isEnablePrediction 开关即可。
    ppmcColl->setUseTriangleMeshConnectivityPrediction(options.isEnablePrediction());
}

void bindPPMCPredictionOnDecompress(
    std::shared_ptr<SymbolReadOperator> symbolReader,
    const PMSFHeader& header,
    std::shared_ptr<MCGAL::Mesh> baseMesh) {
    auto ppmcRdr = std::dynamic_pointer_cast<PPMCSymbolReadOperator>(symbolReader);
    if (!ppmcRdr) return;

    ppmcRdr->setEnablePrediction(header.isEnablePrediction());

    // 历史代码里还枚举了 base mesh 的 facet_degree 判 pureTriangle —— 按 P2.a 对齐
    // 也统一改用 header 的 prediction 开关。保留遍历注释仅作文档。
    (void)baseMesh;
    ppmcRdr->setUseTriangleMeshConnectivityPrediction(header.isEnablePrediction());
}

void ppmcPreQuantize(std::shared_ptr<MCGAL::Mesh> mesh,
                     PMSFHeader& header,
                     const CompressOptions& options) {
    (void)header;
    // ⚠️ P1 对齐：显式把顶点位置替换为量化后的离散位置。仅量化开启时生效。
    if (options.isEnableQuantization()) {
        mesh->quantizeVertexPositions();
    }
}

}  // namespace pmsf
