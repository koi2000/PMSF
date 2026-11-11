#pragma once

#include "PPMC/compress/PPMCSymbolCollectOperator.h"
#include "compress/SymbolCollectOperator.h"
#include <memory>
#include <vector>

class PPMCSegmentationSymbolCollectOperator : public SymbolCollectOperator {
  public:
    bool preservesLiftedBaseMesh() const override { return true; }

    PPMCSegmentationSymbolCollectOperator() = default;

    void init(std::shared_ptr<MCGAL::Mesh> mesh,
              std::vector<MCGAL::Halfedge*> seeds,
              PMSFHeader header) override;

    void collect(MCGAL::Halfedge* seed) override;
    void collect(std::vector<MCGAL::Halfedge*> seeds) override;
    int exportToBuffer(char* buffer, bool enableQuantization = false, bool enableCompress = false) override;

    void setEnablePrediction(bool v) override;
    void setConnectivityPredictionFaces(bool v);
    void setConnectivityPredictionEdges(bool v);
    void setUseTriangleMeshConnectivityPrediction(bool v);
    const pmsf::PMSFSymbolTable* pmsfSymbolTable() const override { return &paperTable_; }
    const pmsf::PMSFSymbolPayloadTable* paperSymbolPayloadTable() const override;

  private:
    void ensureCollectors(std::size_t count);

    std::vector<std::unique_ptr<PPMCSymbolCollectOperator>> collectors_;
    bool prediction_ = false;
    bool facePrediction_ = false;
    bool edgePrediction_ = false;
    bool triangleBalance_ = false;
    int roundsCollected_ = 0;
    pmsf::PMSFSymbolTable paperTable_;
    mutable pmsf::PMSFSymbolPayloadTable paperPayloadTable_;
};
