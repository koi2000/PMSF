/*
 * DefaultPPMCLiteSymbolReadOperator
 *
 * Mirrors DefaultPPMCLiteSymbolCollectOperator. Inherits the v3 per-edge
 * decoder (OptimizedSymbolReadOperator) and layers Frenet inverse-rotation
 * on top of the geometry path.
 *
 * Phase-A only (matches encoder): the bit-plane RC models are unchanged;
 * only the residual's coordinate system is rotated. Because frenet is
 * bijective, running invFrenetRotation on each decoded (α,β,γ) triple
 * recovers the original (rx,ry,rz) residual, which the subsequent
 * reconstruction path (BasicReconstructOperator::insert_point, when
 * prediction is enabled) adds to the barycenter to get the absolute
 * quantized position.
 *
 * Decoder cannot pre-decode residuals in importFromRangeCodedBuffer()
 * (the base class does), because Frenet inverse-rotation requires the
 * gate halfedge which is only known at BFS-time. So this class DEFERS
 * the point stream decode exactly the way the base class already does
 * for per-edge sym decoding (keeps range-coder state live; consumes it
 * inside collectFromDecoded).
 */

#ifndef DEFAULT_PPMC_LITE_SYMBOL_READ_OPERATOR_H
#define DEFAULT_PPMC_LITE_SYMBOL_READ_OPERATOR_H

#include "decompress/OptimizedSymbolReadOperator.h"
#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"
#include <cstddef>
#include <vector>

class DefaultPPMCLiteSymbolReadOperator : public OptimizedSymbolReadOperator {
  public:
    DefaultPPMCLiteSymbolReadOperator() = default;
    ~DefaultPPMCLiteSymbolReadOperator();

    /// Entry called by ColumnarDeserializeOperator when
    /// `blob_layout == CODEC_006`. Parses header + facet/edge/
    /// vertex sym streams (eagerly) and keeps the point stream's range
    /// coder live (lazily decoded in collectFromDecoded).
    void importFromRangeCodedBuffer(char* buffer, size_t& dataOffset,
                                    unsigned nbQuantBits) override;

    /// BFS over the reconstructed mesh. Same v3 per-edge rule as the base
    /// class; additionally, per split face:
    ///   1. peek the gate halfedge `h` (BFS-popped this iteration)
    ///   2. decode 3 residual components from the live bit-plane RC
    ///   3. build Frenet frame from `h`'s face normal
    ///   4. invFrenetRotation → (rx,ry,rz) stored on the face
    /// The ReconstructOperator then adds the barycenter to yield the
    /// absolute quantized removed-vertex position.
    void collectFromDecoded(MCGAL::Halfedge* seed,
                            std::vector<MCGAL::Vertex*>& vertices,
                            std::vector<MCGAL::Halfedge*>& halfedge,
                            std::vector<MCGAL::Facet*>& facets) override;

  private:
    /// Range-coder state for the deferred point stream.
    bool pointRCInitialized_ = false;
    rangecoder pointRC_{};
    size_t pointRCOffset_ = 0;
    unsigned nbQuantBits_ = 0;  // captured from importFromRangeCodedBuffer for decode-time
    bool useResidualBitPlaneLite_ = false;
    // Bit-plane models (one per bitplane, two clusters "first-1" / "after-1").
    std::vector<qsmodel> bpFirst_;
    std::vector<qsmodel> bpAfter_;
    // Adaptive-coord model (used only when useResidualBitPlane_ is false).
    bool coordModelInitialized_ = false;
    qsmodel coordModel_{};
};

#endif  // DEFAULT_PPMC_LITE_SYMBOL_READ_OPERATOR_H
