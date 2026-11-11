/*
 * DefaultPPMCLiteSymbolCollectOperator
 *
 * A "lite" port of PPMC's geometry compression trick onto the Default /
 * OptimizedSymbolCollectOperator's v3 per-edge BFS. The only algorithmic
 * change vs OptimizedSymbolCollectOperator is this:
 *
 *   Phase A (opt-in while parity is being validated):
 *     before bit-plane RC-encoding each split-face's quantized residual
 *     `r = Q(removedPos) - Q(barycenter)`, optionally rotate it through
 *     a Frenet local frame built from the BFS gate halfedge's face normal:
 *         (rx, ry, rz) → (alpha, beta, gamma)
 *     where alpha/beta are tangent components (concentrated near zero
 *     for smooth/flat regions) and gamma is the normal component.
 *     Decoder applies `invFrenetRotation` symmetrically when the same
 *     `PMSF_LITE_ENABLE_FRENET=1` opt-in is set. The default stream keeps
 *     Cartesian residuals so BPV/time cannot regress from a non-default
 *     transform during paper-storage migration.
 *
 * Scope / non-goals (see docs/PPMC_PARQUETV2_NATIVE.md §2.3.2):
 *   - Phase B (area-based face-split prediction): NOT implemented yet.
 *   - Phase C (per-LOD adaptive RC reset with min/range header): NOT yet.
 *   Both are deferred. The unified paper sidecar records these mode bits
 *   explicitly so `Delta_r` consumers can distinguish the default
 *   Cartesian Lite payload from opt-in Frenet experiments without adding
 *   another storage backend.
 *
 * Round-trip invariant:
 *   encoder and decoder share the same BFS traversal order (inherited
 *   from OptimizedSymbolCollectOperator / OptimizedSymbolReadOperator),
 *   so the gate halfedge picked per split-face is geometrically
 *   identical on both sides. frenetRotation + invFrenetRotation are
 *   bijective, but the opt-in remains off until encoder/decoder frame
 *   construction is proven bit-exact across all mesh states.
 *
 * Wiring:
 *   codec_version = 4      (Parquet V2 metadata)
 *   blob_layout   = 6      (BlobLayout::CODEC_006)
 *   Old v3 / v2 files still decode via their own paths; no change to
 *   OptimizedSymbolCollectOperator / OptimizedSymbolReadOperator.
 */

#ifndef DEFAULT_PPMC_LITE_SYMBOL_COLLECT_OPERATOR_H
#define DEFAULT_PPMC_LITE_SYMBOL_COLLECT_OPERATOR_H

#include "compress/OptimizedSymbolCollectOperator.h"
#include "Halfedge.h"
#include "Point.h"
#include <deque>
#include <vector>

/// DefaultPPMCLiteSymbolCollectOperator extends the v3 per-edge collector
/// with Frenet-rotated geometry residuals. Public interface is identical
/// to OptimizedSymbolCollectOperator; ColumnarSerializeOperator detects
/// the subclass via dynamic_cast and stamps `blob_layout = 6 /
/// codec_version = 4` accordingly.
class DefaultPPMCLiteSymbolCollectOperator : public OptimizedSymbolCollectOperator {
  public:
    DefaultPPMCLiteSymbolCollectOperator() = default;

    /// Overrides only the RC path (bit-plane geometry stream); the BFS /
    /// facet-sym / edge-sym logic in the base class is unchanged and
    /// reused verbatim (same per-edge, v3 semantics).
    int exportToBuffer(char* buffer,
                       bool enableQuantization = false,
                       bool enableCompress = false) override;
    int exportToBufferWithRangeCoder(char* buffer, bool enableQuantization = false) override;

    /// Exposes the base Optimized F/E/V streams through the paper model and
    /// annotates the Lite geometry payload mode as side-channel symbols:
    ///   geometry_basis: 0=Cartesian residual, 1=Frenet residual
    ///   area_prediction: 0 until Phase B is implemented
    ///   adaptive_lod_rc_reset: 0 until Phase C is implemented
    const pmsf::PMSFSymbolTable* pmsfSymbolTable() const override;

    /// Collects the BFS gate halfedge for each split-face in the same
    /// order as `ipointQueues` so the Frenet basis on the encoder matches
    /// what the decoder computes. Overrides the base `collect(seed)` to
    /// slot this tracking in **alongside** the inherited logic — the
    /// facet/edge/vertex symbol streams emitted are byte-identical to
    /// what the base class would have produced.
    void collect(MCGAL::Halfedge* seed) override;
    void collect(std::vector<MCGAL::Halfedge*> seeds) override;

  private:
    /// Per-round pre-rotated residuals: we compute the Frenet rotation at
    /// collect-time (while the mesh is still consistent with the BFS state
    /// that produced this round's symbols), then store the rotated (α,β,γ)
    /// as an integer triple. Doing it at export-time would dereference
    /// halfedge pointers whose underlying mesh has been mutated by later
    /// rounds' eliminate operations — which caused a hang/crash during
    /// initial testing. Parallel to ipointQueues[i] (one entry per split
    /// face per round).
    std::vector<std::deque<MCGAL::Vector3i>> rotatedResidualQueues_;
};

#endif  // DEFAULT_PPMC_LITE_SYMBOL_COLLECT_OPERATOR_H
