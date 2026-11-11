#ifndef PMSF_CORE_CANONICAL_SYMBOL_COLLECTOR_H
#define PMSF_CORE_CANONICAL_SYMBOL_COLLECTOR_H

#include "core/CanonicalSymbolEncoder.h"
#include "core/Symbol.h"
#include "core/SymbolStreamSpec.h"
#include "PMSFCollectorSymbolProjection.h"
#include "compress/SymbolCollectOperator.h"

#include <vector>

namespace pmsf {

/// Convenience wrapper that turns any collector exposing paper rounds
/// (or Basic/Optimized queues) into a canonical encoded payload.
class CanonicalSymbolCollector {
public:
    CanonicalSymbolCollector(SymbolCollectOperator* collector,
                             const std::vector<SymbolStreamSpec>& specs)
        : collector_(collector), specs_(specs) {}

    void setUseResidual(bool use) { useResidual_ = use; }

    /// Build the canonical payload from the collector's current state.
    /// Call this after the compression loop has finished.
    CanonicalSymbolPayload buildPayload() const {
        PMSFSymbolStream stream = pmsfRoleSymbolStreamFromCollector(collector_);

        // For collectors that compute RD-prediction residuals, copy them into the
        // canonical residual fields before any reordering so they stay aligned with
        // the round they belong to.
        if (useResidual_ && collector_) {
            if (auto* optimized = dynamic_cast<OptimizedSymbolCollectOperator*>(
                    collector_)) {
                const auto& residuals = optimized->getIpointQueues();
                for (std::size_t i = 0; i < stream.size() && i < residuals.size(); ++i) {
                    stream[i].removedVertexResiduals.assign(residuals[i].begin(),
                                                            residuals[i].end());
                }
            }
        }

        // The collector stores rounds in compression order (finest LOD first).
        // The decoder consumes rounds from the base mesh upward (coarsest first),
        // so reverse the stream before encoding to match the decompression order.
        std::reverse(stream.begin(), stream.end());
        CanonicalSymbolEncoder encoder(specs_);
        if (collector_ && collector_->mesh_ && collector_->mesh_->enableQuantization &&
            collector_->header_.isEnableQuantization()) {
            encoder.setQuantization(collector_->mesh_->i_nbQuantBits,
                                    collector_->mesh_->bboxMin,
                                    collector_->mesh_->f_quantStep);
        }
        if (useResidual_) {
            encoder.setUseResidual(true);
        }
        for (const auto& round : stream) {
            encoder.addRound(round);
        }
        return encoder.finish();
    }

    /// Rebuild the paper symbol stream from a canonical payload.
    static PMSFSymbolStream decodePayload(const CanonicalSymbolPayload& payload,
                                           const std::vector<SymbolStreamSpec>& specs) {
        CanonicalSymbolDecoder decoder(payload, specs);
        return decoder.decode();
    }

private:
    SymbolCollectOperator* collector_;
    std::vector<SymbolStreamSpec> specs_;
    bool useResidual_ = false;
};

}  // namespace pmsf

#endif  // PMSF_CORE_CANONICAL_SYMBOL_COLLECTOR_H
