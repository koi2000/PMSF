/*
 * @file PaperDecompressFactoryRegistrationParquet.cpp
 * @brief Paper backend decompress triples (M2b pilot).
 *
 * Kept in a COLUMNAR_SOURCES-only translation unit: `impl/common/DecompressFactoryRegistration.cpp`
 * is also compiled into non-Parquet tests, and referencing PaperDeserializeOperator there
 * would create a link dependency on Arrow/Parquet for those targets.
 */
#include "algorithm/RetriProtocolRegistry.h"
#include "core/DecompressOperatorRegistry.h"

#include "decompress/OptimizedSymbolReadOperator.h"
#include "decompress/PaperDeserializeOperator.h"
#include <stdexcept>

#include "PPMC/decompress/PPMCSymbolReadOperator.h"
#include "PPMC/reconstruct/PPMCReconstructOperator.h"
#include "Conquest/decompress/ConquestSymbolReadOperator.h"
#include "Conquest/reconstruct/ConquestReconstructOperator.h"
#include "reconstruct/BasicReconstructOperator.h"

namespace pmsf {
namespace {

DecompressOperatorTriple makePpmcPaperTriple() {
    auto reader = std::make_shared<PPMCSymbolReadOperator>();
    auto reconstructor = std::make_shared<PPMCReconstructOperator>();
    auto deserializer = std::make_shared<PaperDeserializeOperator>(reader, reconstructor);
    return {deserializer, reader, reconstructor};
}

DecompressOperatorTriple makeOptimizedPaperTriple() {
    auto reader = std::make_shared<OptimizedSymbolReadOperator>();
    auto reconstructor = std::make_shared<BasicReconstructOperator>();
    auto deserializer = std::make_shared<PaperDeserializeOperator>(reader, reconstructor);
    return {deserializer, reader, reconstructor};
}

// Conquest layout15 is carried verbatim in the PaperStream base/legacy bundle;
// it must be decoded by the Conquest reader/reconstructor, not the generic
// Optimized pair. Without this registration the paper art is registered as an
// unknown algorithm and silently falls back to a mismatched triple.
DecompressOperatorTriple makeConquestPaperTriple() {
    auto reader = std::make_shared<ConquestSymbolReadOperator>();
    auto reconstructor = std::make_shared<ConquestReconstructOperator>(reader);
    auto deserializer = std::make_shared<PaperDeserializeOperator>(reader, reconstructor);
    return {deserializer, reader, reconstructor};
}

// ---------------------------------------------------------------------------
// M4-P0: Paper backend retri deserializer factory.
//
// The retri protocol seam hands us the native reader/reconstructor pair for a
// protocol id (impl/common/RetriOverride.h resolves it through the registered
// reader-pair factory). Paper artifacts live in the single-file `_paper.parquet`
// container, so the deserializer must be PaperDeserializeOperator, not the
// columnar one.
//
// Fail-closed: a non-empty protocol whose native reader is not implemented must
// throw here. Silently returning the generic paper triple would decode a retri
// artifact with the wrong layout (silent corruption), which the M4-P0 brief
// explicitly forbids. Empty/non-retri protocols return nullptr so the existing
// paper triple (default/efoss/ppmc/conquest and the bridge matrix) is kept.
// ---------------------------------------------------------------------------
/// @brief Paper retri deserializer: PaperDeserializeOperator + the retri decode
///        side effects (protocol env + deferred kVPropRDR1 attach) that the
///        columnar `RetriContextDeserializer` applies. Missing side effects are a
///        real crash (RD-family predictors assert on the vertex property), so the
///        attach is re-applied in deserializeBaseMesh() when the pool is ready.
class PaperRetriContextDeserializer : public PaperDeserializeOperator {
  public:
    PaperRetriContextDeserializer(std::shared_ptr<SymbolReadOperator> reader,
                                  std::shared_ptr<ReconstructOperator> reconstructor,
                                  std::string protocol)
        : PaperDeserializeOperator(reader, reconstructor),
          protocol_(std::move(protocol)) {}

    void init(std::string path) override {
        PaperDeserializeOperator::init(std::move(path));
        pmsf::applyRetriDecodeSideEffects(protocol_);
    }

    std::shared_ptr<MCGAL::Mesh> deserializeBaseMesh() override {
        // ContextPool may only be registered after init() (runDecompress defers
        // it to the original mesh size), so re-apply the idempotent attach here.
        pmsf::applyRetriDecodeSideEffects(protocol_);
        return PaperDeserializeOperator::deserializeBaseMesh();
    }

  private:
    std::string protocol_;
};

std::shared_ptr<DeserializeOperator> makePaperRetriDeserializer(
    const std::string& protocol, std::shared_ptr<SymbolReadOperator> defaultReader,
    std::shared_ptr<ReconstructOperator> defaultReconstructor) {
    if (protocol.empty() || protocol == "none") {
        return nullptr;
    }
    std::shared_ptr<SymbolReadOperator> reader;
    std::shared_ptr<ReconstructOperator> reconstructor;
    if (!pmsf::makeRetriReaderPair(protocol, reader, reconstructor) || !reader ||
        !reconstructor) {
        throw std::runtime_error(
            "[Paper][retri] native reader for protocol '" + protocol +
            "' is not implemented; refusing to silently fall back to the generic "
            "paper triple (would decode with the wrong blob layout)");
    }
    (void)defaultReader;
    (void)defaultReconstructor;
    return std::make_shared<PaperRetriContextDeserializer>(reader, reconstructor,
                                                          protocol);
}

struct PaperRegistrations {
    PaperRegistrations() {
        for (const char* name : {"default", "aptm", "pcatm", "pclttm", "rd"}) {
            NamedDecompressOperatorRegistrar(name, Backend::Paper, makeOptimizedPaperTriple);
        }
        // PaperSerializeOperator uses the framework Optimized collector for QEM
        // (QEMAlgorithm does not override collectorFactory), so pair it with the
        // Optimized reader; the Basic pairing was only for the legacy V2 path.
        NamedDecompressOperatorRegistrar("qem", Backend::Paper, makeOptimizedPaperTriple);
        NamedDecompressOperatorRegistrar("ppmc", Backend::Paper, makePpmcPaperTriple);
        // EFOSS uses the same Optimized collector/reader as default; register it
        // explicitly because its name differs from the generic preset list.
        NamedDecompressOperatorRegistrar("efoss", Backend::Paper, makeOptimizedPaperTriple);
        NamedDecompressOperatorRegistrar("conquest", Backend::Paper, makeConquestPaperTriple);
    }
} g_paperRegistrations;

struct PaperRetriRegistrations {
    PaperRetriRegistrations() {
        pmsf::registerRetriPaperDeserializerFactory(makePaperRetriDeserializer);
    }
} g_paperRetriRegistrations;

}  // namespace
}  // namespace pmsf