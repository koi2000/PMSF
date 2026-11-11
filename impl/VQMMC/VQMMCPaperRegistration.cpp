// SPDX-License-Identifier: Apache-2.0
#include "VQMMC/VQMMCPaperRegistration.h"

#include "VQMMC/VQMMCPaperOpaqueIO.h"

#if PMSF_VQMMC_HAS_OPAQUE_PAPER

#include "core/Backend.h"
#include "core/DecompressOperatorRegistry.h"
#include "VQMMC/VQMMCPaperDeserializeOperator.h"
#include "VQMMC/VQMMCPostProcessor.h"
#include "VQMMC/VQMMCReconstructOperator.h"
#include "VQMMC/VQMMCStrictSymbolReadOperator.h"

#include <memory>

namespace pmsf {
namespace {

DecompressOperatorTriple makeVQMMCPaperTripleImpl() {
    auto reader = std::make_shared<VQMMCStrictSymbolReadOperator>();
    auto reconstructor = std::make_shared<VQMMCReconstructOperator>();
    auto postProcessor = std::make_shared<VQMMCPostProcessor>();
    auto deserializer = std::make_shared<VQMMCPaperDeserializeOperator>(
        reader, reconstructor, postProcessor);
    return {deserializer, reader, reconstructor};
}

struct VQMMCPaperRegistration {
    VQMMCPaperRegistration() {
        NamedDecompressOperatorRegistrar(
            "vqmmc", Backend::Paper,
            []() { return makeVQMMCPaperTripleImpl(); });
    }
} g_vqmmcPaperRegistration;

}  // namespace

DecompressOperatorTriple makeVQMMCPaperTriple() {
    return makeVQMMCPaperTripleImpl();
}

}  // namespace pmsf

#endif  // PMSF_VQMMC_HAS_OPAQUE_PAPER
