// SPDX-License-Identifier: Apache-2.0
//
// VQMMCPaperRegistration.h — M5-5 (task-35) paper decompress triple factory.
#ifndef PMSF_VQMMC_PAPER_REGISTRATION_H
#define PMSF_VQMMC_PAPER_REGISTRATION_H

#include "core/DecompressPipeline.h"
#include "VQMMC/VQMMCPaperOpaqueIO.h"

#if PMSF_VQMMC_HAS_OPAQUE_PAPER
namespace pmsf {
/// @brief Paper backend triple for the opaque VQMMC canonical-payload artifact:
///        opaque reader deserializer + strict reader + reconstructor.
DecompressOperatorTriple makeVQMMCPaperTriple();
}  // namespace pmsf
#endif

#endif  // PMSF_VQMMC_PAPER_REGISTRATION_H
