// SPDX-License-Identifier: Apache-2.0
//
// VQMMCPaperOpaqueIO.h — M5-5 (task-35) opaque-payload bridge detection.
//
// VQMMC paper backend keeps the canonical binary artifact verbatim inside a
// PaperStream v2 opaque-payload container (P2BL single blob, zlib-raw payload).
// The storage helper (`src/storage/PaperStreamOpaquePayloadV2.h`) lands with
// storage-architect's M5-5 patch; until then the bridge is compiled out and the
// paper backend keeps its historical fail-fast behaviour. This keeps the tree
// green for every other row while the helper is in review.
#ifndef PMSF_VQMMC_PAPER_OPAQUE_IO_H
#define PMSF_VQMMC_PAPER_OPAQUE_IO_H

#if defined(COLUMNAR_SUPPORT) && defined(__has_include)
#if __has_include("storage/PaperStreamOpaquePayloadV2.h")
#define PMSF_VQMMC_HAS_OPAQUE_PAPER 1
#endif
#endif

#ifndef PMSF_VQMMC_HAS_OPAQUE_PAPER
#define PMSF_VQMMC_HAS_OPAQUE_PAPER 0
#endif

#endif  // PMSF_VQMMC_PAPER_OPAQUE_IO_H
