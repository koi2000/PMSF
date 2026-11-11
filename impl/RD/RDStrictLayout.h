// SPDX-License-Identifier: Apache-2.0
//
// RDStrictLayout.h
//
// Shared constants for the paper-aligned RD storage path.  This path is kept
// separate from RD_PATCH(24), which is the current experimental patch-conquest
// stream with several scheme-version ablations.

#ifndef PMSF_CORE_RD_STRICT_LAYOUT_H
#define PMSF_CORE_RD_STRICT_LAYOUT_H

#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂

#include <cstdint>

namespace pmsf::rdstrict {

constexpr int kBlobLayout = kPaperBlobLayoutRDStrict;
constexpr int kSchemeVersion = kPaperBlobSchemeVersionRDStrict;
constexpr int kStreamCount = kPaperBlobStreamCountRDStrict;

enum StreamId : int {
    Connectivity = 0,
    Geometry = 1,
    SideInfo = 2,
};

enum class ConnectivitySymbol : std::uint8_t {
    NullPatch = 0,
    Val3 = 1,
    Val4 = 2,
    Val5 = 3,
    Val6 = 4,
    ValOther = 5,
};

// Little-endian ASCII tags used inside individual RD_STRICT substreams.
constexpr std::uint32_t kSideInfoMagic = 0x31534452u;  // "RDS1"
constexpr std::uint32_t kGeometryMagic = 0x31474452u;  // "RDG1"

}  // namespace pmsf::rdstrict

#endif  // PMSF_CORE_RD_STRICT_LAYOUT_H
