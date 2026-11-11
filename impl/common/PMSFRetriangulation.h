// SPDX-License-Identifier: Apache-2.0
//
// PMSFRetriangulation.h — PR-U (2026-05-12) + Phase 1 Figure 9 extension.
// Paper-aligned retriangulation rules for both Ahn et al. 2011 Fig.1
// and Alliez & Desbrun 2001 Fig.9.
//
// paperRetriangulate(valence): Ahn et al. 2011 Fig.1 fixed patterns
// paperRetriangulateFig9(valence, tagLeft, tagRight): shared executable model
// for Alliez & Desbrun 2001 Fig.9. It is not yet a literal imported 64-entry
// table; callers that need strict reproduction should validate it against CSV
// ground truth before enabling it by default.

#ifndef PMSF_CORE_PAPER_RETRIANGULATION_H
#define PMSF_CORE_PAPER_RETRIANGULATION_H

#include <array>
#include <cstdint>
#include <vector>

namespace pmsf {

struct RetriangulationResult {
    std::vector<std::array<int, 3>> triangles;   // triangle vertex indices into ring
    std::vector<std::array<int, 2>> diagonals;   // diagonal edge pairs
    std::vector<int> plusTagIdx;                  // indices of Plus-tagged ring vertices
    std::vector<int> minusTagIdx;                 // indices of Minus-tagged ring vertices
};

/// Ahn et al. 2011 Fig.1 — valence-only fixed patterns.
RetriangulationResult paperRetriangulate(int valence);

/// Alliez & Desbrun 2001 Fig.9 — tag-driven executable model.
/// tagLeft/tagRight: 1=Plus, 2=Minus (from common/PCLTTMTagTable.h Tag enum).
/// Returns the retriangulation pattern for the given valence and gate tags.
RetriangulationResult paperRetriangulateFig9(int valence, int tagLeft, int tagRight);

/// Literal Fig.9 descriptor, independently transcribed from the local paper.
/// The ring starts at target(h), proceeds along next(h), and ends at source(h).
/// Existing paperRetriangulateFig9 is the frozen engineering approximation.
RetriangulationResult paperRetriangulateFig9Exact(int valence, int sourceTag, int targetTag);

/// MEPP2/Alliez-Desbrun valence type table.
/// sourceTag/targetTag use MCGAL::Vertex::TagFlag-compatible integers:
/// 1=Plus, 2=Minus. The halfedge orientation is the paper input boundary h;
/// sourceTag is target(opposite(h)), targetTag is target(h).
int paperMepp2TypeFromBoundaryTags(int valence, int sourceTag, int targetTag);

/// Tags assigned to target(next(h)), target(next(next(h))), ... by the MEPP2
/// Retriangulation/Remove_Edges tables for the given type. Values are
/// 1=Plus, 2=Minus.
std::vector<int> paperMepp2InteriorTags(int type);

}  // namespace pmsf

#endif  // PMSF_CORE_PAPER_RETRIANGULATION_H
