#ifndef PMSF_COMPRESS_PAPER_COLLECTOR_SYMBOL_PROJECTION_H
#define PMSF_COMPRESS_PAPER_COLLECTOR_SYMBOL_PROJECTION_H

#include "compress/BasicSymbolCollectOperator.h"
#include "compress/OptimizedSymbolCollectOperator.h"
#include "compress/SymbolCollectOperator.h"
#include "RD/compress/RDTagSymbolCollectOperator.h"
#include "PCLTTM/compress/PCLTTMTagSymbolCollectOperator.h"
#include "PCATM/compress/PCATMColorSymbolCollectOperator.h"
#include "PCATM/compress/PCATMZColorSymbolCollectOperator.h"
#include "PCATM/compress/PCATMPureSymbolCollectOperator.h"
#include <typeinfo>
#include "core/DefaultCodecPMSFSideChannel.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include <deque>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace pmsf {

struct PaperDefaultCodecQueueView {
    const std::vector<std::deque<char>>* facets = nullptr;
    const std::vector<std::deque<char>>* edges = nullptr;
    const std::vector<std::deque<char>>* vertices = nullptr;
    const std::vector<std::deque<MCGAL::Point>>* points = nullptr;
    const std::vector<std::deque<MCGAL::Point>>* vertexPoints = nullptr;
};

inline bool paperDefaultCodecQueueViewValid(
    const PaperDefaultCodecQueueView& view) {
    return view.facets != nullptr &&
           view.edges != nullptr &&
           view.vertices != nullptr &&
           view.points != nullptr &&
           view.vertexPoints != nullptr;
}

inline PaperDefaultCodecQueueView paperDefaultCodecQueuesFromCollector(
    SymbolCollectOperator* collector) {
    PaperDefaultCodecQueueView view;
    if (auto* basic = dynamic_cast<BasicSymbolCollectOperator*>(collector)) {
        view.facets = &basic->getFacetSymbolQueues();
        view.edges = &basic->getEdgeSymbolQueues();
        view.vertices = &basic->getVertexSymbolQueues();
        view.points = &basic->getPointQueues();
        view.vertexPoints = &basic->getVertexPointQueues();
    } else if (auto* optimized =
                   dynamic_cast<OptimizedSymbolCollectOperator*>(collector)) {
        view.facets = &optimized->getFacetSymbolQueues();
        view.edges = &optimized->getEdgeSymbolQueues();
        view.vertices = &optimized->getVertexSymbolQueues();
        view.points = &optimized->getPointQueues();
        view.vertexPoints = &optimized->getVertexPointQueues();
    }
    return view;
}

inline SymbolStream paperSymbolStreamFromDefaultCodecQueues(
    const PaperDefaultCodecQueueView& view) {
    SymbolStream stream;
    if (!paperDefaultCodecQueueViewValid(view)) {
        return stream;
    }

    const std::size_t roundCount = view.facets->size();
    stream.reserve(roundCount);
    for (std::size_t round = 0; round < roundCount; ++round) {
        RoundSymbols symbols;
        setDefaultCodecPaperRoundSymbolsFromQueues(symbols,
                                                   round,
                                                   *view.facets,
                                                   *view.edges,
                                                   *view.vertices,
                                                   *view.points,
                                                   *view.vertexPoints);
        stream.push_back(std::move(symbols));
    }
    return stream;
}

// ---------------------------------------------------------------------------
// RD_TAG (layout 18, sv=1) canonical projection.
//   facetSymbols      -> 5-ary valence code per poly face
//   triangulationEdgeSymbols -> isPoly bit per BFS face
//   removedVertexResiduals   -> RD-prediction residual per poly face
//   side "rd_tag/valence_other" -> inline uint8 escapes for VAL_OTHER
// ---------------------------------------------------------------------------
inline PMSFSymbolStream paperRoleSymbolStreamFromRDTagCollector(
    const RDTagSymbolCollectOperator* rd) {
    PMSFSymbolStream stream;
    if (!rd) return stream;

    const auto& valenceQueues = rd->gateValenceQueues();
    const auto& otherValQueues = rd->gateOtherValQueues();
    const auto& isPolyQueues = rd->gateIsPolyQueues();
    const auto& residualQueues = rd->gateResidualQueues();

    const std::size_t numRounds = isPolyQueues.size();
    stream.reserve(numRounds);
    for (std::size_t r = 0; r < numRounds; ++r) {
        PaperRoundSymbols round;
        stream.push_back(std::move(round));
    }
    return stream;
}

// ---------------------------------------------------------------------------
// CODEC_014 (layout 14, sv=3) canonical projection.
//   facetSymbols             -> 5-ary valence code per poly face
//   triangulationEdgeSymbols -> isPoly bit per BFS face
//   vertexSymbols            -> edge-sym bits (per poly face 3 bits)
//   flipEdgeSymbols          -> color bit per BFS face
//   removedVertexResiduals   -> RD-prediction residual per poly face
//   side "pcatm_color/valence_other" -> inline uint8 escapes
// ---------------------------------------------------------------------------
inline PMSFSymbolStream paperRoleSymbolStreamFromPCATMColorCollector(
    const PCATMColorSymbolCollectOperator* pc) {
    PMSFSymbolStream stream;

    const auto& valenceQueues = pc->gateValenceQueues();
    const auto& otherValQueues = pc->gateOtherValQueues();
    const auto& isPolyQueues = pc->gateIsPolyQueues();
    const auto& edgeSymQueues = pc->gateEdgeSymQueues();
    const auto& colorQueues = pc->gateColorQueues();
    const auto& residualQueues = pc->gateResidualQueues();

    const std::size_t numRounds = isPolyQueues.size();
    stream.reserve(numRounds);
    for (std::size_t r = 0; r < numRounds; ++r) {
        PaperRoundSymbols round;
        stream.push_back(std::move(round));
    }
    return stream;
}

// ---------------------------------------------------------------------------
// Patch-color family (CODEC_017 layout 17 / CODEC_020 layout 20, sv=1)
// canonical projection.  Same as CODEC_014 but without the edge-sym stream.
//   facetSymbols             -> 5-ary valence code per poly face
//   triangulationEdgeSymbols -> isPoly bit per BFS face
//   flipEdgeSymbols          -> color bit per BFS face
//   removedVertexResiduals   -> RD-prediction residual per poly face
//   side "patch_color/valence_other" -> inline uint8 escapes
// ---------------------------------------------------------------------------
inline PMSFSymbolStream paperRoleSymbolStreamFromPatchColorCollector(
    const PCATMZColorSymbolCollectOperator* pc) {
    PMSFSymbolStream stream;

    const auto& valenceQueues = pc->gateValenceQueues();
    const auto& otherValQueues = pc->gateOtherValQueues();
    const auto& isPolyQueues = pc->gateIsPolyQueues();
    const auto& colorQueues = pc->gateColorQueues();
    const auto& residualQueues = pc->gateResidualQueues();

    const std::size_t numRounds = isPolyQueues.size();
    stream.reserve(numRounds);
        for (std::size_t r = 0; r < numRounds; ++r) {
        PaperRoundSymbols round;
        if (r < valenceQueues.size()) {
            round.facetSymbols.assign(valenceQueues[r].begin(),
                                      valenceQueues[r].end());
        }
        if (r < isPolyQueues.size()) {
            round.triangulationEdgeSymbols.assign(isPolyQueues[r].begin(),
                                                  isPolyQueues[r].end());
        }
        if (r < colorQueues.size()) {
            round.flipEdgeSymbols.assign(colorQueues[r].begin(),
                                         colorQueues[r].end());
        }
        if (r < residualQueues.size()) {
            round.removedVertexResiduals.assign(residualQueues[r].begin(),
                                                residualQueues[r].end());
        }
        if (r < otherValQueues.size() && !otherValQueues[r].empty()) {
            auto& side = round.side["patch_color/valence_other"];
            side.reserve(otherValQueues[r].size());
            for (std::uint8_t v : otherValQueues[r]) {
                side.push_back(static_cast<std::int8_t>(v));
            }
        }
                stream.push_back(std::move(round));
    }
    return stream;
}

// ---------------------------------------------------------------------------
// CODEC_021 (layout 21, sv=1/2) canonical projection.
//   facetSymbols             -> 5-ary valence code per poly face
//   triangulationEdgeSymbols -> isPoly bit per BFS face
//   removedVertexResiduals   -> RD-prediction residual per poly face
//   side "pcatm_pure/valence_other" -> inline uint8 escapes
// ---------------------------------------------------------------------------
inline PMSFSymbolStream paperRoleSymbolStreamFromPCATMPureCollector(
    const PCATMPureSymbolCollectOperator* pc) {
    PMSFSymbolStream stream;
    if (!pc) return stream;

    const auto& valenceQueues = pc->gateValenceQueues();
    const auto& otherValQueues = pc->gateOtherValQueues();
    const auto& isPolyQueues = pc->gateIsPolyQueues();
    const auto& residualQueues = pc->gateResidualQueues();

    const std::size_t numRounds = isPolyQueues.size();
    stream.reserve(numRounds);
    for (std::size_t r = 0; r < numRounds; ++r) {
        PaperRoundSymbols round;
        if (r < valenceQueues.size()) {
            round.facetSymbols.assign(valenceQueues[r].begin(),
                                      valenceQueues[r].end());
        }
        if (r < isPolyQueues.size()) {
            round.triangulationEdgeSymbols.assign(isPolyQueues[r].begin(),
                                                  isPolyQueues[r].end());
        }
        if (r < residualQueues.size()) {
            round.removedVertexResiduals.assign(residualQueues[r].begin(),
                                                residualQueues[r].end());
        }
        if (r < otherValQueues.size() && !otherValQueues[r].empty()) {
            auto& side = round.side["pcatm_pure/valence_other"];
            side.reserve(otherValQueues[r].size());
            for (std::uint8_t v : otherValQueues[r]) {
                side.push_back(static_cast<std::int8_t>(v));
            }
        }
        stream.push_back(std::move(round));
    }
    return stream;
}

// ---------------------------------------------------------------------------
// CODEC_016 (layout 16, sv=3) canonical projection.
//   facetSymbols      -> fused face sym {NotPoly, PolyUntagged, PolyPlus, PolyMinus}
//   removedVertexResiduals -> RD-prediction residual per poly face
//   side "pclttm_tag/valence"       -> 5-ary valence code per poly face
//   side "pclttm_tag/valence_other" -> inline uint8 escapes
// ---------------------------------------------------------------------------
inline PMSFSymbolStream paperRoleSymbolStreamFromPCLTTMTagCollector(
    const PCLTTMTagSymbolCollectOperator* pcl) {
    PMSFSymbolStream stream;
    if (!pcl) return stream;

    const auto& fusedQueues = pcl->gateFusedFaceSymQueues();
    const auto& valenceQueues = pcl->gateValenceQueues();
    const auto& otherValQueues = pcl->gateOtherValQueues();
    const auto& residualQueues = pcl->gateResidualQueues();

    const std::size_t numRounds = fusedQueues.size();
    stream.reserve(numRounds);
    for (std::size_t r = 0; r < numRounds; ++r) {
        PaperRoundSymbols round;
        if (r < fusedQueues.size()) {
            round.facetSymbols.assign(fusedQueues[r].begin(),
                                      fusedQueues[r].end());
        }
        if (r < residualQueues.size()) {
            round.removedVertexResiduals.assign(residualQueues[r].begin(),
                                                residualQueues[r].end());
        }
        if (r < valenceQueues.size() && !valenceQueues[r].empty()) {
            auto& side = round.side["pclttm_tag/valence"];
            side.reserve(valenceQueues[r].size());
            for (std::uint8_t v : valenceQueues[r]) {
                side.push_back(static_cast<std::int8_t>(v));
            }
        }
        if (r < otherValQueues.size() && !otherValQueues[r].empty()) {
            auto& side = round.side["pclttm_tag/valence_other"];
            side.reserve(otherValQueues[r].size());
            for (std::uint8_t v : otherValQueues[r]) {
                side.push_back(static_cast<std::int8_t>(v));
            }
        }
        stream.push_back(std::move(round));
    }
    return stream;
}

inline bool paperCollectorCanProjectToSymbolStream(
    SymbolCollectOperator* collector) {
    if (collector == nullptr) {
        return false;
    }
    if (collector->rounds() != nullptr) {
        return true;
    }
    if (dynamic_cast<const RDTagSymbolCollectOperator*>(collector) != nullptr) {
        return true;
    }
    if (dynamic_cast<const PCLTTMTagSymbolCollectOperator*>(collector) !=
        nullptr) {
        return true;
    }
    if (dynamic_cast<const PCATMColorSymbolCollectOperator*>(collector) !=
        nullptr) {
        return true;
    }
    if (dynamic_cast<const PCATMZColorSymbolCollectOperator*>(collector) !=
        nullptr) {
        return true;
    }
    return paperDefaultCodecQueueViewValid(
        paperDefaultCodecQueuesFromCollector(collector));
}

inline SymbolStream paperSymbolStreamFromCollector(
    SymbolCollectOperator* collector) {
    if (collector == nullptr) {
        return {};
    }
    if (const auto* rounds = collector->rounds()) {
        return storageStreamFromUnifiedRounds(*rounds);
    }
    return paperSymbolStreamFromDefaultCodecQueues(
        paperDefaultCodecQueuesFromCollector(collector));
}

// New canonical-facing entry point: returns PMSFSymbolStream directly,
// preserving algorithm-specific residuals / side channels.
inline SymbolCollectOperator* rawCollectorPtr(SymbolCollectOperator* c) {
    return c;
}

inline SymbolCollectOperator* rawCollectorPtr(const SymbolCollectOperator* c) {
    return const_cast<SymbolCollectOperator*>(c);
}

inline PMSFSymbolStream pmsfRoleSymbolStreamFromCollector(
    SymbolCollectOperator* collector) {
    if (collector == nullptr) {
        return {};
    }
    if (auto* rd = dynamic_cast<RDTagSymbolCollectOperator*>(collector)) {
        return paperRoleSymbolStreamFromRDTagCollector(rd);
    }
    // PCATM hierarchy: PCATMColor -> PCLTTMTag -> PCATMZColor -> PCATMPaper -> PCATMPure.
    // Check the derived layouts first, because PCATMZColor/PCATMPaper/PCATMPure are
    // also PCLTTMTag instances and would otherwise be mis-dispatched.
    if (typeid(*collector) == typeid(PCATMColorSymbolCollectOperator)) {
        return paperRoleSymbolStreamFromPCATMColorCollector(
            static_cast<PCATMColorSymbolCollectOperator*>(collector));
    }
    if (typeid(*collector) == typeid(PCATMZColorSymbolCollectOperator)) {
        return paperRoleSymbolStreamFromPatchColorCollector(
            static_cast<PCATMZColorSymbolCollectOperator*>(collector));
    }
    if (typeid(*collector) == typeid(PCATMPMSFSymbolCollectOperator)) {
        return paperRoleSymbolStreamFromPatchColorCollector(
            static_cast<PCATMPMSFSymbolCollectOperator*>(collector));
    }
    if (typeid(*collector) == typeid(PCATMPureSymbolCollectOperator)) {
        return paperRoleSymbolStreamFromPCATMPureCollector(
            static_cast<PCATMPureSymbolCollectOperator*>(collector));
    }
    if (auto* pcl = dynamic_cast<PCLTTMTagSymbolCollectOperator*>(collector)) {
        return paperRoleSymbolStreamFromPCLTTMTagCollector(pcl);
    }
    return toPMSFSymbolStream(paperSymbolStreamFromCollector(collector));
}

inline bool paperRoundSymbolsEqual(const PaperRoundSymbols& a,
                                   const PaperRoundSymbols& b) {
    auto eq = [](const auto& x, const auto& y) {
        if (x.size() != y.size()) return false;
        return std::equal(x.begin(), x.end(), y.begin());
    };
    if (!eq(a.facetSymbols, b.facetSymbols) ||
        !eq(a.triangulationEdgeSymbols, b.triangulationEdgeSymbols) ||
        !eq(a.flipEdgeSymbols, b.flipEdgeSymbols) ||
        !eq(a.vertexSymbols, b.vertexSymbols) ||
        !eq(a.removedVertexResiduals, b.removedVertexResiduals) ||
        !eq(a.shiftedVertexResiduals, b.shiftedVertexResiduals) ||
        !eq(a.removedVertexCoords, b.removedVertexCoords) ||
        !eq(a.shiftedVertexCoords, b.shiftedVertexCoords)) {
        return false;
    }
    // Side channels: a missing empty entry is equivalent to an explicit empty
    // entry (the canonical decoder always materializes side-channel slots).
    auto sideEmptyOrMissing = [](const auto& map, const std::string& name) {
        auto it = map.find(name);
        return it == map.end() || it->second.empty();
    };
    for (const auto& kv : a.side) {
        auto it = b.side.find(kv.first);
        if (kv.second.empty()) {
            if (it != b.side.end() && !it->second.empty()) return false;
            continue;
        }
        if (it == b.side.end()) return false;
        if (kv.second.size() != it->second.size()) return false;
        for (std::size_t i = 0; i < kv.second.size(); ++i) {
            if (kv.second[i].index() != it->second[i].index()) return false;
            bool same = std::visit(
                [&](const auto& v) {
                    using T = std::decay_t<decltype(v)>;
                    if constexpr (std::is_same_v<T, MCGAL::Point>) {
                        const auto& other = std::get<T>(it->second[i]);
                        return v.x() == other.x() && v.y() == other.y() &&
                               v.z() == other.z();
                    } else {
                        return v == std::get<T>(it->second[i]);
                    }
                },
                kv.second[i]);
            if (!same) return false;
        }
    }
    for (const auto& kv : b.side) {
        auto it = a.side.find(kv.first);
        if (kv.second.empty()) {
            if (it != a.side.end() && !it->second.empty()) return false;
            continue;
        }
        if (it == a.side.end()) return false;
    }
    return true;
}

inline bool paperSymbolStreamEqual(const PMSFSymbolStream& a,
                                   const PMSFSymbolStream& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (!paperRoundSymbolsEqual(a[i], b[i])) return false;
    }
    return true;
}

inline SymbolStream requirePMSFSymbolStreamFromCollector(
    SymbolCollectOperator* collector,
    const char* context) {
    if (!paperCollectorCanProjectToSymbolStream(collector)) {
        throw std::runtime_error(
            std::string(context) +
            ": collector must expose paper rounds or Basic/Optimized queues");
    }
    return paperSymbolStreamFromCollector(collector);
}

inline PMSFSymbolStream requirePaperRoleSymbolStreamFromCollector(
    SymbolCollectOperator* collector,
    const char* context) {
    if (!paperCollectorCanProjectToSymbolStream(collector)) {
        throw std::runtime_error(
            std::string(context) +
            ": collector must expose paper rounds or Basic/Optimized queues");
    }
    return pmsfRoleSymbolStreamFromCollector(collector);
}

}  // namespace pmsf

#endif  // PMSF_COMPRESS_PAPER_COLLECTOR_SYMBOL_PROJECTION_H
