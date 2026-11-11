// SPDX-License-Identifier: Apache-2.0
//
// Map RD-style patch conquest symbols into the paper storage relation:
//   Delta_r^p = (F_r, E_r^t, E_r^f, V_r, C_r^f, C_r^v) + explicit side data.
//
// RD_PATCH / RD_STRICT emit a 6-symbol patch alphabet rather than the legacy
// BFS boolean streams. The paper-level projection stores the NULL/non-NULL
// patch flag in F_r and the actual RD alphabet code in V_r, while preserving
// layer, gate bucket and geometry residual streams as named side-channels.
// This keeps the physical backend algorithm-independent without throwing away
// the codec-specific information needed for replay.

#ifndef PMSF_COMPRESS_RD_PAPER_SYMBOL_MAPPER_H
#define PMSF_COMPRESS_RD_PAPER_SYMBOL_MAPPER_H

#include "common/patch/RDPatchPipeline.h"
#include "RD/boundary/RDPatchPMSFBoundary.h"
#include "core/PMSFBoundarySideChannel.h"
#include "core/Symbol.h"
#include "storage/PMSFStorageContextSideChannel.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include <algorithm>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace pmsf {

inline std::string rdPaperChannelName(const std::string& prefix,
                                      const std::string& suffix) {
    return paperBoundarySideName(prefix, suffix);
}

inline void appendRdPaperI32(PaperRoundSymbols& delta,
                             const std::string& prefix,
                             const std::string& suffix,
                             std::int32_t value) {
    appendPMSFBoundarySideI32(delta, prefix, suffix, value);
}

inline void appendRdPaperI32Vector(PaperRoundSymbols& delta,
                                   const std::string& prefix,
                                   const std::string& suffix,
                                   const std::vector<int>& values) {
    appendPaperI32SideChannelValues(
        delta, rdPaperChannelName(prefix, suffix), values);
}

inline void appendRdPatchPMSFBoundary(PaperRoundSymbols& delta,
                                       const std::string& prefix) {
    if (prefix != "rd_patch") {
        return;
    }
    appendPMSFBoundaryAbsentExactPair(
        delta, prefix, "paper",
        rdPatchPaperExactRequiredForPrefix(prefix) ? 1 : 0);
}

inline PMSFSymbolTable pmsfSymbolTableFromRDPatchResult(
    const RDPatchPipelineResult& result,
    const std::string& channelPrefix) {
    PMSFSymbolTable table;
    if (result.symbols.empty() && result.numVerticesIn == 0 &&
        result.numVerticesOut == 0) {
        return table;
    }

    PMSFSymbolRecord& row = appendPMSFSymbolRecord(table, 0, 0, 0);
    appendPMSFStorageContextSymbolMapping(row.delta, channelPrefix, 1);
    appendRdPaperI32(row.delta, channelPrefix, "patch_conquest", 1);
    appendRdPatchPMSFBoundary(row.delta, channelPrefix);

    for (const RDPatchSymbol& sym : result.symbols) {
        const bool emitsVertex = sym.kind != RDPatchSymbolKind::NULL_PATCH;
        appendPaperScalarSymbolForRole(
            row.delta, PMSFSymbolRole::Facet, emitsVertex ? 1 : 0);
        appendPaperScalarSymbolForRole(
            row.delta,
            PMSFSymbolRole::Vertex,
            static_cast<std::int32_t>(sym.kind));
        appendRdPaperI32(row.delta, channelPrefix, "symbol",
                         static_cast<std::int32_t>(sym.kind));
        appendRdPaperI32(row.delta, channelPrefix, "escape_valence",
                         static_cast<std::int32_t>(sym.escape_valence));
        appendRdPaperI32(row.delta, channelPrefix, "phase",
                         static_cast<std::int32_t>(sym.phase));
        appendRdPaperI32(row.delta, channelPrefix, "bucket",
                         static_cast<std::int32_t>(sym.bucket));
        appendRdPaperI32(row.delta, channelPrefix, "layer",
                         static_cast<std::int32_t>(sym.layer));
    }

    appendRdPaperI32(row.delta, channelPrefix, "num_vertices_in",
                     static_cast<std::int32_t>(result.numVerticesIn));
    appendRdPaperI32(row.delta, channelPrefix, "num_vertices_out",
                     static_cast<std::int32_t>(result.numVerticesOut));
    appendRdPaperI32(row.delta, channelPrefix, "num_decimated_total",
                     static_cast<std::int32_t>(result.numDecimatedTotal));
    appendRdPaperI32(row.delta, channelPrefix, "num_null_patches",
                     static_cast<std::int32_t>(result.numNullPatches));
    appendRdPaperI32(row.delta, channelPrefix, "num_dec_cycles",
                     static_cast<std::int32_t>(result.numDecCycles));
    appendRdPaperI32(row.delta, channelPrefix, "num_steps",
                     static_cast<std::int32_t>(result.numSteps));
    appendRdPaperI32(row.delta, channelPrefix, "reached_max_steps",
                     result.reachedMaxSteps ? 1 : 0);
    for (int i = 0; i < 6; ++i) {
        appendRdPaperI32(row.delta, channelPrefix, "bucket_hits",
                         static_cast<std::int32_t>(result.bucketHits[i]));
    }
    for (int i = 0; i < 3; ++i) {
        appendRdPaperI32(row.delta, channelPrefix, "first_gate_v0_q",
                         static_cast<std::int32_t>(result.firstGateV0Q[i]));
        appendRdPaperI32(row.delta, channelPrefix, "first_gate_v1_q",
                         static_cast<std::int32_t>(result.firstGateV1Q[i]));
    }

    appendRdPaperI32Vector(row.delta, channelPrefix, "vv_residual_x",
                           result.vvResidualsX);
    appendRdPaperI32Vector(row.delta, channelPrefix, "vv_residual_y",
                           result.vvResidualsY);
    appendRdPaperI32Vector(row.delta, channelPrefix, "vv_residual_z",
                           result.vvResidualsZ);
    const std::size_t residualCount = std::min(
        result.vvResidualsX.size(),
        std::min(result.vvResidualsY.size(), result.vvResidualsZ.size()));
    for (std::size_t i = 0; i < residualCount; ++i) {
        appendPaperCoordinateForRole(
            row.delta,
            PMSFSymbolRole::RemovedVertexCoord,
            static_cast<double>(result.vvResidualsX[i]),
            static_cast<double>(result.vvResidualsY[i]),
            static_cast<double>(result.vvResidualsZ[i]));
    }
    appendRdPaperI32Vector(row.delta, channelPrefix, "vv_abs_qx",
                           result.vvAbsQX);
    appendRdPaperI32Vector(row.delta, channelPrefix, "vv_abs_qy",
                           result.vvAbsQY);
    appendRdPaperI32Vector(row.delta, channelPrefix, "vv_abs_qz",
                           result.vvAbsQZ);
    appendRdPaperI32Vector(row.delta, channelPrefix, "vv_decode_face_rank",
                           result.vvDecodeFaceRanks);
    appendRdPaperI32Vector(row.delta, channelPrefix, "vv_removed_vertex_id",
                           result.vvRemovedVertexIds);
    appendRdPaperI32Vector(row.delta, channelPrefix, "vv_ring_vertex_id",
                           result.vvRingVertexIds);
    appendRdPaperI32Vector(row.delta, channelPrefix, "vv_ring_qx",
                           result.vvRingQX);
    appendRdPaperI32Vector(row.delta, channelPrefix, "vv_ring_qy",
                           result.vvRingQY);
    appendRdPaperI32Vector(row.delta, channelPrefix, "vv_ring_qz",
                           result.vvRingQZ);

    for (unsigned char value : result.vvRingSizes) {
        appendRdPaperI32(row.delta, channelPrefix, "vv_ring_size",
                         static_cast<std::int32_t>(value));
    }
    for (unsigned char value : result.vvInputOffsets) {
        appendRdPaperI32(row.delta, channelPrefix, "vv_input_offset",
                         static_cast<std::int32_t>(value));
    }
    for (const auto& q : result.layerSeedQs) {
        for (int value : q) {
            appendRdPaperI32(row.delta, channelPrefix, "layer_seed_q",
                             static_cast<std::int32_t>(value));
        }
    }
    for (const auto& q : result.symbolInputQs) {
        for (int value : q) {
            appendRdPaperI32(row.delta, channelPrefix, "symbol_input_q",
                             static_cast<std::int32_t>(value));
        }
    }
    appendRdPaperI32(row.delta, channelPrefix, "locator_rank_prefix",
                     static_cast<std::int32_t>(result.locatorRankPrefix));
    appendRdPaperI32(row.delta, channelPrefix, "locator_ring_prefix",
                     static_cast<std::int32_t>(result.locatorRingPrefix));
    appendRdPaperI32(row.delta, channelPrefix, "leaky_factors",
                     static_cast<std::int32_t>(result.leakyFactors));

    return table;
}

}  // namespace pmsf

#endif  // PMSF_COMPRESS_RD_PAPER_SYMBOL_MAPPER_H
