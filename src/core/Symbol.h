/*
 * @file Symbol.h
 * @brief Unified per-round symbol representation aligned with the paper:
 *        Delta_r = (F_r, E_r^t, E_r^f, V_r, C_r^f, C_r^v), plus typed
 *        side-channel rows for algorithm-specific coding details.
 *
 * Replaces the ad-hoc "four parallel std::vector<std::deque<...>>" convention
 * that every SymbolCollectOperator/SymbolReadOperator reinvents today.
 *
 * Design goals:
 *   - Fixed-shape core channels match the paper's symbolic layer while keeping
 *     the historical field names used by the existing byte streams.
 *   - Typed side-channel rows (std::variant) let Phase 11 Parquet-V3 carry
 *     residuals, codebook indices, context labels, anchor masks etc. WITHOUT
 *     forcing every algorithm to subclass SymbolCollectOperator.
 *   - Zero virtual calls on the hot path — collectors fill a RoundSymbols
 *     value directly; visitors consume it through a concrete iterator.
 *
 * This header is part of the Phase 9 refactor (see docs/ARCHITECTURE_V3.md
 * when it lands, and the plan file jaunty-inventing-puzzle.md).
 */
#ifndef PMSF_CORE_SYMBOL_H
#define PMSF_CORE_SYMBOL_H

#include "Point.h"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace pmsf {

/// Kind of the core fixed-shape channels. The ordinal values are stable.
/// Parquet-V2 deserializers already rely on them implicitly via column names.
///
/// Mapping to the paper:
///   FacetSymbol  -> F_r. The paper describes the common case as flags;
///                   storage uses int32 so algorithm-specific alphabets can
///                   share the same physical column without truncation.
///   EdgeSymbol   -> E_r^t by default (ring-triangulation inserted edges)
///   VertexSymbol -> V_r. Boolean adjustment flags are the default paper
///                   interpretation; valence/patch alphabets are stored here
///                   only when they are the emitted vertex-associated symbol.
///   Point        -> C_r^f
///   VertexPoint  -> C_r^v
///
/// E_r^f (edge-flip adjustment edges) is represented explicitly in
/// PaperRoundSymbols::flipEdgeSymbols. Existing collectors that only expose
/// the legacy EdgeSymbol channel leave E_r^f empty, which is the correct
/// paper-level value for algorithms without edge-flip adjustment.
enum class CoreChannel : std::uint8_t {
    FacetSymbol = 0,   ///< splittable flag per visited facet (F_r)
    EdgeSymbol = 1,    ///< added-edge flag per visited halfedge pair (E_r)
    VertexSymbol = 2,  ///< adjusted flag per visited vertex (V_r, optional)
    Point = 3,         ///< inserted-point position emitted when facet flag is 1 (P_r)
    VertexPoint = 4,   ///< adjusted-vertex position emitted when vertex flag is 1 (VP_r)
    _Count
};

/// Supported side-channel cell types. Keep this list small and stable; it
/// maps 1:1 to Parquet physical types in UnifiedSerializeOperator.
///
/// - Int8  / Int16 / Int32 map to residuals, context labels, tag ids.
/// - UInt16 is the canonical codebook-index cell type.
/// - Float maps to PCA/curvature/Frenet scalar channels.
/// - Vec3f is the go-to residual/anchor type.
/// - Bool is the canonical anchor-mask type.
/// - Bytes carries opaque payloads that still benefit from Parquet RG pushdown
///   but aren't worth a bespoke schema slot.
using SideCell = std::variant<
    bool,
    std::int8_t,
    std::int16_t,
    std::int32_t,
    std::uint16_t,
    float,
    MCGAL::Point,         ///< stored as 3 float columns in Parquet
    std::vector<std::uint8_t>
>;

/// A named, typed side-channel column. Values are LIST-typed inside a round.
/// Shape::Scalar => one cell per symbol; Shape::Vec3 is implicit for Point cells.
struct SideChannelColumn {
    std::string name;
    std::vector<SideCell> values;
};

using SymbolCode = std::int32_t;
using PMSFSymbolCodeDeque = std::deque<SymbolCode>;

/// A single round's worth of symbols, ready for serialization.
/// This is the canonical in-memory container both compress-side collectors
/// and decompress-side readers agree on.
struct RoundSymbols {
    std::deque<SymbolCode> facetSymbols;    ///< F_r, int32 code alphabet
    std::deque<SymbolCode> edgeSymbols;     ///< E_r^t in the paper model
    std::deque<SymbolCode> vertexSymbols;   ///< V_r, int32 code alphabet
    std::deque<MCGAL::Point> points;        ///< C_r^f — one entry per facetSymbol==1
    std::deque<MCGAL::Point> vertexPoints;  ///< C_r^v — one entry per vertexSymbol==1

    /// Named side-channel values for this round. Key = channel name.
    /// Filled by algorithm-specific Predictor / EntropyCoder / Codebook hooks.
    /// Empty in the default fast path, zero-cost when unused.
    std::unordered_map<std::string, std::vector<SideCell>> side;

    void clear() noexcept {
        facetSymbols.clear();
        edgeSymbols.clear();
        vertexSymbols.clear();
        points.clear();
        vertexPoints.clear();
        side.clear();
    }

    std::size_t totalCoreCells() const noexcept {
        return facetSymbols.size() + edgeSymbols.size() + vertexSymbols.size()
             + points.size() + vertexPoints.size();
    }
};

/// Paper-faithful Delta_r container. This is the storage-facing contract:
/// every algorithm-specific emitted symbol must either map into these six
/// streams or live in `side` as an explicit codec-side refinement. Keeping
/// this separate from RoundSymbols lets legacy code continue to use the old
/// field names while new storage code can reason in paper terminology.
struct PaperRoundSymbols {
    std::deque<SymbolCode> facetSymbols;                ///< F_r
    std::deque<SymbolCode> triangulationEdgeSymbols;    ///< E_r^t
    std::deque<SymbolCode> flipEdgeSymbols;             ///< E_r^f
    std::deque<SymbolCode> vertexSymbols;               ///< V_r
    std::deque<MCGAL::Point> removedVertexCoords; ///< C_r^f
    std::deque<MCGAL::Point> shiftedVertexCoords; ///< C_r^v
    /// Optional signed integer residuals used when the canonical payload stores
    /// geometry as RD-prediction residuals instead of absolute coordinates.
    std::deque<MCGAL::PointInt> removedVertexResiduals;
    std::deque<MCGAL::PointInt> shiftedVertexResiduals;
    std::unordered_map<std::string, std::vector<SideCell>> side;

    void clear() noexcept {
        facetSymbols.clear();
        triangulationEdgeSymbols.clear();
        flipEdgeSymbols.clear();
        vertexSymbols.clear();
        removedVertexCoords.clear();
        shiftedVertexCoords.clear();
        side.clear();
    }

    std::size_t totalCoreCells() const noexcept {
        return facetSymbols.size() + triangulationEdgeSymbols.size()
             + flipEdgeSymbols.size() + vertexSymbols.size()
             + removedVertexCoords.size() + shiftedVertexCoords.size();
    }
};

enum class PMSFSymbolRole : std::uint8_t {
    Facet = 0,              ///< F_r
    TriangulationEdge = 1,  ///< E_r^t
    FlipEdge = 2,           ///< E_r^f
    Vertex = 3,             ///< V_r
    RemovedVertexCoord = 4, ///< C_r^f
    ShiftedVertexCoord = 5  ///< C_r^v
};

inline constexpr std::int32_t kPMSFSymbolDeltaTupleArity = 6;

inline constexpr const char* paperSymbolRoleName(PMSFSymbolRole role) {
    switch (role) {
        case PMSFSymbolRole::Facet:
            return "F_r";
        case PMSFSymbolRole::TriangulationEdge:
            return "E_t_r";
        case PMSFSymbolRole::FlipEdge:
            return "E_f_r";
        case PMSFSymbolRole::Vertex:
            return "V_r";
        case PMSFSymbolRole::RemovedVertexCoord:
            return "C_f_r";
        case PMSFSymbolRole::ShiftedVertexCoord:
            return "C_v_r";
    }
    return "unknown";
}

inline constexpr const char* paperSymbolRoleSideChannelName(
    PMSFSymbolRole role) {
    switch (role) {
        case PMSFSymbolRole::Facet:
            return "paper_F_i32";
        case PMSFSymbolRole::TriangulationEdge:
            return "paper_Et_i32";
        case PMSFSymbolRole::FlipEdge:
            return "paper_Ef_i32";
        case PMSFSymbolRole::Vertex:
            return "paper_V_i32";
        case PMSFSymbolRole::RemovedVertexCoord:
            return "paper_Cf_vec3";
        case PMSFSymbolRole::ShiftedVertexCoord:
            return "paper_Cv_vec3";
    }
    return "paper_unknown";
}

inline constexpr bool paperSymbolRoleIsCoordinate(PMSFSymbolRole role) {
    return role == PMSFSymbolRole::RemovedVertexCoord ||
           role == PMSFSymbolRole::ShiftedVertexCoord;
}

inline constexpr bool paperSymbolRoleIsScalarSymbol(PMSFSymbolRole role) {
    return !paperSymbolRoleIsCoordinate(role);
}

inline std::size_t paperRoundSymbolRoleCellCount(
    const PaperRoundSymbols& delta,
    PMSFSymbolRole role) {
    switch (role) {
        case PMSFSymbolRole::Facet:
            return delta.facetSymbols.size();
        case PMSFSymbolRole::TriangulationEdge:
            return delta.triangulationEdgeSymbols.size();
        case PMSFSymbolRole::FlipEdge:
            return delta.flipEdgeSymbols.size();
        case PMSFSymbolRole::Vertex:
            return delta.vertexSymbols.size();
        case PMSFSymbolRole::RemovedVertexCoord:
            return delta.removedVertexCoords.size();
        case PMSFSymbolRole::ShiftedVertexCoord:
            return delta.shiftedVertexCoords.size();
    }
    return 0;
}

inline std::deque<SymbolCode>* paperScalarSymbolDequeForRole(
    PaperRoundSymbols& delta,
    PMSFSymbolRole role) {
    switch (role) {
        case PMSFSymbolRole::Facet:
            return &delta.facetSymbols;
        case PMSFSymbolRole::TriangulationEdge:
            return &delta.triangulationEdgeSymbols;
        case PMSFSymbolRole::FlipEdge:
            return &delta.flipEdgeSymbols;
        case PMSFSymbolRole::Vertex:
            return &delta.vertexSymbols;
        case PMSFSymbolRole::RemovedVertexCoord:
        case PMSFSymbolRole::ShiftedVertexCoord:
            return nullptr;
    }
    return nullptr;
}

inline const std::deque<SymbolCode>* paperScalarSymbolDequeForRole(
    const PaperRoundSymbols& delta,
    PMSFSymbolRole role) {
    switch (role) {
        case PMSFSymbolRole::Facet:
            return &delta.facetSymbols;
        case PMSFSymbolRole::TriangulationEdge:
            return &delta.triangulationEdgeSymbols;
        case PMSFSymbolRole::FlipEdge:
            return &delta.flipEdgeSymbols;
        case PMSFSymbolRole::Vertex:
            return &delta.vertexSymbols;
        case PMSFSymbolRole::RemovedVertexCoord:
        case PMSFSymbolRole::ShiftedVertexCoord:
            return nullptr;
    }
    return nullptr;
}

inline std::deque<MCGAL::Point>* paperCoordinateDequeForRole(
    PaperRoundSymbols& delta,
    PMSFSymbolRole role) {
    switch (role) {
        case PMSFSymbolRole::RemovedVertexCoord:
            return &delta.removedVertexCoords;
        case PMSFSymbolRole::ShiftedVertexCoord:
            return &delta.shiftedVertexCoords;
        case PMSFSymbolRole::Facet:
        case PMSFSymbolRole::TriangulationEdge:
        case PMSFSymbolRole::FlipEdge:
        case PMSFSymbolRole::Vertex:
            return nullptr;
    }
    return nullptr;
}

inline const std::deque<MCGAL::Point>* paperCoordinateDequeForRole(
    const PaperRoundSymbols& delta,
    PMSFSymbolRole role) {
    switch (role) {
        case PMSFSymbolRole::RemovedVertexCoord:
            return &delta.removedVertexCoords;
        case PMSFSymbolRole::ShiftedVertexCoord:
            return &delta.shiftedVertexCoords;
        case PMSFSymbolRole::Facet:
        case PMSFSymbolRole::TriangulationEdge:
        case PMSFSymbolRole::FlipEdge:
        case PMSFSymbolRole::Vertex:
            return nullptr;
    }
    return nullptr;
}

template <typename Codes>
inline bool setPaperScalarSymbolsForRole(PaperRoundSymbols& delta,
                                         PMSFSymbolRole role,
                                         Codes&& values) {
    if (auto* target = paperScalarSymbolDequeForRole(delta, role)) {
        *target = std::forward<Codes>(values);
        return true;
    }
    return false;
}

template <typename Points>
inline bool setPaperCoordinatesForRole(PaperRoundSymbols& delta,
                                       PMSFSymbolRole role,
                                       Points&& values) {
    if (auto* target = paperCoordinateDequeForRole(delta, role)) {
        *target = std::forward<Points>(values);
        return true;
    }
    return false;
}

inline bool appendPaperScalarSymbolForRole(PaperRoundSymbols& delta,
                                           PMSFSymbolRole role,
                                           SymbolCode value) {
    if (auto* target = paperScalarSymbolDequeForRole(delta, role)) {
        target->push_back(value);
        return true;
    }
    return false;
}

inline bool appendPaperCoordinateForRole(PaperRoundSymbols& delta,
                                         PMSFSymbolRole role,
                                         const MCGAL::Point& value) {
    if (auto* target = paperCoordinateDequeForRole(delta, role)) {
        target->push_back(value);
        return true;
    }
    return false;
}

inline bool appendPaperCoordinateForRole(PaperRoundSymbols& delta,
                                         PMSFSymbolRole role,
                                         double x,
                                         double y,
                                         double z) {
    return appendPaperCoordinateForRole(delta, role, MCGAL::Point(x, y, z));
}

inline void appendRoundFacetSymbol(RoundSymbols& round, SymbolCode value) {
    round.facetSymbols.push_back(value);
}

inline void appendRoundTriangulationEdgeSymbol(RoundSymbols& round,
                                               SymbolCode value) {
    round.edgeSymbols.push_back(value);
}

inline void appendRoundVertexSymbol(RoundSymbols& round, SymbolCode value) {
    round.vertexSymbols.push_back(value);
}

inline void appendRoundRemovedVertexCoord(RoundSymbols& round,
                                          const MCGAL::Point& value) {
    round.points.push_back(value);
}

inline void appendRoundRemovedVertexCoord(RoundSymbols& round,
                                          double x,
                                          double y,
                                          double z) {
    appendRoundRemovedVertexCoord(round, MCGAL::Point(x, y, z));
}

inline void appendRoundShiftedVertexCoord(RoundSymbols& round,
                                          const MCGAL::Point& value) {
    round.vertexPoints.push_back(value);
}

inline void appendRoundShiftedVertexCoord(RoundSymbols& round,
                                          double x,
                                          double y,
                                          double z) {
    appendRoundShiftedVertexCoord(round, MCGAL::Point(x, y, z));
}

template <typename T>
inline std::deque<SymbolCode>
toPMSFSymbolCodeDeque(const std::deque<T>& in) {
    std::deque<SymbolCode> out;
    out.resize(in.size());
    for (std::size_t i = 0; i < in.size(); ++i) {
        if constexpr (std::is_same_v<T, char>) {
            out[i] = static_cast<SymbolCode>(
                static_cast<unsigned char>(in[i]));
        } else {
            out[i] = static_cast<SymbolCode>(in[i]);
        }
    }
    return out;
}


template <typename Codes>
inline void setPaperFacetSymbols(PaperRoundSymbols& delta,
                                 Codes&& values) {
    setPaperScalarSymbolsForRole(
        delta, PMSFSymbolRole::Facet, std::forward<Codes>(values));
}


template <typename Codes>
inline void setPaperTriangulationEdgeSymbols(PaperRoundSymbols& delta,
                                             Codes&& values) {
    setPaperScalarSymbolsForRole(delta,
                                 PMSFSymbolRole::TriangulationEdge,
                                 std::forward<Codes>(values));
}

inline void clearPaperTriangulationEdgeSymbols(PaperRoundSymbols& delta) {
    setPaperScalarSymbolsForRole(delta,
                                 PMSFSymbolRole::TriangulationEdge,
                                 std::deque<SymbolCode>{});
}


template <typename Codes>
inline void setPaperFlipEdgeSymbols(PaperRoundSymbols& delta,
                                    Codes&& values) {
    setPaperScalarSymbolsForRole(
        delta, PMSFSymbolRole::FlipEdge, std::forward<Codes>(values));
}

inline void clearPaperFlipEdgeSymbols(PaperRoundSymbols& delta) {
    setPaperScalarSymbolsForRole(delta,
                                 PMSFSymbolRole::FlipEdge,
                                 std::deque<SymbolCode>{});
}


template <typename Codes>
inline void setPaperVertexSymbols(PaperRoundSymbols& delta,
                                  Codes&& values) {
    setPaperScalarSymbolsForRole(
        delta, PMSFSymbolRole::Vertex, std::forward<Codes>(values));
}

inline void clearPaperVertexSymbols(PaperRoundSymbols& delta) {
    setPaperScalarSymbolsForRole(delta,
                                 PMSFSymbolRole::Vertex,
                                 std::deque<SymbolCode>{});
}

inline void appendPaperRemovedVertexCoord(PaperRoundSymbols& delta,
                                          const MCGAL::Point& value) {
    appendPaperCoordinateForRole(
        delta, PMSFSymbolRole::RemovedVertexCoord, value);
}

template <typename Points>
inline void setPaperRemovedVertexCoords(PaperRoundSymbols& delta,
                                        Points&& values) {
    setPaperCoordinatesForRole(delta,
                               PMSFSymbolRole::RemovedVertexCoord,
                               std::forward<Points>(values));
}

inline void appendPaperRemovedVertexCoord(PaperRoundSymbols& delta,
                                          double x,
                                          double y,
                                          double z) {
    appendPaperRemovedVertexCoord(delta, MCGAL::Point(x, y, z));
}

inline void appendPaperShiftedVertexCoord(PaperRoundSymbols& delta,
                                          const MCGAL::Point& value) {
    appendPaperCoordinateForRole(
        delta, PMSFSymbolRole::ShiftedVertexCoord, value);
}

template <typename Points>
inline void setPaperShiftedVertexCoords(PaperRoundSymbols& delta,
                                        Points&& values) {
    setPaperCoordinatesForRole(delta,
                               PMSFSymbolRole::ShiftedVertexCoord,
                               std::forward<Points>(values));
}

inline void appendPaperShiftedVertexCoord(PaperRoundSymbols& delta,
                                          double x,
                                          double y,
                                          double z) {
    appendPaperShiftedVertexCoord(delta, MCGAL::Point(x, y, z));
}

template <typename TargetRoundSymbols, typename SourceRoundSymbols>
inline void setPMSFSideChannelsFromRound(TargetRoundSymbols& target,
                                          const SourceRoundSymbols& source) {
    target.side = source.side;
}

inline PaperRoundSymbols toPaperRoundSymbols(const RoundSymbols& r) {
    PaperRoundSymbols out;
    setPaperScalarSymbolsForRole(out,
                                 PMSFSymbolRole::Facet,
                                 toPMSFSymbolCodeDeque(r.facetSymbols));
    setPaperScalarSymbolsForRole(out,
                                 PMSFSymbolRole::TriangulationEdge,
                                 toPMSFSymbolCodeDeque(r.edgeSymbols));
    setPaperScalarSymbolsForRole(out,
                                 PMSFSymbolRole::Vertex,
                                 toPMSFSymbolCodeDeque(r.vertexSymbols));
    setPaperCoordinatesForRole(out,
                               PMSFSymbolRole::RemovedVertexCoord,
                               r.points);
    setPaperCoordinatesForRole(out,
                               PMSFSymbolRole::ShiftedVertexCoord,
                               r.vertexPoints);
    setPMSFSideChannelsFromRound(out, r);
    return out;
}

/// A compressed mesh = one base mesh + an ordered sequence of RoundSymbols,
/// oldest (deepest LOD) first. SerializeOperator writes rounds back-to-front
/// today; that convention is orthogonal to this container and is preserved.
using SymbolStream = std::vector<RoundSymbols>;
using PMSFSymbolStream = std::vector<PaperRoundSymbols>;

/// A relation row for the paper's random-access representation:
///   {Delta_r^p}_{p in P, 1 <= r <= R}
///
/// Storage backends should converge on this table-shaped form instead of
/// creating one physical schema per algorithm. Non-segmented codecs use
/// partitionId = 0. Algorithms that internally organize symbols by clusters
/// or regions should map those buckets to partitionId and their progressive
/// stages to lodId before reaching the physical writer.
struct PMSFSymbolRecord {
    std::int32_t objectId = 0;
    std::int32_t partitionId = 0;
    std::int32_t lodId = 0;
    PaperRoundSymbols delta;
};

using PMSFSymbolTable = std::vector<PMSFSymbolRecord>;

inline void setPMSFSymbolRecordKey(PMSFSymbolRecord& row,
                                    std::int32_t objectId,
                                    std::int32_t partitionId,
                                    std::int32_t lodId) {
    row.objectId = objectId;
    row.partitionId = partitionId;
    row.lodId = lodId;
}

inline PMSFSymbolRecord makePMSFSymbolRecord(
    std::int32_t objectId = 0,
    std::int32_t partitionId = 0,
    std::int32_t lodId = 0) {
    PMSFSymbolRecord row;
    setPMSFSymbolRecordKey(row, objectId, partitionId, lodId);
    return row;
}

inline PMSFSymbolRecord& appendPMSFSymbolRecord(PMSFSymbolTable& table,
                                                  std::int32_t objectId = 0,
                                                  std::int32_t partitionId = 0,
                                                  std::int32_t lodId = 0) {
    table.push_back(makePMSFSymbolRecord(objectId, partitionId, lodId));
    return table.back();
}

inline PMSFSymbolRecord& ensurePMSFSymbolRecord(PMSFSymbolTable& table,
                                                  std::size_t rowIndex,
                                                  std::int32_t objectId = 0,
                                                  std::int32_t partitionId = 0) {
    while (table.size() <= rowIndex) {
        appendPMSFSymbolRecord(
            table,
            objectId,
            partitionId,
            static_cast<std::int32_t>(table.size()));
    }
    return table[rowIndex];
}

/// Physical payload row for entropy-coded symbols that are already compressed
/// by a codec-specific model but still belong to the paper relation cell
/// (object_id, partition_id, lod_id, Delta_r^p). This lets the physical layer
/// store compact payloads without preserving one storage backend per
/// algorithm. The payload name identifies the logical substream, not an
/// algorithm-specific table.
struct PMSFSymbolPayloadRecord {
    std::int32_t objectId = 0;
    std::int32_t partitionId = 0;
    std::int32_t lodId = 0;
    std::string payloadName;
    std::string codecName;
    std::string payloadRole;
    std::vector<std::uint8_t> payload;
};

using PMSFSymbolPayloadTable = std::vector<PMSFSymbolPayloadRecord>;

inline void setPMSFSymbolPayloadRecordKey(PMSFSymbolPayloadRecord& row,
                                           std::int32_t objectId,
                                           std::int32_t partitionId,
                                           std::int32_t lodId) {
    row.objectId = objectId;
    row.partitionId = partitionId;
    row.lodId = lodId;
}

inline PMSFSymbolPayloadRecord makePMSFSymbolPayloadRecord(
    std::int32_t objectId = 0,
    std::int32_t partitionId = 0,
    std::int32_t lodId = 0,
    std::string payloadName = {},
    std::string codecName = {},
    std::string payloadRole = {}) {
    PMSFSymbolPayloadRecord row;
    setPMSFSymbolPayloadRecordKey(row, objectId, partitionId, lodId);
    row.payloadName = std::move(payloadName);
    row.codecName = std::move(codecName);
    row.payloadRole = std::move(payloadRole);
    return row;
}

inline void setPMSFSymbolPayloadRoleName(PMSFSymbolPayloadRecord& row,
                                          std::string payloadRole) {
    row.payloadRole = std::move(payloadRole);
}

inline void setPMSFSymbolPayloadCodecName(PMSFSymbolPayloadRecord& row,
                                           std::string codecName) {
    row.codecName = std::move(codecName);
}

inline void setPMSFSymbolPayloadName(PMSFSymbolPayloadRecord& row,
                                      std::string payloadName) {
    row.payloadName = std::move(payloadName);
}

inline void setPMSFSymbolPayloadBytes(PMSFSymbolPayloadRecord& row,
                                       const void* data,
                                       std::size_t size) {
    row.payload.clear();
    if (data != nullptr && size > 0) {
        const auto* bytes = reinterpret_cast<const std::uint8_t*>(data);
        row.payload.assign(bytes, bytes + size);
    }
}

inline void setPMSFSymbolPayloadBytes(PMSFSymbolPayloadRecord& row,
                                       std::vector<std::uint8_t> payload) {
    row.payload = std::move(payload);
}

inline PMSFSymbolStream toPMSFSymbolStream(const SymbolStream& stream) {
    PMSFSymbolStream out;
    out.reserve(stream.size());
    for (const auto& r : stream) {
        out.push_back(toPaperRoundSymbols(r));
    }
    return out;
}

inline PMSFSymbolTable toPMSFSymbolTable(const PMSFSymbolStream& stream,
                                           std::int32_t objectId = 0,
                                           std::int32_t partitionId = 0) {
    PMSFSymbolTable out;
    out.reserve(stream.size());
    for (std::size_t r = 0; r < stream.size(); ++r) {
        PMSFSymbolRecord& row = appendPMSFSymbolRecord(
            out,
            objectId, partitionId, static_cast<std::int32_t>(r));
        row.delta = stream[r];
    }
    return out;
}

inline PMSFSymbolTable toPMSFSymbolTable(const SymbolStream& stream,
                                           std::int32_t objectId = 0,
                                           std::int32_t partitionId = 0) {
    return toPMSFSymbolTable(
        toPMSFSymbolStream(stream),
        objectId,
        partitionId);
}

template <typename AugmentRow>
inline bool copyPMSFSymbolTableWithRowAugmentation(
    const PMSFSymbolTable* base,
    PMSFSymbolTable& out,
    AugmentRow augmentRow) {
    out.clear();
    if (base == nullptr || base->empty()) {
        return false;
    }
    out = *base;
    for (auto& row : out) {
        augmentRow(row);
    }
    return !out.empty();
}

template <typename AugmentRow>
inline bool copyPMSFSymbolTableWithIndexedRowAugmentation(
    const PMSFSymbolTable* base,
    PMSFSymbolTable& out,
    AugmentRow augmentRow) {
    out.clear();
    if (base == nullptr || base->empty()) {
        return false;
    }
    out = *base;
    for (std::size_t rowIndex = 0; rowIndex < out.size(); ++rowIndex) {
        augmentRow(rowIndex, out[rowIndex]);
    }
    return !out.empty();
}

/// Register a side-channel at pipeline construction time. Used by
/// CompressPipeline::Builder::withSideChannel<T>(...). The runtime value
/// carried in `SideCell` must match `cellType` at fill time (debug-checked).
struct SideChannelSpec {
    std::string name;
    enum class Type : std::uint8_t {
        Bool, Int8, Int16, Int32, UInt16, Float, Vec3f, Bytes
    } cellType;
    enum class Shape : std::uint8_t {
        Scalar,  ///< one cell per symbol
        Vec3     ///< one Vec3f (MCGAL::Point) cell per symbol, emitted as 3 Parquet columns
    } shape = Shape::Scalar;

    /// @brief Per-Round vs Per-Symbol. Today everything is PerRoundList because
    /// rounds are the Parquet row-group unit; we keep the enum for future
    /// Parquet-V3 variants that might want scalar-per-group metadata.
    enum class Scope : std::uint8_t {
        PerRoundList
    } scope = Scope::PerRoundList;
};

inline void appendPMSFSideChannelSpec(std::vector<SideChannelSpec>& specs,
                                       SideChannelSpec spec) {
    specs.push_back(std::move(spec));
}

}  // namespace pmsf

#endif  // PMSF_CORE_SYMBOL_H
