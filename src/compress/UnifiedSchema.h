/*
 * @file UnifiedSchema.h
 * @brief Parquet V3 schema definitions — dynamic side-channel columns.
 *
 * Phase 11 of the refactor. V3 is STRICTLY ADDITIVE relative to V2:
 *   - no existing algorithm is migrated in this phase,
 *   - no byte of V2 output changes,
 *   - V3 lives in parallel and is opt-in for new algorithms.
 *
 * Legacy single-file V3 prototype adds over V2:
 *   - typed side-channel columns registered at build time (residual,
 *     codebook_idx, context_label, anchor_mask, ...),
 *   - a historical placeholder for a 5th optional `_codebook.parquet`.
 *     Algorithm migrations no longer use per-algorithm codebook tables:
 *     learned codebooks / rc containers are exposed as paper payload
 *     relations and written through UnifiedMultiTableSerializeOperator's
 *     `_v3_blob.parquet`,
 *   - a schema-descriptor row in `_metadata.parquet` so the V3
 *     deserializer can rebuild the schema at open time without any
 *     build-time knowledge.
 *
 * Implementation note: this header defines the declarative schema only.
 * The actual Serialize / Deserialize operators build on top of the Arrow
 * APIs and are in UnifiedSerializeOperator.{h,cpp} /
 * UnifiedDeserializeOperator.{h,cpp}.
 */
#ifndef PMSF_COMPRESS_UNIFIED_SCHEMA_H
#define PMSF_COMPRESS_UNIFIED_SCHEMA_H

#include "core/Symbol.h"

#include <string>
#include <vector>

namespace pmsf {

/// Legacy declarative V3 file layout. One SymbolStream -> five Parquet files
/// sharing the same basename:
///
///   <base>_metadata.parquet   schema descriptor + compression params
///   <base>_base_mesh.parquet  base mesh per group (same semantics as V2)
///   <base>_symbols.parquet    per-round symbols + typed side-channels
///   <base>_boundary.parquet   inter-group stitching (same semantics as V2)
///   <base>_codebook.parquet   optional legacy placeholder
///
/// The first four files keep V2's Row-Group-per-group strategy so that
/// `setTargetGroupId(i)` and `setMaxRounds(k)` random-access continue to
/// work. Current algorithm backends should not add new codebook tables here;
/// compact binary payloads belong in the unified paper payload relation.
struct UnifiedLayout {
    std::string basename;
    std::vector<SideChannelSpec> sideChannels;
    bool hasCodebookTable = false;
};

inline constexpr const char* kUnifiedRoundColumn = "round";
inline constexpr const char* kUnifiedPositionColumn = "pos";
inline constexpr const char* kUnifiedChannelColumn = "channel";
inline constexpr const char* kUnifiedChannelNameColumn = "channel_name";
inline constexpr const char* kUnifiedBoolValueColumn = "bool_val";
inline constexpr const char* kUnifiedSymbolI32Column = "symbol_i32";
inline constexpr const char* kUnifiedPointXColumn = "px";
inline constexpr const char* kUnifiedPointYColumn = "py";
inline constexpr const char* kUnifiedPointZColumn = "pz";

/// Map a SideCell type to the Parquet physical type name (as a string,
/// so this header stays independent of <arrow/api.h>). The serializer
/// translates these to the actual Arrow types.
inline const char* parquetTypeName(SideChannelSpec::Type t) {
    switch (t) {
        case SideChannelSpec::Type::Bool:   return "bool";
        case SideChannelSpec::Type::Int8:   return "int8";
        case SideChannelSpec::Type::Int16:  return "int16";
        case SideChannelSpec::Type::Int32:  return "int32";
        case SideChannelSpec::Type::UInt16: return "uint16";
        case SideChannelSpec::Type::Float:  return "float32";
        case SideChannelSpec::Type::Vec3f:  return "float32_vec3";
        case SideChannelSpec::Type::Bytes:  return "binary";
    }
    return "unknown";
}

/// Canonical column names for a side-channel. Vec3f maps to 3 columns
/// (suffixed _x/_y/_z) so it compresses well under BYTE_STREAM_SPLIT+ZSTD,
/// exactly like V2's point_x/y/z.
inline std::vector<std::string> sideChannelColumnNames(const SideChannelSpec& s) {
    std::vector<std::string> names;
    const std::string base = std::string("sc_") + s.name;
    if (s.shape == SideChannelSpec::Shape::Vec3) {
        names.push_back(base + "_x");
        names.push_back(base + "_y");
        names.push_back(base + "_z");
    } else {
        names.push_back(base);
    }
    return names;
}

/// Descriptor row format persisted in `_metadata.parquet` so the V3
/// deserializer can reconstruct the side-channel schema at open time
/// without the algorithm having to re-declare it.
///
/// One row per registered side channel; `name`, `type_name`, `shape_name`
/// are stored as Arrow strings. At read time the deserializer round-trips
/// back into SideChannelSpec.
struct SideChannelDescriptorRow {
    std::string name;
    std::string type_name;  // see parquetTypeName()
    std::string shape_name; // "Scalar" | "Vec3"
};

inline SideChannelDescriptorRow makeDescriptor(const SideChannelSpec& s) {
    return {
        s.name,
        parquetTypeName(s.cellType),
        s.shape == SideChannelSpec::Shape::Vec3 ? "Vec3" : "Scalar",
    };
}

}  // namespace pmsf

#endif  // PMSF_COMPRESS_UNIFIED_SCHEMA_H
