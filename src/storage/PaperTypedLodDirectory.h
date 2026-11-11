// SPDX-License-Identifier: Apache-2.0
#pragma once

#include "storage/PaperStreamCompactCodec.h"
#include <limits>
#include <map>
#include <stdexcept>

namespace pmsf {

struct PaperTypedStreamSpan {
    const char* name;
    std::uint64_t offset;
    std::uint64_t bytes;
};

// Logical byte ranges in the fixed role order, followed by named extensions.
// Parquet column chunks have their own physical file offsets in the footer;
// these ranges index the relation payload, not hypothetical contiguous file I/O.
inline std::vector<PaperTypedStreamSpan> paperTypedStreamSpans(const PaperLodRow& row) {
    std::vector<PaperTypedStreamSpan> spans;
    std::uint64_t offset=row.payloadByteOffset;
    auto append=[&](const char* name,std::uint64_t bytes) {
        if(bytes>std::numeric_limits<std::uint64_t>::max()-offset)throw std::runtime_error("typed LOD directory offset overflow");
        spans.push_back({name,offset,bytes});offset+=bytes;
    };
    append("F",row.F.size());append("Et",row.Et.size());append("Ef",row.Ef.size());append("V",row.V.size());
    append("Cf",row.CfF.size()*sizeof(float));append("Cv",row.CvF.size()*sizeof(float));
    append("CfQ",row.CfQ.size()*sizeof(std::int32_t));append("CvQ",row.CvQ.size()*sizeof(std::int32_t));
    append("extensions",row.extension.empty()?0:packPaperExtensions(row).size());
    return spans;
}
inline std::uint64_t paperTypedLodByteLength(const PaperLodRow& row) {
    const auto spans=paperTypedStreamSpans(row);
    return spans.back().offset+spans.back().bytes-row.payloadByteOffset;
}
inline void assignPaperTypedLodDirectory(PaperStreamTable& table) {
    std::map<std::int32_t,std::uint64_t> cursor;
    for(auto& row:table.lods) {
        row.payloadByteOffset=cursor[row.partitionId];row.payloadByteLength=paperTypedLodByteLength(row);
        cursor[row.partitionId]=row.payloadByteOffset+row.payloadByteLength;
    }
}
inline void requirePaperTypedLodDirectory(const PaperStreamTable& table) {
    std::map<std::int32_t,std::uint64_t> cursor;
    std::map<std::int32_t,std::int32_t> lod;
    for(const auto& row:table.lods) {
        if(row.lodId!=lod[row.partitionId]++||row.payloadByteOffset!=cursor[row.partitionId]||row.payloadByteLength!=paperTypedLodByteLength(row))
            throw std::runtime_error("typed LOD payload directory mismatch");
        cursor[row.partitionId]=row.payloadByteOffset+row.payloadByteLength;
    }
}
} // namespace pmsf
