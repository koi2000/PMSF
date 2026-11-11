#include "compress/CanonicalArrowTableReader.h"

#include "storage/PMSFSymbolStorageAdapter.h"

#include <arrow/array.h>
#include <cstring>

namespace pmsf {

namespace {

std::int32_t readI32(const std::uint8_t*& p, const std::uint8_t* end) {
    if (p + sizeof(std::int32_t) > end) {
        throw std::runtime_error("CanonicalArrowTableReader: truncated i32");
    }
    std::int32_t v = 0;
    std::memcpy(&v, p, sizeof(v));
    p += sizeof(v);
    return v;
}

std::int8_t readI8(const std::uint8_t*& p, const std::uint8_t* end) {
    if (p >= end) {
        throw std::runtime_error("CanonicalArrowTableReader: truncated i8");
    }
    std::int8_t v = static_cast<std::int8_t>(*p);
    ++p;
    return v;
}

const SymbolStreamSpec* findSpecByRole(const std::vector<SymbolStreamSpec>& specs,
                                       SymbolStreamRole role) {
    for (const auto& s : specs) {
        if (s.role == role) return &s;
    }
    return nullptr;
}

const SymbolStreamSpec* findSpecByName(const std::vector<SymbolStreamSpec>& specs,
                                       const std::string& name) {
    for (const auto& s : specs) {
        if (s.name == name) return &s;
    }
    return nullptr;
}

}  // namespace

CanonicalSymbolPayload CanonicalArrowTableReader::readPayload(
    const arrow::Table& bitTable,
    const arrow::Table& residualTable,
    const arrow::Table* lodTable,
    const arrow::Table* sideTable,
    const std::vector<SymbolStreamSpec>& specs) {
    (void)lodTable;

    CanonicalSymbolPayload payload;
    payload.groupId = 0;
    payload.numRounds = 0;

    // ---------- bit table ----------
    auto gidCol = bitTable.GetColumnByName(kParquetGroupIdColumn);
    auto ridCol = bitTable.GetColumnByName(kParquetRoundIdColumn);
    auto sidCol = bitTable.GetColumnByName(kParquetStreamIdColumn);
    auto modeCol = bitTable.GetColumnByName(kParquetEncodingModeColumn);
    auto dataCol = bitTable.GetColumnByName(kParquetBitDataColumn);
    if (gidCol && ridCol && sidCol && dataCol) {
        auto gidArr = std::static_pointer_cast<arrow::Int32Array>(gidCol->chunk(0));
        auto ridArr = std::static_pointer_cast<arrow::Int32Array>(ridCol->chunk(0));
        auto sidArr = std::static_pointer_cast<arrow::Int32Array>(sidCol->chunk(0));
        auto modeArr = modeCol
                           ? std::static_pointer_cast<arrow::Int8Array>(modeCol->chunk(0))
                           : nullptr;
        auto dataArr = std::static_pointer_cast<arrow::BinaryArray>(dataCol->chunk(0));

        for (int64_t r = 0; r < bitTable.num_rows(); ++r) {
            CanonicalEncodedStream stream;
            stream.roundId = ridArr->Value(r);
            stream.streamId = sidArr->Value(r);
            stream.alphabetBits = modeArr ? static_cast<std::int32_t>(modeArr->Value(r)) : 0;

            const auto role = static_cast<SymbolStreamRole>(stream.streamId);
            stream.role = role;
            if (const auto* spec = findSpecByRole(specs, role)) {
                stream.name = spec->name;
            }

            int32_t len = 0;
            const std::uint8_t* ptr =
                reinterpret_cast<const std::uint8_t*>(dataArr->GetValue(r, &len));
            stream.bytes.assign(ptr, ptr + len);

            payload.numRounds = std::max(payload.numRounds, stream.roundId + 1);
            if (gidArr && r < gidArr->length()) {
                payload.groupId = gidArr->Value(r);
            }
            payload.bitStreams.push_back(std::move(stream));
        }
    }

    // ---------- residual table (geometry) ----------
    auto resGidCol = residualTable.GetColumnByName(kParquetGroupIdColumn);
    auto resRidCol = residualTable.GetColumnByName(kParquetRoundIdColumn);
    auto resModeCol = residualTable.GetColumnByName(kParquetEncodingModeColumn);
    auto resDataCol = residualTable.GetColumnByName(kParquetResidualDataColumn);
    if (resRidCol && resDataCol) {
        auto ridArr = std::static_pointer_cast<arrow::Int32Array>(resRidCol->chunk(0));
        auto dataArr = std::static_pointer_cast<arrow::BinaryArray>(resDataCol->chunk(0));
        auto modeArr = resModeCol
                           ? std::static_pointer_cast<arrow::Int8Array>(resModeCol->chunk(0))
                           : nullptr;

        for (int64_t r = 0; r < residualTable.num_rows(); ++r) {
            CanonicalEncodedGeometry geom;
            geom.roundId = ridArr->Value(r);

            int32_t len = 0;
            const std::uint8_t* ptr =
                reinterpret_cast<const std::uint8_t*>(dataArr->GetValue(r, &len));
            const std::uint8_t* p = ptr;
            const std::uint8_t* end = ptr + len;

            if (len < static_cast<int32_t>(sizeof(std::int8_t) + sizeof(std::int32_t))) {
                throw std::runtime_error("CanonicalArrowTableReader: empty residual row");
            }

            // point sub-block: format + count + data
            const std::int8_t pointFormat = readI8(p, end);
            geom.coordFormat = pointFormat;
            const std::int32_t pointCount = readI32(p, end);
            if (pointCount < 0 || pointCount > 10000000) {
                throw std::runtime_error(
                    "CanonicalArrowTableReader: bogus pointCount=" +
                    std::to_string(pointCount) + " len=" + std::to_string(len));
            }
            geom.pointCount = pointCount;
            std::size_t pointDataLen = 0;
            if (pointFormat == 1) {
                const std::int8_t bits = *p;
                geom.coordBits = bits;
                pointDataLen = 1 + (static_cast<std::size_t>(pointCount) * 3 *
                                    static_cast<std::size_t>(bits) + 7) / 8;
            } else if (pointFormat == 3) {
                if (p + 1 + 1 + sizeof(std::int32_t) > end) {
                    throw std::runtime_error(
                        "CanonicalArrowTableReader: truncated residual bit-plane header");
                }
                if (static_cast<std::int8_t>(*p) != 0) {
                    throw std::runtime_error(
                        "CanonicalArrowTableReader: unsupported residual mode");
                }
                geom.coordBits = static_cast<std::int8_t>(*(p + 1));
                std::int32_t payloadLen = 0;
                std::memcpy(&payloadLen, p + 2, sizeof(payloadLen));
                const std::size_t headerAfterCount = 1 + 1 + sizeof(std::int32_t);
                pointDataLen = headerAfterCount + static_cast<std::size_t>(payloadLen);
            } else {
                pointDataLen = static_cast<std::size_t>(pointCount) * 3 * sizeof(float);
            }
            const std::size_t pointBytesLen =
                sizeof(std::int8_t) + sizeof(std::int32_t) + pointDataLen;
            (void)pointBytesLen;
            if (p + pointDataLen > end) {
                throw std::runtime_error("CanonicalArrowTableReader: truncated point bytes");
            }
            geom.pointBytes.assign(p - sizeof(std::int32_t) - sizeof(std::int8_t),
                                   p + pointDataLen);
            p += pointDataLen;

            // vertex point sub-block: format + count + data
            if (p >= end) {
                // no vertex points
                payload.numRounds = std::max(payload.numRounds, geom.roundId + 1);
                if (resGidCol && resGidCol->num_chunks() > 0 &&
                    resGidCol->chunk(0)->length() > r) {
                    auto gidArr =
                        std::static_pointer_cast<arrow::Int32Array>(resGidCol->chunk(0));
                    payload.groupId = gidArr->Value(r);
                }
                payload.geometry.push_back(std::move(geom));
                continue;
            }
            const std::int8_t vertexFormat = readI8(p, end);
            const std::int32_t vertexPointCount = readI32(p, end);
            if (vertexPointCount < 0 || vertexPointCount > 10000000) {
                throw std::runtime_error(
                    "CanonicalArrowTableReader: bogus vertexPointCount=" +
                    std::to_string(vertexPointCount));
            }
            geom.vertexPointCount = vertexPointCount;
            std::size_t vertexDataLen = 0;
            if (vertexFormat == 1) {
                const std::int8_t bits = *p;
                vertexDataLen = 1 + (static_cast<std::size_t>(vertexPointCount) * 3 *
                                     static_cast<std::size_t>(bits) + 7) / 8;
            } else if (vertexFormat == 3) {
                if (p + 1 + 1 + sizeof(std::int32_t) > end) {
                    throw std::runtime_error(
                        "CanonicalArrowTableReader: truncated residual vertex bit-plane header");
                }
                if (static_cast<std::int8_t>(*p) != 0) {
                    throw std::runtime_error(
                        "CanonicalArrowTableReader: unsupported residual mode");
                }
                std::int32_t payloadLen = 0;
                std::memcpy(&payloadLen, p + 2, sizeof(payloadLen));
                const std::size_t headerAfterCount = 1 + 1 + sizeof(std::int32_t);
                vertexDataLen = headerAfterCount + static_cast<std::size_t>(payloadLen);
            } else {
                vertexDataLen = static_cast<std::size_t>(vertexPointCount) * 3 * sizeof(float);
            }
            if (p + vertexDataLen > end) {
                throw std::runtime_error(
                    "CanonicalArrowTableReader: truncated vertex point bytes");
            }
            geom.vertexPointBytes.assign(
                p - sizeof(std::int32_t) - sizeof(std::int8_t), p + vertexDataLen);
            p += vertexDataLen;

            payload.numRounds = std::max(payload.numRounds, geom.roundId + 1);
            if (resGidCol && resGidCol->num_chunks() > 0 &&
                resGidCol->chunk(0)->length() > r) {
                auto gidArr =
                    std::static_pointer_cast<arrow::Int32Array>(resGidCol->chunk(0));
                payload.groupId = gidArr->Value(r);
            }
            payload.geometry.push_back(std::move(geom));
        }
    }

    // ---------- side table ----------
    if (sideTable) {
        auto sideGidCol = sideTable->GetColumnByName(kParquetGroupIdColumn);
        auto sideRidCol = sideTable->GetColumnByName(kParquetRoundIdColumn);
        auto nameCol = sideTable->GetColumnByName("channel_name");
        auto modeCol2 = sideTable->GetColumnByName(kParquetEncodingModeColumn);
        auto dataCol2 = sideTable->GetColumnByName("side_data");
        if (sideRidCol && nameCol && dataCol2) {
            auto ridArr =
                std::static_pointer_cast<arrow::Int32Array>(sideRidCol->chunk(0));
            auto nameArr =
                std::static_pointer_cast<arrow::StringArray>(nameCol->chunk(0));
            auto modeArr = modeCol2
                               ? std::static_pointer_cast<arrow::Int8Array>(modeCol2->chunk(0))
                               : nullptr;
            auto dataArr =
                std::static_pointer_cast<arrow::BinaryArray>(dataCol2->chunk(0));

            for (int64_t r = 0; r < sideTable->num_rows(); ++r) {
                CanonicalEncodedSideChannel side;
                side.roundId = ridArr->Value(r);
                side.name = std::string(nameArr->Value(r));
                side.alphabetBits = modeArr ? static_cast<std::int32_t>(modeArr->Value(r)) : 0;

                if (const auto* spec = findSpecByName(specs, side.name)) {
                    side.cellType = spec->cellType;
                    side.shape = spec->shape;
                }

                int32_t len = 0;
                const std::uint8_t* ptr =
                    reinterpret_cast<const std::uint8_t*>(dataArr->GetValue(r, &len));
                side.bytes.assign(ptr, ptr + len);

                payload.numRounds = std::max(payload.numRounds, side.roundId + 1);
                if (sideGidCol && sideGidCol->num_chunks() > 0 &&
                    sideGidCol->chunk(0)->length() > r) {
                    auto gidArr =
                        std::static_pointer_cast<arrow::Int32Array>(sideGidCol->chunk(0));
                    payload.groupId = gidArr->Value(r);
                }
                payload.sideChannels.push_back(std::move(side));
            }
        }
    }

    return payload;
}

}  // namespace pmsf
