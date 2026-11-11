#include "compress/CanonicalArrowTableBuilder.h"

#include "storage/PMSFSymbolStorageAdapter.h"

#include <arrow/builder.h>

namespace pmsf {

namespace {

void appendI32(std::vector<std::uint8_t>& out, std::int32_t v) {
    const std::uint8_t* p = reinterpret_cast<const std::uint8_t*>(&v);
    out.insert(out.end(), p, p + sizeof(v));
}

}  // namespace

CanonicalArrowTableBuilder::CanonicalArrowTableBuilder(
    const CanonicalSymbolPayload& payload)
    : payload_(payload) {}

std::shared_ptr<arrow::Table> CanonicalArrowTableBuilder::buildBitTable() const {
    auto schema = arrow::schema({
        arrow::field(kParquetGroupIdColumn, arrow::int32(), false),
        arrow::field(kParquetRoundIdColumn, arrow::int32(), false),
        arrow::field(kParquetStreamIdColumn, arrow::int32(), false),
        arrow::field(kParquetEncodingModeColumn, arrow::int8(), false),
        arrow::field(kParquetBitDataColumn, arrow::binary(), false),
    });

    arrow::Int32Builder gidB, ridB, sidB;
    arrow::Int8Builder modeB;
    arrow::BinaryBuilder dataB;

    for (const auto& s : payload_.bitStreams) {
        (void)gidB.Append(payload_.groupId);
        (void)ridB.Append(s.roundId);
        (void)sidB.Append(s.streamId);
        (void)modeB.Append(static_cast<std::int8_t>(s.alphabetBits == 1 ? 0 : 1));
        (void)dataB.Append(reinterpret_cast<const char*>(s.bytes.data()),
                           static_cast<std::int32_t>(s.bytes.size()));
    }

    std::shared_ptr<arrow::Array> gidA, ridA, sidA, modeA, dataA;
    (void)gidB.Finish(&gidA);
    (void)ridB.Finish(&ridA);
    (void)sidB.Finish(&sidA);
    (void)modeB.Finish(&modeA);
    (void)dataB.Finish(&dataA);
    return arrow::Table::Make(schema, {gidA, ridA, sidA, modeA, dataA},
                              static_cast<int64_t>(payload_.bitStreams.size()));
}

std::shared_ptr<arrow::Table> CanonicalArrowTableBuilder::buildResidualTable() const {
    auto schema = arrow::schema({
        arrow::field(kParquetGroupIdColumn, arrow::int32(), false),
        arrow::field(kParquetRoundIdColumn, arrow::int32(), false),
        arrow::field(kParquetEncodingModeColumn, arrow::int8(), false),
        arrow::field(kParquetResidualDataColumn, arrow::binary(), false),
    });

    arrow::Int32Builder gidB, ridB;
    arrow::Int8Builder modeB;
    arrow::BinaryBuilder dataB;

    for (const auto& g : payload_.geometry) {
        std::vector<std::uint8_t> bytes;
        bytes.insert(bytes.end(), g.pointBytes.begin(), g.pointBytes.end());
        bytes.insert(bytes.end(), g.vertexPointBytes.begin(),
                     g.vertexPointBytes.end());

        (void)gidB.Append(payload_.groupId);
        (void)ridB.Append(g.roundId);
        (void)modeB.Append(g.coordFormat);
        (void)dataB.Append(reinterpret_cast<const char*>(bytes.data()),
                           static_cast<std::int32_t>(bytes.size()));
    }

    std::shared_ptr<arrow::Array> gidA, ridA, modeA, dataA;
    (void)gidB.Finish(&gidA);
    (void)ridB.Finish(&ridA);
    (void)modeB.Finish(&modeA);
    (void)dataB.Finish(&dataA);
    return arrow::Table::Make(schema, {gidA, ridA, modeA, dataA},
                              static_cast<int64_t>(payload_.geometry.size()));
}

std::shared_ptr<arrow::Table> CanonicalArrowTableBuilder::buildLodTable() const {
    auto schema = arrow::schema({
        arrow::field(kParquetGroupIdColumn, arrow::int32(), false),
        arrow::field(kParquetRoundIdColumn, arrow::int32(), false),
        arrow::field(kParquetBitStartColumn, arrow::list(arrow::int32()), false),
        arrow::field(kParquetBitLenColumn, arrow::list(arrow::int32()), false),
        arrow::field(kParquetResidualStartColumn, arrow::int32(), false),
        arrow::field(kParquetResidualLenColumn, arrow::int32(), false),
        arrow::field("header_blob", arrow::binary(), false),
    });

    arrow::Int32Builder gidB, ridB, resStartB, resLenB;
    arrow::BinaryBuilder headerB;
    auto bitStartVb = std::make_shared<arrow::Int32Builder>();
    auto bitLenVb = std::make_shared<arrow::Int32Builder>();
    arrow::ListBuilder bitStartLb(arrow::default_memory_pool(), bitStartVb);
    arrow::ListBuilder bitLenLb(arrow::default_memory_pool(), bitLenVb);

    // Group bit streams by round id.
    std::unordered_map<std::int32_t, std::vector<const CanonicalEncodedStream*>> byRound;
    for (const auto& s : payload_.bitStreams) {
        byRound[s.roundId].push_back(&s);
    }

    for (const auto& g : payload_.geometry) {
        (void)gidB.Append(payload_.groupId);
        (void)ridB.Append(g.roundId);
        (void)bitStartLb.Append();
        (void)bitLenLb.Append();
        int cur = 0;
        for (const auto* s : byRound[g.roundId]) {
            (void)bitStartVb->Append(cur);
            (void)bitLenVb->Append(static_cast<std::int32_t>(s->bytes.size()));
            cur += static_cast<std::int32_t>(s->bytes.size());
        }
        (void)resStartB.Append(0);
        std::int32_t resLen = static_cast<std::int32_t>(
            g.pointBytes.size() + g.vertexPointBytes.size());
        (void)resLenB.Append(resLen);
        (void)headerB.Append("", 0);
    }

    std::shared_ptr<arrow::Array> gidA, ridA, bitStartA, bitLenA, resStartA, resLenA, headerA;
    (void)gidB.Finish(&gidA);
    (void)ridB.Finish(&ridA);
    (void)bitStartLb.Finish(&bitStartA);
    (void)bitLenLb.Finish(&bitLenA);
    (void)resStartB.Finish(&resStartA);
    (void)resLenB.Finish(&resLenA);
    (void)headerB.Finish(&headerA);
    return arrow::Table::Make(schema,
                              {gidA, ridA, bitStartA, bitLenA, resStartA, resLenA, headerA},
                              static_cast<int64_t>(payload_.geometry.size()));
}

std::shared_ptr<arrow::Table> CanonicalArrowTableBuilder::buildSideTable() const {
    auto schema = arrow::schema({
        arrow::field(kParquetGroupIdColumn, arrow::int32(), false),
        arrow::field(kParquetRoundIdColumn, arrow::int32(), false),
        arrow::field("channel_name", arrow::utf8(), false),
        arrow::field(kParquetEncodingModeColumn, arrow::int8(), false),
        arrow::field("side_data", arrow::binary(), false),
    });

    arrow::Int32Builder gidB, ridB;
    arrow::StringBuilder nameB;
    arrow::Int8Builder modeB;
    arrow::BinaryBuilder dataB;

    for (const auto& s : payload_.sideChannels) {
        // Preserve one row per side channel per round so the decoder can rely on
        // the registered spec order (round-major) when reading the table back.
        (void)gidB.Append(payload_.groupId);
        (void)ridB.Append(s.roundId);
        (void)nameB.Append(s.name);
        (void)modeB.Append(static_cast<std::int8_t>(s.alphabetBits == 1 ? 0 : 1));
        (void)dataB.Append(reinterpret_cast<const char*>(s.bytes.data()),
                           static_cast<std::int32_t>(s.bytes.size()));
    }

    std::shared_ptr<arrow::Array> gidA, ridA, nameA, modeA, dataA;
    (void)gidB.Finish(&gidA);
    (void)ridB.Finish(&ridA);
    (void)nameB.Finish(&nameA);
    (void)modeB.Finish(&modeA);
    (void)dataB.Finish(&dataA);
    return arrow::Table::Make(schema, {gidA, ridA, nameA, modeA, dataA},
                              nameA->length());
}

}  // namespace pmsf
