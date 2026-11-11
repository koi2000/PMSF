#ifndef PMSF_STORAGE_SEGMENTED_GROUP_CHANNEL_H
#define PMSF_STORAGE_SEGMENTED_GROUP_CHANNEL_H

// The segmented group channel is shared by the binary layout probe and the
// segmented decoder.  Keep this header independent of the mesh and codec
// implementations so it can also be tested as a standalone wire parser.

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <limits>
#include <utility>
#include <vector>

namespace pmsf {

struct SegmentedGroupRun {
    std::int8_t gid = 0;
    std::uint32_t length = 0;
};

struct ParsedSegmentedGroupChannel {
    int baseFacetCount = 0;
    std::uint32_t runCount = 0;
    bool varintRunLengths = false;
    std::size_t endOffset = 0;
    std::vector<SegmentedGroupRun> runs;
};

namespace segmented_group_channel_detail {

inline bool readInt(const char* data,
                    std::size_t size,
                    std::size_t& at,
                    int& out) {
    static_assert(sizeof(int) == 4,
                  "segmented blobs use a four-byte native int");
    if (data == nullptr || at > size || size - at < sizeof(int)) {
        return false;
    }
    std::memcpy(&out, data + at, sizeof(int));
    at += sizeof(int);
    return true;
}

inline bool readCanonicalVarUInt32(const char* data,
                                   std::size_t size,
                                   std::size_t& at,
                                   std::uint32_t& out) {
    if (data == nullptr || at > size) {
        return false;
    }

    std::size_t cursor = at;
    std::uint32_t value = 0;
    for (unsigned byteIndex = 0; byteIndex < 5; ++byteIndex) {
        if (cursor >= size) {
            return false;
        }

        const std::uint8_t byte =
            static_cast<std::uint8_t>(data[cursor++]);
        const std::uint8_t payload =
            static_cast<std::uint8_t>(byte & 0x7fu);

        // A uint32 has only four bits available in its fifth ULEB128 byte.
        // The continuation bit must also be clear: there is no sixth byte.
        if (byteIndex == 4 &&
            ((byte & 0x80u) != 0 || payload > 0x0fu)) {
            return false;
        }

        value |= static_cast<std::uint32_t>(payload) << (byteIndex * 7);
        if ((byte & 0x80u) == 0) {
            // A terminating zero payload after another byte is an overlong
            // representation (for example 0x81 0x00 for the value 1).
            if (byteIndex != 0 && payload == 0) {
                return false;
            }
            at = cursor;
            out = value;
            return true;
        }
    }
    return false;
}

inline bool readRunLength(const char* data,
                          std::size_t size,
                          std::size_t& at,
                          bool varintRunLengths,
                          std::uint32_t& out) {
    if (varintRunLengths) {
        return readCanonicalVarUInt32(data, size, at, out);
    }

    int fixedLength = 0;
    if (!readInt(data, size, at, fixedLength) || fixedLength <= 0) {
        return false;
    }
    out = static_cast<std::uint32_t>(fixedLength);
    return true;
}

inline bool readGid(const char* data,
                    std::size_t size,
                    std::size_t& at,
                    std::int8_t& out) {
    if (data == nullptr || at > size || size - at < sizeof(out)) {
        return false;
    }
    std::memcpy(&out, data + at, sizeof(out));
    at += sizeof(out);
    return true;
}

}  // namespace segmented_group_channel_detail

/// Parse and validate the base-facet ownership channel.
///
/// The channel starts with two native four-byte ints:
///   [baseFacetCount][rleRunCount]
///
/// A positive rleRunCount uses the legacy `(gid:int8, length:int32)` form.
/// A negative rleRunCount uses `(gid:int8, length:ULEB128<uint32>)`.
/// The parser validates the complete channel before allocating `out->runs`
/// or returning the end offset.  In particular, all lengths are bounded by
/// the actual live-facet count and their sum must own every live facet.
///
/// `offset` is updated only on success.  `out` may be null when the caller
/// only needs validation.
inline bool parseSegmentedGroupChannel(
    const char* data,
    std::size_t size,
    std::size_t& offset,
    int groupCount,
    int liveFacetCount,
    ParsedSegmentedGroupChannel* out = nullptr) {
    using namespace segmented_group_channel_detail;

    if (data == nullptr || groupCount <= 0 || groupCount > 128 ||
        liveFacetCount < 0 || offset > size) {
        return false;
    }

    const std::size_t channelStart = offset;
    std::size_t cursor = channelStart;
    int baseFacetCount = 0;
    int encodedRunCount = 0;
    if (!readInt(data, size, cursor, baseFacetCount) ||
        !readInt(data, size, cursor, encodedRunCount) ||
        baseFacetCount < 0 ||
        baseFacetCount != liveFacetCount ||
        encodedRunCount == std::numeric_limits<int>::min()) {
        return false;
    }

    const bool varintRunLengths = encodedRunCount < 0;
    const std::uint32_t runCount =
        varintRunLengths
            ? static_cast<std::uint32_t>(
                  -(static_cast<std::int64_t>(encodedRunCount)))
            : static_cast<std::uint32_t>(encodedRunCount);

    // Positive lengths imply that a non-empty facet set needs at least one
    // run, while an empty facet set has exactly zero runs.
    if ((baseFacetCount == 0 && runCount != 0) ||
        (baseFacetCount != 0 && runCount == 0) ||
        runCount > static_cast<std::uint32_t>(baseFacetCount)) {
        return false;
    }

    // Every run consumes one gid byte and at least one length byte in the
    // varint form, or one gid byte and four length bytes in the legacy form.
    // This bounds the scan before any run vector can be allocated.
    const std::size_t minimumBytesPerRun = varintRunLengths ? 2u : 5u;
    const std::uint64_t minimumRunCapacity =
        static_cast<std::uint64_t>((size - cursor) / minimumBytesPerRun);
    if (static_cast<std::uint64_t>(runCount) > minimumRunCapacity) {
        return false;
    }

    // First pass: validate ownership without allocating a run vector.
    const std::size_t runsStart = cursor;
    std::uint64_t runSum = 0;
    int previousGid = -1;
    for (std::uint32_t run = 0; run < runCount; ++run) {
        std::int8_t gid = 0;
        if (!readGid(data, size, cursor, gid)) {
            return false;
        }
        const int gidValue = static_cast<int>(gid);
        if (gidValue < 0 || gidValue >= groupCount ||
            (run != 0 && gidValue == previousGid)) {
            return false;
        }

        std::uint32_t runLength = 0;
        if (!readRunLength(data, size, cursor, varintRunLengths, runLength) ||
            runLength == 0 ||
            runLength > static_cast<std::uint32_t>(baseFacetCount) ||
            runSum + runLength >
                static_cast<std::uint64_t>(baseFacetCount)) {
            return false;
        }
        runSum += runLength;
        previousGid = gidValue;
    }

    if (runSum != static_cast<std::uint64_t>(baseFacetCount)) {
        return false;
    }

    // Second pass: only now materialize the validated ownership runs.
    if (out != nullptr) {
        std::vector<SegmentedGroupRun> runs;
        runs.reserve(static_cast<std::size_t>(runCount));
        cursor = runsStart;
        for (std::uint32_t run = 0; run < runCount; ++run) {
            std::int8_t gid = 0;
            std::uint32_t runLength = 0;
            const bool gidOk = readGid(data, size, cursor, gid);
            const bool lengthOk =
                gidOk && readRunLength(data, size, cursor,
                                       varintRunLengths, runLength);
            if (!lengthOk) {
                // The first pass already proved this impossible.  Keep the
                // failure path defensive in case this helper is changed.
                return false;
            }
            runs.push_back({gid, runLength});
        }
        out->baseFacetCount = baseFacetCount;
        out->runCount = runCount;
        out->varintRunLengths = varintRunLengths;
        out->endOffset = cursor;
        out->runs = std::move(runs);
    }

    offset = cursor;
    return true;
}

}  // namespace pmsf

#endif
