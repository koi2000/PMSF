/*
 * @file        ConnectivityBlobContainer.cpp
 * @brief       ConnectivityBlobContainer v3 实现。
 */

#include "ConnectivityBlobContainer.h"

#include <cstring>
#include <stdexcept>
#include <string>

namespace pmsf {

// ----------------------- Header -------------------------------------------

void ConnectivityBlobHeader::writeTo(BitWriter& w) const {
    if (stream_offsets.size() !=
        static_cast<size_t>(num_rounds) * static_cast<size_t>(num_streams) + 1) {
        throw std::runtime_error(
            "ConnectivityBlobHeader::writeTo: stream_offsets size mismatch "
            "(expect R*S+1, got " +
            std::to_string(stream_offsets.size()) + ")");
    }
    w.writeBits(magic, 32);
    w.writeBits(blob_layout, 16);
    w.writeBits(scheme_version, 16);
    w.writeBits(num_rounds, 32);
    w.writeBits(num_streams, 32);
    for (uint32_t off : stream_offsets) {
        w.writeBits(off, 32);
    }
}

ConnectivityBlobHeader ConnectivityBlobHeader::readFrom(BitReader& r) {
    ConnectivityBlobHeader h;
    h.magic = static_cast<uint32_t>(r.readBits(32));
    if (h.magic != kConnectivityMagic) {
        throw std::runtime_error(
            "ConnectivityBlobHeader::readFrom: bad magic 0x" +
            std::to_string(h.magic) + ", expected 'CONN' (0x4E4E4F43)");
    }
    h.blob_layout    = static_cast<uint16_t>(r.readBits(16));
    h.scheme_version = static_cast<uint16_t>(r.readBits(16));
    h.num_rounds     = static_cast<uint32_t>(r.readBits(32));
    h.num_streams    = static_cast<uint32_t>(r.readBits(32));
    const size_t off_count =
        static_cast<size_t>(h.num_rounds) * static_cast<size_t>(h.num_streams) + 1;
    h.stream_offsets.resize(off_count);
    for (size_t i = 0; i < off_count; ++i) {
        h.stream_offsets[i] = static_cast<uint32_t>(r.readBits(32));
    }
    return h;
}

size_t ConnectivityBlobHeader::headerByteSize() const {
    // 4 (magic) + 2 (blob_layout) + 2 (scheme_version) + 4 (num_rounds)
    //   + 4 (num_streams) + 4 * off_count
    const size_t off_count =
        static_cast<size_t>(num_rounds) * static_cast<size_t>(num_streams) + 1;
    return 4 + 2 + 2 + 4 + 4 + 4 * off_count;
}

// ----------------------- Builder ------------------------------------------

ConnectivityBlobBuilder::ConnectivityBlobBuilder(int blob_layout,
                                                 int scheme_version,
                                                 int num_rounds,
                                                 int num_streams)
    : num_rounds_(num_rounds),
      num_streams_(num_streams),
      blob_layout_(blob_layout),
      scheme_version_(scheme_version) {
    if (num_rounds < 0 || num_streams < 0) {
        throw std::invalid_argument("ConnectivityBlobBuilder: negative rounds/streams");
    }
    cells_.assign(num_rounds_, std::vector<std::vector<uint8_t>>(num_streams_));
    filled_.assign(static_cast<size_t>(num_rounds_) * num_streams_, 0u);
}

ConnectivityBlobBuilder::ConnectivityBlobBuilder(
    const PaperConnectivityBlobSpec& spec)
    : ConnectivityBlobBuilder(spec.blobLayout,
                              spec.schemeVersion,
                              spec.roundCount,
                              spec.streamCount) {}

void ConnectivityBlobBuilder::appendStreamBytes(int round_id,
                                                int stream_id,
                                                const std::vector<uint8_t>& bytes) {
    appendStreamBytes(round_id, stream_id, std::span<const uint8_t>(bytes.data(), bytes.size()));
}

void ConnectivityBlobBuilder::appendStreamBytes(int round_id,
                                                int stream_id,
                                                std::span<const uint8_t> bytes) {
    if (round_id < 0 || round_id >= num_rounds_ ||
        stream_id < 0 || stream_id >= num_streams_) {
        throw std::out_of_range(
            "ConnectivityBlobBuilder::appendStreamBytes: (round,stream) out of range");
    }
    const size_t idx = static_cast<size_t>(round_id) * num_streams_ + stream_id;
    if (filled_[idx]) {
        throw std::runtime_error(
            "ConnectivityBlobBuilder::appendStreamBytes: (round=" +
            std::to_string(round_id) + ", stream=" +
            std::to_string(stream_id) + ") written twice");
    }
    cells_[round_id][stream_id].assign(bytes.begin(), bytes.end());
    filled_[idx] = 1u;
}

std::vector<uint8_t> ConnectivityBlobBuilder::finalize() {
    // 1) 构造 header (stream_offsets 计算)。
    ConnectivityBlobHeader h;
    h.magic          = kConnectivityMagic;
    h.blob_layout    = static_cast<uint16_t>(blob_layout_);
    h.scheme_version = static_cast<uint16_t>(scheme_version_);
    h.num_rounds     = static_cast<uint32_t>(num_rounds_);
    h.num_streams    = static_cast<uint32_t>(num_streams_);

    const size_t off_count =
        static_cast<size_t>(num_rounds_) * static_cast<size_t>(num_streams_) + 1;
    h.stream_offsets.assign(off_count, 0);
    uint32_t cursor = 0;
    for (int r = 0; r < num_rounds_; ++r) {
        for (int s = 0; s < num_streams_; ++s) {
            h.stream_offsets[static_cast<size_t>(r) * num_streams_ + s] = cursor;
            cursor += static_cast<uint32_t>(cells_[r][s].size());
        }
    }
    h.stream_offsets[off_count - 1] = cursor;  // payload total size sentinel

    // 2) 写 header。
    BitWriter w;
    h.writeTo(w);

    // 3) 追加 payload (按 (r, s) 顺序串接)。
    for (int r = 0; r < num_rounds_; ++r) {
        for (int s = 0; s < num_streams_; ++s) {
            const auto& cell = cells_[r][s];
            if (!cell.empty()) {
                w.writeBytes(cell.data(), cell.size());
            }
        }
    }

    auto blob = w.finalize();

    // 4) 复位 builder 状态（允许复用）。
    cells_.clear();
    filled_.clear();
    num_rounds_  = 0;
    num_streams_ = 0;
    return blob;
}

// ----------------------- Reader -------------------------------------------

ConnectivityBlobReader::ConnectivityBlobReader(const std::vector<uint8_t>& blob)
    : owned_(blob) {
    view_ = std::span<const uint8_t>(owned_.data(), owned_.size());
    parse();
}

ConnectivityBlobReader::ConnectivityBlobReader(std::span<const uint8_t> blob)
    : view_(blob) {
    parse();
}

void ConnectivityBlobReader::parse() {
    if (view_.size() < 16) {
        throw std::runtime_error(
            "ConnectivityBlobReader: blob too small for header (" +
            std::to_string(view_.size()) + " bytes)");
    }
    BitReader r(view_.data(), view_.size());
    header_          = ConnectivityBlobHeader::readFrom(r);
    const size_t hsz = header_.headerByteSize();
    if (hsz > view_.size()) {
        throw std::runtime_error(
            "ConnectivityBlobReader: header size exceeds blob (" +
            std::to_string(hsz) + " > " + std::to_string(view_.size()) + ")");
    }
    payload_      = view_.data() + hsz;
    payload_size_ = view_.size() - hsz;
    // sentinel 对 payload 总长的校验
    if (!header_.stream_offsets.empty()) {
        uint32_t payload_declared = header_.stream_offsets.back();
        if (payload_declared > payload_size_) {
            throw std::runtime_error(
                "ConnectivityBlobReader: declared payload size (" +
                std::to_string(payload_declared) + ") > available (" +
                std::to_string(payload_size_) + ")");
        }
    }
}

std::span<const uint8_t> ConnectivityBlobReader::getStream(int round_id, int stream_id) const {
    const int R = static_cast<int>(header_.num_rounds);
    const int S = static_cast<int>(header_.num_streams);
    if (round_id < 0 || round_id >= R || stream_id < 0 || stream_id >= S) {
        throw std::out_of_range("ConnectivityBlobReader::getStream: out of range");
    }
    const size_t idx  = static_cast<size_t>(round_id) * S + stream_id;
    const uint32_t b  = header_.stream_offsets[idx];
    const uint32_t e  = (stream_id + 1 < S)
                            ? header_.stream_offsets[idx + 1]
                            : header_.stream_offsets[static_cast<size_t>(round_id + 1) * S];
    if (e < b || e > payload_size_) {
        throw std::runtime_error("ConnectivityBlobReader::getStream: corrupt offsets");
    }
    return std::span<const uint8_t>(payload_ + b, e - b);
}

size_t ConnectivityBlobReader::roundTotalBytes(int round_id) const {
    const int R = static_cast<int>(header_.num_rounds);
    const int S = static_cast<int>(header_.num_streams);
    if (round_id < 0 || round_id >= R) {
        throw std::out_of_range("ConnectivityBlobReader::roundTotalBytes: out of range");
    }
    const uint32_t b = header_.stream_offsets[static_cast<size_t>(round_id) * S];
    const uint32_t e = header_.stream_offsets[static_cast<size_t>(round_id + 1) * S];
    return static_cast<size_t>(e - b);
}

}  // namespace pmsf
