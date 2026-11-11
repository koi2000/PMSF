#include "core/CanonicalSymbolEncoder.h"

#include "entropy/RangeCoderEntropyCoder.h"
#include "entropy/StreamEntropyCoder.h"
#include "MeshUtils.h"
#include "../rangecoder/qsmodel.h"
#include "../rangecoder/rangecod.h"

#include <algorithm>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace pmsf {

namespace {


void appendI32(std::vector<std::uint8_t>& out, std::int32_t v) {
    const std::uint8_t* p = reinterpret_cast<const std::uint8_t*>(&v);
    out.insert(out.end(), p, p + sizeof(v));
}

std::int32_t readI32(const std::uint8_t*& p, const std::uint8_t* end) {
    if (p + sizeof(std::int32_t) > end) {
        throw std::runtime_error("CanonicalSymbolDecoder: truncated i32");
    }
    std::int32_t v = 0;
    std::memcpy(&v, p, sizeof(v));
    p += sizeof(v);
    return v;
}

void appendI8(std::vector<std::uint8_t>& out, std::int8_t v) {
    out.push_back(static_cast<std::uint8_t>(v));
}

std::int8_t readI8(const std::uint8_t*& p, const std::uint8_t* end) {
    if (p >= end) throw std::runtime_error("CanonicalSymbolDecoder: truncated i8");
    std::int8_t v = static_cast<std::int8_t>(*p);
    ++p;
    return v;
}

// Pack 0/1 symbols into bits. Returns bytes.
std::vector<std::uint8_t> bitPack(const std::vector<std::uint32_t>& symbols) {
    std::vector<std::uint8_t> out;
    out.resize((symbols.size() + 7) / 8, 0);
    for (std::size_t i = 0; i < symbols.size(); ++i) {
        if (symbols[i]) out[i / 8] |= static_cast<std::uint8_t>(1 << (i % 8));
    }
    return out;
}

std::vector<std::uint32_t> bitUnpack(const std::uint8_t* p,
                                     std::size_t byteLen,
                                     std::size_t count) {
    std::vector<std::uint32_t> out;
    out.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        std::size_t byteIdx = i / 8;
        if (byteIdx >= byteLen) break;
        out.push_back((p[byteIdx] >> (i % 8)) & 1u);
    }
    return out;
}

// Determine required unsigned bits for a signed/unsigned integer range.
int requiredBits(std::int64_t minVal, std::int64_t maxVal) {
    std::uint64_t range = static_cast<std::uint64_t>(maxVal - minVal);
    if (range == 0) return 1;
    int bits = 0;
    while ((1ULL << bits) <= range) ++bits;
    return std::clamp(bits, 1, 16);
}

// Pack a sequence of unsigned values, each using `bits` bits, into bytes.
std::vector<std::uint8_t> bitPackN(const std::vector<std::uint32_t>& values,
                                   int bits) {
    std::vector<std::uint8_t> out;
    if (values.empty()) return out;
    const std::uint64_t mask = (bits >= 32) ? ~std::uint64_t{0}
                                            : ((std::uint64_t{1} << bits) - 1);
    std::uint64_t buffer = 0;
    int used = 0;
    for (std::uint32_t v : values) {
        buffer |= (static_cast<std::uint64_t>(v) & mask) << used;
        used += bits;
        while (used >= 8) {
            out.push_back(static_cast<std::uint8_t>(buffer & 0xFF));
            buffer >>= 8;
            used -= 8;
        }
    }
    if (used > 0) {
        out.push_back(static_cast<std::uint8_t>(buffer & 0xFF));
    }
    return out;
}

std::vector<std::uint32_t> bitUnpackN(const std::uint8_t* p,
                                      std::size_t byteLen,
                                      std::size_t count,
                                      int bits) {
    std::vector<std::uint32_t> out;
    out.reserve(count);
    const std::uint64_t mask = (bits >= 32) ? ~std::uint64_t{0}
                                            : ((std::uint64_t{1} << bits) - 1);
    std::uint64_t buffer = 0;
    int used = 0;
    std::size_t idx = 0;
    while (out.size() < count) {
        while (used < bits && idx < byteLen) {
            buffer |= static_cast<std::uint64_t>(p[idx++]) << used;
            used += 8;
        }
        out.push_back(static_cast<std::uint32_t>(buffer & mask));
        buffer >>= bits;
        used -= bits;
    }
    (void)byteLen;
    return out;
}

// Encode a sequence of signed residuals using the legacy RD bit-plane model:
// one binary qsmodel per bit-plane, split into "before first 1" and "after
// first 1" clusters.  The input quantBits determines BP_BITS = quantBits + 1.
// Encode a binary (0/1) stream with a single adaptive qsmodel using the same
// parameters as the legacy Optimized path (alphabet=2, lg_totf=10).
std::vector<std::uint8_t> encodeBinaryRangeCode(const std::vector<std::uint32_t>& values) {
    qsmodel model;
    initqsmodel(&model, 2, 10, 1 << 9, nullptr, 1);

    std::vector<char> buf(std::max<std::size_t>(values.size() + 1024, 4096));
    std::size_t off = 0;
    rangecoder rc;
    rc.p_data = buf.data();
    rc.p_dataOffset = &off;
    start_encoding(&rc, 0, 0);

    for (std::uint32_t v : values) {
        int sym = v ? 1 : 0;
        int sy_f = 0, lt_f = 0;
        qsgetfreq(&model, sym, &sy_f, &lt_f);
        encode_shift(&rc, static_cast<freq>(sy_f), static_cast<freq>(lt_f), 10);
        qsupdate(&model, sym);
    }
    done_encoding(&rc);
    deleteqsmodel(&model);
    return std::vector<std::uint8_t>(buf.begin(), buf.begin() + off);
}

// Decode a binary range-code payload back into 0/1 values.
std::vector<std::uint32_t> decodeBinaryRangeCode(const std::uint8_t* data,
                                                 std::size_t len,
                                                 std::size_t count) {
    qsmodel model;
    initqsmodel(&model, 2, 10, 1 << 9, nullptr, 0);

    std::vector<char> staging(data, data + len);
    std::size_t off = 0;
    rangecoder rc;
    rc.p_data = staging.data();
    rc.p_dataOffset = &off;
    if (start_decoding(&rc) != 0) {
        deleteqsmodel(&model);
        throw std::runtime_error(
            "CanonicalSymbolDecoder: binary range-code start_decoding failed");
    }

    std::vector<std::uint32_t> out;
    out.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        int ch = decode_culshift(&rc, 10);
        int sym = qsgetsym(&model, ch);
        int sy_f = 0, lt_f = 0;
        qsgetfreq(&model, sym, &sy_f, &lt_f);
        decode_update(&rc, static_cast<freq>(sy_f), static_cast<freq>(lt_f), 1 << 10);
        qsupdate(&model, sym);
        out.push_back(sym ? 1u : 0u);
    }
    deleteqsmodel(&model);
    return out;
}

std::vector<std::uint8_t> encodeResidualBitPlane(
    const std::vector<std::int32_t>& values, int quantBits) {
    const int bpBits = quantBits + 1;
    std::vector<qsmodel> modelsFirst1(bpBits);
    std::vector<qsmodel> modelsAfter1(bpBits);
    for (int b = 0; b < bpBits; ++b) {
        initqsmodel(&modelsFirst1[b], 2, 10, 1 << 9, nullptr, 1);
        initqsmodel(&modelsAfter1[b], 2, 10, 1 << 9, nullptr, 1);
    }

    std::vector<char> buf(std::max<std::size_t>(values.size() * 6 + 4096, 8192));
    std::size_t off = 0;
    rangecoder rc;
    rc.p_data = buf.data();
    rc.p_dataOffset = &off;
    start_encoding(&rc, 0, 0);

    for (std::int32_t r : values) {
        std::uint32_t z = (static_cast<std::uint32_t>(r) << 1) ^
                          static_cast<std::uint32_t>(r >> 31);
        bool seenOne = false;
        for (int b = bpBits - 1; b >= 0; --b) {
            int bit = static_cast<int>((z >> b) & 1u);
            qsmodel* m = seenOne ? &modelsAfter1[b] : &modelsFirst1[b];
            int sy_f = 0, lt_f = 0;
            qsgetfreq(m, bit, &sy_f, &lt_f);
            encode_shift(&rc, static_cast<freq>(sy_f), static_cast<freq>(lt_f), 10);
            qsupdate(m, bit);
            if (bit) seenOne = true;
        }
    }
    done_encoding(&rc);
    for (int b = 0; b < bpBits; ++b) {
        deleteqsmodel(&modelsFirst1[b]);
        deleteqsmodel(&modelsAfter1[b]);
    }
    return std::vector<std::uint8_t>(buf.begin(), buf.begin() + off);
}

// Decode a bit-plane residual payload back into signed ints.
std::vector<std::int32_t> decodeResidualBitPlane(
    const std::uint8_t* data, std::size_t len, std::size_t count, int quantBits) {
    const int bpBits = quantBits + 1;
    std::vector<qsmodel> modelsFirst1(bpBits);
    std::vector<qsmodel> modelsAfter1(bpBits);
    for (int b = 0; b < bpBits; ++b) {
        initqsmodel(&modelsFirst1[b], 2, 10, 1 << 9, nullptr, 0);
        initqsmodel(&modelsAfter1[b], 2, 10, 1 << 9, nullptr, 0);
    }

    std::vector<char> staging(data, data + len);
    std::size_t off = 0;
    rangecoder rc;
    rc.p_data = staging.data();
    rc.p_dataOffset = &off;
    if (start_decoding(&rc) != 0) {
        for (int b = 0; b < bpBits; ++b) {
            deleteqsmodel(&modelsFirst1[b]);
            deleteqsmodel(&modelsAfter1[b]);
        }
        throw std::runtime_error(
            "CanonicalSymbolDecoder: residual bit-plane start_decoding failed");
    }

    std::vector<std::int32_t> out;
    out.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        std::uint32_t z = 0;
        bool seenOne = false;
        for (int b = bpBits - 1; b >= 0; --b) {
            int ch = decode_culshift(&rc, 10);
            qsmodel* m = seenOne ? &modelsAfter1[b] : &modelsFirst1[b];
            int sym = qsgetsym(m, ch);
            int sy_f = 0, lt_f = 0;
            qsgetfreq(m, sym, &sy_f, &lt_f);
            decode_update(&rc, static_cast<freq>(sy_f), static_cast<freq>(lt_f), 1 << 10);
            qsupdate(m, sym);
            if (sym) {
                z |= (1u << b);
                seenOne = true;
            }
        }
        // Un-ZigZag.
        out.push_back(static_cast<std::int32_t>((z >> 1) ^ (-static_cast<std::int32_t>(z & 1u))));
    }
    for (int b = 0; b < bpBits; ++b) {
        deleteqsmodel(&modelsFirst1[b]);
        deleteqsmodel(&modelsAfter1[b]);
    }
    return out;
}

}  // namespace

// ---------------------------------------------------------------------------
// CanonicalSymbolEncoder
// ---------------------------------------------------------------------------

CanonicalSymbolEncoder::CanonicalSymbolEncoder(
    const std::vector<SymbolStreamSpec>& specs)
    : specs_(specs) {
    payload_.groupId = 0;
    payload_.numRounds = 0;
}

void CanonicalSymbolEncoder::setQuantization(int bits,
                                             MCGAL::Point bboxMin,
                                             float quantStep) {
    useQuantization_ = (bits > 0);
    quantBits_ = std::clamp(bits, 0, 31);
    quantBboxMin_ = bboxMin;
    quantStep_ = quantStep;
}

void CanonicalSymbolEncoder::setUseResidual(bool use) {
    useResidual_ = use;
}

void CanonicalSymbolEncoder::addRound(const PaperRoundSymbols& round) {
    const std::int32_t r = roundsAdded_++;
    if (static_cast<std::size_t>(r) >= scalarBuffers_.size()) {
        scalarBuffers_.resize(r + 1);
        removedResidualBuffers_.resize(r + 1);
        shiftedResidualBuffers_.resize(r + 1);
        removedCoordBuffers_.resize(r + 1);
        shiftedCoordBuffers_.resize(r + 1);
        sideBuffers_.resize(r + 1);
    }

    auto& scalarBuf = scalarBuffers_[r];
    for (const auto& spec : specs_) {
        switch (spec.role) {
            case SymbolStreamRole::Facet:
                scalarBuf[spec.role].insert(
                    scalarBuf[spec.role].end(),
                    round.facetSymbols.begin(), round.facetSymbols.end());
                break;
            case SymbolStreamRole::TriangulationEdge:
                scalarBuf[spec.role].insert(
                    scalarBuf[spec.role].end(),
                    round.triangulationEdgeSymbols.begin(),
                    round.triangulationEdgeSymbols.end());
                break;
            case SymbolStreamRole::FlipEdge:
                scalarBuf[spec.role].insert(
                    scalarBuf[spec.role].end(),
                    round.flipEdgeSymbols.begin(), round.flipEdgeSymbols.end());
                break;
            case SymbolStreamRole::Vertex:
                scalarBuf[spec.role].insert(
                    scalarBuf[spec.role].end(),
                    round.vertexSymbols.begin(), round.vertexSymbols.end());
                break;
            case SymbolStreamRole::RemovedVertexCoord:
            case SymbolStreamRole::ShiftedVertexCoord:
                // Handled once per round in encodeGeometry.
                break;
            case SymbolStreamRole::SideChannel: {
                auto it = round.side.find(spec.name);
                if (it != round.side.end()) {
                    auto& buf = sideBuffers_[r][spec.name];
                    buf.insert(buf.end(), it->second.begin(), it->second.end());
                }
                break;
            }
            default:
                break;
        }
    }

    if (useResidual_) {
        removedResidualBuffers_[r].insert(removedResidualBuffers_[r].end(),
                                          round.removedVertexResiduals.begin(),
                                          round.removedVertexResiduals.end());
        shiftedResidualBuffers_[r].insert(shiftedResidualBuffers_[r].end(),
                                          round.shiftedVertexResiduals.begin(),
                                          round.shiftedVertexResiduals.end());
    } else {
        removedCoordBuffers_[r].insert(removedCoordBuffers_[r].end(),
                                       round.removedVertexCoords.begin(),
                                       round.removedVertexCoords.end());
        shiftedCoordBuffers_[r].insert(shiftedCoordBuffers_[r].end(),
                                       round.shiftedVertexCoords.begin(),
                                       round.shiftedVertexCoords.end());
    }
}

CanonicalSymbolPayload CanonicalSymbolEncoder::finish() {
    payload_.numRounds = roundsAdded_;
    flushScalarStreams();
    for (std::int32_t r = 0; r < roundsAdded_; ++r) {
        encodeGeometry(
            r,
            removedResidualBuffers_[r],
            shiftedResidualBuffers_[r],
            removedCoordBuffers_[r],
            shiftedCoordBuffers_[r]);
    }
    flushSideChannels();
    auto out = std::move(payload_);
    clear();
    return out;
}

void CanonicalSymbolEncoder::clear() {
    payload_ = CanonicalSymbolPayload{};
    payload_.groupId = 0;
    payload_.numRounds = 0;
    roundsAdded_ = 0;
    scalarBuffers_.clear();
    removedResidualBuffers_.clear();
    shiftedResidualBuffers_.clear();
    removedCoordBuffers_.clear();
    shiftedCoordBuffers_.clear();
    sideBuffers_.clear();
}

void CanonicalSymbolEncoder::flushScalarStreams() {
    for (std::int32_t r = 0; r < roundsAdded_; ++r) {
        for (const auto& spec : specs_) {
            switch (spec.role) {
                case SymbolStreamRole::Facet:
                case SymbolStreamRole::TriangulationEdge:
                case SymbolStreamRole::FlipEdge:
                case SymbolStreamRole::Vertex: {
                    auto it = scalarBuffers_[r].find(spec.role);
                    if (it != scalarBuffers_[r].end() && !it->second.empty()) {
                                                encodeScalarStream(spec, it->second, r);
                    }
                    break;
                }
                default:
                    break;
            }
        }
    }
}

void CanonicalSymbolEncoder::encodeScalarStream(
    const SymbolStreamSpec& spec,
    const std::deque<SymbolCode>& symbols,
    std::int32_t roundId) {
    CanonicalEncodedStream out;
    out.name = spec.name;
    out.role = spec.role;
    out.roundId = roundId;
    out.streamId = static_cast<std::int32_t>(spec.role);
    out.symbolCount = static_cast<std::int32_t>(symbols.size());

    std::vector<std::uint32_t> uvals;
    uvals.reserve(symbols.size());
    std::int64_t minVal = 0, maxVal = 0;
    bool first = true;
    for (SymbolCode s : symbols) {
        uvals.push_back(static_cast<std::uint32_t>(s));
        if (first) {
            minVal = maxVal = s;
            first = false;
        } else {
            minVal = std::min<std::int64_t>(minVal, s);
            maxVal = std::max<std::int64_t>(maxVal, s);
        }
    }

    int alphabetBits = spec.alphabetBits;
    if (alphabetBits <= 0) {
        alphabetBits = first ? 1 : requiredBits(minVal, maxVal);
    }
    out.alphabetBits = alphabetBits;

    appendI32(out.bytes, out.symbolCount);
    appendI8(out.bytes, static_cast<std::int8_t>(alphabetBits));

    if (alphabetBits == 1 && spec.encodeWithRangeCoder) {
        // Use the same adaptive binary range coder as the legacy path.
        auto blob = encodeBinaryRangeCode(uvals);
        out.bytes.insert(out.bytes.end(), blob.begin(), blob.end());
    } else if (auto injected = ::pmsf::injectedStreamCoder(spec.name)) {
        // Phase S1（R13）：方案为该流注入了编码器 ⇒ 用它。
        // 边界（S1 有意保留）：`alphabetBits == 1` 的二进制流走各自更前面的
        // 专用分支（binary range code / bitPack），**不**经此注入 —— 编解码
        // 两侧的分支顺序因此是对称的。覆盖二进制流属 S2/S3 的范围。
        injected->reset();
        std::vector<std::uint8_t> blob;
        injected->encode(uvals, blob);
        out.bytes.insert(out.bytes.end(), blob.begin(), blob.end());
    } else if (spec.encodeWithRangeCoder && alphabetBits <= 16) {
        // 无方案 / 该流未覆盖 ⇒ 原有路径原样保留（逐字节兼容）。
        auto ctx = (spec.contextModel == SymbolContextModel::Valence4)
                       ? RangeCoderContextModel::Valence4
                       : RangeCoderContextModel::None;
        RangeCoderEntropyCoder coder(alphabetBits, ctx);
        std::vector<std::uint8_t> blob;
        coder.encode(uvals, blob);
        out.bytes.insert(out.bytes.end(), blob.begin(), blob.end());
    } else {
        // Fallback: raw little-endian int32 array.
        for (auto v : uvals) {
            appendI32(out.bytes, static_cast<std::int32_t>(v));
        }
    }

    payload_.bitStreams.push_back(std::move(out));
}

void CanonicalSymbolEncoder::encodeGeometry(
    std::int32_t roundId,
    const std::deque<MCGAL::PointInt>& removedResiduals,
    const std::deque<MCGAL::PointInt>& shiftedResiduals,
    const std::deque<MCGAL::Point>& removedCoords,
    const std::deque<MCGAL::Point>& shiftedCoords) {
    CanonicalEncodedGeometry geom;
    geom.roundId = roundId;

    auto appendRawPoints = [](std::vector<std::uint8_t>& out,
                              const std::deque<MCGAL::Point>& pts) {
        appendI8(out, 0);  // coordFormat = raw float
        appendI32(out, static_cast<std::int32_t>(pts.size()));
        for (const auto& p : pts) {
            float xyz[3] = {static_cast<float>(p.x()),
                            static_cast<float>(p.y()),
                            static_cast<float>(p.z())};
            const std::uint8_t* raw = reinterpret_cast<const std::uint8_t*>(xyz);
            out.insert(out.end(), raw, raw + sizeof(xyz));
        }
    };

    auto appendQuantizedPoints = [&](std::vector<std::uint8_t>& out,
                                     const std::deque<MCGAL::Point>& pts) {
        appendI8(out, 1);  // coordFormat = quantized integer
        appendI32(out, static_cast<std::int32_t>(pts.size()));
        appendI8(out, static_cast<std::int8_t>(quantBits_));

        std::vector<std::uint32_t> uvals;
        uvals.reserve(pts.size() * 3);
        for (const auto& p : pts) {
            MCGAL::PointInt q = MCGAL::floatPosToInt(p, quantBboxMin_, quantStep_);
            uvals.push_back(static_cast<std::uint32_t>(q.x()));
            uvals.push_back(static_cast<std::uint32_t>(q.y()));
            uvals.push_back(static_cast<std::uint32_t>(q.z()));
        }
        auto packed = bitPackN(uvals, quantBits_);
        out.insert(out.end(), packed.begin(), packed.end());
    };

    auto appendResidualPoints = [&](std::vector<std::uint8_t>& out,
                                    const std::deque<MCGAL::PointInt>& residuals) {
        if (!useQuantization_ || quantBits_ <= 0) {
            throw std::runtime_error(
                "CanonicalSymbolEncoder: residual geometry requires quantization");
        }
        appendI8(out, 3);  // format = signed residual integer (v3 layout)
        appendI32(out, static_cast<std::int32_t>(residuals.size()));

        std::vector<std::int32_t> values;
        values.reserve(residuals.size() * 3);
        for (const auto& r : residuals) {
            values.push_back(r.x());
            values.push_back(r.y());
            values.push_back(r.z());
        }
        std::vector<std::uint8_t> payload =
            encodeResidualBitPlane(values, quantBits_);

        appendI8(out, 0);  // mode = bit-plane signed residuals
        appendI8(out, static_cast<std::int8_t>(quantBits_));
        appendI32(out, static_cast<std::int32_t>(payload.size()));
        out.insert(out.end(), payload.begin(), payload.end());
    };

    if (useResidual_) {
        geom.coordFormat = 3;
        geom.pointCount = static_cast<std::int32_t>(removedResiduals.size());
        geom.vertexPointCount = static_cast<std::int32_t>(shiftedResiduals.size());
        appendResidualPoints(geom.pointBytes, removedResiduals);
        appendResidualPoints(geom.vertexPointBytes, shiftedResiduals);
    } else if (useQuantization_ && quantBits_ > 0) {
        geom.coordFormat = 1;
        geom.coordBits = static_cast<std::int8_t>(quantBits_);
        geom.pointCount = static_cast<std::int32_t>(removedCoords.size());
        geom.vertexPointCount = static_cast<std::int32_t>(shiftedCoords.size());
        appendQuantizedPoints(geom.pointBytes, removedCoords);
        appendQuantizedPoints(geom.vertexPointBytes, shiftedCoords);
    } else {
        geom.coordFormat = 0;
        geom.pointCount = static_cast<std::int32_t>(removedCoords.size());
        geom.vertexPointCount = static_cast<std::int32_t>(shiftedCoords.size());
        appendRawPoints(geom.pointBytes, removedCoords);
        appendRawPoints(geom.vertexPointBytes, shiftedCoords);
    }

    payload_.geometry.push_back(std::move(geom));
}

void CanonicalSymbolEncoder::flushSideChannels() {
    for (std::int32_t r = 0; r < roundsAdded_; ++r) {
        for (const auto& spec : specs_) {
            if (spec.role != SymbolStreamRole::SideChannel) continue;
            auto it = sideBuffers_[r].find(spec.name);
            if (it == sideBuffers_[r].end()) {
                encodeSideChannel(spec, {}, r);
            } else {
                encodeSideChannel(spec, it->second, r);
            }
        }
    }
}

void CanonicalSymbolEncoder::encodeSideChannel(
    const SymbolStreamSpec& spec,
    const std::vector<SideCell>& cells,
    std::int32_t roundId) {
    CanonicalEncodedSideChannel out;
    out.name = spec.name;
    out.cellType = spec.cellType;
    out.shape = spec.shape;
    out.roundId = roundId;
    out.cellCount = static_cast<std::int32_t>(cells.size());

    if (spec.shape == SideChannelSpec::Shape::Vec3) {
        appendI32(out.bytes, out.cellCount);
        for (const auto& cell : cells) {
            MCGAL::Point p;
            bool ok = false;
            std::visit(
                [&](const auto& v) {
                    using T = std::decay_t<decltype(v)>;
                    if constexpr (std::is_same_v<T, MCGAL::Point>) {
                        p = v;
                        ok = true;
                    }
                },
                cell);
            if (!ok) continue;
            float xyz[3] = {static_cast<float>(p.x()),
                            static_cast<float>(p.y()),
                            static_cast<float>(p.z())};
            const std::uint8_t* raw = reinterpret_cast<const std::uint8_t*>(xyz);
            out.bytes.insert(out.bytes.end(), raw, raw + sizeof(xyz));
        }
        out.alphabetBits = 0;
    } else {
        // Scalar side channel: convert to uint32 and range-code.
        std::vector<std::uint32_t> uvals;
        uvals.reserve(cells.size());
        std::int64_t minVal = 0, maxVal = 0;
        bool first = true;
        bool allBool = true;
        for (const auto& cell : cells) {
            std::int64_t v = 0;
            bool isBool = false;
            std::visit(
                [&](const auto& value) {
                    using T = std::decay_t<decltype(value)>;
                    if constexpr (std::is_same_v<T, bool>) {
                        v = value ? 1 : 0;
                        isBool = true;
                    } else if constexpr (std::is_arithmetic_v<T>) {
                        v = static_cast<std::int64_t>(value);
                    } else {
                        v = 0;
                    }
                },
                cell);
            if (!isBool) allBool = false;
            uvals.push_back(static_cast<std::uint32_t>(v));
            if (first) {
                minVal = maxVal = v;
                first = false;
            } else {
                minVal = std::min(minVal, v);
                maxVal = std::max(maxVal, v);
            }
        }

        int alphabetBits = spec.alphabetBits;
        if (alphabetBits <= 0) {
            alphabetBits = first ? 1 : (allBool ? 1 : requiredBits(minVal, maxVal));
        }
        out.alphabetBits = alphabetBits;

        appendI32(out.bytes, out.cellCount);
        appendI8(out.bytes, static_cast<std::int8_t>(alphabetBits));

        if (alphabetBits == 1) {
            auto packed = bitPack(uvals);
            out.bytes.insert(out.bytes.end(), packed.begin(), packed.end());
        } else if (auto injected = ::pmsf::injectedStreamCoder(spec.name)) {
            // Phase S1（R13）：方案为该流注入了编码器 ⇒ 用它。
            injected->reset();
            std::vector<std::uint8_t> blob;
            injected->encode(uvals, blob);
            out.bytes.insert(out.bytes.end(), blob.begin(), blob.end());
        } else if (alphabetBits <= 16) {
            // 无方案 / 该流未覆盖 ⇒ **原有路径原样保留**（不是"等价重写"，
            // 而是同一段代码 —— 这是默认路径逐字节兼容的来源）。
            auto ctx = (spec.contextModel == SymbolContextModel::Valence4)
                           ? RangeCoderContextModel::Valence4
                           : RangeCoderContextModel::None;
            RangeCoderEntropyCoder coder(alphabetBits, ctx);
            std::vector<std::uint8_t> blob;
            coder.encode(uvals, blob);
            out.bytes.insert(out.bytes.end(), blob.begin(), blob.end());
        } else {
            for (auto v : uvals) {
                appendI32(out.bytes, static_cast<std::int32_t>(v));
            }
        }
    }

    payload_.sideChannels.push_back(std::move(out));
}

// ---------------------------------------------------------------------------
// CanonicalSymbolDecoder
// ---------------------------------------------------------------------------

CanonicalSymbolDecoder::CanonicalSymbolDecoder(
    const CanonicalSymbolPayload& payload,
    const std::vector<SymbolStreamSpec>& specs)
    : payload_(payload), specs_(specs) {}

void CanonicalSymbolDecoder::setQuantization(int bits,
                                             MCGAL::Point bboxMin,
                                             float quantStep) {
    useQuantization_ = (bits > 0);
    quantBits_ = std::clamp(bits, 0, 31);
    quantBboxMin_ = bboxMin;
    quantStep_ = quantStep;
}

std::vector<PaperRoundSymbols> CanonicalSymbolDecoder::decode() const {
    std::vector<PaperRoundSymbols> rounds;
    rounds.resize(payload_.numRounds);

    // bitStreams and sideChannels are stored round-major.
    std::size_t bitIdx = 0;
    std::size_t sideIdx = 0;

    for (int r = 0; r < payload_.numRounds; ++r) {
        auto& round = rounds[r];

        for (const auto& spec : specs_) {
            switch (spec.role) {
                case SymbolStreamRole::Facet:
                    if (bitIdx < payload_.bitStreams.size())
                        decodeScalarStream(spec, payload_.bitStreams[bitIdx++],
                                           round.facetSymbols);
                    break;
                case SymbolStreamRole::TriangulationEdge:
                    if (bitIdx < payload_.bitStreams.size())
                        decodeScalarStream(spec, payload_.bitStreams[bitIdx++],
                                           round.triangulationEdgeSymbols);
                    break;
                case SymbolStreamRole::FlipEdge:
                    if (bitIdx < payload_.bitStreams.size())
                        decodeScalarStream(spec, payload_.bitStreams[bitIdx++],
                                           round.flipEdgeSymbols);
                    break;
                case SymbolStreamRole::Vertex:
                    if (bitIdx < payload_.bitStreams.size())
                        decodeScalarStream(spec, payload_.bitStreams[bitIdx++],
                                           round.vertexSymbols);
                    break;
                case SymbolStreamRole::RemovedVertexCoord:
                case SymbolStreamRole::ShiftedVertexCoord:
                    break;
                case SymbolStreamRole::SideChannel:
                    if (sideIdx < payload_.sideChannels.size())
                        decodeSideChannel(spec, payload_.sideChannels[sideIdx++],
                                          round);
                    break;
                default:
                    break;
            }
        }

        if (r < static_cast<int>(payload_.geometry.size())) {
            decodeGeometry(payload_.geometry[r], round);
        }
    }

    return rounds;
}

void CanonicalSymbolDecoder::decodeScalarStream(
    const SymbolStreamSpec& spec,
    const CanonicalEncodedStream& encoded,
    std::deque<SymbolCode>& out) const {
    const std::uint8_t* p = encoded.bytes.data();
    const std::uint8_t* end = p + encoded.bytes.size();

    const std::int32_t count = readI32(p, end);
    const std::int8_t alphabetBits = readI8(p, end);

    std::vector<std::uint32_t> uvals;
    if (alphabetBits == 1) {
        uvals = decodeBinaryRangeCode(p, end - p, static_cast<std::size_t>(count));
    } else if (auto injected = ::pmsf::injectedStreamCoder(spec.name)) {
        // Phase S1（R13）：与编码侧同一采集缝，保证两侧用同一个 coder。
        injected->reset();
        std::size_t consumed = injected->decode(p, end - p,
                                                static_cast<std::size_t>(count),
                                                uvals);
        (void)consumed;
    } else if (alphabetBits <= 16) {
        auto ctx = (spec.contextModel == SymbolContextModel::Valence4)
                       ? RangeCoderContextModel::Valence4
                       : RangeCoderContextModel::None;
        RangeCoderEntropyCoder coder(alphabetBits, ctx);
        std::size_t consumed = coder.decode(p, end - p,
                                            static_cast<std::size_t>(count),
                                            uvals);
        (void)consumed;
    } else {
        for (std::int32_t i = 0; i < count; ++i) {
            uvals.push_back(static_cast<std::uint32_t>(readI32(p, end)));
        }
    }

    out.clear();
    for (auto v : uvals) out.push_back(static_cast<SymbolCode>(v));
}

void CanonicalSymbolDecoder::decodeGeometry(
    const CanonicalEncodedGeometry& geom,
    PaperRoundSymbols& round) const {
    auto readRawPoints = [](const std::vector<std::uint8_t>& bytes,
                            std::deque<MCGAL::Point>& out) {
        const std::uint8_t* p = bytes.data();
        const std::uint8_t* end = p + bytes.size();
        if (p >= end) return;
        const std::int8_t format = readI8(p, end);
        if (format != 0) {
            throw std::runtime_error(
                "CanonicalSymbolDecoder: expected raw-float geometry format");
        }
        const std::int32_t count = readI32(p, end);
        for (std::int32_t i = 0; i < count; ++i) {
            if (p + 3 * sizeof(float) > end) break;
            float xyz[3];
            std::memcpy(xyz, p, sizeof(xyz));
            p += sizeof(xyz);
            out.emplace_back(xyz[0], xyz[1], xyz[2]);
        }
    };

    auto readQuantizedPoints = [&](const std::vector<std::uint8_t>& bytes,
                                   std::deque<MCGAL::Point>& out) {
        const std::uint8_t* p = bytes.data();
        const std::uint8_t* end = p + bytes.size();
        if (p >= end) return;
        const std::int8_t format = readI8(p, end);
        if (format != 1) {
            throw std::runtime_error(
                "CanonicalSymbolDecoder: expected quantized geometry format");
        }
        const std::int32_t count = readI32(p, end);
        const std::int8_t bits = readI8(p, end);
        if (count <= 0) return;
        if (bits <= 0 || bits > 31) {
            throw std::runtime_error(
                "CanonicalSymbolDecoder: invalid quantized geometry bits");
        }
        const std::size_t totalValues = static_cast<std::size_t>(count) * 3;
        const std::size_t packedBits = totalValues * static_cast<std::size_t>(bits);
        const std::size_t packedBytes = (packedBits + 7) / 8;
        if (p + packedBytes > end) {
            throw std::runtime_error(
                "CanonicalSymbolDecoder: truncated quantized geometry bytes");
        }
        auto uvals = bitUnpackN(p, packedBytes, totalValues, bits);
        for (std::size_t i = 0; i + 2 < uvals.size(); i += 3) {
            MCGAL::PointInt q(static_cast<std::int32_t>(uvals[i]),
                              static_cast<std::int32_t>(uvals[i + 1]),
                              static_cast<std::int32_t>(uvals[i + 2]));
            out.push_back(MCGAL::intPosToFloat(q, quantBboxMin_, quantStep_));
        }
    };

    auto readResidualPoints = [&](const std::vector<std::uint8_t>& bytes,
                                  std::deque<MCGAL::PointInt>& out) {
        const std::uint8_t* p = bytes.data();
        const std::uint8_t* end = p + bytes.size();
        if (p >= end) return;
        const std::int8_t format = readI8(p, end);
        if (format != 3) {
            throw std::runtime_error(
                "CanonicalSymbolDecoder: expected residual geometry format v3");
        }
        const std::int32_t count = readI32(p, end);
        const std::int8_t mode = readI8(p, end);
        if (mode != 0) {
            throw std::runtime_error(
                "CanonicalSymbolDecoder: unsupported residual geometry mode");
        }
        const std::int8_t bits = readI8(p, end);
        const std::int32_t payloadLen = readI32(p, end);
        if (count <= 0) return;
        if (payloadLen < 0 || p + payloadLen > end) {
            throw std::runtime_error(
                "CanonicalSymbolDecoder: truncated residual geometry payload");
        }
        const std::size_t totalValues = static_cast<std::size_t>(count) * 3;
        auto vals = decodeResidualBitPlane(p, payloadLen, totalValues, bits);
        for (std::size_t i = 0; i + 2 < vals.size(); i += 3) {
            out.emplace_back(vals[i], vals[i + 1], vals[i + 2]);
        }
        p += payloadLen;
    };

    if (geom.coordFormat == 3 ||
        (!geom.pointBytes.empty() && geom.pointBytes[0] == 3)) {
        readResidualPoints(geom.pointBytes, round.removedVertexResiduals);
        readResidualPoints(geom.vertexPointBytes, round.shiftedVertexResiduals);
    } else if (geom.coordFormat == 1 ||
               (!geom.pointBytes.empty() && geom.pointBytes[0] == 1)) {
        if (!useQuantization_) {
            throw std::runtime_error(
                "CanonicalSymbolDecoder: quantized geometry requires "
                "setQuantization() before decode()");
        }
        readQuantizedPoints(geom.pointBytes, round.removedVertexCoords);
        readQuantizedPoints(geom.vertexPointBytes, round.shiftedVertexCoords);
    } else {
        readRawPoints(geom.pointBytes, round.removedVertexCoords);
        readRawPoints(geom.vertexPointBytes, round.shiftedVertexCoords);
    }
}

void CanonicalSymbolDecoder::decodeSideChannel(
    const SymbolStreamSpec& spec,
    const CanonicalEncodedSideChannel& encoded,
    PaperRoundSymbols& round) const {
    const std::uint8_t* p = encoded.bytes.data();
    const std::uint8_t* end = p + encoded.bytes.size();

    if (encoded.shape == SideChannelSpec::Shape::Vec3) {
        const std::int32_t count = readI32(p, end);
        auto& out = round.side[encoded.name];
        out.clear();
        out.reserve(count);
        for (std::int32_t i = 0; i < count; ++i) {
            if (p + 3 * sizeof(float) > end) break;
            float xyz[3];
            std::memcpy(xyz, p, sizeof(xyz));
            p += sizeof(xyz);
            out.emplace_back(MCGAL::Point(xyz[0], xyz[1], xyz[2]));
        }
        return;
    }

    const std::int32_t count = readI32(p, end);
    const std::int8_t alphabetBits = readI8(p, end);

    std::vector<std::uint32_t> uvals;
    if (alphabetBits == 1) {
        uvals = bitUnpack(p, end - p, static_cast<std::size_t>(count));
    } else if (auto injected = ::pmsf::injectedStreamCoder(spec.name)) {
        // Phase S1（R13）：与编码侧同一采集缝，保证两侧用同一个 coder。
        injected->reset();
        std::size_t consumed = injected->decode(p, end - p,
                                                static_cast<std::size_t>(count),
                                                uvals);
        (void)consumed;
    } else if (alphabetBits <= 16) {
        auto ctx = (spec.contextModel == SymbolContextModel::Valence4)
                       ? RangeCoderContextModel::Valence4
                       : RangeCoderContextModel::None;
        RangeCoderEntropyCoder coder(alphabetBits, ctx);
        std::size_t consumed = coder.decode(p, end - p,
                                            static_cast<std::size_t>(count),
                                            uvals);
        (void)consumed;
    } else {
        for (std::int32_t i = 0; i < count; ++i) {
            uvals.push_back(static_cast<std::uint32_t>(readI32(p, end)));
        }
    }

    auto& out = round.side[encoded.name];
    out.clear();
    out.reserve(uvals.size());
    for (auto v : uvals) {
        switch (encoded.cellType) {
            case SideChannelSpec::Type::Bool:
                out.emplace_back(v != 0);
                break;
            case SideChannelSpec::Type::Int8:
                out.emplace_back(static_cast<std::int8_t>(v));
                break;
            case SideChannelSpec::Type::Int16:
                out.emplace_back(static_cast<std::int16_t>(v));
                break;
            case SideChannelSpec::Type::Int32:
                out.emplace_back(static_cast<std::int32_t>(v));
                break;
            case SideChannelSpec::Type::UInt16:
                out.emplace_back(static_cast<std::uint16_t>(v));
                break;
            case SideChannelSpec::Type::Float:
                out.emplace_back(static_cast<float>(v));
                break;
            default:
                out.emplace_back(static_cast<std::int32_t>(v));
                break;
        }
    }
}

}  // namespace pmsf
