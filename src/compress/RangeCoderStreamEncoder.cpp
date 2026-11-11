/*****************************************************************************
 * @Author: koi
 * @Description: RangeCoderStreamEncoder 实现。
 *****************************************************************************/


#include "compress/RangeCoderStreamEncoder.h"
#include "compress/ResidualAxisContext.h"
#include "Point.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <memory>

namespace pmsf {

namespace {

constexpr int kDefaultQsRescale = 1 << 9;

/// P16（2026-09-25）：qsmodel 的 **rescale** 旋钮（自适应速度/精度）。
///
/// `initqsmodel(m, n, lg_totf, rescale, init, compress)` 的第四个参数是
/// 「重标定间隔」上界：模型初始 `rescale = n>>4 | 2`（起步快），每次真正
/// 重标定时翻倍直到 `targetrescale`。它**不改变字母表、不改变码流语义**，
/// 只改变频率估计的跟踪速度，因此是纯编码器参数：
///   - 不动量化位深；
///   - 不动质量（符号序列相同 ⇒ 解码网格逐位相同，前提是解码端同值）；
///   - 不是容器重包装。
///
/// 默认值 = 原硬编码值（`1<<9`）⇒ **默认路径零字节变化**；仅当环境变量置位
/// 时才偏离，用于「先量后调」的扫描。实测最优只有 −21 B（见
/// docs/BPV_REDUCTION_PLAN_2026-09-25.md §4），故保持默认。
int qsRescaleOverride() {
    static const int v = [] {
        const char* e = std::getenv("PMSF_QS_RESCALE");
        if (e == nullptr || *e == '\0') {
            return kDefaultQsRescale;
        }
        const int parsed = std::atoi(e);
        // qsmodel 要求 rescale <= 1<<lg_totf（此处 lg_totf=10）。
        return (parsed >= 2 && parsed <= (1 << 10)) ? parsed : kDefaultQsRescale;
    }();
    return v;
}

}  // namespace

AdaptiveSymbolEncoder::AdaptiveSymbolEncoder(int alphabetSize,
                                             int scaleBits,
                                             int initCount)
    : alphabetSize_(alphabetSize), scaleBits_(scaleBits) {
    initqsmodel(&model_, alphabetSize, scaleBits, initCount, nullptr, 1);
}

AdaptiveSymbolEncoder::~AdaptiveSymbolEncoder() {
    deleteqsmodel(&model_);
}

AdaptiveSymbolEncoder::AdaptiveSymbolEncoder(AdaptiveSymbolEncoder&& other) noexcept
    : alphabetSize_(other.alphabetSize_), scaleBits_(other.scaleBits_) {
    std::memcpy(&model_, &other.model_, sizeof(model_));
    std::memset(&other.model_, 0, sizeof(other.model_));
    other.alphabetSize_ = 0;
    other.scaleBits_ = 0;
}

AdaptiveSymbolEncoder& AdaptiveSymbolEncoder::operator=(AdaptiveSymbolEncoder&& other) noexcept {
    if (this != &other) {
        deleteqsmodel(&model_);
        alphabetSize_ = other.alphabetSize_;
        scaleBits_ = other.scaleBits_;
        std::memcpy(&model_, &other.model_, sizeof(model_));
        std::memset(&other.model_, 0, sizeof(other.model_));
        other.alphabetSize_ = 0;
        other.scaleBits_ = 0;
    }
    return *this;
}

void AdaptiveSymbolEncoder::encode(rangecoder* rc, int sym) {
    if (sym < 0 || sym >= alphabetSize_) {
        std::cerr << "[AdaptiveSymbolEncoder] symbol " << sym
                  << " out of range [0, " << alphabetSize_ << ")" << std::endl;
        sym = 0;
    }
    int syfreq, ltfreq;
    qsgetfreq(&model_, sym, &syfreq, &ltfreq);
    encode_shift(rc, syfreq, ltfreq, scaleBits_);
    qsupdate(&model_, sym);
}

size_t encodeUint8Queues(const std::vector<std::deque<uint8_t>>& queues,
                         int alphabetSize,
                         bool reverseRounds,
                         char* out,
                         size_t maxSize) {
    size_t rcOffset = 0;
    rangecoder rc;
    rc.p_data = out;
    rc.p_dataOffset = &rcOffset;
    start_encoding(&rc, 0, 0);

    // P16：第四个实参是 qsmodel 的 rescale（非「初始计数」）；默认 1<<9。
    AdaptiveSymbolEncoder encoder(alphabetSize, 10, qsRescaleOverride());

    int start = reverseRounds ? static_cast<int>(queues.size()) - 1 : 0;
    int end   = reverseRounds ? -1 : static_cast<int>(queues.size());
    int step  = reverseRounds ? -1 : 1;

    for (int i = start; i != end; i += step) {
        for (uint8_t v : queues[i]) {
            encoder.encode(&rc, static_cast<int>(v));
        }
    }

    return done_encoding(&rc);
}

size_t encodeMultiModelSegments(const std::vector<MultiModelSegment>& segments,
                                bool reverseRounds,
                                char* out,
                                size_t maxSize) {
    if (segments.empty()) {
        return 0;
    }
    const size_t R = segments[0].queues->size();
    for (const auto& seg : segments) {
        if (!seg.queues || seg.queues->size() != R) {
            throw std::runtime_error(
                "[encodeMultiModelSegments] segment round count mismatch");
        }
    }

    size_t rcOffset = 0;
    rangecoder rc;
    rc.p_data = out;
    rc.p_dataOffset = &rcOffset;
    start_encoding(&rc, 0, 0);

    std::vector<AdaptiveSymbolEncoder> encoders;
    encoders.reserve(segments.size());
    for (const auto& seg : segments) {
        encoders.emplace_back(seg.alphabetSize);
    }

    int start = reverseRounds ? static_cast<int>(R) - 1 : 0;
    int end   = reverseRounds ? -1 : static_cast<int>(R);
    int step  = reverseRounds ? -1 : 1;

    for (int i = start; i != end; i += step) {
        for (size_t s = 0; s < segments.size(); ++s) {
            for (uint8_t v : (*segments[s].queues)[i]) {
                encoders[s].encode(&rc, static_cast<int>(v));
            }
        }
    }

    return done_encoding(&rc);
}

size_t encodeContextAdaptiveUint8Queue(
    const std::deque<uint8_t>& symbols,
    int alphabetSize,
    int numContexts,
    const std::function<int(int prevSymbol)>& deriveCtx,
    const std::deque<uint8_t>* escapeSymbols,
    int escapeAlphabetSize,
    int escapeSymbol,
    char* out,
    size_t /*maxSize*/) {
    size_t rcOffset = 0;
    rangecoder rc;
    rc.p_data = out;
    rc.p_dataOffset = &rcOffset;
    start_encoding(&rc, 0, 0);

    std::vector<AdaptiveSymbolEncoder> models;
    models.reserve(numContexts);
    for (int i = 0; i < numContexts; ++i) {
        models.emplace_back(alphabetSize);
    }

    std::unique_ptr<AdaptiveSymbolEncoder> escapeEncoder;
    if (escapeSymbols != nullptr && escapeAlphabetSize > 0) {
        escapeEncoder = std::make_unique<AdaptiveSymbolEncoder>(escapeAlphabetSize);
    }

    size_t escCursor = 0;
    int prev = -1;
    for (uint8_t symU : symbols) {
        int sym = static_cast<int>(symU);
        int ctx = deriveCtx(prev);
        if (ctx < 0) ctx = 0;
        if (ctx >= numContexts) ctx = numContexts - 1;
        models[ctx].encode(&rc, sym);

        if (escapeEncoder && sym == escapeSymbol) {
            int esc = (escapeSymbols && escCursor < escapeSymbols->size())
                          ? static_cast<int>((*escapeSymbols)[escCursor++])
                          : 0;
            escapeEncoder->encode(&rc, esc);
        }
        prev = sym;
    }

    return done_encoding(&rc);
}

size_t encodeValence5WithEscape(
    const std::vector<std::deque<uint8_t>>& valenceQueues,
    const std::vector<std::deque<uint8_t>>& escapeQueues,
    bool reverseRounds,
    char* out,
    size_t maxSize) {
    if (valenceQueues.size() != escapeQueues.size()) {
        throw std::runtime_error(
            "[encodeValence5WithEscape] valence/escape queue round count mismatch");
    }

    size_t rcOffset = 0;
    rangecoder rc;
    rc.p_data = out;
    rc.p_dataOffset = &rcOffset;
    start_encoding(&rc, 0, 0);

    AdaptiveSymbolEncoder valEncoder(5);
    AdaptiveSymbolEncoder escapeEncoder(256);

    int start = reverseRounds ? static_cast<int>(valenceQueues.size()) - 1 : 0;
    int end   = reverseRounds ? -1 : static_cast<int>(valenceQueues.size());
    int step  = reverseRounds ? -1 : 1;

    for (int i = start; i != end; i += step) {
        const auto& codesQ = valenceQueues[i];
        const auto& otherQ = escapeQueues[i];
        size_t otherCursor = 0;
        for (uint8_t c : codesQ) {
            int sym = static_cast<int>(c);
            if (sym >= 5) {
                std::cerr << "[encodeValence5WithEscape] symbol " << sym
                          << " >= 5 (clamped to V_OTHER)" << std::endl;
                sym = 4;  // V_OTHER index
            }
            valEncoder.encode(&rc, sym);
            if (sym == 4) {
                int esym = (otherCursor < otherQ.size())
                               ? static_cast<int>(otherQ[otherCursor++])
                               : 0;
                escapeEncoder.encode(&rc, esym);
            }
        }
    }

    return done_encoding(&rc);
}

size_t encodeUint8Queue(const std::deque<uint8_t>& queue,
                        int alphabetSize,
                        char* out,
                        size_t maxSize) {
    // 利用多轮函数：构造一个只含 queue 的临时 vector，不反转轮次。
    std::vector<std::deque<uint8_t>> wrap;
    wrap.push_back(queue);
    return encodeUint8Queues(wrap, alphabetSize, false, out, maxSize);
}

size_t encodeResidualBitPlaneQueues(
    const std::vector<std::deque<MCGAL::PointInt>>& queues,
    int bpBits,
    bool reverseRounds,
    char* out,
    size_t maxSize) {
    size_t rcOffset = 0;
    rangecoder rc;
    rc.p_data = out;
    rc.p_dataOffset = &rcOffset;
    start_encoding(&rc, 0, 0);

    std::vector<qsmodel> bpFirst1(bpBits);
    std::vector<qsmodel> bpAfter1(bpBits);
    for (int b = 0; b < bpBits; b++) {
        initqsmodel(&bpFirst1[b], 2, 10, 1 << 9, nullptr, 1);
        initqsmodel(&bpAfter1[b], 2, 10, 1 << 9, nullptr, 1);
    }

    auto encodeResidual = [&](int32_t r) {
        uint32_t z = (static_cast<uint32_t>(r) << 1) ^
                     static_cast<uint32_t>(r >> 31);
        bool seenOne = false;
        for (int b = bpBits - 1; b >= 0; b--) {
            int bit = static_cast<int>((z >> b) & 1u);
            qsmodel* m = seenOne ? &bpAfter1[b] : &bpFirst1[b];
            int syfreq, ltfreq;
            qsgetfreq(m, bit, &syfreq, &ltfreq);
            encode_shift(&rc, syfreq, ltfreq, 10);
            qsupdate(m, bit);
            if (bit) seenOne = true;
        }
    };

    int start = reverseRounds ? static_cast<int>(queues.size()) - 1 : 0;
    int end   = reverseRounds ? -1 : static_cast<int>(queues.size());
    int step  = reverseRounds ? -1 : 1;

    for (int i = start; i != end; i += step) {
        for (const auto& res : queues[i]) {
            encodeResidual(res.x());
            encodeResidual(res.y());
            encodeResidual(res.z());
        }
    }

    size_t sz = done_encoding(&rc);
    for (int b = 0; b < bpBits; b++) {
        deleteqsmodel(&bpFirst1[b]);
        deleteqsmodel(&bpAfter1[b]);
    }
return sz;
}

size_t encodeResidualBitPlaneQueue(const std::deque<MCGAL::PointInt>& queue,
                                   int bpBits,
                                   char* out,
                                   size_t maxSize) {
    std::vector<std::deque<MCGAL::PointInt>> wrap;
    wrap.push_back(queue);
    return encodeResidualBitPlaneQueues(wrap, bpBits, false, out, maxSize);
}

size_t encodeResidualBitPlaneQueuePerComponent(const std::deque<MCGAL::PointInt>& queue,
                                               int bpBits,
                                               char* out,
                                               size_t /*maxSize*/) {
    size_t rcOffset = 0;
    rangecoder rc;
    rc.p_data = out;
    rc.p_dataOffset = &rcOffset;
    start_encoding(&rc, 0, 0);

    std::vector<qsmodel> bpFirst1(3 * bpBits);
    std::vector<qsmodel> bpAfter1(3 * bpBits);
    for (int b = 0; b < 3 * bpBits; b++) {
        initqsmodel(&bpFirst1[b], 2, 10, 1 << 9, nullptr, 1);
        initqsmodel(&bpAfter1[b], 2, 10, 1 << 9, nullptr, 1);
    }

    auto encodeComponent = [&](int comp, int32_t r) {
        uint32_t z = (static_cast<uint32_t>(r) << 1) ^
                     static_cast<uint32_t>(r >> 31);
        bool seenOne = false;
        const int base = comp * bpBits;
        for (int b = bpBits - 1; b >= 0; b--) {
            int bit = static_cast<int>((z >> b) & 1u);
            qsmodel* m = seenOne ? &bpAfter1[base + b] : &bpFirst1[base + b];
            int syfreq, ltfreq;
            qsgetfreq(m, bit, &syfreq, &ltfreq);
            encode_shift(&rc, syfreq, ltfreq, 10);
            qsupdate(m, bit);
            if (bit) seenOne = true;
        }
    };

    for (const auto& res : queue) {
        encodeComponent(0, res.x());
        encodeComponent(1, res.y());
        encodeComponent(2, res.z());
    }

    size_t sz = done_encoding(&rc);
    for (int b = 0; b < 3 * bpBits; b++) {
        deleteqsmodel(&bpFirst1[b]);
        deleteqsmodel(&bpAfter1[b]);
    }
    return sz;
}

namespace {

int bitWidthForResiduals(const std::vector<int>& vals) {
    uint32_t maxZ = 0;
    for (int32_t r : vals) {
        uint32_t z = (static_cast<uint32_t>(r) << 1) ^ static_cast<uint32_t>(r >> 31);
        if (z > maxZ) maxZ = z;
    }
    if (maxZ == 0) return 0;
    int bits = 0;
    while (maxZ != 0) {
        ++bits;
        maxZ >>= 1;
    }
    return std::min(bits, 31);
}

size_t encodeAdaptiveBitPlaneInternal(const std::vector<int>& rx,
                                      const std::vector<int>& ry,
                                      const std::vector<int>& rz,
                                      char* out,
                                      size_t /*maxSize*/,
                                      bool planeMajor,
                                      bool biasedFirstOneInit,
                                      int minComponentWidth,
                                      const std::vector<int>* planeOrder) {
    const size_t N = rx.size();
    const std::vector<int>* comps[3] = {&rx, &ry, &rz};
    int bits[3] = {
        bitWidthForResiduals(rx),
        bitWidthForResiduals(ry),
        bitWidthForResiduals(rz),
    };
    for (int& b : bits) b = std::max(b, minComponentWidth);
    const int offsets[3] = {0, bits[0], bits[0] + bits[1]};
    const int numCtx = bits[0] + bits[1] + bits[2];
    if (N == 0 || numCtx == 0) return 0;

    size_t rcOffset = 0;
    rangecoder rc;
    rc.p_data = out;
    rc.p_dataOffset = &rcOffset;
    start_encoding(&rc, 0, 0);

    std::vector<qsmodel> bpFirst1(numCtx), bpAfter1(numCtx);
    int firstOneInit[2] = {992, 32};
    // P16：默认 1<<9 ⇒ 零字节变化；见 qsRescaleOverride()。
    const int qsRescale = qsRescaleOverride();
    for (int i = 0; i < numCtx; ++i) {
        initqsmodel(&bpFirst1[i], 2, 10, qsRescale,
                    biasedFirstOneInit ? firstOneInit : nullptr, 1);
        initqsmodel(&bpAfter1[i], 2, 10, qsRescale, nullptr, 1);
    }

    // Non-bucket joint-axis contexts: hash the fully decoded earlier axes
    // into a fixed model pool.  Fixed-size hashing keeps model overhead bounded
    // while letting y depend on x and z depend on (x,y).
    constexpr int kHashY = 1;
    constexpr int kHashZ = 32;
    const bool hashCtx = []() {
        const char* e = std::getenv("PMSF_MMSE_RESID_HASH_CTX");
        return e && *e && e[0] != '0';
    }() && planeMajor;
    std::vector<uint16_t> hashY, hashXY;
    std::vector<qsmodel> bpYFirst1, bpYAfter1, bpZFirst1, bpZAfter1;
    if (hashCtx) {
        hashY.resize(N);
        hashXY.resize(N);
        for (size_t i = 0; i < N; ++i) {
            const uint32_t ux = static_cast<uint32_t>(rx[i]);
            const uint32_t uy = static_cast<uint32_t>(ry[i]);
            hashY[i] = static_cast<uint16_t>((ux * 2654435761u) & (kHashY - 1));
            hashXY[i] = static_cast<uint16_t>(
                ((ux * 73856093u) ^ (uy * 19349663u)) & (kHashZ - 1));
        }
        const int yModels = bits[1] * kHashY;
        const int zModels = bits[2] * kHashZ;
        bpYFirst1.resize(yModels); bpYAfter1.resize(yModels);
        bpZFirst1.resize(zModels); bpZAfter1.resize(zModels);
        for (int i = 0; i < yModels; ++i) {
            initqsmodel(&bpYFirst1[i], 2, 10, qsRescale,
                        biasedFirstOneInit ? firstOneInit : nullptr, 1);
            initqsmodel(&bpYAfter1[i], 2, 10, qsRescale, nullptr, 1);
        }
        for (int i = 0; i < zModels; ++i) {
            initqsmodel(&bpZFirst1[i], 2, 10, qsRescale,
                        biasedFirstOneInit ? firstOneInit : nullptr, 1);
            initqsmodel(&bpZAfter1[i], 2, 10, qsRescale, nullptr, 1);
        }
    }

    auto encodeCompBitsVertexMajor = [&](int comp, int32_t r, std::vector<unsigned char>& seen) {
        const int width = bits[comp];
        if (width == 0) return;
        uint32_t z = (static_cast<uint32_t>(r) << 1) ^ static_cast<uint32_t>(r >> 31);
        for (int b = width - 1; b >= 0; --b) {
            int bit = static_cast<int>((z >> b) & 1u);
            const int mi = offsets[comp] + b;
            qsmodel* mp = seen[comp] ? &bpAfter1[mi] : &bpFirst1[mi];
            int sf, lf;
            qsgetfreq(mp, bit, &sf, &lf);
            encode_shift(&rc, sf, lf, 10);
            qsupdate(mp, bit);
            if (bit) seen[comp] = 1;
        }
    };


    if (planeMajor) {
        std::vector<unsigned char> seenOne[3];
        for (int a = 0; a < 3; ++a) seenOne[a].assign(N, 0);
        std::vector<int> naturalOrder;
        const std::vector<int>* order = planeOrder;
        if (order == nullptr || order->empty()) {
            for (int comp = 0; comp < 3; ++comp) {
                for (int b = bits[comp] - 1; b >= 0; --b)
                    naturalOrder.push_back(comp * 32 + b);
            }
            order = &naturalOrder;
        }
        for (size_t qi = 0; qi < order->size(); ++qi) {
            const int plane = (*order)[qi];
            const int comp = plane / 32;
            const int b0 = plane % 32;
            if (comp < 0 || comp > 2 || b0 < 0 || b0 >= bits[comp]) continue;
            const int width = bits[comp];
            if (width == 0) continue;
            (void)width;
            const int b = b0;
            // P16 候选上下文探针（保留注释）；R-D 顺序只改变平面访问次序。
            {
                const int mi = offsets[comp] + b;
                for (size_t i = 0; i < N; ++i) {
                    int32_t r = (*comps[comp])[i];
                    uint32_t z = (static_cast<uint32_t>(r) << 1) ^ static_cast<uint32_t>(r >> 31);
                    int bit = static_cast<int>((z >> b) & 1u);
                    qsmodel* mp = nullptr;
                    if (hashCtx && comp == 1) {
                        const int idx = b * kHashY + hashY[i];
                        mp = seenOne[comp][i] ? &bpYAfter1[idx] : &bpYFirst1[idx];
                    } else if (hashCtx && comp == 2) {
                        const int idx = b * kHashZ + hashXY[i];
                        mp = seenOne[comp][i] ? &bpZAfter1[idx] : &bpZFirst1[idx];
                    } else {
                        mp = seenOne[comp][i] ? &bpAfter1[mi] : &bpFirst1[mi];
                    }
                    int sf, lf;
                    qsgetfreq(mp, bit, &sf, &lf);
                    encode_shift(&rc, sf, lf, 10);
                    qsupdate(mp, bit);
                    if (bit) seenOne[comp][i] = 1;
                }
            }
        }
    } else {
        std::vector<unsigned char> seen(3, 0);
        for (size_t i = 0; i < N; ++i) {
            std::fill(seen.begin(), seen.end(), 0);
            encodeCompBitsVertexMajor(0, rx[i], seen);
            encodeCompBitsVertexMajor(1, ry[i], seen);
            encodeCompBitsVertexMajor(2, rz[i], seen);
        }
    }

    size_t sz = done_encoding(&rc);
    for (int i = 0; i < numCtx; ++i) {
        deleteqsmodel(&bpFirst1[i]);
        deleteqsmodel(&bpAfter1[i]);
    }
    for (auto& m : bpYFirst1) deleteqsmodel(&m);
    for (auto& m : bpYAfter1) deleteqsmodel(&m);
    for (auto& m : bpZFirst1) deleteqsmodel(&m);
    for (auto& m : bpZAfter1) deleteqsmodel(&m);

        return sz;
}

}  // namespace

size_t encodeResidualBitPlaneQueueAdaptiveVertexMajor(const std::vector<int>& rx,
                                                      const std::vector<int>& ry,
                                                      const std::vector<int>& rz,
                                                      char* out,
                                                      size_t maxSize,
                                                      int minComponentWidth) {
    return encodeAdaptiveBitPlaneInternal(rx, ry, rz, out, maxSize, false, false,
                                          minComponentWidth, nullptr);
}

size_t encodeResidualBitPlaneQueueAdaptivePlaneMajor(const std::vector<int>& rx,
                                                     const std::vector<int>& ry,
                                                     const std::vector<int>& rz,
                                                     char* out,
                                                     size_t maxSize,
                                                     bool biasedFirstOneInit,
                                                     int minComponentWidth) {
    return encodeAdaptiveBitPlaneInternal(rx, ry, rz, out, maxSize, true, biasedFirstOneInit,
                                          minComponentWidth, nullptr);
}

size_t encodeResidualBitPlaneQueueAdaptivePlaneMajorOrdered(
    const std::vector<int>& rx,
    const std::vector<int>& ry,
    const std::vector<int>& rz,
    char* out,
    size_t maxSize,
    const std::vector<int>& planeOrder,
    bool biasedFirstOneInit,
    int minComponentWidth) {
    return encodeAdaptiveBitPlaneInternal(rx, ry, rz, out, maxSize, true,
                                          biasedFirstOneInit, minComponentWidth,
                                          &planeOrder);
}

MultiModelRangeEncoder::MultiModelRangeEncoder(char* out, size_t* offset)
    : offset_(offset) {
    rc_.p_data = out;
    rc_.p_dataOffset = offset_;
    start_encoding(&rc_, 0, 0);
}

MultiModelRangeEncoder::~MultiModelRangeEncoder() {
    for (qsmodel& m : models_) {
        deleteqsmodel(&m);
    }
}

int MultiModelRangeEncoder::addModel(const ModelSpec& spec) {
    models_.emplace_back();
    initqsmodel(&models_.back(), spec.alphabetSize, spec.scaleBits,
                spec.rescale, const_cast<int*>(spec.initCounts), 1);
    specs_.push_back(spec);
    return static_cast<int>(models_.size()) - 1;
}

void MultiModelRangeEncoder::encodeRawBit(int bit) {
    if (bit < 0) bit = 0;
    if (bit > 1) bit = 1;
    encode_shift(&rc_, 1, static_cast<freq>(bit), 1);
}

void MultiModelRangeEncoder::encodeRawByte(uint8_t v) {
    encode_byte(&rc_, v);
}

void MultiModelRangeEncoder::encodeRawShort(uint16_t v) {
    encode_short(&rc_, v);
}

void MultiModelRangeEncoder::encodeRawSymbol(int sym, int bits) {
    if (bits <= 0) return;
    encode_shift(&rc_, 1, static_cast<freq>(sym), bits);
}

void MultiModelRangeEncoder::encodeRawSymbolWithFreq(int sym,
                                                     int syfreq,
                                                     int ltfreq,
                                                     int scaleBits) {
    encode_shift(&rc_, syfreq, ltfreq, scaleBits);
}

void MultiModelRangeEncoder::encode(int modelId, int sym) {
    if (modelId < 0 || static_cast<size_t>(modelId) >= models_.size()) {
        throw std::runtime_error("[MultiModelRangeEncoder] invalid model id");
    }
    const ModelSpec& spec = specs_[static_cast<size_t>(modelId)];
    if (sym < 0 || sym >= spec.alphabetSize) {
        std::cerr << "[MultiModelRangeEncoder] symbol " << sym
                  << " out of range [0, " << spec.alphabetSize << ")"
                  << std::endl;
        sym = 0;
    }
    qsmodel& m = models_[static_cast<size_t>(modelId)];
    int syfreq, ltfreq;
    qsgetfreq(&m, sym, &syfreq, &ltfreq);
    encode_shift(&rc_, syfreq, ltfreq, spec.scaleBits);
    qsupdate(&m, sym);
}

size_t MultiModelRangeEncoder::finish() {
    return done_encoding(&rc_);
}

}  // namespace pmsf
