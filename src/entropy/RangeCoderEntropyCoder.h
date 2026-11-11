/*
 * @file RangeCoderEntropyCoder.h
 * @brief EntropyCoder adapter over the existing src/rangecoder/ library.
 *
 * Wraps the `rangecod.h` + `qsmodel.h` C API that BasicSymbolCollectOperator
 * already uses for its per-round quantized-point streams. Algorithms that
 * want the classical PMSF behaviour — adaptive qsmodel with a power-of-two
 * alphabet — can pick this coder via the Builder and pay zero LOC to
 * reproduce the legacy output.
 *
 * The adapter is deliberately thin: it owns an output buffer, forwards
 * encode/decode to the C entrypoints, and carries just enough state
 * (alphabet size in bits, encoded block size) to make round-trip reliable.
 */
#ifndef PMSF_ENTROPY_RANGE_CODER_ENTROPY_CODER_H
#define PMSF_ENTROPY_RANGE_CODER_ENTROPY_CODER_H

#include "EntropyCoder.h"
#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"

#include <algorithm>
#include <cstring>
#include <stdexcept>

namespace pmsf {

/// Optional context model used by the range coder.
enum class RangeCoderContextModel : std::uint8_t {
    None = 0,
    Valence4 = 1
};

/// @param alphabet_bits log2(alphabet). Must equal the caller's quantization
///        bit-width; outside symbols are clamped into range.
class RangeCoderEntropyCoder final : public EntropyCoder {
public:
    /// @brief 标记「按**精确**字母表大小构造」，不补齐到 2 的幂。
    ///
    /// 为什么需要：底层 `qsmodel` 本来就接受任意 n（`initqsmodel(m, int n, ...)`），
    /// 但此前这个封装只暴露 `alphabet_bits` 并把 sizes 撑成 `1 << bits`。
    /// 当调用方的实际字母表不是 2 的幂时（例如某层残差的 span = 458），
    /// 撑成 512 会白白多花 `log2(512/458) ≈ 0.16` bit/符号。
    /// 用 tag 而不是重载，避免 `RangeCoderEntropyCoder(458)` 与
    /// `RangeCoderEntropyCoder(9)` 在调用点长得一样、读错意思。
    struct ExactAlphabet {};

    explicit RangeCoderEntropyCoder(int alphabet_bits,
                                    RangeCoderContextModel ctx = RangeCoderContextModel::None)
        : alphabet_bits_(std::clamp(alphabet_bits, 1, 16)), context_(ctx) {}

    RangeCoderEntropyCoder(int exact_alphabet_size, ExactAlphabet,
                           RangeCoderContextModel ctx = RangeCoderContextModel::None)
        : alphabet_bits_(0), exact_size_(std::clamp(exact_alphabet_size, 2, 1 << 16)), context_(ctx) {}

    void reset() override {}

    /// @brief 实际字母表大小：精确模式用精确值，bits 模式用 1<<bits。
    int alphabetSize() const {
        return exact_size_ > 0 ? exact_size_ : (1 << alphabet_bits_);
    }

    void encode(const std::vector<std::uint32_t>& symbols,
                std::vector<std::uint8_t>& out) override {
        if (context_ == RangeCoderContextModel::Valence4 && alphabet_bits_ == 3) {
            encodeValence4(symbols, out);
            return;
        }
        const int alphabet_size = alphabetSize();
        const int lg_totf = std::max(10, alphabet_bits_);
        std::vector<char> buf(std::max<std::size_t>(symbols.size() * 2 + 1024, 4096));
        std::size_t off = 0;
        rangecoder rc;
        rc.p_data = buf.data();
        rc.p_dataOffset = &off;
        start_encoding(&rc, 0, 0);
        qsmodel model;
        initqsmodel(&model, alphabet_size, lg_totf,
                    1 << std::max(0, lg_totf - 1), nullptr, 1);
        for (std::uint32_t s : symbols) {
            if (static_cast<int>(s) >= alphabet_size) s = alphabet_size - 1;
            int sy_f = 0, lt_f = 0;
            qsgetfreq(&model, static_cast<int>(s), &sy_f, &lt_f);
            encode_shift(&rc, static_cast<freq>(sy_f), static_cast<freq>(lt_f),
                         static_cast<freq>(lg_totf));
            qsupdate(&model, static_cast<int>(s));
        }
        done_encoding(&rc);
        deleteqsmodel(&model);
        out.insert(out.end(), buf.begin(), buf.begin() + off);
    }

    std::size_t decode(const std::uint8_t* in,
                       std::size_t in_bytes,
                       std::size_t count,
                       std::vector<std::uint32_t>& out) override {
        if (context_ == RangeCoderContextModel::Valence4 && alphabet_bits_ == 3) {
            return decodeValence4(in, in_bytes, count, out);
        }
        const int alphabet_size = alphabetSize();
        const int lg_totf = std::max(10, alphabet_bits_);
        std::size_t off = 0;
        rangecoder rc;
        // The range coder C API expects a mutable char* buffer; we stage a
        // private copy so we never mutate caller memory.
        std::vector<char> staging(in, in + in_bytes);
        rc.p_data = staging.data();
        rc.p_dataOffset = &off;
        if (start_decoding(&rc) != 0) {
            throw std::runtime_error("RangeCoderEntropyCoder: start_decoding failed");
        }
        qsmodel model;
        initqsmodel(&model, alphabet_size, lg_totf,
                    1 << std::max(0, lg_totf - 1), nullptr, 0);
        out.reserve(out.size() + count);
        for (std::size_t i = 0; i < count; ++i) {
            int ch = decode_culshift(&rc, lg_totf);
            int sym = qsgetsym(&model, ch);
            int sy_f = 0, lt_f = 0;
            qsgetfreq(&model, sym, &sy_f, &lt_f);
            decode_update(&rc, static_cast<freq>(sy_f),
                          static_cast<freq>(lt_f), static_cast<freq>(1 << lg_totf));
            qsupdate(&model, sym);
            out.push_back(static_cast<std::uint32_t>(sym));
        }
        done_decoding(&rc);
        deleteqsmodel(&model);
        return off;
    }

private:
    int alphabet_bits_;
    int exact_size_ = 0;  // >0 表示「精确字母表」模式;
    RangeCoderContextModel context_;

    static int deriveValence4Ctx(int prev) {
        if (prev < 0) return 0;
        if (prev == 1 || prev == 2) return 1;  // VAL_3 / VAL_4
        if (prev == 3 || prev == 4) return 2;  // VAL_5 / VAL_6
        return 3;
    }

    void encodeValence4(const std::vector<std::uint32_t>& symbols,
                        std::vector<std::uint8_t>& out) {
        constexpr int kAlphabetSize = 8;
        constexpr int kLgTotf = 10;
        constexpr int kRescale = 1 << 9;
        constexpr int kNumCtx = 4;

        std::vector<char> buf(std::max<std::size_t>(symbols.size() * 2 + 1024, 4096));
        std::size_t off = 0;
        rangecoder rc;
        rc.p_data = buf.data();
        rc.p_dataOffset = &off;
        start_encoding(&rc, 0, 0);

        qsmodel models[kNumCtx];
        for (int i = 0; i < kNumCtx; ++i) {
            initqsmodel(&models[i], kAlphabetSize, kLgTotf, kRescale, nullptr, 1);
        }

        int prev = -1;
        for (std::uint32_t s : symbols) {
            int sym = static_cast<int>(s) & (kAlphabetSize - 1);
            int ctx = deriveValence4Ctx(prev);
            int sy_f = 0, lt_f = 0;
            qsgetfreq(&models[ctx], sym, &sy_f, &lt_f);
            encode_shift(&rc, static_cast<freq>(sy_f), static_cast<freq>(lt_f),
                         static_cast<freq>(kLgTotf));
            qsupdate(&models[ctx], sym);
            prev = sym;
        }
        done_encoding(&rc);
        for (int i = 0; i < kNumCtx; ++i) deleteqsmodel(&models[i]);
        out.insert(out.end(), buf.begin(), buf.begin() + off);
    }

    std::size_t decodeValence4(const std::uint8_t* in,
                               std::size_t in_bytes,
                               std::size_t count,
                               std::vector<std::uint32_t>& out) {
        constexpr int kAlphabetSize = 8;
        constexpr int kLgTotf = 10;
        constexpr int kRescale = 1 << 9;
        constexpr int kNumCtx = 4;

        std::size_t off = 0;
        rangecoder rc;
        std::vector<char> staging(in, in + in_bytes);
        rc.p_data = staging.data();
        rc.p_dataOffset = &off;
        if (start_decoding(&rc) != 0) {
            throw std::runtime_error(
                "RangeCoderEntropyCoder: valence4 start_decoding failed");
        }

        qsmodel models[kNumCtx];
        for (int i = 0; i < kNumCtx; ++i) {
            initqsmodel(&models[i], kAlphabetSize, kLgTotf, kRescale, nullptr, 0);
        }

        out.reserve(out.size() + count);
        int prev = -1;
        for (std::size_t i = 0; i < count; ++i) {
            int ctx = deriveValence4Ctx(prev);
            int ch = decode_culshift(&rc, kLgTotf);
            int sym = qsgetsym(&models[ctx], ch);
            int sy_f = 0, lt_f = 0;
            qsgetfreq(&models[ctx], sym, &sy_f, &lt_f);
            decode_update(&rc, static_cast<freq>(sy_f), static_cast<freq>(lt_f),
                          static_cast<freq>(1 << kLgTotf));
            qsupdate(&models[ctx], sym);
            out.push_back(static_cast<std::uint32_t>(sym));
            prev = sym;
        }
        done_decoding(&rc);
        for (int i = 0; i < kNumCtx; ++i) deleteqsmodel(&models[i]);
        return off;
    }
};

}  // namespace pmsf

#endif  // PMSF_ENTROPY_RANGE_CODER_ENTROPY_CODER_H
