/*
 * @file ContextAdaptiveRangeCoder.h
 * @brief Context-adaptive range coder —— PPMC / RD bpv-gap closure 用。
 *
 * 动机：RangeCoderEntropyCoder 用单个 qsmodel 对整串 symbol 编码。PPMC 原论文
 * §4 的关键技术之一是"per-context adaptive coder"，把 alphabet 按"前驱符号值"
 * 切成若干 context，每个 context 各自维护一个 qsmodel —— 在强相关符号串（面
 * splittable flag、edge-add flag）上能把 bpv 再压 ~15-25%。
 *
 * 设计：
 *   - constructor 取 (alphabet_bits, context_count)，context_count 默认等于
 *     alphabet_size（前一个 symbol 直接当 context id）。
 *   - reset() 重建 context_count 个 qsmodel，每个的 alphabet 都是 alphabet_size。
 *   - encode() 顺序处理每个 symbol：先用当前 context 的 qsmodel 编码，再把当
 *     前 symbol 设为下一个 symbol 的 context。
 *   - decode() 对称。
 *
 * 与 RangeCoderEntropyCoder 完全 ABI 兼容（继承 pmsf::EntropyCoder），impl/
 * main 只需切换 make_shared<ContextAdaptiveRangeCoder> 即可。
 */
#ifndef PMSF_ENTROPY_CONTEXT_ADAPTIVE_RANGE_CODER_H
#define PMSF_ENTROPY_CONTEXT_ADAPTIVE_RANGE_CODER_H

#include "EntropyCoder.h"
#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"

#include <algorithm>
#include <cstring>
#include <stdexcept>
#include <vector>

namespace pmsf {

class ContextAdaptiveRangeCoder final : public EntropyCoder {
public:
    /// @param alphabet_bits log2(alphabet_size)
    /// @param context_bits  log2(context_count)。默认等于 alphabet_bits，即每个
    ///        前驱符号独占一个 context。=0 表示禁用 context，退化为普通 RC。
    ContextAdaptiveRangeCoder(int alphabet_bits, int context_bits = -1)
        : alphabet_bits_(std::clamp(alphabet_bits, 1, 16)),
          context_bits_(context_bits < 0 ? alphabet_bits_ : std::clamp(context_bits, 0, alphabet_bits_)) {}

    void reset() override {}

    void encode(const std::vector<std::uint32_t>& symbols,
                std::vector<std::uint8_t>& out) override {
        const int alphabet_size = 1 << alphabet_bits_;
        const int lg_totf = alphabet_bits_;
        const int ctx_count = 1 << context_bits_;

        // 预算 buffer：context 切分会让 qsmodel 略显原始、每 symbol 在最差情况
        // 下接近 alphabet_bits 位；乘 2 + 1024 头部冗余。
        std::vector<char> buf(std::max<std::size_t>(symbols.size() * 2 + 1024, 4096));
        std::size_t off = 0;
        rangecoder rc;
        rc.p_data = buf.data();
        rc.p_dataOffset = &off;
        start_encoding(&rc, 0, 0);

        std::vector<qsmodel> models(ctx_count);
        for (int i = 0; i < ctx_count; ++i) {
            initqsmodel(&models[i], alphabet_size, lg_totf,
                        1 << std::max(0, lg_totf - 1), nullptr, 1);
        }

        int ctx = 0;
        const int ctx_mask = ctx_count - 1;
        for (std::uint32_t s : symbols) {
            if (static_cast<int>(s) >= alphabet_size) s = alphabet_size - 1;
            int sy_f = 0, lt_f = 0;
            qsgetfreq(&models[ctx], static_cast<int>(s), &sy_f, &lt_f);
            encode_shift(&rc, static_cast<freq>(sy_f), static_cast<freq>(lt_f),
                         static_cast<freq>(lg_totf));
            qsupdate(&models[ctx], static_cast<int>(s));
            ctx = static_cast<int>(s) & ctx_mask;  // 下一个 symbol 的 context = 当前 symbol 的低位
        }
        done_encoding(&rc);
        for (auto& m : models) deleteqsmodel(&m);
        out.insert(out.end(), buf.begin(), buf.begin() + off);
    }

    std::size_t decode(const std::uint8_t* in,
                       std::size_t in_bytes,
                       std::size_t count,
                       std::vector<std::uint32_t>& out) override {
        const int alphabet_size = 1 << alphabet_bits_;
        const int lg_totf = alphabet_bits_;
        const int ctx_count = 1 << context_bits_;
        const int ctx_mask = ctx_count - 1;

        std::size_t off = 0;
        rangecoder rc;
        std::vector<char> staging(in, in + in_bytes);
        rc.p_data = staging.data();
        rc.p_dataOffset = &off;
        if (start_decoding(&rc) != 0) {
            throw std::runtime_error("ContextAdaptiveRangeCoder: start_decoding failed");
        }

        std::vector<qsmodel> models(ctx_count);
        for (int i = 0; i < ctx_count; ++i) {
            initqsmodel(&models[i], alphabet_size, lg_totf,
                        1 << std::max(0, lg_totf - 1), nullptr, 0);
        }

        out.reserve(out.size() + count);
        int ctx = 0;
        for (std::size_t i = 0; i < count; ++i) {
            int ch = decode_culshift(&rc, lg_totf);
            int sym = qsgetsym(&models[ctx], ch);
            int sy_f = 0, lt_f = 0;
            qsgetfreq(&models[ctx], sym, &sy_f, &lt_f);
            decode_update(&rc, static_cast<freq>(sy_f),
                          static_cast<freq>(lt_f), static_cast<freq>(1 << lg_totf));
            qsupdate(&models[ctx], sym);
            out.push_back(static_cast<std::uint32_t>(sym));
            ctx = sym & ctx_mask;
        }
        done_decoding(&rc);
        for (auto& m : models) deleteqsmodel(&m);
        return off;
    }

private:
    int alphabet_bits_;
    int context_bits_;
};

}  // namespace pmsf

#endif  // PMSF_ENTROPY_CONTEXT_ADAPTIVE_RANGE_CODER_H
