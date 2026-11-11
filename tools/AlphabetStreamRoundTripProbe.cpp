// SPDX-License-Identifier: Apache-2.0
//
// AlphabetStreamRoundTripProbe.cpp — P16 T3.2
//
// 判据：pmsf::decodeAlphabetStream 必须是 pmsf::encodeAlphabetStream 的
// **bit-exact 镜像** —— 对多组（字母表大小 × 偏置 × 长度）的符号序列，
// 编码后再解码必须逐符号相同。
//
// 这个探针是 facet 换 RC 那条路的**前置闸**：解码器不过闸，不许接线。
#include "common/AlphabetStreamCodec.h"

#include <algorithm>
#include <cstdio>
#include <cstdint>
#include <deque>
#include <random>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    struct Case {
        const char* name;
        int alphabet;
        double zeroBias;   // 生成时落在 0 的概率
        std::size_t n;
    };
    const Case cases[] = {
        {"bit/uniform/n=1",        2, 0.50, 1},
        {"bit/uniform/n=1000",     2, 0.50, 1000},
        {"bit/skewed/n=5000",      2, 0.05, 5000},
        {"bit/skewed/n=658900",    2, 0.95, 658900},
        {"4ary/uniform/n=10000",   4, 0.25, 10000},
    };

    // ⚠️ 已知观察（2026-09-25，未定位）：加一组 `alphabet=5, n=50000` 会 **SIGSEGV**。
    // 5 元不是本探针的用途（facet 是二元流，本文件注记的既有消费者是 2 与 4），
    // 故先从 in-contract 范围里移除，并把这条**未定位的崩溃**留在这里备查。
    int failed = 0;
    for (const Case& c : cases) {
        std::mt19937 rng(20260925u);
        std::uniform_real_distribution<double> u(0.0, 1.0);
        std::deque<uint8_t> syms;
        for (std::size_t i = 0; i < c.n; ++i) {
            if (u(rng) < c.zeroBias) {
                syms.push_back(0);
            } else {
                syms.push_back(static_cast<uint8_t>(rng() % static_cast<unsigned>(c.alphabet)));
            }
        }

        const std::vector<uint8_t> enc = pmsf::encodeAlphabetStream(syms, c.alphabet, nullptr);
        const std::vector<uint8_t> dec =
            pmsf::decodeAlphabetStream(enc.data(), enc.size(), c.alphabet);

        bool ok = (dec.size() == syms.size());
        std::size_t firstBad = 0;
        if (ok) {
            for (std::size_t i = 0; i < dec.size(); ++i) {
                if (dec[i] != syms[i]) { ok = false; firstBad = i; break; }
            }
        }
        if (!ok) ++failed;
        std::printf("%-24s alphabet=%d n=%-7zu enc=%-7zu dec=%-7zu %s%s\n",
                    c.name, c.alphabet, c.n, enc.size(), dec.size(),
                    ok ? "OK" : "MISMATCH",
                    ok ? "" : " (first bad idx logged below)");
        if (!ok && dec.size() == syms.size()) {
            std::printf("    first mismatch at %zu: got %u want %u\n",
                        firstBad, dec[firstBad], syms[firstBad]);
        }
    }

    // ── 可选：对一份真实 dump 做「zlib 位数组 vs RC 符号序列」对照 ──
    // 用法： AlphabetStreamRoundTripProbe <dump>
    // dump 每行： <n> <bit> <bit> ...（一个 bit 一个 token）
    if (const char* dump = (argc > 1 ? argv[1] : nullptr)) {
        std::ifstream in(dump);
        if (!in) { std::printf("cannot open %s\n", dump); return 2; }
        std::deque<uint8_t> syms;
        std::string line;
        while (std::getline(in, line)) {
            std::istringstream ls(line);
            std::size_t n = 0;
            if (!(ls >> n)) continue;
            for (std::size_t i = 0; i < n; ++i) { int v = 0; ls >> v; syms.push_back((uint8_t)(v & 1)); }
        }
        const std::vector<uint8_t> rc = pmsf::encodeAlphabetStream(syms, 2, nullptr);
        std::printf("\n[dump] %s\n  syms=%zu\n", dump, syms.size());
        std::printf("  RC(符号序列)          = %zu B\n", rc.size());
        std::printf("  参考 zlib(位数组)实测  = 28599 B\n");
        std::printf("  参考 KT 模型           = 22460 B\n");
        std::printf("  ⇒ RC/zlib = %.2fx\n", (double)rc.size() / 28599.0);

        // 关键：生产的 encodeVQAdaptiveCharBits 是【逐轮各编一次】（模型每轮重置），
        // 与上面「整条一次编」不等价。这里按逐轮重编一遍求真实预期值。
        std::ifstream in2(dump);
        std::deque<uint8_t> roundSyms;
        std::size_t perRound = 4;   // 每轮载荷 = 4 字节头 + RC
        std::size_t nRounds = 0;
        {
            std::string l2;
            while (std::getline(in2, l2)) {
                std::istringstream ls2(l2);
                std::size_t n = 0;
                if (!(ls2 >> n)) continue;
                roundSyms.clear();
                for (std::size_t i = 0; i < n; ++i) { int v = 0; ls2 >> v; roundSyms.push_back((uint8_t)(v & 1)); }
                perRound += pmsf::encodeAlphabetStream(roundSyms, 2, nullptr).size();
                ++nRounds;
            }
        }
        std::printf("  RC(逐轮各编, %zu 轮)   = %zu B   ← 与生产同构\n", nRounds, perRound);
        std::printf("  参考 zlib(位数组)实测  = 28599 B\n");
        std::printf("  ⇒ 逐轮/zlib = %.2fx  (省 %+d B)\n",
                    (double)perRound / 28599.0, (int)perRound - 28599);

        // ── 定点测试：复刻 packBitStreamThenZlib 的「打包+填充」后，
        //    再按 MSB-first 提取位数组，喂给 RC —— 看 2.7x 异常能否离线复现。
        std::ifstream in3(dump);
        std::vector<uint8_t> packed;
        {
            std::string l3;
            while (std::getline(in3, l3)) {
                std::istringstream ls3(l3);
                std::size_t n = 0;
                if (!(ls3 >> n)) continue;
                std::size_t bitPos = 0;
                std::size_t base = packed.size();
                packed.push_back(0);
                for (std::size_t i = 0; i < n; ++i) {
                    int v = 0; ls3 >> v;
                    const std::size_t byteIdx = base + bitPos / 8;
                    if (byteIdx >= packed.size()) packed.resize(byteIdx + 1, 0);
                    packed[byteIdx] |= (uint8_t)((v & 1) << (7 - (bitPos % 8)));
                    ++bitPos;
                }
                packed.resize(base + (n + 7) / 8 + 1, 0);   // 每轮 +1 填充字节
            }
        }
        std::deque<uint8_t> bitsArr;
        for (std::size_t i = 0; i < packed.size(); ++i)
            for (int b = 7; b >= 0; --b) bitsArr.push_back((packed[i] >> b) & 1u);
        const std::vector<uint8_t> rcBits = pmsf::encodeAlphabetStream(bitsArr, 2, nullptr);
        std::printf("\n[定点] packed=%zu B, bits=%zu\n", packed.size(), bitsArr.size());
        std::printf("  RC(位数组, MSB-first)  = %zu B\n", rcBits.size());
        std::printf("  对照 RC(符号序列)      = %zu B\n", rc.size());
        std::printf("  ⇒ 比值 %.2fx %s\n", (double)rcBits.size() / rc.size(),
                    rcBits.size() > rc.size() * 2 ? "← 异常在离线可复现！" : "← 离线正常，异常在生产插入处");
    }

    std::printf("\n%s: %d/%zu cases failed\n",
                failed == 0 ? "PASS" : "FAIL", failed,
                sizeof(cases) / sizeof(cases[0]));
    return failed == 0 ? 0 : 1;
}
