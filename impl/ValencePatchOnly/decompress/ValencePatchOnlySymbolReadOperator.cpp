/*
 * @file        ValencePatchOnlySymbolReadOperator.cpp
 * @brief       方案 V — Valence-Patch-Only decoder 实现。
 *
 * 与 collector 字节级对称：解 ConnectivityBlobContainer 的 3 个子流，
 * 投影回父类 RDGateOnly v3 协议的 5 个解码队列，让父类 collectFromDecoded
 * 接管 BFS / mesh 重建。
 */

#include "ValencePatchOnlySymbolReadOperator.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂

#include "compress/ConnectivityBlobContainer.h"
#include "ValencePatchOnly/compress/ValencePatchOnlySymbolCollectOperator.h"  // ValencePatchCode enum
#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"
#include "Point.h"

#include <cstring>
#include <iostream>
#include <span>
#include <stdexcept>

namespace {

// 父类 RDValenceCode = {VAL_3=0, VAL_4=1, VAL_5=2, VAL_6=3, VAL_OTHER=4}（5 元）。
// 方案 V 6 元：{NULL=0, VAL_3=1, VAL_4=2, VAL_5=3, VAL_6=4, VAL_OTHER=5}。
// 6 元 → (isPoly, 5元) 投影：
//   sym6 == 0 → isPoly=0, 不产生 5 元码字。
//   sym6 ∈ {1..5} → isPoly=1, 5 元码字 = sym6 - 1。
//
// 与 collector 的 mapFiveToSixIfPoly() 严格逆映射。
inline bool sym6IsPoly(uint8_t s6) { return s6 != 0; }
inline uint8_t sym6ToSym5(uint8_t s6) { return static_cast<uint8_t>(s6 - 1); }

}  // namespace

void ValencePatchOnlySymbolReadOperator::importFromRangeCodedBuffer(
    char* buffer,
    size_t& dataOffset,
    unsigned nbQuantBits) {
    // ---- 1) 解 ConnectivityBlobContainer ----
    // collector 写出的总字节数 = blob.size()；ColumnarDeserializeOperator 已经把
    // _symbols.parquet 的 BINARY 列内容拷到 rcBlob_，并通过 dataOffset=0 调进来。
    // 我们读完整个 blob 即可。
    //
    // 注意：blob 长度是 Columnar 调用方传入的 rcBlob_ 总长，但 Columnar deserializer
    // 没有把 size 传给我们（接口签名只有 buffer / dataOffset / nbQuantBits）。
    // ConnectivityBlobReader 通过 magic + header 自带的 stream_offsets 校验，
    // 所以只要 buffer 起点对就能解，多余的字节不会被读。我们假设
    // dataOffset 之后就是完整 blob（Columnar dispatch 行为符合此假设）。

    // 我们没有 blob 长度，但 ConnectivityBlobReader 需要一个 span。用一个保守的上限：
    // 把 buffer + dataOffset 后的所有字节都当作 blob 的潜在范围。Reader 内部
    // 通过 header.stream_offsets sentinel 确定真实 payload 长度（已带边界校验）。
    //
    // 风险点：如果 Columnar deserialize 后续还要消费 buffer 末尾的其它数据，
    // 这种过度 reader 不会破坏 buffer，但会错失边界检查。Phase 1 阶段
    // ColumnarDeserializeOperator 对 layout=9 case 只做单次解码 —— 没有后续
    // 字节消费，安全。
    //
    // 实务做法：我们假设 Columnar 把 rc_blob 的尾部当作"buffer 边界"。对应位置
    // case 9 的实现里我们会让它传 size。当前先用一个比较大的"软上限"探测：
    //   span = (buffer + dataOffset, max_in_practice)
    // 然后由 Reader 的 magic 与 sentinel 失败时报错。
    //
    // 经过权衡：我们无法可靠从签名拿到 size。最稳的选择是让派生 Read 在
    // ColumnarDeserializeOperator case 9 里被显式 init 时传 buffer 长度
    // —— 但接口签名暂不变。临时方案：用 BitReader 自己沿 header 读，header 大小
    // 推算后只取必要字节。这正是 ConnectivityBlobReader 已经做的事 —— 它从
    // header.stream_offsets 后缀 sentinel 推 payload 总长，自验证。

    // 我们这里采用：传一个非常大的 span（4GB），让 Reader 内部按真实 sentinel 截断。
    constexpr size_t kArbitraryLargeView = static_cast<size_t>(1) << 32;
    std::span<const uint8_t> view(reinterpret_cast<const uint8_t*>(buffer + dataOffset),
                                   kArbitraryLargeView);

    pmsf::ConnectivityBlobReader rd(view);
    const auto& hdr = rd.header();
    const auto spec = pmsf::valencePatchOnlyConnectivityBlobSpec(
        static_cast<std::int32_t>(hdr.num_rounds));
    pmsf::requirePaperConnectivityBlobHeaderMatchesSpec(
        hdr, spec, "ValencePatchOnly");

    const int numRounds = static_cast<int>(hdr.num_rounds);

    // ---- 2) Quantization invariant ----
    rcUsingResidualLocal_ = header_.isEnablePrediction() && header_.isEnableQuantization();
    if (!rcUsingResidualLocal_) {
        throw std::runtime_error(
            "[ValencePatchOnly decoder] expects enablePrediction && enableQuantization");
    }
    const int BP_BITS = static_cast<int>(nbQuantBits) + 1;

    // ---- 3) 准备每轮访问数（faceBitsPerRound_）----
    // 在解 stream 0 时按轮统计 isPoly bit 数，自然得到。
    faceBitsPerRound_.assign(numRounds, 0);

    // ---- 4) round-major 解 3 段流 ----
    // ConnectivityBlobBuilder 写入时按 (round, stream) 平铺，所以这里也按 round
    // 序遍历，每轮 3 段独立 start_decoding/done_decoding（与 collector 对称）。
    // qsmodel 频率精度：必须与 collector 字节对称 —— lg_totf=10 (totf=1024)。
    // 见 ValencePatchOnlySymbolCollectOperator.cpp 同段注释。
    constexpr int alphabet_size = 8;
    constexpr int lg_totf = 10;
    constexpr int rescale = 1 << 9;

    for (int r = 0; r < numRounds; ++r) {
        // ---- stream 0: [int32 numFaces][int32 numOther][bytes 4-context RC] ----
        {
        auto sl = rd.getStream(r, pmsf::kPaperBlobStreamValencePatchOnlyValence6);
            if (sl.size() < 8) {
                throw std::runtime_error(
                    "[ValencePatchOnly decoder] stream 0 too small for header");
            }
            int32_t numFaces = 0, numOther = 0;
            std::memcpy(&numFaces, sl.data(), 4);
            std::memcpy(&numOther, sl.data() + 4, 4);
            faceBitsPerRound_[r] = numFaces;

            std::vector<char> rcStaging(sl.begin() + 8, sl.end());
            size_t rcOffset = 0;
            rangecoder rc;
            rc.p_data = rcStaging.data();
            rc.p_dataOffset = &rcOffset;
            if (start_decoding(&rc) != 0) {
                throw std::runtime_error(
                    "[ValencePatchOnly decoder] valence6 RC start_decoding failed at round=" +
                    std::to_string(r));
            }

            qsmodel ms[4];
            for (int i = 0; i < 4; ++i) {
                initqsmodel(&ms[i], alphabet_size, lg_totf, rescale, nullptr, 0);
            }
            qsmodel esc;
            initqsmodel(&esc, 256, 10, 1 << 9, nullptr, 0);

            int prev = -1;
            int otherSeen = 0;
            for (int k = 0; k < numFaces; ++k) {
                int ctx = (prev == -1) ? 0
                       : (prev == 0)  ? 1
                       : (prev == 5)  ? 3
                                      : 2;
                int syfreq, ltfreq;
                int ltval = decode_culshift(&rc, lg_totf);
                int sym = qsgetsym(&ms[ctx], ltval);
                qsgetfreq(&ms[ctx], sym, &syfreq, &ltfreq);
                decode_update(&rc, syfreq, ltfreq, 1 << lg_totf);
                qsupdate(&ms[ctx], sym);

                if (sym6IsPoly(static_cast<uint8_t>(sym))) {
                    rcDecodedIsPoly_.push_back(1);
                    rcDecodedValCodes_.push_back(sym6ToSym5(static_cast<uint8_t>(sym)));
                    if (sym == static_cast<int>(ValencePatchCode::VAL_OTHER)) {
                        int eltval = decode_culshift(&rc, 10);
                        int esym = qsgetsym(&esc, eltval);
                        int esyfreq, eltfreq;
                        qsgetfreq(&esc, esym, &esyfreq, &eltfreq);
                        decode_update(&rc, esyfreq, eltfreq, 1 << 10);
                        qsupdate(&esc, esym);
                        rcDecodedOtherVals_.push_back(static_cast<uint8_t>(esym));
                        otherSeen++;
                    }
                } else {
                    rcDecodedIsPoly_.push_back(0);
                }
                prev = sym;
            }
            done_decoding(&rc);
            for (auto& m : ms) deleteqsmodel(&m);
            deleteqsmodel(&esc);

            if (otherSeen != numOther) {
                std::cerr << "[ValencePatchOnly decoder] WARN round=" << r
                          << " V_OTHER count mismatch (header=" << numOther
                          << " decoded=" << otherSeen << ")" << std::endl;
            }
        }

        // ---- stream 1: edgeSym (per polyFace 3 bit, RC 2 元) ----
        // 我们在头部记一下 numEdgeSyms 才能知道读多少个 sym。同 stream 0 加 4-byte 头。
        {
        auto sl = rd.getStream(r, pmsf::kPaperBlobStreamValencePatchOnlyEdgeSym);
            if (sl.size() < 4) {
                throw std::runtime_error(
                    "[ValencePatchOnly decoder] stream 1 too small for header");
            }
            int32_t numEdgeSyms = 0;
            std::memcpy(&numEdgeSyms, sl.data(), 4);
            std::vector<char> rcStaging(sl.begin() + 4, sl.end());
            size_t rcOffset = 0;
            rangecoder rc;
            rc.p_data = rcStaging.data();
            rc.p_dataOffset = &rcOffset;
            if (start_decoding(&rc) != 0) {
                throw std::runtime_error(
                    "[ValencePatchOnly decoder] edgeSym start_decoding failed");
            }
            qsmodel m;
            initqsmodel(&m, 2, 10, 1 << 9, nullptr, 0);
            for (int k = 0; k < numEdgeSyms; ++k) {
                int syfreq, ltfreq;
                int ltval = decode_culshift(&rc, 10);
                int sym = qsgetsym(&m, ltval);
                qsgetfreq(&m, sym, &syfreq, &ltfreq);
                decode_update(&rc, syfreq, ltfreq, 1 << 10);
                qsupdate(&m, sym);
                rcDecodedEdgeSyms_.push_back(static_cast<uint8_t>(sym));
            }
            done_decoding(&rc);
            deleteqsmodel(&m);
        }

        // ---- stream 2: residual (bit-plane RC) ----
        {
        auto sl = rd.getStream(r, pmsf::kPaperBlobStreamValencePatchOnlyResidual);
            if (sl.size() < 4) {
                throw std::runtime_error(
                    "[ValencePatchOnly decoder] stream 2 too small for header");
            }
            int32_t numCoords = 0;
            std::memcpy(&numCoords, sl.data(), 4);
            std::vector<char> rcStaging(sl.begin() + 4, sl.end());
            size_t rcOffset = 0;
            rangecoder rc;
            rc.p_data = rcStaging.data();
            rc.p_dataOffset = &rcOffset;
            if (start_decoding(&rc) != 0) {
                throw std::runtime_error(
                    "[ValencePatchOnly decoder] residual start_decoding failed");
            }
            std::vector<qsmodel> bpFirst1(BP_BITS), bpAfter1(BP_BITS);
            for (int b = 0; b < BP_BITS; ++b) {
                initqsmodel(&bpFirst1[b], 2, 10, 1 << 9, nullptr, 0);
                initqsmodel(&bpAfter1[b], 2, 10, 1 << 9, nullptr, 0);
            }
            auto decResidualBP = [&]() -> int32_t {
                uint32_t z = 0;
                bool seenOne = false;
                for (int b = BP_BITS - 1; b >= 0; --b) {
                    qsmodel* mp = seenOne ? &bpAfter1[b] : &bpFirst1[b];
                    int syfreq, ltfreq;
                    int ltval = decode_culshift(&rc, 10);
                    int sym = qsgetsym(mp, ltval);
                    qsgetfreq(mp, sym, &syfreq, &ltfreq);
                    decode_update(&rc, syfreq, ltfreq, 1 << 10);
                    qsupdate(mp, sym);
                    z = (z << 1) | static_cast<uint32_t>(sym);
                    if (sym) seenOne = true;
                }
                return static_cast<int32_t>((z >> 1) ^ -static_cast<int32_t>(z & 1));
            };
            int coordsConsumed = 0;
            while (coordsConsumed < numCoords) {
                int32_t rx = decResidualBP();
                int32_t ry = decResidualBP();
                int32_t rz = decResidualBP();
                rcDecodedResiduals_.push_back(MCGAL::PointInt(rx, ry, rz));
                coordsConsumed += 3;
            }
            done_decoding(&rc);
            for (int b = 0; b < BP_BITS; ++b) {
                deleteqsmodel(&bpFirst1[b]);
                deleteqsmodel(&bpAfter1[b]);
            }
        }
    }

    // dataOffset 推进：等于整个 blob 的字节数（header + payload）。
    // ConnectivityBlobReader 没有直接暴露这个；用 header.headerByteSize() +
    // 末尾 sentinel 推。
    const size_t blobSize = hdr.headerByteSize() +
                            (hdr.stream_offsets.empty() ? 0 : hdr.stream_offsets.back());
    dataOffset += blobSize;

    const char* env = std::getenv("PMSF_BPV_DIAG");
    if (env && env[0] == '1') {
        std::cerr << "[ValencePatchOnly decoder] rounds=" << numRounds
                  << " totalIsPolyBits=" << rcDecodedIsPoly_.size()
                  << " totalValCodes=" << rcDecodedValCodes_.size()
                  << " totalOtherVals=" << rcDecodedOtherVals_.size()
                  << " totalEdgeSyms=" << rcDecodedEdgeSyms_.size()
                  << " totalResiduals=" << rcDecodedResiduals_.size()
                  << " blobSize=" << blobSize << std::endl;
    }
}
