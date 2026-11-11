// SPDX-License-Identifier: Apache-2.0
//
// APTMTagSymbolReadOperator.h — APTM P1 geometry coder prototype, decoder side.
//
// 与 `APTMTagSymbolCollectOperator` 对称：APTM Tag 产物 = CODEC_016 容器
// （layout=16，3 段流：valence / fusedFaceSym / residual），但 scheme_version 是
// APTM 自己的 64/65，residual 的语义不是 Cartesian 而是：
//   - 64 (frenet)：可逆整数 lifting 的 Frenet 分量 (c_n,c_t,c_b)；
//   - 65 (polar) ：(R, θq, φq) 三元组（θ/φ 各 3-bit，论文 §4.2）。
//
// 实现策略（零复制父类解码逻辑）：
//   1. peep 容器 sv：若为 64/65，则把 blob 拷到本地缓冲、把 sv patch 回 3；
//   2. 调父类 `PCLTTMTagSymbolReadOperator::importFromRangeCodedBuffer` —— 它按
//      CODEC_016/sv=3 解析 3 段流，把 residual 三元组装进 rcDecodedResiduals_
//      （对 64 来说这**就是** Frenet 分量；对 65 来说是未展开的 polar 三元组）；
//   3. polar：把队列里的 (R,θq,φq) 逐项展开成 Frenet 分量（与 encoder 展开公式
//      逐位一致）；frenet：不动；
//   4. 置 `rcUseFrenetResidual_ = true` ⇒ 父类 `collectFromDecoded` 在还原每个
//      polyFace 时自动调 `unprojectResidualFromFrenet`（= 可逆 lift 的整数逆，
//      用**量化整点位置**构造 frame，与 encoder 逐位同源）。
//
// 这样 APTM 不需要复制父类 400 行的 BFS / v0-fan 对角线反演；P2 若要做真正的
// 跨轮 decoder 模拟（polar 的跨轮对称），才需要替换 `collectFromDecoded`。
//
// 解码器选择：APTMAlgorithm::makeDecompressOptions 在本 reader 启用时显式装配
// `ColumnarDeserializeOperator(this, PCLTTMTagReconstructOperator)`，并按
// `afterBaseMeshHook` 补挂 `kVPropRDR1`（父类 Frenet 路径的预测器状态）。

#ifndef PMSF_APTM_DECOMPRESS_APTMTAG_SYMBOL_READ_OPERATOR_H
#define PMSF_APTM_DECOMPRESS_APTMTAG_SYMBOL_READ_OPERATOR_H

#include "APTMTagSchemeVersion.h"
#include "PCLTTM/decompress/PCLTTMTagSymbolReadOperator.h"

#include <cstdint>
#include <cstdlib>
#include <string>

namespace pmsf {
namespace aptm_tag {

/// 本 reader 支持的 APTM scheme_version（见 collector 头）。
inline bool isAPTMTagSchemeVersion(std::uint16_t sv) {
    return sv == kSchemeVersionFrenet || sv == kSchemeVersionPolar;
}

}  // namespace aptm_tag
}  // namespace pmsf

class APTMTagSymbolReadOperator final : public PCLTTMTagSymbolReadOperator {
  public:
    APTMTagSymbolReadOperator() = default;

    /// 解析 CODEC_016 容器：sv=3 时原样委托父类；sv=64/65 时先 patch 再委托，
    /// 随后按 APTM 语义展开 residual 队列并打开父类的 Frenet 反投影分支。
    void importFromRangeCodedBuffer(char* buffer,
                                    size_t& dataOffset,
                                    unsigned nbQuantBits) override;

    int lastSchemeVersion() const { return lastSchemeVersion_; }
    int lastRawResidualCount() const { return lastRawResidualCount_; }

  private:
    int lastSchemeVersion_ = 0;
    int lastRawResidualCount_ = 0;
};

#endif  // PMSF_APTM_DECOMPRESS_APTMTAG_SYMBOL_READ_OPERATOR_H
