// SPDX-License-Identifier: Apache-2.0
//
// PCATMZColorSymbolReadOperator.h — PCATM Z-Color (Phase 4 PR-C, BlobLayout=17,
// sv=1) decoder 端。
//
// 协议（sv=1, 4 段流 ConnectivityBlobContainer v3, magic='CONN'）：
//   stream 0 (valence):   5-元 4-context RC + V_OTHER inline escape
//   stream 1 (isPolyBit): 2-元 RC，per BFS face 1 bit
//   stream 2 (colorBits): 2-元 RC，per BFS face 1 bit (RED=0/BLUE=1)
//   stream 3 (residual):  bit-plane RC（含 Phase 3 Step 1 二阶 curvature predictor）
//
// 几何还原路径（与 PCLTTM Tag sv=3 / RD Tag sv=1 同源）：
//   decoder 按 v0-fan 拓扑推 N-3 条对角线，不消费 color 流做对角线决定——
//   color 流保留作"协议忠实留位"+可选 cross-check（PMSF_PCATM_ZCOLOR_VALIDATE=1
//   时 decoder 端 BFS 会把读回的 color 与 face.triangleColor_ 对比，检测
//   encoder/decoder BFS 同序性）。
//
// **与 PCLTTMTagSymbolReadOperator sv=3 的差异（最小化）**：
//   - num_streams 3 → 4，stream 2 为 2-元 colorBits（而 PCLTTM 无 color）；
//   - stream 1 为 2-元 isPolyBit（PCLTTM 是 4-元 fusedFaceSym）；
//   - stream 3 是 residual（PCLTTM 是 stream 2）；
//   - 不填 rcDecodedVertexTags_（保持空）。
//   - collectFromDecoded 不重写：完全复用父类 PCLTTMTag 的 v0-fan 反演路径。
//
// 详见 docs/paper_alignment/PHASE4_PLAN.md §3.4 (PR-C)。

#ifndef PMSF_DECOMPRESS_PCATM_ZCOLOR_SYMBOL_READ_OPERATOR_H
#define PMSF_DECOMPRESS_PCATM_ZCOLOR_SYMBOL_READ_OPERATOR_H

#include "PCLTTM/decompress/PCLTTMTagSymbolReadOperator.h"

#include <cstdint>
#include <deque>

class PCATMZColorSymbolReadOperator : public PCLTTMTagSymbolReadOperator {
  public:
    PCATMZColorSymbolReadOperator() = default;

    /// @brief 解析 BlobLayout=17 sv=1 的 ConnectivityBlobContainer v3 4 段流。
    ///        填充 rcDecodedValCodes_ / rcDecodedOtherVals_ / rcDecodedIsPoly_
    ///        / rcDecodedColors_ / rcDecodedResiduals_。
    ///        **不填** rcDecodedVertexTags_（保持空）。
    void importFromRangeCodedBuffer(char* buffer,
                                    size_t& dataOffset,
                                    unsigned nbQuantBits) override;

    // collectFromDecoded 不重写：复用父类 PCLTTMTagSymbolReadOperator 的
    // v0-fan 对角线反演 + 二阶 curvature pred 还原路径。color 流不参与拓扑
    // 推断。若需 cross-check，在本类后续版本中 override collectFromDecoded
    // 加 PMSF_PCATM_ZCOLOR_VALIDATE=1 assertion 即可。

  protected:
    /// @brief color stream（per BFS face 1 bit, RED=0/BLUE=1）。
    ///        Phase 4 PR-C 阶段不参与对角线推断，保留作 protocol 字段。
    ///        未来 PR-C.2 可接入 union-find 推 patch 边界。
    std::deque<uint8_t> rcDecodedColors_;
};

#endif  // PMSF_DECOMPRESS_PCATM_ZCOLOR_SYMBOL_READ_OPERATOR_H
