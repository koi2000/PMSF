/*
 * @Description: Valence-code symbol collector（PCLTTM 论文连通性编码路径）
 *
 * 码流布局（复用 Optimized 的 BFS 序列结构，只把 facet-sym 升级为 7 元码）：
 *   valenceCodeQueues   : round -> deque<uint8_t>  每个面一个 7 元码
 *   edgeSymbolQueues    : round -> deque<uint8_t>  每个 halfedge 一个 1-bit 对角线标记
 *   pointQueues         : round -> deque<Point>    polyFace 原坐标
 *   ipointQueues        : round -> deque<PointInt> polyFace 残差坐标（预测路径使用）
 *   vertexSymbolQueues  : round -> deque<uint8_t>  (可选 Adjustment)
 *   vertexPointQueues   : round -> deque<Point>    (可选 Adjustment)
 *
 * 码字字母表 ValenceCode（enum 同名）：
 *   NULL_PATCH (0) : 未被 decimate 的普通面
 *   FAN_CHILD  (1) : decimate 后 fan triangulation 产生的子三角形
 *   POLY_V_3..6 (2..5) : polyFace，对应 removedVertex.valence ∈ {3,4,5,6}
 *   POLY_V_OTHER (6)   : 其它异常度数（容错）
 *
 * 解码端（ValenceSymbolReadOperator + ValenceReconstructOperator）：
 *   - 先读 base mesh、所有码字、所有 edge bits 与 point 残差；
 *   - BFS 顺序复盘：每个面消费一个 7 元码；POLY_V_* 则额外读一个 point（残差或原坐标）；
 *     每个 halfedge 消费一个对角线 bit；
 *   - 重建阶段：把所有对角线 halfedge merge_face 掉，还原 polygon；再对所有 POLY_V_*
 *     facet 调用 insert_point 插入被移除顶点。
 *
 * Parquet V2 兼容：serialize 侧通过 BlobLayout::VALENCE_RC 标识，rc_blob 字节流与
 *                  BasicSerializeOperator 的输出一致；解码侧在 ColumnarDeserializeOperator
 *                  中识别 VALENCE_RC layout 后调用 ValenceSymbolReadOperator 的入口。
 *
 * 分区 (segmentation) 兼容：
 *   - 与 OptimizedSymbolCollectOperator 相同，PMSF 按 group 切换 mesh_/seeds_，
 *     每个 group 独立跑 BFS；collect(vector<Halfedge*>) 内部循环多 seed。
 */
#ifndef VALENCE_SYMBOLCOLLECTOPERATOR_H
#define VALENCE_SYMBOLCOLLECTOPERATOR_H

#include "SymbolCollectOperator.h"
#include <cstdint>
#include <deque>
#include <memory>
#include <vector>

namespace MCGAL {
class Point;
class PointInt;
class Halfedge;
class Facet;
}  // namespace MCGAL

enum class ValenceCode : uint8_t {
    NULL_PATCH     = 0,
    FAN_CHILD      = 1,
    POLY_VAL_3     = 2,
    POLY_VAL_4     = 3,
    POLY_VAL_5     = 4,
    POLY_VAL_6     = 5,
    POLY_VAL_OTHER = 6,
    NUM_CODES      = 7
};

class ValenceSymbolCollectOperator : public SymbolCollectOperator {
  public:
    ValenceSymbolCollectOperator() = default;

    void collect(MCGAL::Halfedge* seed) override;
    void collect(std::vector<MCGAL::Halfedge*> seeds) override;

    int exportToBufferWithRangeCoder(char* buffer, bool enableQuantization = false) override;
    bool supportsRangeCoderExport() const override { return true; }
    int exportToBuffer(char* buffer, bool enableQuantization = false, bool enableCompress = false) override;

    // --- Parquet V2 访问器 ---
    const std::vector<std::deque<uint8_t>>& getValenceCodeQueues() const { return valenceCodeQueues_; }
    const std::vector<std::deque<uint8_t>>& getEdgeSymbolQueues() const { return edgeSymbolQueues_; }
    const std::vector<std::deque<MCGAL::Point>>& getPointQueues() const { return pointQueues_; }
    const std::vector<std::deque<MCGAL::PointInt>>& getIpointQueues() const { return ipointQueues_; }
    const std::vector<std::deque<uint8_t>>& getVertexSymbolQueues() const { return vertexSymbolQueues_; }
    const std::vector<std::deque<MCGAL::Point>>& getVertexPointQueues() const { return vertexPointQueues_; }
    const std::vector<pmsf::RoundSymbols>* rounds() const override { return &rounds_; }
    const pmsf::PMSFSymbolTable* pmsfSymbolTable() const override;

  public:
    /// @brief 把 valence 整数映射到 ValenceCode。对外公开方便 ReadOperator 反向用。
    static ValenceCode encodePolyValence(int v);
    /// @brief 码字 → valence 数值；POLY_V_OTHER 返回 0（由解码端 fallback 处理）。
    static int decodePolyValence(ValenceCode c);

  private:
    void collectOneRound(MCGAL::Halfedge* seed,
                        std::deque<uint8_t>& valenceCode,
                        std::deque<uint8_t>& edgeSym,
                        std::deque<MCGAL::Point>& points,
                        std::deque<MCGAL::PointInt>& ipoints,
                        std::deque<uint8_t>& vertexSym,
                        std::deque<MCGAL::Point>& vertexPoints);
    void appendRoundMirror(const std::deque<uint8_t>& valenceCode,
                           const std::deque<uint8_t>& edgeSym,
                           const std::deque<MCGAL::Point>& points,
                           const std::deque<MCGAL::PointInt>& ipoints,
                           const std::deque<uint8_t>& vertexSym,
                           const std::deque<MCGAL::Point>& vertexPoints);

  private:
    std::vector<std::deque<uint8_t>> valenceCodeQueues_;
    std::vector<std::deque<uint8_t>> edgeSymbolQueues_;
    std::vector<std::deque<MCGAL::Point>> pointQueues_;
    std::vector<std::deque<MCGAL::PointInt>> ipointQueues_;
    std::vector<std::deque<uint8_t>> vertexSymbolQueues_;
    std::vector<std::deque<MCGAL::Point>> vertexPointQueues_;
    std::vector<pmsf::RoundSymbols> rounds_;
};

#endif
