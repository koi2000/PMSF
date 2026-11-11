/*
 * @Author: koi
 * @Date: 2025-09-01 09:37:03
 * @Description:
 */
#ifndef SYMBOLREADOPERATOR_H
#define SYMBOLREADOPERATOR_H

#include "core/Symbol.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Point.h"
#include "header/PMSFHeader.h"
#include <vector>

/**
 * @brief 符号收集操作符
 * @details 需要对数据结构和函数进行抽象，
 */
class SymbolReadOperator {
  public:
    SymbolReadOperator() = default;

    /// @brief 解码端「本轮该面可分裂」的置位，所有 SymbolReadOperator 共用。
    ///
    /// 为什么不能直接 `f->setSplittable()`：
    ///
    ///   - `MCGAL::Facet::isSplittable()` 把 `removedFlag_ == Removed` 也算作
    ///     可分裂（见 deps/MCGAL/include/Facet.h），编码端正是据此为**已移除的面**
    ///     写出 symbol=1 的；
    ///   - 而 `Mesh::resetState()` 会跳过已移除的面，它们的 `splittableFlag_`
    ///     跨轮残留 —— 于是下一轮 BFS 再次走到这个面时，
    ///     `assert(splittableFlag_ == Unknown)` 直接中断。
    ///     实测样本：某算法二进制解码第 4 轮（facet removedFlag=1,
    ///     splittableFlag=1）。
    ///
    /// 先判再置与直接置位在语义上完全等价（`isSplittable()` 本来就为真），
    /// 因此解码输出逐字节不变，只是不再撞断言。
    static void markFacetSplittableThisRound(MCGAL::Facet* f) {
        if (f->isUnsplittable()) {
            f->resetSplittableFlag();
        }
        if (!f->isSplittable()) {
            f->setSplittable();
        }
    }

    virtual void init(std::vector<size_t> offsets, PMSFHeader header) {
        this->offsets = offsets;
        this->header_ = header;
    }

    virtual void setPMSFSymbolContext(const pmsf::PMSFSymbolTable* symbols,
                                       const pmsf::PMSFSymbolPayloadTable* payloads) {
        pmsfSymbolTable_ = symbols;
        paperSymbolPayloadTable_ = payloads;
    }

    const pmsf::PMSFSymbolTable* pmsfSymbolTable() const {
        return pmsfSymbolTable_;
    }

    const pmsf::PMSFSymbolPayloadTable* paperSymbolPayloadTable() const {
        return paperSymbolPayloadTable_;
    }

    virtual void collect(std::vector<MCGAL::Halfedge*> seeds,
                         char* buffer,
                         size_t& dataOffset,
                         std::vector<MCGAL::Vertex*>& vertices,
                         std::vector<MCGAL::Halfedge*>& halfedge,
                         std::vector<MCGAL::Facet*>& facets) = 0;
    // 收集一轮的信息
    virtual void collect(MCGAL::Halfedge* seed,
                         char* buffer,
                         size_t& dataOffset,
                         std::vector<MCGAL::Vertex*>& vertices,
                         std::vector<MCGAL::Halfedge*>& halfedge,
                         std::vector<MCGAL::Facet*>& facets) = 0;
    /// @brief 从"range coder 编码后的"整块符号+几何缓冲区一次性解码所有 round 的符号/残差
    /// @details 默认空实现；OptimizedSymbolReadOperator 覆盖。
    virtual void importFromRangeCodedBuffer(char* buffer, size_t& dataOffset, unsigned nbQuantBits) {
        (void)buffer; (void)dataOffset; (void)nbQuantBits;
    }

    /// @brief 从 Parquet 解析的结构化残差数据导入内部队列（RD 预测路径）
    /// @details 默认空实现；OptimizedSymbolReadOperator 覆盖。
    virtual void importResidualsFromParquet(const std::vector<MCGAL::PointInt>& residuals) {
        (void)residuals;
    }

    /// @brief 从 Parquet 解析的结构化符号数据导入内部队列（多表方案）
    /// @param facetSymbols 面片符号（BFS 顺序）
    /// @param edgeSymbols 边符号（BFS 顺序）
    /// @param vertexSymbols 顶点符号（BFS 顺序，可为空）
    /// @param points 插入点坐标
    virtual void importFromParquet(const std::vector<bool>& facetSymbols,
                                   const std::vector<bool>& edgeSymbols,
                                   const std::vector<bool>& vertexSymbols,
                                   const std::vector<MCGAL::Point>& points) {
        (void)facetSymbols; (void)edgeSymbols; (void)vertexSymbols; (void)points;
    }

    /// @brief 扩展版 Parquet 导入：支持“面片点流 + 顶点点流”双流输入
    /// @details 默认实现会把两路点流拼接后回退到旧接口，保证向后兼容
    virtual void importFromParquetExtended(const std::vector<bool>& facetSymbols,
                                           const std::vector<bool>& edgeSymbols,
                                           const std::vector<bool>& vertexSymbols,
                                           const std::vector<MCGAL::Point>& points,
                                           const std::vector<MCGAL::Point>& vertexPoints) {
        if (vertexPoints.empty()) {
            importFromParquet(facetSymbols, edgeSymbols, vertexSymbols, points);
            return;
        }
        std::vector<MCGAL::Point> merged = points;
        merged.insert(merged.end(), vertexPoints.begin(), vertexPoints.end());
        importFromParquet(facetSymbols, edgeSymbols, vertexSymbols, merged);
    }

    /// @brief 从已导入的内部队列中按 BFS 顺序消费符号并重建（多表方案）
    virtual void collectFromDecoded(MCGAL::Halfedge* seed,
                                    std::vector<MCGAL::Vertex*>& vertices,
                                    std::vector<MCGAL::Halfedge*>& halfedge,
                                    std::vector<MCGAL::Facet*>& facets) {
        (void)seed; (void)vertices; (void)halfedge; (void)facets;
    }

    void enableQuantization(MCGAL::Point bboxMin, float f_quantStep) {
        enableQuantization_ = true;
        bboxMin_ = bboxMin;
        f_quantStep_ = f_quantStep;
    }

    // codec_version 来自 Columnar metadata 表。ColumnarDeserializeOperator 在
    // 调度解码前调用 setCodecVersion 把值透传给 reader。非 Parquet 路径默认 0
    // （Binary 没有版本字段），reader 的 Binary 入口对 0 的处理与最新版一致 ——
    // 压缩侧 OptimizedSymbolCollectOperator::collect 已经只产出 per-edge 流，
    // Binary 文件因此与 codec_version>=3 语义绑定，重新跑 bench 即可。
    void setCodecVersion(int v) { codecVersion_ = v; }
    int getCodecVersion() const { return codecVersion_; }
    /**
     * Quantize a position
     */
    MCGAL::PointInt getQuantizedPos(MCGAL::Point p) const {
        return MCGAL::PointInt((p.x() - bboxMin_.x()) / (f_quantStep_ * (1 << i_curQuantizationId)),
                        (p.y() - bboxMin_.y()) / (f_quantStep_ * (1 << i_curQuantizationId)),
                        (p.z() - bboxMin_.z()) / (f_quantStep_ * (1 << i_curQuantizationId)));
    }

    bool enableQuantization_ = false;
    int i_curQuantizationId = 0;
    MCGAL::Point bboxMin_;
    float f_quantStep_;
    int groupId = -1;
    std::vector<size_t> offsets;
    PMSFHeader header_;
    const pmsf::PMSFSymbolTable* pmsfSymbolTable_ = nullptr;
    const pmsf::PMSFSymbolPayloadTable* paperSymbolPayloadTable_ = nullptr;
    // 0 = 未设置（走 Binary 路径默认最新语义），>=3 = per-edge（Default/Optimized 新方案）
    int codecVersion_ = 0;
};
#endif
