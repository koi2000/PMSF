/*
 * @file ConquestSymbolReadOperator.h
 * @brief valence-conquest 的**符号读取器**（解码端，薄）。
 *
 * 职责只有一个：把 `exportToBufferWithRangeCoder` 写出的字节流按
 * `pmsf::conquest_blob` 布局读回内存（编码端与解码端**同一个头文件**，
 * 见 `src/conquest/ConquestBlobFormat.h`），供 `ConquestReconstructOperator`
 * 逐层取走。
 *
 * **不做的事**（刻意）：
 *   - 不解码几何、不动网格 —— 那是 `ConquestReconstructOperator` +
 *     `pmsf::decodeLayer` 的事（那是本次唯一被逐层验证过的解码实体）；
 *   - 不读 `nbQuantBits` 参数 —— 量化网格**随流走**（蓝图 §54 的教训：
 *     量化器的定义经有损中间表示会伪装成"被测对象错了"）。
 */
#ifndef PMSF_CONQUEST_DECOMPRESS_CONQUEST_SYMBOL_READ_OPERATOR_H
#define PMSF_CONQUEST_DECOMPRESS_CONQUEST_SYMBOL_READ_OPERATOR_H

#include "conquest/ConquestBlobFormat.h"
#include "decompress/SymbolReadOperator.h"

#include <cstddef>
#include <vector>

namespace pmsf {

class ConquestSymbolReadOperator : public SymbolReadOperator {
  public:
    ConquestSymbolReadOperator() = default;
    ~ConquestSymbolReadOperator() = default;

    /// @brief 告诉 reader 缓冲区**真实长度**。
    ///
    /// ⚠️ **必须调用**：框架的 `importFromRangeCodedBuffer(char*, size_t&, unsigned)`
    /// **不传长度**，若 reader 拿猜的上界去解析就会**越界读**（实测：用 BUFFER_SIZE
    /// 当上界、真实 blob 只有 4136 B，解析器一路读进未映射页 ⇒ `vector<char>` 范围
    /// 拷贝段错，崩溃点离本 codec 很远）。调用方在调用导入前设一次。
    void setBufferSize(std::size_t n) {
        bufferSize_ = n;
    }
    std::size_t bufferSize() const { return bufferSize_; }

    /// @brief 主入口：把整流读进内存。失败时 `decodedOk()` 为 false。
    void importFromRangeCodedBuffer(char* buffer, size_t& dataOffset, unsigned nbQuantBits) override;

    // 框架的两个 `collect` 是**纯虚**，必须实现。本 layout 走"队列已装填"那一支
    // （`paperBlobLayoutUsesDecodedQueues(15) == true`），逐轮真正的动作在
    // `collectFromDecoded` + 重建器里，所以这两个只做防御性返回。
    void collect(std::vector<MCGAL::Halfedge*> seeds,
                 char* buffer,
                 size_t& dataOffset,
                 std::vector<MCGAL::Vertex*>& vertices,
                 std::vector<MCGAL::Halfedge*>& halfedge,
                 std::vector<MCGAL::Facet*>& facets) override;
    void collect(MCGAL::Halfedge* seed,
                 char* buffer,
                 size_t& dataOffset,
                 std::vector<MCGAL::Vertex*>& vertices,
                 std::vector<MCGAL::Halfedge*>& halfedge,
                 std::vector<MCGAL::Facet*>& facets) override;

    /// @brief 每轮被解序列化器调用一次。本类只记下"这一轮开始了"，
    /// 真正的解码由重建器在紧接着的 `reconstruct` 里做（顺序由框架保证）。
    void collectFromDecoded(MCGAL::Halfedge* seed,
                            std::vector<MCGAL::Vertex*>& vertices,
                            std::vector<MCGAL::Halfedge*>& halfedge,
                            std::vector<MCGAL::Facet*>& facets) override;

    // ---- 供重建器读 --------------------------------------------------------
    bool decodedOk() const {
        return decodedOk_;
    }
    const conquest_blob::Stream& stream() const {
        return stream_;
    }
    /// @brief 取走下一层的记录并推进游标；越界返回 nullptr。
    /// **由重建器在 `reconstruct` 里调用**（框架保证 `collectFromDecoded`
    /// 先于 `reconstruct`，所以"每轮取一层"与轮号一一对应）。
    const conquest_blob::LayerRecord* takeNextLayer();
    std::size_t layerCursor() const {
        return layerIndex_;
    }

  protected:
    // Alternative framework transports may install validated logical layers;
    // traversal and mesh reconstruction still use this same reader cursor.
    void installDecodedStream(conquest_blob::Stream stream);

  private:
    conquest_blob::Stream stream_;
    std::size_t layerIndex_ = 0;
    std::size_t bufferSize_ = 0;
    bool decodedOk_ = false;
};

}  // namespace pmsf

#endif  // PMSF_CONQUEST_DECOMPRESS_CONQUEST_SYMBOL_READ_OPERATOR_H
