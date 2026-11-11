/*****************************************************************************
 * @Author: koi
 * @Description: 通用 range-coder 符号流编码辅助函数。
 * 阶段 G：把各算法 SymbolCollectOperator 中重复的 qsmodel 初始化、round 逆序遍历、
 *        2/N 元 adaptive 编码等逻辑抽象到框架，算法实现只需声明“哪些队列、alphabet 大小”。
 *****************************************************************************/
#ifndef PMSF_COMPRESS_RANGE_CODER_STREAM_ENCODER_H
#define PMSF_COMPRESS_RANGE_CODER_STREAM_ENCODER_H

#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"
#include "Point.h"

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <stdexcept>
#include <vector>

namespace pmsf {

/// @brief 封装 range-coder + qsmodel 的通用 2/N 元 adaptive 符号流编码。
///        默认参数与各论文 collector 的历史实现保持一致（scale=10, init=1<<9）。
class AdaptiveSymbolEncoder {
  public:
    /// @param alphabetSize 符号表大小（如 2 表示 bit，5 表示 valence 5 元码）。
    explicit AdaptiveSymbolEncoder(int alphabetSize,
                                   int scaleBits = 10,
                                   int initCount = 1 << 9);
    ~AdaptiveSymbolEncoder();

    AdaptiveSymbolEncoder(const AdaptiveSymbolEncoder&) = delete;
    AdaptiveSymbolEncoder& operator=(const AdaptiveSymbolEncoder&) = delete;

    AdaptiveSymbolEncoder(AdaptiveSymbolEncoder&& other) noexcept;
    AdaptiveSymbolEncoder& operator=(AdaptiveSymbolEncoder&& other) noexcept;

    /// @brief 编码一个符号。rangecoder 必须已通过 start_encoding 初始化。
    void encode(rangecoder* rc, int sym);

    int alphabetSize() const { return alphabetSize_; }
    int scaleBits() const { return scaleBits_; }

  private:
    qsmodel model_;
    int alphabetSize_;
    int scaleBits_;
};

/// @brief 管理单个 rangecoder 会话内的多个 qsmodel，适合需要交错编码的算法
///        （如 PPMC 的连接性/几何符号、RDPatch 的分层 NULL/valence/escape 流）。
///        每个 model 独立配置 alphabet、scale、rescale 与可选初始计数。
class MultiModelRangeEncoder {
  public:
    struct ModelSpec {
        int alphabetSize;   ///< 符号表大小
        int scaleBits;      ///< 总频率为 1 << scaleBits
        int rescale;        ///< qsmodel rescale 间隔
        const int* initCounts = nullptr;  ///< 初始频率数组（大小 alphabetSize），nullptr 表示均匀
    };

    /// @param out    输出缓冲区指针。
    /// @param offset 写入偏移指针；编码从 *offset 开始，finish 后 *offset 已更新。
    MultiModelRangeEncoder(char* out, size_t* offset);
    ~MultiModelRangeEncoder();

    MultiModelRangeEncoder(const MultiModelRangeEncoder&) = delete;
    MultiModelRangeEncoder& operator=(const MultiModelRangeEncoder&) = delete;

    /// @brief 注册一个新的 qsmodel，返回 model id（从 0 开始）。
    int addModel(const ModelSpec& spec);

    /// @brief 无模型编码一个原始 bit（scale=1）。
    void encodeRawBit(int bit);
    /// @brief 无模型编码一个原始 byte（scale=8）。
    void encodeRawByte(uint8_t v);
    /// @brief 无模型编码一个原始 short（scale=16）。
    void encodeRawShort(uint16_t v);
    /// @brief 无模型编码一个定长原始符号（syfreq=1，scale=bits）。
    void encodeRawSymbol(int sym, int bits);
    /// @brief 无模型编码一个符号，使用调用方提供的固定频率（用于静态模型）。
    void encodeRawSymbolWithFreq(int sym, int syfreq, int ltfreq, int scaleBits);

    /// @brief 使用指定 model 编码一个符号并更新模型。
    void encode(int modelId, int sym);

    /// @brief 结束编码，返回本次会话写入的字节数（不含起始 *offset）。
    size_t finish();

    /// @brief 当前已写入字节数（含 rangecoder 内部状态，未必最终）。
    size_t bytesWritten() const { return *offset_; }

    /// @brief 底层 rangecoder，仅在需要直接调用底层 API 时使用。
    rangecoder* rc() { return &rc_; }

  private:
    rangecoder rc_;
    size_t* offset_;
    std::vector<qsmodel> models_;
    std::vector<ModelSpec> specs_;
};

/// @brief 把多轮 uint8 符号队列编码到一个 range-coder 输出缓冲区。
/// @param queues        per-round 符号队列（每个 round 一个 deque<uint8_t>）。
/// @param alphabetSize  符号表大小。
/// @param reverseRounds true 表示从 queues 末尾轮到第 0 轮编码（与 decoder 重建顺序对齐）。
/// @param out           输出缓冲区。
/// @param maxSize       输出缓冲区最大字节数。
/// @return 实际写入字节数。
/// @throws std::runtime_error 若符号值超出 alphabetSize 或缓冲区溢出。
size_t encodeUint8Queues(const std::vector<std::deque<uint8_t>>& queues,
                         int alphabetSize,
                         bool reverseRounds,
                         char* out,
                         size_t maxSize);

/// @brief 对单个 uint8 队列编码（无 round 维度）。返回实际写入字节数。
size_t encodeUint8Queue(const std::deque<uint8_t>& queue,
                        int alphabetSize,
                        char* out,
                        size_t maxSize);

/// @brief 对多个 uint8 队列分别编码，每个队列使用独立的 encoder；返回每段编码后字节。
///        用于 V_OTHER escape：valence 码与 escape 字节可共用同一 rangecoder，但用两个 model。
struct MultiModelSegment {
    const std::vector<std::deque<uint8_t>>* queues = nullptr;
    int alphabetSize = 0;
};
size_t encodeMultiModelSegments(const std::vector<MultiModelSegment>& segments,
                                bool reverseRounds,
                                char* out,
                                size_t maxSize);

/// @brief 编码 5 元 valence 流（含 V_OTHER inline 256 元 escape）。
///        同一 rangecoder 内交替使用 5 元 model 与 256 元 escape model。
size_t encodeValence5WithEscape(
    const std::vector<std::deque<uint8_t>>& valenceQueues,
    const std::vector<std::deque<uint8_t>>& escapeQueues,
    bool reverseRounds,
    char* out,
    size_t maxSize);

/// @brief 上下文 adaptive 符号流编码：每个 context 一个独立 qsmodel，可选 inline escape。
/// @param symbols        待编码符号队列。
/// @param alphabetSize   主 model 符号表大小。
/// @param numContexts    context 数量。
/// @param deriveCtx      根据**上一个已编码符号**推导当前 context 的回调；
///                       首次调用时 prevSymbol = -1。
/// @param escapeSymbols  可选的 inline escape 字节队列；当 symbols[i] == escapeSymbol 时
///                       从该队列消费一个 escape 字节并用 escape model 编码。可传 nullptr。
/// @param escapeAlphabetSize escape model 的 alphabet 大小；escapeSymbols 为 nullptr 时忽略。
/// @param escapeSymbol   触发 escape 的主 model 符号值。
/// @return 实际写入字节数。
size_t encodeContextAdaptiveUint8Queue(
    const std::deque<uint8_t>& symbols,
    int alphabetSize,
    int numContexts,
    const std::function<int(int prevSymbol)>& deriveCtx,
    const std::deque<uint8_t>* escapeSymbols,
    int escapeAlphabetSize,
    int escapeSymbol,
    char* out,
    size_t maxSize);

/// @brief 把每轮一个 PointInt 残差队列按 bit-plane 编码。
/// @param queues   per-round 残差队列。
/// @param bpBits   bit-plane 位数（通常 = nbQuantBits + 1）。
/// @param reverseRounds 同 encodeUint8Queues。
/// @param out      输出缓冲区。
/// @param maxSize  输出缓冲区最大字节数。
/// @return 实际写入字节数。
size_t encodeResidualBitPlaneQueues(
    const std::vector<std::deque<MCGAL::PointInt>>& queues,
    int bpBits,
    bool reverseRounds,
    char* out,
    size_t maxSize);

/// @brief 对单个 PointInt 残差队列按 bit-plane 编码。
size_t encodeResidualBitPlaneQueue(const std::deque<MCGAL::PointInt>& queue,
                                   int bpBits,
                                   char* out,
                                   size_t maxSize);

/// @brief 按分量独立模型的 residual bit-plane 编码（每个 component × bit 一个 model）。
///        编码顺序：每个 residual 的 x、y、z 依次从高位到低位编码。
size_t encodeResidualBitPlaneQueuePerComponent(const std::deque<MCGAL::PointInt>& queue,
                                               int bpBits,
                                               char* out,
                                               size_t maxSize);

/// @brief 自适应位宽、按顶点顺序扫描的分量 bit-plane 编码（RDPatch sv=6）。
/// @param minComponentWidth 每个分量最少编码位数（MMSE 使用 1，RDPatch 使用 0）。
size_t encodeResidualBitPlaneQueueAdaptiveVertexMajor(const std::vector<int>& rx,
                                                      const std::vector<int>& ry,
                                                      const std::vector<int>& rz,
                                                      char* out,
                                                      size_t maxSize,
                                                      int minComponentWidth = 0);

/// @brief 自适应位宽、按 component -> bitplane -> vertex 顺序扫描的编码（RDPatch sv=7）。
/// @param minComponentWidth 每个分量最少编码位数（MMSE 使用 1，RDPatch 使用 0）。
size_t encodeResidualBitPlaneQueueAdaptivePlaneMajor(const std::vector<int>& rx,
                                                     const std::vector<int>& ry,
                                                     const std::vector<int>& rz,
                                                     char* out,
                                                     size_t maxSize,
                                                     bool biasedFirstOneInit = false,
                                                     int minComponentWidth = 0);

/// SAME stream format as encodeResidualBitPlaneQueueAdaptivePlaneMajor, but the
/// bit planes are visited in planeOrder (plane id = axis*32 + bit). Used by the
/// RD paper stage 4 R-D-optimized bitplane order prototype; the order list is
/// written by the caller into its stream header.
size_t encodeResidualBitPlaneQueueAdaptivePlaneMajorOrdered(
    const std::vector<int>& rx,
    const std::vector<int>& ry,
    const std::vector<int>& rz,
    char* out,
    size_t maxSize,
    const std::vector<int>& planeOrder,
    bool biasedFirstOneInit = false,
    int minComponentWidth = 0);

}  // namespace pmsf

#endif  // PMSF_COMPRESS_RANGE_CODER_STREAM_ENCODER_H
