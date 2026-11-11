/*
 * @Author: koi
 * @Date: 2025-09-01 10:04:28
 * @Description:
 */
#ifndef PMSF_HEADER_H
#define PMSF_HEADER_H

#include "BufferUtils.h"

class PMSFHeader {
  protected:
    int round = 0;
    int groupNumber = 0;
    bool enableCompress = false;
    bool enableQuantization = false;
    bool enableSegmentation = false;
    bool enableAdjustment = false;
    bool enablePrediction = false;
    /// @brief 是否对 simplify 后的 polygon face 做 fan-triangulation。
    /// @details 默认 true（历史行为）：VertexRemovalEliminateOperator::triangulate
    ///          会把每个 N-gon 切成 N-2 个三角形并给对角线打 isAdded() 标记。
    ///          false 时跳过三角化 —— 与 PPMC 一致，mesh 中间态可能含 N-gon；
    ///          配合 OptimizedSymbolCollectOperator 的 "second-pop + skip both
    ///          unsplittable" 规则可降低 connectivity BPV。
    ///          **二进制序列化时用反向位 "triangulationDisabled" 存储**，
    ///          保证老 .pmsf 文件（bit=0）解码到 enableTriangulation=true。
    bool enableTriangulation = true;

    /// @brief PPMC 论文 §3.3 face connectivity prediction：用面积均值预测 facet sym。
    /// @details 默认 false（历史行为）。=true 时 PPMCSymbolCollect/Read 走面积比较
    ///          路径，命中预测的 face 用极低概率码字（BPV 显著下降）。
    ///          **要求 encoder/decoder 浮点累加完全对称**——只在 PPMC reader/reconstructor
    ///          路径下消费此 bit；OptimizedSymbol* 路径忽略。
    ///          **序列化反向存** `connectivityPredictionFacesDisabled`，老 .pmsf 默认 false。
    bool enableConnectivityPredictionFaces = false;

    /// @brief PPMC 论文 §3.3 edge connectivity prediction：用边长均值预测 edge sym。
    /// @details 默认 false（历史行为）。条件同上：仅在 PPMC reader/reconstructor 路径下消费。
    ///          **序列化反向存** `connectivityPredictionEdgesDisabled`。
    bool enableConnectivityPredictionEdges = false;

    /// @brief PPMC 论文 §3.3 triangle-mesh balance 预测（仅纯三角网格生效）。
    /// @details 默认 false。=true 时 PPMCSymbolCollect/Read 在面预测分支额外走 1-ring
    ///          facet sym 平衡度判据。非纯三角输入会被 PMSF 入口处的 isPureTriangle()
    ///          guard 自动关闭。**序列化反向存** `triangleMeshBalanceDisabled`。
    bool enableTriangleMeshBalance = false;

    /// @brief PPMC 论文 §3.1 local curvature prediction（Laplacian 残差修正）。
    /// @details 默认 false（历史行为，bit 9）；老 .pmsf/头部（bit 9=0）解码得到 false。
    ///          仅在 PPMC reader/reconstructor 路径下消费，encoder/decoder 必须同时开启。
    bool enableCurvaturePrediction = false;

  public:
    PMSFHeader() = default;

    // Builder模式
    class Builder {
      private:
        int _round = 0;
        int _groupNumber = 0;
        bool _enableCompress = false;
        bool _enableQuantization = false;
        bool _enableSegmentation = false;
        bool _enableAdjustment = false;
        bool _enablePrediction = false;
        bool _enableTriangulation = true;  // 默认 = 历史行为
        bool _enableConnectivityPredictionFaces = false;
        bool _enableConnectivityPredictionEdges = false;
        bool _enableTriangleMeshBalance = false;
        bool _enableCurvaturePrediction = false;

      public:
        Builder& setRound(int r) {
            _round = r;
            return *this;
        }
        Builder& setGroupNumber(int groupNumber) {
            _groupNumber = groupNumber;
            return *this;
        }
        Builder& setEnableCompress(bool v) {
            _enableCompress = v;
            return *this;
        }
        Builder& setEnableQuantization(bool v) {
            _enableQuantization = v;
            return *this;
        }
        Builder& setEnableSegmentation(bool v) {
            _enableSegmentation = v;
            return *this;
        }
        Builder& setEnableAdjustMent(bool v) {
            _enableAdjustment = v;
            return *this;
        }
        Builder& setEnablePrediction(bool v) {
            _enablePrediction = v;
            return *this;
        }
        Builder& setEnableTriangulation(bool v) {
            _enableTriangulation = v;
            return *this;
        }
        Builder& setEnableConnectivityPredictionFaces(bool v) {
            _enableConnectivityPredictionFaces = v;
            return *this;
        }
        Builder& setEnableConnectivityPredictionEdges(bool v) {
            _enableConnectivityPredictionEdges = v;
            return *this;
        }
        Builder& setEnableTriangleMeshBalance(bool v) {
            _enableTriangleMeshBalance = v;
            return *this;
        }
        Builder& setEnableCurvaturePrediction(bool v) {
            _enableCurvaturePrediction = v;
            return *this;
        }
        PMSFHeader build() {
            PMSFHeader header;
            header.round = _round;
            header.groupNumber = _groupNumber;
            header.enableCompress = _enableCompress;
            header.enableQuantization = _enableQuantization;
            header.enableSegmentation = _enableSegmentation;
            header.enableAdjustment = _enableAdjustment;
            header.enablePrediction = _enablePrediction;
            header.enableTriangulation = _enableTriangulation;
            header.enableConnectivityPredictionFaces = _enableConnectivityPredictionFaces;
            header.enableConnectivityPredictionEdges = _enableConnectivityPredictionEdges;
            header.enableTriangleMeshBalance = _enableTriangleMeshBalance;
            header.enableCurvaturePrediction = _enableCurvaturePrediction;
            return header;
        }
    };

    // getter
    int getRound() const {
        return round;
    }
    int getGroupNumber() const {
        return groupNumber;
    }
    bool isEnableCompress() const {
        return enableCompress;
    }
    bool isEnableQuantization() const {
        return enableQuantization;
    }
    bool isEnableSegmentation() const {
        return enableSegmentation;
    }
    bool isEnableAdjustment() const {
        return enableAdjustment;
    }
    bool isEnablePrediction() const {
        return enablePrediction;
    }
    bool isEnableTriangulation() const {
        return enableTriangulation;
    }
    bool isEnableConnectivityPredictionFaces() const {
        return enableConnectivityPredictionFaces;
    }
    bool isEnableConnectivityPredictionEdges() const {
        return enableConnectivityPredictionEdges;
    }
    bool isEnableTriangleMeshBalance() const {
        return enableTriangleMeshBalance;
    }
    bool isEnableCurvaturePrediction() const {
        return enableCurvaturePrediction;
    }

    // setter
    void setRound(int r) {
        round = r;
    }
    void setGroupNumber(int g) {
        groupNumber = g;
    }
    void setEnableCompress(bool v) {
        enableCompress = v;
    }
    void setEnableQuantization(bool v) {
        enableQuantization = v;
    }
    void setEnableSegmentation(bool v) {
        enableSegmentation = v;
    }
    void setEnableAdjustment(bool v) {
        enableAdjustment = v;
    }
    void setEnablePrediction(bool v) {
        enablePrediction = v;
    }
    void setEnableTriangulation(bool v) {
        enableTriangulation = v;
    }
    void setEnableConnectivityPredictionFaces(bool v) {
        enableConnectivityPredictionFaces = v;
    }
    void setEnableConnectivityPredictionEdges(bool v) {
        enableConnectivityPredictionEdges = v;
    }
    void setEnableTriangleMeshBalance(bool v) {
        enableTriangleMeshBalance = v;
    }
    void setEnableCurvaturePrediction(bool v) {
        enableCurvaturePrediction = v;
    }

    // 完整的序列化方法
    virtual void serialize(char* buffer, size_t& offset) const {
        writeInt(buffer, offset, round);
        unsigned bitOffset = 0;
        writeBool(buffer, bitOffset, offset, enableCompress);
        writeBool(buffer, bitOffset, offset, enableQuantization);
        writeBool(buffer, bitOffset, offset, enableSegmentation);
        writeBool(buffer, bitOffset, offset, enableAdjustment);
        writeBool(buffer, bitOffset, offset, enablePrediction);
        // bit 5: triangulationDisabled = !enableTriangulation。反向位以
        // 保证老 .pmsf（5 位用完后剩下位为 0）解码得到 enableTriangulation=true。
        writeBool(buffer, bitOffset, offset, !enableTriangulation);
        // bits 6-8: 3 个 PPMC 风格预测开关，正向位（默认 false）。
        // 老 .pmsf（这些位 = 0）解码后 enableConnectivityPrediction*=false / enableTriangleMeshBalance=false。
        writeBool(buffer, bitOffset, offset, enableConnectivityPredictionFaces);
        writeBool(buffer, bitOffset, offset, enableConnectivityPredictionEdges);
        writeBool(buffer, bitOffset, offset, enableTriangleMeshBalance);
        // bit 9: PPMC local curvature prediction（正向位；老头部 bit=0 → false）。
        writeBool(buffer, bitOffset, offset, enableCurvaturePrediction);
        offset += 1;
        bitOffset = 0;
        if (enableSegmentation) {
            writeInt(buffer, offset, groupNumber);
        }
    }

    // 反序列化方法（可选）
    virtual void deserialize(char* buffer, size_t& offset) {
        round = readInt(buffer, offset);
        unsigned bitOffset = 0;
        enableCompress = readBool(buffer, bitOffset, offset);
        enableQuantization = readBool(buffer, bitOffset, offset);
        enableSegmentation = readBool(buffer, bitOffset, offset);
        enableAdjustment = readBool(buffer, bitOffset, offset);
        enablePrediction = readBool(buffer, bitOffset, offset);
        // bit 5 读反向位：老 .pmsf 中该位 = 0 → triangulationDisabled=false
        //                                     → enableTriangulation=true（默认历史行为）。
        bool triangulationDisabled = readBool(buffer, bitOffset, offset);
        enableTriangulation = !triangulationDisabled;
        // bits 6-8: 默认 false（老文件 = 0 → 默认行为）。
        enableConnectivityPredictionFaces = readBool(buffer, bitOffset, offset);
        enableConnectivityPredictionEdges = readBool(buffer, bitOffset, offset);
        enableTriangleMeshBalance = readBool(buffer, bitOffset, offset);
        enableCurvaturePrediction = readBool(buffer, bitOffset, offset);
        bitOffset = 0;
        offset += 1;
        if (enableSegmentation) {
            groupNumber = readInt(buffer, offset);
        }
    }
};

#endif