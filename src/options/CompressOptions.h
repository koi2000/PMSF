/*
 * @Author: koi
 * @Date: 2025-08-27 21:18:59
 * @Description: 存储用于压缩所使用的各种option
 */

#ifndef COMPRESSION_OPTIONS_H
#define COMPRESSION_OPTIONS_H

#include "../elimination/EliminateOperator.h"
#include "../segmentation/SegmentationOption.h"
#include "../selection/SelectOperator.h"
#include "../preprocess/PreProcessOperator.h"
#include "../postprocess/PostProcessOperator.h"
#include <memory>
#include <string>

class CompressOptions {
  public:
    CompressOptions() = default;
    ~CompressOptions() = default;

    class Builder {
      public:
        Builder()
            : round_(0),
              enablePrediction_(false),
              enableQuantization_(false),
              enableAdjustment_(false) {}

        Builder& setRound(const int r) {
            round_ = r;
            return *this;
        }

        Builder& setPath(const std::string& p) {
            path_ = p;
            return *this;
        }
        Builder& setOutputPath(const std::string& p) {
            outputPath_ = p;
            return *this;
        }
        Builder& setSelect(const std::shared_ptr<SelectOperator>& s) {
            select_ = s;
            return *this;
        }
        Builder& setEliminate(const std::shared_ptr<EliminateOperator>& e) {
            eliminate_ = e;
            return *this;
        }
        Builder& setPreProcessOperator(const std::shared_ptr<PreProcessOperator>& p) {
            preProcessOperator_ = p;
            return *this;
        }
        Builder& setPostProcessOperator(const std::shared_ptr<PostProcessOperator>& p) {
            postProcessOperator_ = p;
            return *this;
        }
        Builder& setEnableCompress(bool enable) {
            enableCompress_ = enable;
            return *this;
        }
        Builder& setEnablePrediction(bool enable) {
            enablePrediction_ = enable;
            return *this;
        }
        Builder& setEnableQuantization(bool enable) {
            enableQuantization_ = enable;
            return *this;
        }
        Builder& setEnableSegmentation(bool enable) {
            enableSegmentation_ = enable;
            return *this;
        }
        Builder& setEnableAdjustment(bool enable) {
            enableAdjustment_ = enable;
            return *this;
        }
        /// @brief 是否对 simplify 后的 polygon face 做 fan-triangulation。
        ///        默认 true（历史行为）；false 时 VertexRemovalEliminateOperator
        ///        跳过 triangulate()，配合 OptimizedSymbolCollectOperator 的
        ///        "second-pop + skip both unsplittable" 规则降低 connectivity BPV。
        Builder& setEnableTriangulation(bool enable) {
            enableTriangulation_ = enable;
            return *this;
        }
        /// @brief PPMC 论文 §3.3 face connectivity prediction（默认 false）。
        ///        =true 时 PPMC reader/reconstructor 路径用面积均值预测 facet sym。
        ///        非 PPMC reader 路径忽略此 flag。
        Builder& setEnableConnectivityPredictionFaces(bool enable) {
            enableConnectivityPredictionFaces_ = enable;
            return *this;
        }
        /// @brief PPMC 论文 §3.3 edge connectivity prediction（默认 false）。
        Builder& setEnableConnectivityPredictionEdges(bool enable) {
            enableConnectivityPredictionEdges_ = enable;
            return *this;
        }
        /// @brief PPMC 论文 §3.3 triangle-mesh balance prediction（默认 false）。
        ///        仅纯三角网格生效；非纯三角输入会被 PMSF 入口的 isPureTriangle() guard 关掉。
        Builder& setEnableTriangleMeshBalance(bool enable) {
            enableTriangleMeshBalance_ = enable;
            return *this;
        }
        Builder& setEnableEdgeFlipPostProcess(bool enable) {
            enableEdgeFlipPostProcess_ = enable;
            return *this;
        }
        Builder& setEdgeFlipPostProcessRounds(int rounds) {
            edgeFlipPostProcessRounds_ = rounds > 0 ? rounds : 1;
            return *this;
        }
        Builder& setEdgeFlipPostProcessLongEdgeRatio(double ratio) {
            if (ratio > 0.0) edgeFlipPostProcessLongEdgeRatio_ = ratio;
            return *this;
        }
        Builder& setEdgeFlipPostProcessMinImprovementRatio(double ratio) {
            if (ratio > 0.0) edgeFlipPostProcessMinImprovementRatio_ = ratio;
            return *this;
        }
        Builder& setSegmentationOption(const SegmentationOption& seg) {
            segmentation_ = seg;
            return *this;
        }
        CompressOptions build() {
            CompressOptions options;
            options.round = round_;
            options.path = path_;
            options.outputPath = outputPath_;
            options.select = select_;
            options.eliminate = eliminate_;
            options.enableCompress = enableCompress_;
            options.enablePrediction = enablePrediction_;
            options.enableQuantization = enableQuantization_;
            options.enableSegmentation = enableSegmentation_;
            options.enableAdjustment = enableAdjustment_;
            options.enableTriangulation = enableTriangulation_;
            options.enableConnectivityPredictionFaces = enableConnectivityPredictionFaces_;
            options.enableConnectivityPredictionEdges = enableConnectivityPredictionEdges_;
            options.enableTriangleMeshBalance = enableTriangleMeshBalance_;
            options.enableEdgeFlipPostProcess = enableEdgeFlipPostProcess_;
            options.edgeFlipPostProcessRounds = edgeFlipPostProcessRounds_;
            options.edgeFlipPostProcessLongEdgeRatio = edgeFlipPostProcessLongEdgeRatio_;
            options.edgeFlipPostProcessMinImprovementRatio = edgeFlipPostProcessMinImprovementRatio_;
            options.segmentation = segmentation_;
            options.preProcessOperator = preProcessOperator_;
            options.postProcessOperator = postProcessOperator_;
            return options;
        }

      private:
        int round_;
        std::string path_;
        std::string outputPath_;
        bool enableCompress_ = false;
        bool enablePrediction_ = false;
        bool enableQuantization_ = false;
        bool enableSegmentation_ = false;
        bool enableAdjustment_ = false;
        bool enableTriangulation_ = true;  // 默认 = 历史行为
        bool enableConnectivityPredictionFaces_ = false;
        bool enableConnectivityPredictionEdges_ = false;
        bool enableTriangleMeshBalance_ = false;
        bool enableEdgeFlipPostProcess_ = false;
        int edgeFlipPostProcessRounds_ = 1;
        double edgeFlipPostProcessLongEdgeRatio_ = 1.35;
        double edgeFlipPostProcessMinImprovementRatio_ = 0.90;
        SegmentationOption segmentation_;
        std::shared_ptr<SelectOperator> select_;
        std::shared_ptr<EliminateOperator> eliminate_;
        std::shared_ptr<PreProcessOperator> preProcessOperator_;
        std::shared_ptr<PostProcessOperator> postProcessOperator_;
    };

    const std::string& getPath() const {
        return path;
    }

    const std::string& getOutputPath() const {
        return outputPath;
    }
    const int getRound() const {
        return round;
    }
    void setRound(const int r) {
        round = r;
    }
    const std::shared_ptr<SelectOperator>& getSelect() const {
        return select;
    }
    const std::shared_ptr<EliminateOperator>& getEliminate() const {
        return eliminate;
    }
    // Phase 5 PR-E：允许 baseline impl main 在构造 options 之后、run 之前
    // 覆盖 eliminate（例如 PMSF_PCATM_USE_RETRI_ZCOLOR=1 时从 PCATMEliminate 切到
    // PCATMZColorEliminate 以保证量化整数 barycenter 的 bit-exact 对齐）。
    // 默认 baseline 路径根本不调用这个 setter，字节级零回归。
    void setEliminate(const std::shared_ptr<EliminateOperator>& e) {
        eliminate = e;
    }
    // PHASE59 物品 3 前置：两遍式分区压缩要给**勘察遍**换一个独立的产物基名。
    // 勘察遍的产物是废件（它只为算出「顶点序号 → 分区」表而跑），与分区遍共用
    // 基名时会留下陈旧的 `_bit` / `_lod` 表，把解码端的 blob 重组污染成
    // pass1/pass2 混合体（见 removePaperArtifactsForBase 的注释）。
    // 只有分区分支（segments > 1）调用它 ⇒ 默认路径零变化。
    void setOutputPath(const std::string& p) {
        outputPath = p;
    }
    // Phase 5 PR-E：QEM 借 RD_TAG 协议时，必须关掉 adjustment（QEM
    // VertexRemovalEliminate 在 adjust=true 时把 vertex.position 改为 quadric
    // optimal placement，与 RDTag decoder 的 barycenter 还原假设不符 → round
    // 3 join_face assert）。env 默认 off 时 baseline 不调用 → 零回归。
    void setEnableAdjustment(bool enable) {
        enableAdjustment = enable;
    }
    const std::shared_ptr<PreProcessOperator>& getPreProcessOperator() const {
        return preProcessOperator;
    }
    const std::shared_ptr<PostProcessOperator>& getPostProcessOperator() const {
        return postProcessOperator;
    }
    bool isEnableCompress() const {
        return enableCompress;
    }
    bool isEnablePrediction() const {
        return enablePrediction;
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
    /// @brief 是否对 simplify 后的 polygon face 做 fan-triangulation（默认 true）。
    bool isEnableTriangulation() const {
        return enableTriangulation;
    }
    /// @brief PPMC §3.3 face connectivity prediction（默认 false）。
    bool isEnableConnectivityPredictionFaces() const {
        return enableConnectivityPredictionFaces;
    }
    /// @brief PPMC §3.3 edge connectivity prediction（默认 false）。
    bool isEnableConnectivityPredictionEdges() const {
        return enableConnectivityPredictionEdges;
    }
    /// @brief PPMC §3.3 triangle-mesh balance prediction（默认 false，仅纯三角网格）。
    bool isEnableTriangleMeshBalance() const {
        return enableTriangleMeshBalance;
    }
    // ---- Phase 1：契约侧（CompressOptionsTuner）需要的 setter ----
    // 这些开关原先只能经 Builder 在构造时设置，算法迁入 ISimplificationAlgorithm
    // 后需要在 CompressOptions 已构造好之后再调整，因此补上同名 setter。
    void setEnableTriangulation(bool enable) {
        enableTriangulation = enable;
    }
    void setEnableConnectivityPredictionFaces(bool enable) {
        enableConnectivityPredictionFaces = enable;
    }
    void setEnableConnectivityPredictionEdges(bool enable) {
        enableConnectivityPredictionEdges = enable;
    }
    void setEnableTriangleMeshBalance(bool enable) {
        enableTriangleMeshBalance = enable;
    }

    /// Debug/testing override (env-driven); normally configured via Builder.
    void forceEnableEdgeFlipPostProcess(bool enable, int rounds) {
        enableEdgeFlipPostProcess = enable;
        if (rounds > 0) edgeFlipPostProcessRounds = rounds;
    }

    /// @brief default 后处理：每轮压缩后执行 edge-flip，并按轮写 side-symbol。
    bool isEnableEdgeFlipPostProcess() const {
        return enableEdgeFlipPostProcess;
    }
    int getEdgeFlipPostProcessRounds() const {
        return edgeFlipPostProcessRounds;
    }
    double getEdgeFlipPostProcessLongEdgeRatio() const {
        return edgeFlipPostProcessLongEdgeRatio;
    }
    double getEdgeFlipPostProcessMinImprovementRatio() const {
        return edgeFlipPostProcessMinImprovementRatio;
    }

    const SegmentationOption& getSegmentationOption() const {
        return segmentation;
    }
    void setSegmentationOption(const SegmentationOption& s) {
        segmentation = s;
    }

  private:
    bool enableCompress = false;
    bool enablePrediction = false;
    bool enableQuantization = false;
    bool enableSegmentation = false;
    bool enableAdjustment = false;
    bool enableTriangulation = true;  // 默认 = 历史行为
    bool enableConnectivityPredictionFaces = false;
    bool enableConnectivityPredictionEdges = false;
    bool enableTriangleMeshBalance = false;
    bool enableEdgeFlipPostProcess = false;
    int edgeFlipPostProcessRounds = 1;
    double edgeFlipPostProcessLongEdgeRatio = 1.35;
    double edgeFlipPostProcessMinImprovementRatio = 0.90;

    std::string path;
    std::string outputPath;
    int round = 0;
    std::shared_ptr<SelectOperator> select;
    std::shared_ptr<EliminateOperator> eliminate;
    SegmentationOption segmentation;
    std::shared_ptr<PreProcessOperator> preProcessOperator;
    std::shared_ptr<PostProcessOperator> postProcessOperator;
};

#endif
