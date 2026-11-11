/*
 * @Author: koi
 * @Date: 2025-09-02 10:39:10
 * @Description:
 */
#ifndef OPTIMIZED_BASIC_DESERIALIZE_OPERATOR_H
#define OPTIMIZED_BASIC_DESERIALIZE_OPERATOR_H

#include "DeserializeOperator.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "decompress/SymbolReadOperator.h"
#include "postprocess/EdgeFlipPostProcessOperator.h"
// PHASE60 物品 3：分区（layout-32）产物的后端无关解码镜像，实现在 impl/common/
// （二进制与列式两个消费者，见该文件头注释）。列式侧同源使用。
#include "SegmentedRangeCodedDecode.h"
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <memory>
#include <utility>
#include <vector>
#include <unordered_set>

class OptimizedBasicDeserializeOperator : public DeserializeOperator {
  public:
    OptimizedBasicDeserializeOperator(std::shared_ptr<SymbolReadOperator> symbolReadOperator,
                                      std::shared_ptr<ReconstructOperator> reconstructOperator)
        : DeserializeOperator(symbolReadOperator, reconstructOperator) {}

    ~OptimizedBasicDeserializeOperator() = default;

    // void init(std::string path) override;

    void setSeeds(std::vector<MCGAL::Halfedge*> seeds) override {
        seed = seeds[0];
    }

    std::shared_ptr<MCGAL::Mesh> deserializeBaseMesh() override;

    std::shared_ptr<MCGAL::Mesh> exportMesh() override;

    void deserializeOneRound() override;
    void applyBoundaryPatchesForRound(int boundaryRound);

    bool isBoundaryV3Payload() const override { return boundaryApplyMode_ == 71; }
    void finalizeBoundaryInverses(int rounds) override;

    /// PHASE60 物品 3：本产物是否为分区（layout-32）blob。
    ///
    /// 前置条件是「header 声明了分区且开了预测」—— 只有 `segments > 1` 的配置
    /// 才同时满足（`SimplificationAlgorithmRegistry.cpp:271` 把
    /// `enableSegmentation` 置成 `segments > 1`，:491 据此换用
    /// `SegmentedOptimizedSymbolCollectOperator`，它是 layout-32 的唯一写出方）。
    ///
    /// 但**不能**靠 `header_.getGroupNumber()` 判别：实测二进制产物里它恒为 0
    /// （只有 `ColumnarSerializeOperator.cpp:130` 会赋值），故真正的判别只能
    /// 来自 blob 自身的逐字段约束，见 `pmsf::segmentedRangeCodedBlobMatches`。
    bool isSegmentedRangeCodedPayload() const {
        if (!header_.isEnablePrediction() || !header_.isEnableSegmentation() || buffer_ == nullptr || mesh_ == nullptr || totalSize <= dataOffset_) {
            return false;
        }
        std::size_t liveVertices = 0;
        for (MCGAL::Vertex* v : mesh_->vertices()) {
            if (v && !v->isRemoved()) {
                ++liveVertices;
            }
        }
        std::size_t liveFacets = 0;
        for (MCGAL::Facet* f : mesh_->faces()) {
            if (f && !f->isRemoved()) {
                ++liveFacets;
            }
        }
        return pmsf::segmentedRangeCodedBlobMatches(buffer_ + dataOffset_, totalSize - dataOffset_, static_cast<int>(liveVertices),
                                                    static_cast<int>(liveFacets));
    }

    MCGAL::Halfedge* seed = nullptr;
    std::vector<MCGAL::Vertex*> vertices;
    std::vector<MCGAL::Halfedge*> halfedges;
    std::vector<MCGAL::Facet*> facets;
    bool hasInterleavedEdgeFlipPayload_ = false;
    int edgeFlipDecodedRounds_ = 0;
    std::vector<int> edgeFlipRoundCounts_;
    std::vector<int> edgeFlipPrefix_;
    std::vector<std::pair<int, int>> edgeFlipSymbols_;
    std::vector<EdgeFlipPostProcessOperator::CoordSymbol> edgeFlipCoordSymbols_;
    std::vector<int> edgeFlipRoundBitCounts_;
    std::vector<int> edgeFlipRoundBitPrefix_;
    std::vector<unsigned char> edgeFlipRoundBits_;
    bool edgeFlipSeedBfsOrder_ = false;
    // V9 payload: one bit per all-edge in canonical (endpoint-q sorted) order; the
    // decoder rebuilds the same list from its own mesh state, so no per-flip
    // coordinate symbols and no shared BFS root are needed.
    bool edgeFlipCanonicalBits_ = false;
    std::vector<int> boundaryRoundBitCounts_;
    std::vector<int> boundaryRoundResidualPrefix_;
    std::vector<int> boundaryRoundBitPrefix_;
    std::vector<unsigned char> boundaryRoundBits_;
    std::vector<int> boundaryRoundResiduals_;
    int boundaryApplyMode_ = 0;
    std::unordered_set<int> boundaryAppliedRounds_;

    // PHASE60 物品 3：分区（layout-32）产物的每分区解码镜像。与列式
    // `ColumnarDeserializeOperator::segmentedDecode_` 是同一份实现。
    pmsf::SegmentedRangeCodedDecode segmentedDecode_;
    bool segmentedRangeCoded_ = false;
};

#endif
