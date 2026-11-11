// SPDX-License-Identifier: Apache-2.0
//
// APTMConquestCollector.h — APTM P3 / P3-P2（A&D valence/null_patch connectivity
// + APTM 锚点 + 可选 APTM 几何 hook）。
//
// 论文背景（Cheng/Jin/Liu, Pacific Graphics 2006）：
//   APTM 的 connectivity 就是 Alliez & Desbrun (SIGGRAPH 2001) 的 valence-driven
//   decimating/cleaning conquest —— 论文 §4.1 原文："The connectivity encoding step
//   is not changed in our approach, **except that we respect anchors and send more
//   null_patch codes to encode them**."
//
// 框架里已经有一份**逐符号对拍验证过**的 A&D valence-conquest 原生 codec
// （P13.3/P13.4）：编码 `pmsf::FrontierConquestEngine::decimateLayer`、解码
// `pmsf::decodeLayer`（4 网格 × 8 层链式 topoGeomOk=1）。P3 的“实现”= 复用这份
// 资产 + APTM 锚点注入（`AnchorVetoPredicate`，见 impl/Conquest/hooks/），
// 不重写引擎（重写只会引入分岔）。
//
// 本类在 **P3-P1** 基础上增加了**几何 hook 接缝**（P3-P2 / Arch-2 S1+S3）：
//   - 自己驱动引擎（复制父类 collect 的 ~60 行，因为要在 hooks 里同时注入
//     AnchorVetoPredicate 与 APTM 几何 hook；父类的 env 注入只支持锚点）；
//   - 自己持有 layers_ 并按 conquest_blob v2 写出（父类 stream_ 私有；布局与父类
//     逐字节同构：layers 逆序写、同一 writeStream）；
//   - 几何模式由 `PMSF_APTM_CONQUEST_GEOM` 控制：native（默认，P3-P1 原路径）/
//     lossy（S1 因果实验）/ polar（论文 §4.2 Frenet+θφ 3-bit，S3）。
//   - 导出后解析 conquest_blob 原始字节，给出逐分量
//     `[APTM-Conquest bpv] layers= symbols= nullPatch= connectivityBytes=
//      geometryBytes= containerBytes= totalBlob= verts= faces= anchors=`，
//     polar/lossy 模式再加一行 `[APTM-Conquest geom]` 统计。
//
// 兼容性：native 且未设 env ⇒ hook 集合只有 AnchorVeto（与 P3-P1 相同）⇒
// 产物应与 P3-P1 逐字节一致（56,841 B @ r16，实测核对）。默认 APTM 路径
// （无 `PMSF_APTM_USE_CONQUEST_P3`）根本不实例化本类，84,496 B 逐字节不变。

#ifndef PMSF_APTM_COMPRESS_APTM_CONQUEST_COLLECTOR_H
#define PMSF_APTM_COMPRESS_APTM_CONQUEST_COLLECTOR_H

#include "Conquest/compress/ConquestSymbolCollectOperator.h"
#include "Configuration.h"
#include "conquest/ConquestBlobFormat.h"
#include "conquest/ConquestDecodeLayer.h"
#include "conquest/ConquestMeshPrimitives.h"
#include "conquest/FrontierConquestEngine.h"
#include "feature/CurvatureHysteresisDetector.h"
#include "header/PMSFHeader.h"
#include "hooks/APTMConquestGeometryHook.h"

#include "Mesh.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <unistd.h>

namespace pmsf {

class APTMConquestCollector final : public ConquestSymbolCollectOperator {
  public:
    APTMConquestCollector() = default;

    /// 在父类 `init()`（计算量化网格并把工作网格吸附到格中心）**之前**检测锚点：
    /// APTM §3 的 anchor 判定作用在**输入网格的原始几何**上（与 default APTM 的
    /// AnchorPreprocessor 一致）。检测到的 poolId 在删减全程稳定（顶点只删不建）。
    void init(std::shared_ptr<MCGAL::Mesh> mesh,
              std::vector<MCGAL::Halfedge*> seeds,
              PMSFHeader header) override {
        meshCopy_ = mesh;
        if (mesh) {
            vertexCount_ = static_cast<long long>(mesh->nb_vertices);
            faceCount_ = static_cast<long long>(mesh->nb_faces);
            pmsf::CurvatureHysteresisDetector detector;  // 1.7 / 1.2（论文默认）
            const pmsf::FeatureSet anchors = detector.detect(mesh);
            anchors_.clear();
            anchors_.reserve(anchors.size());
            for (std::uint32_t id : anchors) anchors_.insert(id);
            anchorCount_ = anchors_.size();
            std::fprintf(stderr,
                         "[APTM-Conquest] detected anchors: %zu "
                         "(curvature hysteresis 1.7/1.2, APTM §3)\n",
                         anchorCount_);
        }
        ConquestSymbolCollectOperator::init(std::move(mesh), std::move(seeds),
                                            std::move(header));
        streamStep_ = quantGrid().step;
        const auto mode = aptm_conquest::resolveGeometryMode();
        arch2Iters_ = aptm_conquest::resolveArch2Iters();
        arch2On_ = (mode != aptm_conquest::GeometryMode::Native) &&
                   arch2Iters_ > 0;
        if (arch2On_) {
            std::fprintf(stderr,
                         "[APTM-Conquest] Arch-2 enabled: iters=%d "
                         "(encoder-side decode simulation + fixed point)\n",
                         arch2Iters_);
        }
        if (mode != aptm_conquest::GeometryMode::Native) {
            std::fprintf(stderr,
                         "[APTM-Conquest] geometry mode=%s levels=%d "
                         "lossyBits=%d\n",
                         (mode == aptm_conquest::GeometryMode::Polar) ? "polar"
                                                                     : "lossy",
                         aptm_conquest::resolvePolarLevels(),
                         aptm_conquest::resolveLossyBits());
        }
    }

    /// 收下引擎几何残差（整数域 cellDelta）。
    class DeltaSink final : public pmsf::GeometrySink {
      public:
        explicit DeltaSink(std::vector<std::array<int, 3>>* out) : out_(out) {}
        void onRemovedVertex(const int cellDelta[3]) override {
            out_->push_back({cellDelta[0], cellDelta[1], cellDelta[2]});
        }

      private:
        std::vector<std::array<int, 3>>* out_;
    };

    /// 一层 = 一条 LayerRecord（与父类语义一致，但 hooks 由本类构造）。
    void collect(MCGAL::Halfedge* seed) override {
        if (!seed || !meshCopy_) return;
        const auto mode = aptm_conquest::resolveGeometryMode();
        std::shared_ptr<aptm_conquest::APTMConquestResidualQuantizer> quant;
        std::shared_ptr<aptm_conquest::APTMRecordingPredictor> recPred;
        std::vector<std::array<int, 3>> layerTargets;
        if (arch2On_) {
            // Arch-2：录制每个被删顶点的 target cell（= 原网格位置的量化格号），
            // 供导出期 fixed-point 重算 delta 时用。
            recPred = std::make_shared<aptm_conquest::APTMRecordingPredictor>();
            recPred->onRemoved = [this, &layerTargets](std::uint32_t centerId) {
                if (!meshCopy_) return;
                MCGAL::Vertex* v =
                    MCGAL::contextPool.getVertexByIndexInSubPool(
                        meshCopy_->meshId(), static_cast<int>(centerId));
                if (!v) return;
                const auto& g = quantGrid();
                pmsf::ResidualQuantizer q;
                layerTargets.push_back(
                    {q.toCell(v->x(), g.bboxMin[0], g.step),
                     q.toCell(v->y(), g.bboxMin[1], g.step),
                     q.toCell(v->z(), g.bboxMin[2], g.step)});
            };
        }
        pmsf::EngineHooksPtr hooks = aptm_conquest::makeAPTMConquestHooks(
            &anchors_, mode, &quant, recPred);
        pmsf::FrontierConquestEngine engine(hooks);
        engine.setQuantGrid(quantGrid());

        pmsf::conquest_blob::LayerRecord layer;
        // 种子坐标随层记录（池号在解码端不可比；次序与父类一致：
        // seedPos[0]=tgt(seed)、seedPos[1]=tgt(opp(seed))）。
        if (MCGAL::Vertex* a = seed->end_vertex()) {
            layer.seedPos[0][0] = a->x();
            layer.seedPos[0][1] = a->y();
            layer.seedPos[0][2] = a->z();
        }
        if (MCGAL::Halfedge* o = seed->opposite()) {
            if (MCGAL::Vertex* b = o->end_vertex()) {
                layer.seedPos[1][0] = b->x();
                layer.seedPos[1][1] = b->y();
                layer.seedPos[1][2] = b->z();
            }
        }
        pmsf::ConquestState st;
        DeltaSink sink(&layer.deltas);
        engine.decimateLayer(meshCopy_.get(), st, seed, &layer.symbols, &sink);
        unhandledType0_ += engine.unhandledType0Count();
        streamStep_ = quantGrid().step;

        // 累计几何 hook 统计（用于证据：R/角度分布）。
        if (quant) {
            geomCount_ += quant->count();
            geomPolarCount_ += quant->polarCount();
            geomZero_ += quant->zeroResidual();
            geomFallback_ += quant->frameFallback();
            if (quant->rMax() > geomRMax_) geomRMax_ = quant->rMax();
            for (int i = 0; i < 8; ++i) {
                thetaHist_[i] += quant->thetaHist()[i];
                phiHist_[i] += quant->phiHist()[i];
            }
            lastMode_ = quant->mode();
            lastLevels_ = quant->levels();
        }
        layers_.push_back(std::move(layer));
        if (arch2On_) targetCells_.push_back(std::move(layerTargets));
    }

    void collect(std::vector<MCGAL::Halfedge*> seeds) override {
        if (!seeds.empty()) collect(seeds[0]);
    }

    /// 按 conquest_blob v2 写出（与父类逐字节同构：layers 逆序、同一 writeStream）。
    int exportToBufferWithRangeCoder(char* buffer,
                                     bool enableQuantization = false) override {
        (void)enableQuantization;
        if (buffer == nullptr || layers_.empty()) return -1;
        if (arch2On_) applyArch2();
        pmsf::conquest_blob::Stream out;
        out.step = streamStep_;
        const auto& g = quantGrid();
        out.bboxMin[0] = g.bboxMin[0];
        out.bboxMin[1] = g.bboxMin[1];
        out.bboxMin[2] = g.bboxMin[2];
        out.layers.assign(layers_.rbegin(), layers_.rend());
        const std::size_t need =
            pmsf::conquest_blob::detail::serializedSize(out);
        if (need > (std::size_t)BUFFER_SIZE) return -1;
        const int bytes = pmsf::conquest_blob::writeStream(
            out, buffer, (std::size_t)BUFFER_SIZE);
        if (bytes <= 0) return bytes;
        parseAndReportComponents(buffer, static_cast<std::size_t>(bytes));
        reportGeomStats();
        return bytes;
    }

    std::size_t anchorCount() const { return anchorCount_; }

    // ---- 逐分量探针（证据脚本读日志）----
    int lastSymbolsBytes() const { return symBytes_; }
    int lastDeltasBytes() const { return deltaBytes_; }
    int lastContainerBytes() const { return containerBytes_; }
    int lastSymbolCount() const { return symbolCount_; }
    int lastNullPatchCount() const { return nullPatchCount_; }
    int lastLayerCount() const { return layerCount_; }

  private:
    /// Arch-2（S2）：**单趟 decoder 精确模拟**，把几何残差改成「decoder 将看到的位置」。
    ///
    /// 旧写法（迭代 Jacobi 不动点）为什么不对：它用 decodeLayer 的 cur-delta 重建位置
    /// 去算 pred，但换用新 delta 后 decoder 位置又变 ⇒ 帧/预测不同步、出现漂移尖峰。
    ///
    /// 正确做法（LIFO 解码顺序、单趟、无迭代）：
    ///   decoder 对第 idx 层每个插点 k 的动作是**完全可复现**的：
    ///     ring = 该插点的环邻居（拓扑，解码顺序里都已先处理）
    ///     pred = 环位置算术平均；predCell = toCell(pred)
    ///     d    = inverseTransform(stored_delta, PatchView(ring 位置))
    ///     pos  = bm + (predCell + d + 0.5) * step        ← 这就是 decoder 的坐标
    ///   我们从 base mesh 出发、按解码顺序重放拓扑（decodeLayer 只用来建环/给池号），
    ///   同时自己维护 `posMap: 池号 → 正在被 decoder 重建的坐标`：
    ///     raw   = target_cell - toCell(pred_from_posMap)   （target = 原网格量化格）
    ///     stored= transform(raw, PatchView(ring_from_posMap))
    ///     pos   = bm + (toCell(pred) + inverse(stored) + 0.5) * step  → 写入 posMap
    ///   因为每一步的 pred/ring/PatchView 与 decoder 逐位相同，写出的 stored 让 decoder
    ///   重建出的坐标**逐位等于**我们的 posMap ⇒ 几何误差只剩「target 量化 + polar 角度
    ///   量化」这一层，不再有跨层漂移（拓扑本来就与坐标无关）。
    ///
    /// 迭代次数：真正需要 1 趟；`PMSF_APTM_CONQUEST_ARCH2_ITERS` 作为**验证趟数**，
    /// 第 2 趟应 `changed=0`（= 不动点已成立）。
    void applyArch2() {
        if (arch2Computed_) return;  // 幂等：paper 路径会二次 export（captureNativeBlob）
        arch2Computed_ = true;
        const std::size_t N = layers_.size();
        if (N == 0 || targetCells_.size() != N) {
            std::fprintf(stderr,
                         "[APTM-Conquest arch2] SKIP: target/layer size mismatch "
                         "(%zu vs %zu)\n",
                         targetCells_.size(), N);
            return;
        }
        if (!meshCopy_) return;
        const auto& g = quantGrid();
        char basePath[256];
        std::snprintf(basePath, sizeof(basePath),
                      "/tmp/aptm_arch2_base_%d.off", static_cast<int>(::getpid()));
        meshCopy_->dumpto_oldtype(basePath);
        const auto mode = aptm_conquest::resolveGeometryMode();

        // 初始 cur = pass1 的 stored delta（只用来让 decodeLayer 建拓扑；几何不用它）
        std::vector<std::vector<std::array<int, 3>>> cur(N);
        for (std::size_t i = 0; i < N; ++i) cur[i] = layers_[i].deltas;

        pmsf::ResidualQuantizer qref;
        aptm_conquest::APTMConquestResidualQuantizer qhook(mode);
        int usedIters = 0;
        bool converged = false;
        long long changed = 0;
        double errRms = 0.0;
        long long errMax = 0;

        for (int it = 1; it <= arch2Iters_; ++it) {
            std::shared_ptr<MCGAL::Mesh> sim =
                std::make_shared<MCGAL::Mesh>(std::string(basePath));
            // 池号 → decoder 已重建的坐标；base 顶点不在此表（稳定，直接读网格）。
            std::unordered_map<std::uint32_t, std::array<double, 3>> posMap;
            std::vector<std::vector<aptm_conquest::APTMInsertRecord>> caps(N);
            auto capPred =
                std::make_shared<aptm_conquest::APTMCapturingPredictor>();
            pmsf::EngineHooksPtr decHooks = aptm_conquest::makeAPTMConquestHooks(
                nullptr, mode, nullptr, capPred);

            long long errSqSum = 0, errCnt = 0;
            errMax = 0;
            std::vector<std::vector<std::array<int, 3>>> next(N);
            changed = 0;

            for (std::size_t j = 0; j < N; ++j) {
                const std::size_t idx = N - 1 - j;  // 解码顺序 = 编码逆序
                capPred->out = &caps[idx];
                pmsf::conquest_blob::LayerRecord& L = layers_[idx];
                MCGAL::Vertex* s0 =
                    pmsf::nearestVertex(sim.get(), L.seedPos[0]);
                MCGAL::Vertex* s1 =
                    pmsf::nearestVertex(sim.get(), L.seedPos[1]);
                if (s0 == nullptr || s1 == nullptr) {
                    std::fprintf(stderr,
                                 "[APTM-Conquest arch2] SKIP iter=%d layer=%zu: "
                                 "seed vertex not found\n",
                                 it, idx);
                    std::remove(basePath);
                    return;
                }
                MCGAL::Halfedge* seed = pmsf::halfedgeBetween(s1, s0);
                if (seed == nullptr) {
                    std::fprintf(stderr,
                                 "[APTM-Conquest arch2] SKIP iter=%d layer=%zu: "
                                 "seed halfedge not found\n",
                                 it, idx);
                    std::remove(basePath);
                    return;
                }
                const int vBefore = MCGAL::contextPool.getVindex(0);
                pmsf::ConquestState st;
                pmsf::decodeLayer(sim.get(), st, seed, L.symbols, cur[idx], g,
                                  *decHooks);
                const int vAfter = MCGAL::contextPool.getVindex(0);
                const std::size_t nNew =
                    static_cast<std::size_t>(vAfter - vBefore);
                const auto& tgt = targetCells_[idx];
                const auto& cap = caps[idx];
                if (nNew != tgt.size() || cap.size() != tgt.size()) {
                    std::fprintf(stderr,
                                 "[APTM-Conquest arch2] SKIP iter=%d layer=%zu: "
                                 "size mismatch new=%zu cap=%zu tgt=%zu\n",
                                 it, idx, nNew, cap.size(), tgt.size());
                    std::remove(basePath);
                    return;
                }
                next[idx].resize(tgt.size());
                std::vector<double> ring(3 * 256);
                for (std::size_t k = 0; k < tgt.size(); ++k) {
                    const std::uint32_t newId =
                        static_cast<std::uint32_t>(vBefore) +
                        static_cast<std::uint32_t>(k);
                    const int cnt = cap[k].count;
                    const std::size_t have = cap[k].ringIds.size();
                    // 环位置：已重建顶点查 posMap；base/尚未处理顶点读 sim 网格。
                    for (int a = 0; a < cnt; ++a) {
                        const std::uint32_t id =
                            a < static_cast<int>(have) ? cap[k].ringIds[a] : 0u;
                        auto itPos = posMap.find(id);
                        if (itPos != posMap.end()) {
                            ring[3 * a + 0] = itPos->second[0];
                            ring[3 * a + 1] = itPos->second[1];
                            ring[3 * a + 2] = itPos->second[2];
                        } else {
                            MCGAL::Vertex* v =
                                MCGAL::contextPool.getVertexByIndexInSubPool(
                                    0, static_cast<int>(id));
                            ring[3 * a + 0] = v ? v->x() : 0.0;
                            ring[3 * a + 1] = v ? v->y() : 0.0;
                            ring[3 * a + 2] = v ? v->z() : 0.0;
                        }
                    }
                    double pred[3] = {0.0, 0.0, 0.0};
                    for (int a = 0; a < cnt; ++a) {
                        pred[0] += ring[3 * a + 0];
                        pred[1] += ring[3 * a + 1];
                        pred[2] += ring[3 * a + 2];
                    }
                    if (cnt > 0) {
                        pred[0] /= cnt;
                        pred[1] /= cnt;
                        pred[2] /= cnt;
                    }
                    int predCell[3];
                    for (int a = 0; a < 3; ++a) {
                        predCell[a] = qref.toCell(pred[a], g.bboxMin[a], g.step);
                    }
                    int raw[3];
                    for (int a = 0; a < 3; ++a) {
                        raw[a] = tgt[k][static_cast<std::size_t>(a)] - predCell[a];
                    }
                    pmsf::PatchView pv;
                    pv.neighborPos = ring.data();
                    pv.neighborCount = cnt;
                    pv.quantStep = g.step;
                    pv.bboxMin = g.bboxMin;
                    pv.neighborIds = nullptr;
                    pv.centerId = newId;
                    int stored[3];
                    qhook.transform(raw, stored, pv);
                    next[idx][k] = {stored[0], stored[1], stored[2]};
                    if (next[idx][k] != cur[idx][k]) ++changed;

                    // decoder 将重建出的坐标（逐位同公式）。
                    int dcell[3];
                    qhook.inverseTransform(stored, dcell, pv);
                    std::array<double, 3> pos;
                    for (int a = 0; a < 3; ++a) {
                        const int cell = predCell[a] + dcell[a];
                        const float bm = static_cast<float>(g.bboxMin[a]);
                        const float st = static_cast<float>(g.step);
                        pos[static_cast<std::size_t>(a)] =
                            static_cast<double>(bm +
                                                (static_cast<float>(cell) + 0.5f) * st);
                    }
                    posMap[newId] = pos;

                    // 诊断：decoder 位置 vs target cell 的误差（量化格）。
                    for (int a = 0; a < 3; ++a) {
                        const long long e = static_cast<long long>(
                            predCell[a] + dcell[a] -
                            tgt[k][static_cast<std::size_t>(a)]);
                        const long long ae = e < 0 ? -e : e;
                        errSqSum += ae * ae;
                        if (ae > errMax) errMax = ae;
                        ++errCnt;
                    }
                }
            }
            errRms = errCnt > 0 ? std::sqrt(static_cast<double>(errSqSum) /
                                            static_cast<double>(errCnt))
                                : 0.0;
            geomErrRmsCells_ = errRms;
            geomErrMaxCells_ = errMax;
            usedIters = it;
            cur = std::move(next);
            std::fprintf(stderr,
                         "[APTM-Conquest arch2] iter=%d changed=%lld "
                         "geomErrRmsCells=%.4f geomErrMaxCells=%lld\n",
                         it, changed, errRms, errMax);
            if (changed == 0) {
                converged = true;
                break;
            }
        }
        std::remove(basePath);
        for (std::size_t i = 0; i < N; ++i)
            layers_[i].deltas = std::move(cur[i]);
        std::fprintf(stderr,
                     "[APTM-Conquest arch2] done iters=%d converged=%d "
                     "geomErrRmsCells=%.4f geomErrMaxCells=%lld\n",
                     usedIters, converged ? 1 : 0, errRms, errMax);
        std::fflush(stderr);
    }

    void reportGeomStats() const {
        const auto mode = aptm_conquest::resolveGeometryMode();
        if (mode == aptm_conquest::GeometryMode::Native) return;
        double thetaBits = 0.0, phiBits = 0.0;
        const long long total = thetaHist_[0] + thetaHist_[1] + thetaHist_[2] +
                                thetaHist_[3] + thetaHist_[4] +
                                thetaHist_[5] + thetaHist_[6] +
                                thetaHist_[7];
        for (int i = 0; i < 8; ++i) {
            if (thetaHist_[i] > 0 && total > 0) {
                const double p = (double)thetaHist_[i] / total;
                thetaBits -= p * std::log2(p);
            }
            if (phiHist_[i] > 0 && total > 0) {
                const double p = (double)phiHist_[i] / total;
                phiBits -= p * std::log2(p);
            }
        }
        std::fprintf(stderr,
                     "[APTM-Conquest geom] mode=%s levels=%d transforms=%lld "
                     "zeroResidual=%lld frameFallback=%lld rMax=%d "
                     "thetaEntropyBits=%.3f phiEntropyBits=%.3f\n",
                     (mode == aptm_conquest::GeometryMode::Polar) ? "polar"
                                                                  : "lossy",
                     lastLevels_, geomCount_, geomZero_, geomFallback_,
                     geomRMax_, thetaBits, phiBits);
        std::fflush(stderr);
    }

    /// conquest_blob v2 原始字节布局（src/conquest/ConquestBlobFormat.h）：
    ///   [magic 4][u16 ver][u16 flags][f64 step][f64 bboxMin[3]][u32 layers]
    ///   per layer: [f64 seedPos[2][3]] [u32 numSymbols][u32 symBytes][symBytes]
    ///              [u32 numDeltas] [3 × (i32 lo][u32 span][u32 bytes][bytes])]
    void parseAndReportComponents(const char* buffer, std::size_t bytes) {
        symBytes_ = deltaBytes_ = containerBytes_ = symbolCount_ = 0;
        nullPatchCount_ = layerCount_ = 0;
        pmsf::conquest_blob::Stream s;
        if (!pmsf::conquest_blob::readStream(buffer, bytes, &s)) {
            std::fprintf(stderr,
                         "[APTM-Conquest] WARN conquest_blob readStream failed "
                         "(bytes=%zu)\n",
                         bytes);
            return;
        }
        layerCount_ = static_cast<int>(s.layers.size());
        for (const auto& L : s.layers) {
            symbolCount_ += static_cast<int>(L.symbols.size());
            for (int sym : L.symbols) {
                if (sym == 4) ++nullPatchCount_;  // 4 = null patch（A&D）
            }
        }
        const std::uint8_t* p = reinterpret_cast<const std::uint8_t*>(buffer);
        const std::uint8_t* end = p + bytes;
        auto ok = [&](std::size_t n) {
            return static_cast<std::size_t>(end - p) >= n;
        };
        auto skip = [&](std::size_t n) { p += n; };
        auto rdU32 = [&]() -> std::uint32_t {
            std::uint32_t v = 0;
            std::memcpy(&v, p, 4);
            p += 4;
            return v;
        };
        if (!ok(4 + 2 + 2 + 8 * 4 + 4)) return;
        skip(4 + 2 + 2);  // magic, version, flags
        skip(8 * 4);      // step + bboxMin[3]
        const std::uint32_t layers = rdU32();
        if (layers != s.layers.size()) {
            std::fprintf(stderr,
                         "[APTM-Conquest] WARN layer count mismatch raw=%u "
                         "decoded=%zu\n",
                         layers, s.layers.size());
            return;
        }
        for (std::uint32_t i = 0; i < layers; ++i) {
            if (!ok(2 * 3 * 8 + 8)) return;
            skip(2 * 3 * 8);  // seedPos[2][3]
            rdU32();          // numSymbols
            const std::uint32_t nsBytes = rdU32();
            if (!ok(nsBytes)) return;
            symBytes_ += static_cast<int>(nsBytes);
            skip(nsBytes);
            const std::uint32_t nd = rdU32();
            for (std::uint32_t k = 0; k < 3; ++k) {
                if (!ok(4 + 4 + 4)) return;
                skip(8);      // lo, span
                const std::uint32_t nb = rdU32();
                if (!ok(nb)) return;
                deltaBytes_ += static_cast<int>(nb);
                skip(nb);
            }
            (void)nd;
        }
        containerBytes_ = static_cast<int>(bytes) - symBytes_ - deltaBytes_;
        std::fprintf(stderr,
                     "[APTM-Conquest bpv] layers=%d symbols=%d nullPatch=%d "
                     "connectivityBytes=%d geometryBytes=%d containerBytes=%d "
                     "totalBlob=%zu verts=%lld faces=%lld anchors=%zu\n",
                     layerCount_, symbolCount_, nullPatchCount_, symBytes_,
                     deltaBytes_, containerBytes_, bytes, vertexCount_,
                     faceCount_, anchorCount_);
        std::fflush(stderr);
    }

    bool arch2On_ = false;
    bool arch2Computed_ = false;
    int arch2Iters_ = 0;
    std::vector<std::vector<std::array<int, 3>>> targetCells_;
    double geomErrRmsCells_ = 0.0;
    long long geomErrMaxCells_ = 0;

    std::shared_ptr<MCGAL::Mesh> meshCopy_;
    std::unordered_set<std::uint32_t> anchors_;
    std::vector<pmsf::conquest_blob::LayerRecord> layers_;
    double streamStep_ = 0.0;

    std::size_t anchorCount_ = 0;
    long long vertexCount_ = 0;
    long long faceCount_ = 0;
    std::uint64_t unhandledType0_ = 0;

    // 几何 hook 累计统计
    aptm_conquest::GeometryMode lastMode_ = aptm_conquest::GeometryMode::Native;
    int lastLevels_ = 8;
    long long geomCount_ = 0;
    long long geomPolarCount_ = 0;
    long long geomZero_ = 0;
    long long geomFallback_ = 0;
    int geomRMax_ = 0;
    long long thetaHist_[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    long long phiHist_[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    // 逐分量探针
    int symBytes_ = 0;
    int deltaBytes_ = 0;
    int containerBytes_ = 0;
    int symbolCount_ = 0;
    int nullPatchCount_ = 0;
    int layerCount_ = 0;
};

}  // namespace pmsf

#endif  // PMSF_APTM_COMPRESS_APTM_CONQUEST_COLLECTOR_H
