// SPDX-License-Identifier: Apache-2.0
//
// PCLTTMTagSymbolCollectOperator.cpp — see header for design contract.
//
// 实施策略（阶段 1 / sv=2 / fusedFaceSym 合流）：
//   完全复用父类 PCATMColorSymbolCollectOperator 的 BFS 收集（valence + isPoly +
//   edgeSym + color + residual），其内部把 5 段流塞进 protected gateXxxQueues_。
//   本类**额外**跑一遍同序 BFS，对每个 BFS face emit 1 个 4-元 fusedFaceSym：
//     0 = NotPoly       (face !isSplittable)
//     1 = PolyUntagged  (face isSplittable && proxyHalfedge.vertex.tag == Untagged)
//     2 = PolyPlus      (... tag == Plus)
//     3 = PolyMinus     (... tag == Minus)
//   export 函数：4 段流 ConnectivityBlobBuilder（valence + fusedFaceSym + edgeSym + residual）。
//   旧 sv=1 isPoly 与 vertexTag 已合并进 stream 1。decoder 端按对称拆回。

#include "PCLTTM/compress/PCLTTMTagSymbolCollectOperator.h"
#include "common/ValenceStreamCodec.h"  // pmsf::encodeValence5（P3 抽取）
#include "common/AlphabetStreamCodec.h"  // pmsf::encodeAlphabetStream（P11 抽取）
#include "entropy/StreamEntropyCoder.h"
#include "common/PaperLayoutAliases.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂
#include "core/PMSFBoundarySideChannel.h"
#include "core/PaperGeometrySideChannel.h"
#include "PCLTTM/boundary/PCLTTMPMSFBoundary.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include "BFSVersionManager.h"
#include "compress/ConnectivityBlobContainer.h"
#include "compress/RangeCoderStreamEncoder.h"
#include "Configuration.h"
#include "ContextPool.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Point.h"
#include "Vertex.h"
#include "core.h"
#include "common/Define.h"
#include "RD/compress/RDSymbolCollectOperator.h"  // RDValenceCode
#include "common/RDTagCanonicalCodec.h"  // pmsf::rd_tag::encodeResidualBitplane（P7 抽取）
#include "predict/RDPredictor.h"               // kVPropRDR1, rdCurvaturePrediction
#include "PCLTTM/PCLTTMFrenetLayer.h"
#include "predict/FrenetResidualCodec.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <queue>
#include <stdexcept>
#include <vector>

// =============================================================================
// =============================================================================
// 用法：
// 输出到 /tmp/pclttm_phase2_enc_trace.log（覆写）。
// 解码端对应 dec_trace.log。两份对 diff 找发散点。
namespace {

bool pclttmEnvOn(const char* name) {
    return pmsf::pmsfStorageEnvEnabled(name);
}

void requirePCLTTMTagPaperExactIfRequested() {
    if (!pmsf::boundaryFlagOn(pmsf::pclttm_boundary::kPaperExactSpec)) {
        return;
    }
    throw std::runtime_error(
        "[PCLTTMTag] paper-exact topology/geometry replay is not implemented "
        "for BlobLayout=16; use CODEC_028 only after its exact replay "
        "guards are satisfied.");
}




inline bool pclttmProfileLogs() {
    static const bool enabled = [] {
        const char* env = std::getenv("PMSF_PROFILE");
        return env && env[0] != '\0' && std::strcmp(env, "0") != 0 &&
               std::strcmp(env, "false") != 0 && std::strcmp(env, "FALSE") != 0;
    }();
    return enabled;
}

bool rdtagKeepR1AcrossRounds() {
    const char* e = std::getenv("PMSF_RDTAG_KEEP_R1");
    return e && e[0] == '1';
}

bool rdtagFrenetRoundEnabled(int roundIdx) {
    const char* e = std::getenv("PMSF_RDTAG_FRENET_RESIDUAL");
    const char* exact = std::getenv("PMSF_RDTAG_FRENET_EXACT");
    if ((!e || e[0] != '1') && (!exact || exact[0] != '1')) return false;
    int start = 0;
    int end = 1 << 30;
    if (const char* s = std::getenv("PMSF_RDTAG_FRENET_START")) {
        start = std::atoi(s);
    }
    if (const char* s = std::getenv("PMSF_RDTAG_FRENET_END")) {
        end = std::atoi(s);
    }
    return roundIdx >= start && roundIdx <= end;
}

bool rdtagFrenetExactEnabled() {
    const char* e = std::getenv("PMSF_RDTAG_FRENET_EXACT");
    return e && e[0] == '1';
}

bool rdtagFrenetGateFrameEnabled() {
    const char* e = std::getenv("PMSF_RDTAG_FRENET_GATE_FRAME");
    return e && e[0] == '1';
}

MCGAL::PointInt projectResidualToFrenet(const MCGAL::PointInt& r,
                                        MCGAL::Facet* f,
                                        const MCGAL::Point* bboxMin,
                                        float quantStep);
MCGAL::PointInt unprojectResidualFromFrenet(const MCGAL::PointInt& r,
                                            MCGAL::Facet* f,
                                            const MCGAL::Point* bboxMin,
                                            float quantStep);

}  // namespace

void PCLTTMTagSymbolCollectOperator::collect(MCGAL::Halfedge* seed) {
    static const std::vector<MCGAL::Halfedge*> kEmpty;
    collect(seed, kEmpty);
}

void PCLTTMTagSymbolCollectOperator::collect(std::vector<MCGAL::Halfedge*> seeds) {
    if (seeds.empty()) {
        throw std::runtime_error(
            "[PCLTTMTag] collect(vector<seed>) called with empty seeds");
    }
    static const std::vector<MCGAL::Halfedge*> kEmpty;
    collect(seeds[0], kEmpty);
}

void PCLTTMTagSymbolCollectOperator::collect(MCGAL::Halfedge* seed,
                                             const std::vector<MCGAL::Halfedge*>& gates) {
    // Phase 2 Step 3: 对 mesh 所有非 removed face 调 normalize，让 face.proxyHalfedge
    // 都对齐到 vertex **量化整数坐标**字典序最小的 outgoing halfedge。decoder 端在
    // collectFromDecoded 入口同步 normalize，两端 BFS 顺序 + face.getVertices
    // 顺序 100% 对齐（量化整数 bit-exact 一致；浮点 v.x()/y()/z() 不一致）。
    for (MCGAL::Facet* f : mesh_->faces()) {
        if (f && !f->isRemoved()) {
            f->normalize_proxy_to_min_position_vertex(
                mesh_->bboxMin, mesh_->f_quantStep);
        }
    }

    // ----------------------------------------------------------------------------
    // Phase 3 路径 A — 每轮入口清零所有 allocated vertex 的 r1。
    //   说明：跨轮 r1 复用会破坏 encoder/decoder 对称性 —— 编码端 round=r 对应
    //   解码端 round=R-1-r（LOD 时间轴反向）；跨轮累积无法在两端 mirror。
    //   每轮入口清零保证"本轮独立的 1-环 r1 累积"，与论文公式 (2) 单 LOD 邻居
    //   语义一致。两端在每轮入口都做同样清零（同 mesh 的 vindexs 个 allocated
    //   slot，包括 isRemoved 的，无害）→ bit-exact 对称。
    // ----------------------------------------------------------------------------
    if (!rdtagKeepR1AcrossRounds()) {
        const int meshId = mesh_->meshId();
        const int nAlloc = MCGAL::contextPool.getVindex(meshId);
        for (int i = 0; i < nAlloc; ++i) {
            MCGAL::Vertex* v = MCGAL::contextPool.getVertexByIndexInSubPool(meshId, i);
            MCGAL::PointInt& r1Slot =
                MCGAL::contextPool.property<MCGAL::PointInt>(
                    meshId, MCGAL::kVPropRDR1, v);
            r1Slot = MCGAL::PointInt(0, 0, 0);
        }
    }

    // 1) 调父类 PCATMColor BFS — 它会填 valence/isPoly/edgeSym/color/residual 5 段流。
    PCATMColorSymbolCollectOperator::collect(seed, gates);

    // ----------------------------------------------------------------------------
    // Phase 3 路径 A — 二阶 curvature predictor （在第二遍 BFS 里做）
    // ----------------------------------------------------------------------------
    //   父类已经把 polyFace 的 raw 一阶 residual `r1 = vrPos - barycenter` 顺序入
    //   `gateResidualQueues_.back()`。下面第二遍 BFS 与父类 BFS 严格同序（push 规则
    //   镜像 RDGateOnly v3，已被 Phase 2 验证 byte-identical），按 BFS 顺序遍 polyFace：
    //     1. 计算 `pred_c2 = mean(neighbors' kVPropRDR1)`（论文公式 (2)）；
    //     2. 把 `gateResidualQueues_.back()[k]` 从 r1 改写成 `encoded = r1 - pred_c2`；
    //     3. 把 r1 写到 polyFace 全部 polygon 邻居 vertex 的 `kVPropRDR1`，供 BFS 后
    //        续 face 的 pred_c2 使用。
    //
    //   时序对称：解码端 `PCLTTMTagSymbolReadOperator::collectFromDecoded` 在按
    //   同序 BFS 处理 polyFace 时，先 `r1 = encoded + pred_c2`，再写邻居 r1。两端
    //   "何时写"完全 match —— 这是 §B.1 路径 A 的核心不变量。
    //
    //   注意：跨轮（多个 LOD）r1 累积留在 vertex 上 → 后续轮的 pred_c2 不再恒为 0。
    //         每轮第一个 polyFace 仍可能 pred_c2=0（邻居本轮还没写过），符合预期。
    // ----------------------------------------------------------------------------
    // 第二阶段开始前，**新轮**：清掉所有 active vertex 的 r1（为本轮 BFS 重置）。
    // 注意：跨轮 inheritance 是论文设计意图，但跨轮的 r1 是**上一轮**几何的产物，
    //       直接复用会让 encoder/decoder 严格对齐**前提**：两端跨轮的 r1 完全一致。
    //       但我们这里采取更稳的语义：**每轮内** r1 累积，**每轮结束**不主动清；
    //       下一轮开头又会被覆盖（每个 polyFace 都会写邻居 r1）。base mesh
    //       vertex 在第 0 轮第一个 polyFace 处理前 r1=0（attachVertexProperty
    //       默认初始化为 0），后续每轮第一个 polyFace 用上一轮遗留的 r1 作为邻居预测，
    //       这正是 cross-LOD 信号传递。两端严格同步即可 round-trip。
    //
    //   property 默认初值：`PropertyT<T>::resize(n)` 会 default-construct，
    //   PointInt() 默认是 (0,0,0)，符合预期。

    auto& residualQueueThisRound = gateResidualQueues_.back();
    gateFacetQueues_.emplace_back();
    auto& facetQueueThisRound = gateFacetQueues_.back();
    size_t polyFaceCounter = 0;
    std::vector<pmsf::pclttm_frenet::PatchCode> frenetCodes;

    // (a) Step 1 诊断：env PMSF_RDTAG_LEAKY_DIAG=1 时备份本轮 pred_c2，供 leaky-
    //     factor grid-search 离线分析。默认 off → 零额外内存。
    static const bool kLeakyDiagOn = []() {
        const char* e = std::getenv("PMSF_RDTAG_LEAKY_DIAG");
        return e && e[0] == '1';
    }();
    if (kLeakyDiagOn) {
        gatePredC2Queues_.emplace_back();
        gateFaceNormalXQueues_.emplace_back();
        gateFaceNormalYQueues_.emplace_back();
        gateFaceNormalZQueues_.emplace_back();
        gateGateVecXQueues_.emplace_back();
        gateGateVecYQueues_.emplace_back();
        gateGateVecZQueues_.emplace_back();
    }

    // 2) 同序再次 BFS，对每个 BFS face emit 1 个 4-元 fusedFaceSym。
    //    per face 1 sym（与父类 isPoly 频率严格对齐），合并 isPoly + vertexTag。
    std::deque<uint8_t> fusedSyms;
    int current_version = MCGAL::BFSVersionManager::current_version++;

    const int roundIdxForTrace = static_cast<int>(gateFusedFaceSymQueues_.size());
    const bool frenetResidualThisRound =
        rdtagFrenetRoundEnabled(roundIdxForTrace);
    const bool frenetExact =
        rdtagFrenetExactEnabled();
    if (frenetExact) {
        gateFrenetCorrectionQueues_.emplace_back();
    }
    int traceBfsIdx = 0;
    std::FILE* degreeDump = nullptr;

    std::vector<int> q;
    q.reserve(mesh_ ? mesh_->faces().size() * 2 + 16 : 16);
    std::size_t qHead = 0;
    if (seed != nullptr) {
        q.push_back(seed->poolId());
    }
    while (qHead < q.size()) {
        int hid = q[qHead++];
        MCGAL::Halfedge* h =
            MCGAL::contextPool.getHalfedgeByIndexInSubPool(mesh_->meshId(), hid);
        if (h == nullptr || h->isVisited(current_version) || h->isRemoved()) continue;
        h->setVisited(current_version);

        MCGAL::Facet* f = h->face();
        bool faceFirstVisit = (f != nullptr) && !f->isVisited(current_version);
        if (faceFirstVisit) {
            f->setVisited(current_version);

            uint8_t sym;
            if (!f->isSplittable()) {
                sym = 0;  // NotPoly
            } else {
                MCGAL::Halfedge* hh = f->proxyHalfedge();
                MCGAL::Vertex* v = hh ? hh->vertex() : nullptr;
                uint8_t tag = v ? static_cast<uint8_t>(v->getTag()) : 0;
                // tag: Untagged=0 → sym=1, Plus=1 → sym=2, Minus=2 → sym=3
                if (tag > 2) tag = 0;
                sym = static_cast<uint8_t>(1 + tag);
            }
            fusedSyms.push_back(sym);
            if (degreeDump) {
                std::fprintf(degreeDump, "%u %d\n",
                             static_cast<unsigned>(sym >= 1 ? 1 : 0),
                             static_cast<int>(f->facet_degree()));
            }

            // ---- Phase 3 路径 A：二阶 curvature predictor 处理 polyFace ----
            // sym >= 1 → 是 polyFace（与父类 isPoly=1 同步频率），对应
            // gateResidualQueues_.back()[polyFaceCounter] = r1（一阶 raw residual）。
            //   1) 计算 pred_c2 = mean(neighbors' kVPropRDR1)；
            //   2) rewrite residualQueueThisRound[polyFaceCounter] = r1 - pred_c2；
            //   3) 把 r1 写到 polygon 全部邻居 vertex 的 kVPropRDR1。
            // 解码端在同序 BFS 处理时，先 r1 = encoded + pred_c2，再写邻居 r1，
            // 两端"何时写"严格 match。
            if (sym >= 1 && polyFaceCounter < residualQueueThisRound.size()) {
                facetQueueThisRound.push_back(f);
                MCGAL::PointInt& slot = residualQueueThisRound[polyFaceCounter];
                if (frenetPatches_) {
                    const auto patch = frenetPatches_->find(f);
                    if (patch == frenetPatches_->end()) {
                        throw std::runtime_error("PCLTTM Frenet collector lacks the eliminated patch frame");
                    }
                    frenetCodes.push_back(patch->second);
                    frenetPatches_->erase(patch);
                } else {
                MCGAL::PointInt r1 = slot;  // 父类已写入：r1 = vrPos - barycenter
                MCGAL::PointInt pred_c2 =
                    MCGAL::rdCurvaturePrediction(f, mesh_->meshId());
                MCGAL::PointInt r1ForState = r1;
                // encoded = r1 - pred_c2
                slot = MCGAL::PointInt(r1.x() - pred_c2.x(),
                                       r1.y() - pred_c2.y(),
                                       r1.z() - pred_c2.z());
                if (frenetResidualThisRound) {
                    const MCGAL::PointInt originalEncoded = slot;
                    if (rdtagFrenetGateFrameEnabled()) {
                        const pmsf::FrenetFrame frame = pmsf::safeHalfedgeFrame(h);
                        MCGAL::Vector3i rotated = pmsf::rotateResidualToFrenet(
                            MCGAL::Vector3i(slot.x(), slot.y(), slot.z()),
                            frame,
                            1 << 30);
                        slot = MCGAL::PointInt(rotated[0], rotated[1], rotated[2]);
                    } else {
                        slot = projectResidualToFrenet(
                            slot, f, &mesh_->bboxMin, mesh_->f_quantStep);
                    }
                    MCGAL::PointInt approxEncoded;
                    if (rdtagFrenetGateFrameEnabled()) {
                        const pmsf::FrenetFrame frame = pmsf::safeHalfedgeFrame(h);
                        MCGAL::Vector3i inv = pmsf::rotateResidualFromFrenet(
                            MCGAL::Vector3i(slot.x(), slot.y(), slot.z()),
                            frame);
                        approxEncoded = MCGAL::PointInt(inv[0], inv[1], inv[2]);
                    } else {
                        approxEncoded = unprojectResidualFromFrenet(
                            slot, f, &mesh_->bboxMin, mesh_->f_quantStep);
                    }
                    if (frenetExact) {
                        gateFrenetCorrectionQueues_.back().push_back(
                            MCGAL::PointInt(originalEncoded.x() - approxEncoded.x(),
                                            originalEncoded.y() - approxEncoded.y(),
                                            originalEncoded.z() - approxEncoded.z()));
                    } else {
                        r1ForState = MCGAL::PointInt(approxEncoded.x() + pred_c2.x(),
                                                     approxEncoded.y() + pred_c2.y(),
                                                     approxEncoded.z() + pred_c2.z());
                    }
                }
                // (a) Step 1 诊断：备份 pred_c2 + face normal + gate vector
                //   供 leaky-factor + Frenet 投影离线诊断。
                if (kLeakyDiagOn) {
                    gatePredC2Queues_.back().push_back(pred_c2);
                    MCGAL::Vector3 n = f->computeNormal();
                    // Normalize（computeNormal 可能返回未归一化向量）
                    double nlen = std::sqrt((double)n.x()*n.x() + (double)n.y()*n.y() + (double)n.z()*n.z());
                    if (nlen > 1e-12) {
                        gateFaceNormalXQueues_.back().push_back((float)(n.x() / nlen));
                        gateFaceNormalYQueues_.back().push_back((float)(n.y() / nlen));
                        gateFaceNormalZQueues_.back().push_back((float)(n.z() / nlen));
                    } else {
                        gateFaceNormalXQueues_.back().push_back(0.0f);
                        gateFaceNormalYQueues_.back().push_back(0.0f);
                        gateFaceNormalZQueues_.back().push_back(1.0f);
                    }
                    // gate vector = h.vertex - h.opposite.vertex（在浮点 world 域）
                    MCGAL::Halfedge* hp = f->proxyHalfedge();
                    if (hp && hp->opposite()) {
                        MCGAL::Point P = hp->vertex()->point();
                        MCGAL::Point Q = hp->opposite()->vertex()->point();
                        gateGateVecXQueues_.back().push_back(P.x() - Q.x());
                        gateGateVecYQueues_.back().push_back(P.y() - Q.y());
                        gateGateVecZQueues_.back().push_back(P.z() - Q.z());
                    } else {
                        gateGateVecXQueues_.back().push_back(1.0f);
                        gateGateVecYQueues_.back().push_back(0.0f);
                        gateGateVecZQueues_.back().push_back(0.0f);
                    }
                }
                // 把 r1 写回所有 polygon 邻居 vertex（注意：polygon 的 1-环
                // vertex 即 face.getVertices()，这些是当前 LOD 的 active vertex，
                // 不会是 removed 的 center vertex）。
                MCGAL::Halfedge* start = f->proxyHalfedge();
                MCGAL::Halfedge* hit = start;
                do {
                    MCGAL::Vertex* vn = hit ? hit->vertex() : nullptr;
                    if (vn == nullptr) continue;
                    MCGAL::PointInt& r1Slot =
                        MCGAL::contextPool.property<MCGAL::PointInt>(
                            mesh_->meshId(), MCGAL::kVPropRDR1, vn);
                    r1Slot = r1ForState;
                    hit = hit ? hit->next() : nullptr;
                } while (hit && hit != start);
                }
                polyFaceCounter++;
            }

            // ---- TRACE ----
            traceBfsIdx++;

            // BFS push neighbors —— **必须严格镜像父类 RDGateOnly v3 BFS 规则**
            // (see RDGateOnlySymbolCollectOperator.cpp:184-195)：
            // 不区分 opposite vs next-on-same-face，全部 push（按 next() 走遍 face 一周
            // 把 opposite + hIt 各 push 一次）。否则 BFS 访问的 face 数 ≠ 父类 isPoly 数，
            // 导致 decoder bfs faceCount mismatch。
            MCGAL::Halfedge* hIt = h;
            do {
                MCGAL::Halfedge* hOpp = hIt->opposite();
                if (hOpp != nullptr && !hOpp->isVisited(current_version)) {
                    q.push_back(hOpp->poolId());
                }
                if (hIt != nullptr && !hIt->isVisited(current_version)) {
                    q.push_back(hIt->poolId());
                }
                hIt = hIt->next();
            } while (hIt != h && hIt != nullptr);
        }
    }
    if (degreeDump) std::fclose(degreeDump);

    if (frenetPatches_) {
        if (frenetCodes.size() != residualQueueThisRound.size() || !frenetPatches_->empty()) {
            throw std::runtime_error("PCLTTM Frenet elimination/collection patch counts differ");
        }
        const auto layer=pmsf::pclttm_frenet::chooseLayer(frenetCodes);
        frenetDenominators_.push_back(layer.denominator);
        for(std::size_t i=0;i<layer.coefficients.size();++i) {
            const auto& q=layer.coefficients[i];
            residualQueueThisRound[i]=MCGAL::PointInt(q[0],q[1],q[2]);
        }
        std::cout << "PCLTTM_FRENET encoder layer=" << roundIdxForTrace
                  << " patches=" << frenetCodes.size() << " denominator=" << layer.denominator
                  << " whole_cell_misses=" << std::count_if(frenetCodes.begin(),frenetCodes.end(),[](const auto& p){return !p.wholeExact;})
                  << " degenerate_frames=" << std::count_if(frenetCodes.begin(),frenetCodes.end(),[](const auto& p){return p.degenerate;}) << '\n';
    }
    gateFusedFaceSymQueues_.push_back(std::move(fusedSyms));
    paperTableCache_.clear();

}

// ============================================================================
// 内部 helpers (mirror PCATMColor sv=3 export helpers，但 stream 3 改 3-元 RC)
// ============================================================================

namespace {

struct FrenetFrame {
    double nx = 0.0, ny = 0.0, nz = 1.0;
    double tx = 1.0, ty = 0.0, tz = 0.0;
    double bx = 0.0, by = 1.0, bz = 0.0;
};

// ============================================================================
// PR-FRENET-FIX（task-13）：Frenet frame 的**量化对称**构造。
//
// 问题：encoder 的 frame 用原始浮点顶点位置；decoder 的 frame 用量化重建位置
// （= intPosToFloat(q)，落在 (q+0.5)·step 半格上）。两者法向/门向量有
// ~quantStep/edgeLen 的系统差 ⇒ 残差旋转/反旋转不对合，sv=7 decode 失败
// （coord_max=1.75e-3, topology=false）。
//
// 修法：frenet env 开启时，frame 用到的所有位置先量化到 int 网格再重建
// （公式与 Mesh::floatPosToInt / intPosToFloat 逐位同源），双端 bit-exact。
// frame 函数只在 frenet 分支被调用（encoder env / decoder sv=7|8）⇒ 参数缺失时
// 退回原浮点路径，默认 RD/PCLTTM 产物逐字节不变。
// ============================================================================

/// 量化网格位置：与 Mesh::floatPosToInt（trunc）再 intPosToFloat（+0.5 半格）逐位同源。
/// encoder/decoder 两个 TU 保持同一份实现（decoder 传 bboxMin_/f_quantStep_）
/// ⇒ 两端 frame 的数值源完全一致。
MCGAL::Point frenetGridPoint(MCGAL::Vertex* v,
                             const MCGAL::Point* bboxMin,
                             float quantStep) {
    if (v == nullptr) return MCGAL::Point(0.0f, 0.0f, 0.0f);
    const MCGAL::Point p = v->point();
    if (bboxMin == nullptr || quantStep <= 0.0f) return p;
    const int qx = static_cast<int>((p.x() - bboxMin->x()) / quantStep);
    const int qy = static_cast<int>((p.y() - bboxMin->y()) / quantStep);
    const int qz = static_cast<int>((p.z() - bboxMin->z()) / quantStep);
    return MCGAL::Point(bboxMin->x() + (qx + 0.5) * quantStep,
                        bboxMin->y() + (qy + 0.5) * quantStep,
                        bboxMin->z() + (qz + 0.5) * quantStep);
}

/// 与 Facet::computeNormal 同源（proxyHalfedge 前 3 顶点叉积 + normalize），
/// 仅位置源改走 frenetGridPoint。非 frenet（或参数缺失）时退回原实现。
MCGAL::Vector3 frenetFrameNormal(MCGAL::Facet* f,
                                 const MCGAL::Point* bboxMin,
                                 float quantStep) {
    // 只在 frenet 分支被调用（encoder env / decoder sv=7|8）；参数缺失时退回原路径。
    if (bboxMin == nullptr || quantStep <= 0.0f) {
        return f->computeNormal();
    }
    MCGAL::Halfedge* h = f->proxyHalfedge();
    if (h == nullptr || h->next() == nullptr) {
        return MCGAL::Vector3(0, 0, 0);
    }
    MCGAL::Halfedge* hn = h->next();
    MCGAL::Halfedge* hnn = hn->next();
    MCGAL::Vertex* v3 = hnn ? hnn->vertex() : hn->end_vertex();
    const MCGAL::Point p1 = frenetGridPoint(h->vertex(), bboxMin, quantStep);
    const MCGAL::Point p2 = frenetGridPoint(hn->vertex(), bboxMin, quantStep);
    const MCGAL::Point p3 = frenetGridPoint(v3, bboxMin, quantStep);
    const MCGAL::Vector3 v1(p2.x() - p1.x(), p2.y() - p1.y(), p2.z() - p1.z());
    const MCGAL::Vector3 v2(p3.x() - p1.x(), p3.y() - p1.y(), p3.z() - p1.z());
    return v1.cross(v2).normalize();
}

FrenetFrame residualFrenetFrame(MCGAL::Facet* f,
                                const MCGAL::Point* bboxMin,
                                float quantStep) {
    FrenetFrame frame;
    if (!f) return frame;

    MCGAL::Vector3 n = frenetFrameNormal(f, bboxMin, quantStep);
    double nlen = std::sqrt((double)n.x() * n.x() +
                            (double)n.y() * n.y() +
                            (double)n.z() * n.z());
    if (nlen > 1e-12) {
        frame.nx = n.x() / nlen;
        frame.ny = n.y() / nlen;
        frame.nz = n.z() / nlen;
    }

    MCGAL::Halfedge* hp = f->proxyHalfedge();
    double gx = 1.0, gy = 0.0, gz = 0.0;
    if (hp && hp->opposite() && hp->vertex() && hp->opposite()->vertex()) {
        MCGAL::Point p = frenetGridPoint(hp->vertex(), bboxMin, quantStep);
        MCGAL::Point q =
            frenetGridPoint(hp->opposite()->vertex(), bboxMin, quantStep);
        gx = (double)p.x() - q.x();
        gy = (double)p.y() - q.y();
        gz = (double)p.z() - q.z();
    }

    const double dotGN = gx * frame.nx + gy * frame.ny + gz * frame.nz;
    frame.tx = gx - dotGN * frame.nx;
    frame.ty = gy - dotGN * frame.ny;
    frame.tz = gz - dotGN * frame.nz;
    double tlen = std::sqrt(frame.tx * frame.tx +
                            frame.ty * frame.ty +
                            frame.tz * frame.tz);
    if (tlen <= 1e-12) {
        const bool normalMostlyX =
            std::fabs(frame.nx) >= std::fabs(frame.ny) &&
            std::fabs(frame.nx) >= std::fabs(frame.nz);
        gx = normalMostlyX ? 0.0 : 1.0;
        gy = normalMostlyX ? 1.0 : 0.0;
        gz = 0.0;
        const double dotFallback = gx * frame.nx + gy * frame.ny + gz * frame.nz;
        frame.tx = gx - dotFallback * frame.nx;
        frame.ty = gy - dotFallback * frame.ny;
        frame.tz = gz - dotFallback * frame.nz;
        tlen = std::sqrt(frame.tx * frame.tx +
                         frame.ty * frame.ty +
                         frame.tz * frame.tz);
    }
    if (tlen > 1e-12) {
        frame.tx /= tlen;
        frame.ty /= tlen;
        frame.tz /= tlen;
    }

    frame.bx = frame.ny * frame.tz - frame.nz * frame.ty;
    frame.by = frame.nz * frame.tx - frame.nx * frame.tz;
    frame.bz = frame.nx * frame.ty - frame.ny * frame.tx;
    return frame;
}


// ============================================================================
// PR-FRENET-LIFT（task-13）：可逆整数 lifting 版局部系变换。
//
// 为什么需要：float 旋转 + round 是**不可逆**的（decoder 反旋转后残差有 ±0.5~1
// 量化单位的损失，经 50 层预测链累积到 ~39 quant unit ⇒ sv=7 decode coord_max
// 1.75e-3 / topology false；sv=8 exact correction 修得好但 correction 流
// 4,225 B > frenet 省下的 2,171 B，净亏）。这里把 R（rows=n,t,b）分解成
// Euler ZYX = Rz(α)Ry(β)Rx(γ)，每个 2D 旋转用 3 个 lifting shear
// （u += round(a·v) 这种整数可逆步）实现 ⇒ 整数域**严格可逆**（decoder 逐位逆回），
// 不需要 correction 流。R 由量化位置构造（frenetFrameNormal）⇒ 双端 bit-exact。
// ============================================================================

int liftRound(double v) { return static_cast<int>(std::llround(v)); }

/// 2D rotation [[c,-s],[s,c]] 的 3-lift 分解（a=(c-1)/s, b=s），精确可逆。
void lift2dForward(int& u, int& v, double c, double s) {
    // 近 pi 旋转：R(theta) = N·R(theta-pi)，N=两坐标取负（精确可逆）。
    // 否则 a=(c-1)/s → ∞，lifting 的 rounding 会被放大数千倍（实测 3243 quant）。
    if (c < 0.0) {
        u = -u;
        v = -v;
        c = -c;
        s = -s;
    }
    if (std::fabs(s) < 1e-15) return;  // theta'≈0
    const double a = (c - 1.0) / s;
    u += liftRound(a * static_cast<double>(v));
    v += liftRound(s * static_cast<double>(u));
    u += liftRound(a * static_cast<double>(v));
}

void lift2dInverse(int& u, int& v, double c, double s) {
    if (c < 0.0) {
        u = -u;
        v = -v;
        c = -c;
        s = -s;
    }
    if (std::fabs(s) < 1e-15) return;
    const double a = (c - 1.0) / s;
    u -= liftRound(a * static_cast<double>(v));
    v -= liftRound(s * static_cast<double>(u));
    u -= liftRound(a * static_cast<double>(v));
}

/// R = Rz(alpha)·Ry(beta)·Rx(gamma)（ZYX Euler）。R 的行 = (n, t, b)。
void frenetEulerAngles(const FrenetFrame& fr, double& alpha, double& beta,
                       double& gamma) {
    double sBeta = -fr.bx;  // r20 = -sin(beta)
    if (sBeta > 1.0) sBeta = 1.0;
    if (sBeta < -1.0) sBeta = -1.0;
    beta = std::asin(sBeta);
    if (std::fabs(std::cos(beta)) > 1e-8) {
        gamma = std::atan2(fr.by, fr.bz);  // r21, r22
        alpha = std::atan2(fr.tx, fr.nx);  // r10, r00
    } else {
        // gimbal lock（beta≈±90°）：只有 alpha∓gamma 可辨，取 gamma=0。
        gamma = 0.0;
        alpha = std::atan2(-fr.ny, fr.ty);
    }
}

/// forward: p = R·r（整数 lifting 近似，严格可逆于 liftResidualFromFrenet）。
MCGAL::PointInt liftResidualToFrenet(const MCGAL::PointInt& r,
                                     const FrenetFrame& fr) {
    double alpha = 0.0, beta = 0.0, gamma = 0.0;
    frenetEulerAngles(fr, alpha, beta, gamma);
    int x = r.x(), y = r.y(), z = r.z();
    lift2dForward(y, z, std::cos(gamma), std::sin(gamma));   // Rx(gamma)
    lift2dForward(x, z, std::cos(beta), -std::sin(beta));    // Ry(beta)
    lift2dForward(x, y, std::cos(alpha), std::sin(alpha));   // Rz(alpha)
    return MCGAL::PointInt(x, y, z);
}

/// inverse: r = R^-1·p（liftResidualToFrenet 的整数逆）。
MCGAL::PointInt liftResidualFromFrenet(const MCGAL::PointInt& p,
                                       const FrenetFrame& fr) {
    double alpha = 0.0, beta = 0.0, gamma = 0.0;
    frenetEulerAngles(fr, alpha, beta, gamma);
    int x = p.x(), y = p.y(), z = p.z();
    lift2dInverse(x, y, std::cos(alpha), std::sin(alpha));   // Rz^-1
    lift2dInverse(x, z, std::cos(beta), -std::sin(beta));    // Ry^-1
    lift2dInverse(y, z, std::cos(gamma), std::sin(gamma));   // Rx^-1
    return MCGAL::PointInt(x, y, z);
}

MCGAL::PointInt projectResidualToFrenet(const MCGAL::PointInt& r,
                                        MCGAL::Facet* f,
                                        const MCGAL::Point* bboxMin,
                                        float quantStep) {
    const FrenetFrame frame = residualFrenetFrame(f, bboxMin, quantStep);
    return liftResidualToFrenet(r, frame);
}

MCGAL::PointInt unprojectResidualFromFrenet(const MCGAL::PointInt& r,
                                            MCGAL::Facet* f,
                                            const MCGAL::Point* bboxMin,
                                            float quantStep) {
    const FrenetFrame frame = residualFrenetFrame(f, bboxMin, quantStep);
    return liftResidualFromFrenet(r, frame);
}

constexpr int kNumValenceCodes = 5;

/// stream 0: valence (5-元 + 4-context + V_OTHER inline 256-元 escape)
/// 字节布局: [int32 numPoly][int32 numOther][bytes RC]
std::vector<uint8_t> encodeValence5WithContext(const std::deque<uint8_t>& valCodes,
                                                const std::deque<uint8_t>& otherVals) {
    // Phase S2（R13/R14，与 rd 批同形）：方案可注入本条流的编码器。
    // 注入格式 [i32 numPoly][i32 numOther][injected 码流]；符号展平约定：
    // 按 valCodes 顺序逐符号，遇 VAL_OTHER 紧跟其 escape 字节（otherVals 序），
    // 共 numPoly + numOther 个 uint32 —— 解码侧严格逆变换。
    // 未注入时下面**原有代码一字不动** ⇒ 默认路径逐字节不变。
    if (auto injected = ::pmsf::injectedStreamCoder("valence")) {
        std::vector<std::uint32_t> syms;
        syms.reserve(valCodes.size() + otherVals.size());
        auto itOther = otherVals.begin();
        for (uint8_t v : valCodes) {
            syms.push_back(v);
            if (v == (uint8_t)RDValenceCode::VAL_OTHER) {
                syms.push_back(static_cast<std::uint32_t>(*itOther));
                ++itOther;
            }
        }
        std::vector<std::uint8_t> blob;
        injected->reset();
        injected->encode(syms, blob);
        std::vector<uint8_t> out(8 + blob.size());
        int32_t numPoly  = (int32_t)valCodes.size();
        int32_t numOther = (int32_t)otherVals.size();
        std::memcpy(out.data(),     &numPoly,  4);
        std::memcpy(out.data() + 4, &numOther, 4);
        if (!blob.empty()) std::memcpy(out.data() + 8, blob.data(), blob.size());
        return out;
    }
    // 未注入 ⇒ 交给公共核。其内容与本函数原先注入缝之后的实现**逐字相同**
    // （P3 抽取 2026-09-23；注入缝本身只有本算法用，按计划 §0 规则 3 留在算法目录）。
    return pmsf::encodeValence5(valCodes, otherVals);
}

/// stream 1 (PCLTTM Tag sv=2 专属): 4-元 RC for fusedFaceSym。
/// 字节布局: [int32 numSyms][bytes RC]
///
/// P12 侧流声明化：实现下沉到 `pmsf::encodeAlphabetStream`（P11 抽取的共享件）。
/// 此处只剩**声明**：字母表 = 4、流名 = "fusedFaceSym"（本算法确有注入缝）。
/// 共享件第三消费者的抗凑数依据见计划 §73.3。
std::vector<uint8_t> encodeFusedFaceSym4RC(const std::deque<uint8_t>& syms) {
    return pmsf::encodeAlphabetStream(syms, 4, "fusedFaceSym");
}


}  // namespace

std::vector<std::uint8_t> PCLTTMTagSymbolCollectOperator::encodeFrenetRound(std::size_t r) const {
    pmsf::pclttm_frenet::Layer layer{frenetDenominators_.at(r), {}};
    const auto& residuals=gateResidualQueues_.at(r);
    layer.coefficients.reserve(residuals.size());
    for (const auto& q:residuals) layer.coefficients.push_back(pmsf::pclttm_frenet::toCell(q));
    return pmsf::pclttm_frenet::encodeLayer(layer);
}

const pmsf::PMSFSymbolTable* PCLTTMTagSymbolCollectOperator::pmsfSymbolTable() const {
    if (frenetPatches_) {
        paperTableCache_.clear();
        for (std::size_t r=0;r<gateFusedFaceSymQueues_.size();++r) {
            auto& row=pmsf::appendPMSFSymbolRecord(paperTableCache_,0,0,static_cast<int>(r));
            row.delta.facetSymbols.assign(gateFusedFaceSymQueues_[r].begin(),gateFusedFaceSymQueues_[r].end());
            row.delta.vertexSymbols.assign(gateValenceQueues_[r].begin(),gateValenceQueues_[r].end());
            if (!gateOtherValQueues_[r].empty()) {
                row.delta.side[pmsf::pclttm_frenet::kValenceEscape].emplace_back(
                    std::vector<std::uint8_t>(gateOtherValQueues_[r].begin(),gateOtherValQueues_[r].end()));
            }
            row.delta.side[pmsf::pclttm_frenet::kGeometryStream].emplace_back(encodeFrenetRound(r));
        }
        return &paperTableCache_;
    }
    const pmsf::PMSFSymbolTable* base = RDGateOnlySymbolCollectOperator::pmsfSymbolTable();
    if (!base || base->empty()) {
        return nullptr;
    }
    pmsf::copyPMSFSymbolTableWithRowAugmentation(
        base, paperTableCache_, [](pmsf::PMSFSymbolRecord& row) {
        pmsf::clearPCLTTMTagInheritedTriangulationEdgeSymbols(row.delta);
    });
    const bool tagTableTriangulation = []() {
        const char* e = std::getenv("PMSF_PCLTTM_USE_TAG_TABLE_TRIANGULATION");
        return e && e[0] == '1';
    }();
    const bool frenetExact = rdtagFrenetExactEnabled();
    for (std::size_t r = 0; r < paperTableCache_.size() && r < gateFusedFaceSymQueues_.size(); ++r) {
        pmsf::appendPCLTTMTagRoundSchemaSideChannel(
            paperTableCache_[r].delta,
            gateFusedFaceSymQueues_[r],
            static_cast<std::int32_t>(
                rdtagFrenetRoundEnabled(static_cast<int>(r)) ? 1 : 0),
            static_cast<std::int32_t>(frenetExact ? 1 : 0),
            static_cast<std::int32_t>(tagTableTriangulation ? 1 : 0));

        if (r < gatePredC2Queues_.size()) {
            for (const auto& q : gatePredC2Queues_[r]) {
                pmsf::appendPCLTTMTagPredC2Symbol(
                    paperTableCache_[r].delta, q);
            }
        }
        if (r < gateFrenetCorrectionQueues_.size()) {
            for (const auto& q : gateFrenetCorrectionQueues_[r]) {
                pmsf::appendPCLTTMTagFrenetCorrectionSymbol(
                    paperTableCache_[r].delta, q);
            }
        }
    }
    return paperTableCache_.empty() ? nullptr : &paperTableCache_;
}

// ============================================================================
// export — ConnectivityBlobBuilder v3 (3 段流，sv=3, Phase 2)
//   stream 0: valence (5-元 + 4-context + V_OTHER inline 256-元 escape)
//   stream 1: fusedFaceSym (4-元 RC)
//   stream 2: residual (bit-plane RC)
//   [砍掉] edgeSym — decoder 按 v0-fan 拓扑推对角线（base_mesh.proxyHalfedge
//                    通过 reset_proxy_first 已对齐 encoder v0 语义）
// ============================================================================
int PCLTTMTagSymbolCollectOperator::exportToBufferWithRangeCoder(char* buffer,
                                                                  bool isEnableQuantization) {
    if (buffer == nullptr) return 0;
    requirePCLTTMTagPaperExactIfRequested();
    if (const char* tagTable = std::getenv("PMSF_PCLTTM_USE_TAG_TABLE_TRIANGULATION");
        tagTable && *tagTable && std::strcmp(tagTable, "0") != 0) {
        // CODEC_016/sv=3 has no edgeSym/diagonal stream; the decoder cannot
        // replay a non-fan tag-table triangulation (D25).  Fail fast instead
        // of emitting an artifact that decodes to exploded topology.
        throw std::runtime_error(
            "[PCLTTMTag] PMSF_PCLTTM_USE_TAG_TABLE_TRIANGULATION=1 is "
            "encoder-only and not decodable with BlobLayout=16/sv=3 (D25); "
            "unset it or use a layout that serializes diagonals.");
    }
    if (!enablePrediction_ || !isEnableQuantization) {
        throw std::runtime_error(
            "[PCLTTMTag] exportRC requires enablePrediction && enableQuantization.");
    }

    const int numRounds = (int)gateValenceQueues_.size();
    for (int r = 0; r < numRounds; ++r) {
        if (rdtagFrenetRoundEnabled(r) || rdtagFrenetExactEnabled()) {
            throw std::runtime_error(
                "[PCLTTMTag] BlobLayout=16/sv=3 stores Cartesian residuals only; "
                "PMSF_RDTAG_FRENET_* is not a decodable CODEC_016 mode. "
                "Use an RDTag/RDStrict path that serializes the Frenet mode, "
                "or leave the Frenet env disabled.");
        }
    }
    if ((int)gateIsPolyQueues_.size()        != numRounds ||
        (int)gateResidualQueues_.size()      != numRounds ||
        (int)gateFusedFaceSymQueues_.size()  != numRounds) {
        throw std::runtime_error(
            "[PCLTTMTag sv=3] inconsistent per-round queue sizes "
            "(expects valence/fusedFaceSym/residual aligned by round)");
    }

    for (int r = 0; r < numRounds; ++r) {
        if (gateFusedFaceSymQueues_[r].size() != gateIsPolyQueues_[r].size()) {
            throw std::runtime_error(
                "[PCLTTMTag sv=3] fusedFaceSym/isPoly count mismatch at round=" +
                std::to_string(r) +
                " fused=" + std::to_string(gateFusedFaceSymQueues_[r].size()) +
                " isPoly=" + std::to_string(gateIsPolyQueues_[r].size()));
        }
    }

    // ---- Phase 2 validation: edgeSym 8-元 pattern 分布诊断（opt-in） ----
    const char* envP2Val = std::getenv("PMSF_PHASE2_VALIDATE");
    if (envP2Val && envP2Val[0] == '1') {
        long long patCount[8] = {0};
        long long totalFaceGroups = 0, polyPat[8] = {0}, nonPolyPat[8] = {0};
        for (int r = 0; r < numRounds; ++r) {
            const auto& isPoly = gateIsPolyQueues_[r];
            const auto& edge = gateEdgeSymQueues_[r];
            const int F = (int)isPoly.size();
            for (int k = 0; k < F; ++k) {
                uint8_t b0 = edge[3*k + 0], b1 = edge[3*k + 1], b2 = edge[3*k + 2];
                int pat = (b0 << 2) | (b1 << 1) | b2;
                patCount[pat]++;
                totalFaceGroups++;
                if (isPoly[k]) polyPat[pat]++; else nonPolyPat[pat]++;
            }
        }
        std::cerr << "[PHASE2_VALIDATE edgeSym pattern] total=" << totalFaceGroups << std::endl;
        const char* names[8] = {"000","001","010","011","100","101","110","111"};
        for (int p = 0; p < 8; ++p) {
            if (patCount[p] == 0) continue;
            std::cerr << "  " << names[p]
                      << "  total=" << patCount[p]
                      << "  poly=" << polyPat[p]
                      << "  nonPoly=" << nonPolyPat[p] << std::endl;
        }
    }

    if (frenetPatches_ && frenetDenominators_.size()!=static_cast<std::size_t>(numRounds))
        throw std::runtime_error("PCLTTM Frenet layer directory size mismatch");
    pmsf::ConnectivityBlobBuilder cbb(pmsf::pclttmTagConnectivityBlobSpec(numRounds,
        frenetPatches_ ? pmsf::kPCLTTMTagFrenetVersion : pmsf::kPaperBlobSchemeVersionPCLTTMTag));

    int sumValence = 0, sumFused = 0, sumRes = 0;
    const int BP_BITS = (int)mesh_->i_nbQuantBits + 1;
    using Clock = std::chrono::steady_clock;
    const bool profile = pclttmProfileLogs();
    auto tExportStart = Clock::now();
    double valenceMs = 0.0;
    double fusedMs = 0.0;
    double residualMs = 0.0;
    double appendMs = 0.0;

    for (int i = 0; i < numRounds; ++i) {
        int er = numRounds - 1 - i;

        auto t = Clock::now();
        auto rcVal = encodeValence5WithContext(gateValenceQueues_[er],
                                                gateOtherValQueues_[er]);
        if (profile) {
            valenceMs += std::chrono::duration<double, std::milli>(
                             Clock::now() - t).count();
            t = Clock::now();
        }
        sumValence += (int)rcVal.size();
        cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamPCLTTMTagValence, rcVal);
        if (profile) {
            appendMs += std::chrono::duration<double, std::milli>(
                            Clock::now() - t).count();
            t = Clock::now();
        }

        auto rcFused = encodeFusedFaceSym4RC(gateFusedFaceSymQueues_[er]);
        if (profile) {
            fusedMs += std::chrono::duration<double, std::milli>(
                           Clock::now() - t).count();
            t = Clock::now();
        }
        sumFused += (int)rcFused.size();
        cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamPCLTTMTagFusedFaceSym, rcFused);
        if (profile) {
            appendMs += std::chrono::duration<double, std::milli>(
                            Clock::now() - t).count();
            t = Clock::now();
        }

        auto rcRes = frenetPatches_ ? encodeFrenetRound(er) :
            pmsf::rd_tag::encodeResidualBitplane(gateResidualQueues_[er], BP_BITS, "residual");
        if (profile) {
            residualMs += std::chrono::duration<double, std::milli>(
                              Clock::now() - t).count();
            t = Clock::now();
        }
        sumRes += (int)rcRes.size();
        cbb.appendStreamBytes(i, pmsf::kPaperBlobStreamPCLTTMTagResidual, rcRes);
        if (profile) {
            appendMs += std::chrono::duration<double, std::milli>(
                            Clock::now() - t).count();
        }
    }

    auto tFinalize = Clock::now();
    auto blob = cbb.finalize();
    auto tMemcpy = Clock::now();
    std::memcpy(buffer, blob.data(), blob.size());
    if (profile) {
        auto tDone = Clock::now();
        const double finalizeMs = std::chrono::duration<double, std::milli>(
                                      tMemcpy - tFinalize).count();
        const double memcpyMs = std::chrono::duration<double, std::milli>(
                                    tDone - tMemcpy).count();
        const double totalMs = std::chrono::duration<double, std::milli>(
                                   tDone - tExportStart).count();
        std::fprintf(stderr,
                     "[PCLTTM_EXPORT_PROFILE] rounds=%d valence=%.3fms fused=%.3fms "
                     "residual=%.3fms append=%.3fms finalize=%.3fms memcpy=%.3fms "
                     "total=%.3fms bytes=%zu\n",
                     numRounds, valenceMs, fusedMs, residualMs, appendMs,
                     finalizeMs, memcpyMs, totalMs, blob.size());
    }

    lastFusedFaceSymSize_ = sumFused;

    const char* envBpv = std::getenv("PMSF_BPV_DIAG");
    if (envBpv && envBpv[0] == '1') {
        int overhead = (int)blob.size() - sumValence - sumFused - sumRes;
        // 符号计数：与 RD_TAG 的同名诊断对齐，使两行的"每 BFS face 一条流"
        // 可以直接比个数（字节数只说明花多少，个数才说明方案发多少）。
        long long nFusedSyms = 0, nFusedActive = 0, nValenceSyms = 0,
                  nResidualCoords = 0;
        for (const auto& q : gateFusedFaceSymQueues_) {
            nFusedSyms += (long long)q.size();
            for (uint8_t s : q) nFusedActive += (s != 0 ? 1 : 0);
        }
        for (const auto& q : gateValenceQueues_) nValenceSyms += (long long)q.size();
        for (const auto& q : gateResidualQueues_)
            nResidualCoords += (long long)q.size() * 3;
        std::cerr << "[PCLTTMTag bpv sv=3] valence=" << sumValence
                  << " fusedFaceSym=" << sumFused
                  << " residual=" << sumRes
                  << " containerOverhead=" << overhead
                  << " totalBlob=" << blob.size()
                  << " rounds=" << numRounds << std::endl;
        const long long nVerts =
            (mesh_ != nullptr) ? (long long)mesh_->nb_vertices : 0;
        std::cerr << "[PCLTTMTag sym] fusedSyms=" << nFusedSyms
                  << " fusedActive=" << nFusedActive
                  << " valenceSyms=" << nValenceSyms
                  << " residualCoords=" << nResidualCoords
                  << " verts=" << nVerts;
        if (nVerts > 0) {
            std::cerr << " fusedSymsPerVert=" << (double)nFusedSyms / (double)nVerts;
        }
        std::cerr << std::endl;
    }

    return (int)blob.size();
}

int PCLTTMTagSymbolCollectOperator::exportToBuffer(char* /*buffer*/,
                                                    bool /*enableQuantization*/,
                                                    bool /*enableCompress*/) {
    throw std::runtime_error(
        "[PCLTTMTag] exportToBuffer (non-RC) is not implemented.");
}
