/*
 * @Author: koi
 * @Date: 2026-04-26
 * @Description:
 *   RD 压缩/解压共用的预测辅助：
 *     - 一阶 barycentric 预测（基于当前 polygon 面邻居的量化坐标均值）
 *     - **二阶 curvature 预测**（论文 Sect. 3.1, 公式 (2)）：
 *           r_c       = v_c_int - barycenter_int(N(c))         // 一阶残差
 *           r̂_c       = mean_{i ∈ N(c)} r_i                    // 二阶预测（邻居一阶残差均值）
 *           encoded   = r_c - r̂_c                              // 实际进入熵编码的量
 *       邻居的 r_i 通过 vertex 属性 "rd_r1" 共享给两端，编解码对称。
 *     - 编解码对称：对同一 face 邻居，编码端与解码端得到一致的 predictor
 *
 *   本工具只依赖 "face 的邻居顶点 point + 邻居 vertex 上的 rd_r1 属性" 以及 mesh 的量化参数，
 *   因此解码端在 insert_point 之前可复用同一份逻辑，不依赖被移除顶点的任何信息。
 */
#ifndef RD_PREDICTOR_H
#define RD_PREDICTOR_H

#include "ContextPool.h"
#include "Facet.h"
#include "Global.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "MMSEPredictor.h"  // MMSE paper predictor stack (decoder replay hook)
#include "Point.h"
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <vector>

namespace MCGAL {

/// vertex 属性名：每个顶点的"一阶残差 r1 = v_quant - barycenter_at_insert"，Vector3i 类型。
/// 编码端在 VertexRemovalEliminateOperator::remove_point 写入；
/// 解码端在 BasicReconstructOperator::insert_point 写入。
constexpr const char* kVPropRDR1 = "rd_r1";

/// 取 face 的所有边界顶点（即待插入中心点的 1-环）
inline std::vector<MCGAL::Vertex*> rdCollectPolygonVerts(MCGAL::Facet* face) {
    std::vector<MCGAL::Vertex*> verts;
    if (face == nullptr) return verts;
    MCGAL::Halfedge* st = face->proxyHalfedge();
    if (st == nullptr) return verts;
    // ── PHASE65/D18：**步数上限 + 断链保护** ─────────────────────────────
    //
    // 实测（D18，gdb `catch throw` 抓的分配点栈）：pclttm 解压 dragon 时，
    // `std::bad_alloc` 的分配点就在**本函数的 `push_back`**，而当时网格很**小**
    // （halfedge=18,581 / facets=10,266）——**不是结构爆炸，是这个环走不完**：
    // `next()` 链若**绕成一个不含 `st` 的循环**，`do/while (it != st)` 永不退出，
    // 于是以 **≈1.04 GB/s** 无界 push_back，直到 VmPeak 406 GB 撑爆内存。
    //
    // 这与附 88 的 `Facet::normalize_proxy_to_min_position_vertex` 活锁是
    // **同一个根因**（不闭合的半边环），只是那处表现为自旋、此处表现为爆内存。
    //
    // 修法与本仓库 idiom 一致（`pmsf::boundedRing(v, maxSteps=256)`）：
    // 加步数上限并顺带断链保护。正常面环必闭合 ⇒ 上限**永不触发** ⇒ 字节中性。
    constexpr int kMaxRingSteps = 256;
    MCGAL::Halfedge* it = st;
    bool closed = false;
    for (int steps = 0; steps < kMaxRingSteps; ++steps) {
        verts.push_back(it->vertex());
        MCGAL::Halfedge* nxt = it->next();
        if (nxt == nullptr || nxt == st) { closed = true; break; }
        it = nxt;
    }
    // ── PHASE67 物品5：**D18 探针**（bound-trip 处打 face id + 是否贴边界） ──
    // 在案探针方案：定位 dragon 上「第一处 next() 链不闭合」的 face，
    // 再看它是否贴着边界（dragon 有 4,474 条边界边；解码器零边界分支）。
    // 用 stderr 输出，不改返回值 ⇒ 对产物零影响（纯诊断）。
    if (!closed) {
        static int probeCount = 0;
        if (probeCount < 20) {          // 只打前 20 处，避免刷屏
            ++probeCount;
            // MCGAL 用 opposite()==nullptr 表示边界半边（Facet 无 isBoundary()）。
            // 统计本次走过的半边里有多少条是边界半边 ⇒ 直接回答「是否贴边界」。
            int nBoundary = 0;
            for (MCGAL::Vertex* v : verts) {
                MCGAL::Halfedge* h = v ? v->proxyHalfedge() : nullptr;
                if (h && h->opposite() == nullptr) ++nBoundary;
            }
            std::fprintf(stderr,
                "[D18-probe] ring NOT closed: face=%p proxy=%p nVerts=%zu "
                "st_opp_null=%d boundaryVerts=%d st_face=%p\n",
                static_cast<void*>(face), static_cast<void*>(st), verts.size(),
                ((st->opposite() == nullptr) ? 1 : 0), nBoundary,
                static_cast<void*>(st->face()));
            // ── PHASE70 物品7：**下一级探针**（在案方案的直接实现） ──────────
            // 上一级已把「边界路径」用数据否掉（boundaryVerts=0）。在案的下一步是：
            //   打印该 256 环上**全部半边的 face 指针** ——
            //   若这个环**跨了多个 face** ⇒ 候选原因 (1)「重建时环拼接错误」成立；
            //   若**全属同一 face**       ⇒ 候选原因 (2)「符号流误读」更可能。
            // 这里统计**不同 face 的个数**与前若干个样本（纯诊断，不改返回值）。
            MCGAL::Facet* firstFace = st->face();
            int distinctFaces = 0;
            MCGAL::Facet* seen[4] = {nullptr, nullptr, nullptr, nullptr};
            MCGAL::Halfedge* w = st;
            for (int k = 0; k < kMaxRingSteps; ++k) {
                MCGAL::Facet* f = w->face();
                bool known = false;
                for (MCGAL::Facet* s : seen) {
                    if (s == f) {
                        known = true;
                        break;
                    }
                }
                if (!known) {
                    if (distinctFaces < 4) seen[distinctFaces] = f;
                    ++distinctFaces;
                }
                MCGAL::Halfedge* n2 = w->next();
                if (n2 == nullptr || n2 == st) break;
                w = n2;
            }
            std::fprintf(stderr,
                "[D18-probe-2] ring face census: distinctFaces=%d firstFace=%p "
                "samples=[%p %p %p %p] => %s\n",
                distinctFaces, static_cast<void*>(firstFace),
                static_cast<void*>(seen[0]), static_cast<void*>(seen[1]),
                static_cast<void*>(seen[2]), static_cast<void*>(seen[3]),
                (distinctFaces > 1) ? "CROSS-FACE => ring splicing (setNext) error"
                                    : "SAME-FACE => symbol-stream misread likelier");
            // ── P29 物品3：**第三级探针**（沿「符号流误读」方向再推一级） ────
            // 二级已证：256 环**全在同一个 face 内**（20/20 SAME-FACE）⇒ 环拼接被否。
            // 存活解释是「该 face 内出现了不含 st 的子环」。子环有两种可能来源，
            // 必须分开，否则无法继续：
            //   (i)  **提前闭合**：next() 链回到了环上某个**非 st** 的半边 ⇒ 存在一个
            //        不含起点的环 ⇒ 某次 setNext 把一个半边的 next **指回了环内**
            //        （典型：重建时把「环的入口」写错）；
            //   (ii) **永不闭合**：256 步内**没有任何重复半边** ⇒ 链根本没成环
            //        （next 指向新半边/野指针），是更严重的结构性损坏。
            // 判据（预先登记）：报告**第一个重复出现的半边** h* 及其下标，以及
            // `h* == st` 与否。h*==st 本不该走到这里（closed 会为真）；
            // 故非 (i) 即 (ii)。
            MCGAL::Halfedge* ring[kMaxRingSteps];
            int nRing = 0;
            int firstRepeatIdx = -1;
            int firstRepeatOf = -1;
            {
                MCGAL::Halfedge* w2 = st;
                for (int k = 0; k < kMaxRingSteps; ++k) {
                    bool dup = false;
                    for (int j = 0; j < nRing; ++j) {
                        if (ring[j] == w2) {
                            dup = true;
                            firstRepeatIdx = k;
                            firstRepeatOf = j;
                            break;
                        }
                    }
                    if (dup) break;
                    ring[nRing++] = w2;
                    MCGAL::Halfedge* n3 = w2->next();
                    if (n3 == nullptr) break;
                    w2 = n3;
                }
            }
            std::fprintf(stderr,
                "[D18-probe-3] ring trace: walked=%d firstRepeatIdx=%d firstRepeatOf=%d "
                "ringLen=%d => %s\n",
                (firstRepeatIdx >= 0 ? firstRepeatIdx : nRing), firstRepeatIdx,
                firstRepeatOf, nRing,
                (firstRepeatIdx >= 0)
                    ? "EARLY-CLOSE => a non-st sub-cycle exists (setNext points back into the ring)"
                    : "NEVER-CLOSES => next() chain does not cycle at all (structural corruption)");

            // ── P30 物品3：**第四级探针** —— 把 2-环上每条半边「是谁」打出来 ────
            // 三级已证：链是 st → h1 → h2 → h1，即 {h1,h2} 自成一个 **2-环**，
            // 而 st 只是**指向**它、自己不在环上 ⇒ 「面指针与 next 链不一致」。
            // 现在要回答的是「谁写坏的」，故打印环上每条半边的
            //   (顶点对, face, 是否边界)，以及**顶点对是否与别的面重复**。
            // 判据（预先登记）：
            //   · 若 h1/h2 的顶点对**互为反向**（v_a→v_b 与 v_b→v_a）⇒ 这是一个
            //     **digon（2 边形）**，即重建时**多插了一个面**；
            //   · 若二者的 face 指针**相同**而 next 互指 ⇒ 环被**切断成两段**，
            //     是 `setNext` 把「回到环内某半边」写错的典型形态。
            for (int k = 0; k < nRing && k < 8; ++k) {
                MCGAL::Halfedge* h = ring[k];
                MCGAL::Vertex* v0 = h->vertex();
                MCGAL::Halfedge* nx = h->next();
                MCGAL::Vertex* v1 = nx ? nx->vertex() : nullptr;
                std::fprintf(stderr,
                    "[D18-probe-4] idx=%d he=%p v0=%d v1=%d face=%p opp_null=%d next=%p\n", k,
                    static_cast<void*>(h),
                    v0 ? v0->poolId() : -1, v1 ? v1->poolId() : -1,
                    static_cast<void*>(h->face()),
                    ((h->opposite() == nullptr) ? 1 : 0),
                    static_cast<void*>(nx));
            }
            // 反向判定：h1 与 h2 是否互为反向（digon 签名）
            if (nRing >= 3) {
                MCGAL::Halfedge* h1 = ring[1];
                MCGAL::Halfedge* h2 = ring[2];
                MCGAL::Halfedge* n1 = h1->next();
                MCGAL::Halfedge* n2 = h2->next();
                bool digon = (n1 == h2 && n2 == h1);
                std::fprintf(stderr,
                    "[D18-probe-4] 2-cycle check: h1->next==h2 && h2->next==h1 ? %s "
                    "=> %s\n",
                    digon ? "YES" : "no",
                    digon ? "DIGON: a face with exactly 2 halfedges (duplicate/degenerate face inserted)"
                          : "NOT a digon: the cycle was cut and re-spliced");
            }
        }
    }
    return verts;
}

/// 一阶 barycentric 预测（量化域）：返回 face 内所有邻居 **量化坐标** 的平均
/// 注：邻居顶点 point 是浮点，先 floatPosToInt 再取平均，避免解码端反量化误差。
inline MCGAL::PointInt rdBarycenterQuantized(MCGAL::Facet* face,
                                             std::shared_ptr<MCGAL::Mesh> mesh) {
    auto verts = rdCollectPolygonVerts(face);
    if (verts.empty() || mesh == nullptr) {
        return MCGAL::PointInt(0, 0, 0);
    }
    int64_t sx = 0, sy = 0, sz = 0;
    int n = 0;
    for (MCGAL::Vertex* v : verts) {
        MCGAL::PointInt pi = mesh->floatPosToInt(v->point());
        sx += pi.x();
        sy += pi.y();
        sz += pi.z();
        n++;
    }
    if (n == 0) return MCGAL::PointInt(0, 0, 0);
    // 向零舍入的整数平均（编解码对称），若希望更接近四舍五入可改为 (sx + n/2) / n
    int ax = static_cast<int>(sx / n);
    int ay = static_cast<int>(sy / n);
    int az = static_cast<int>(sz / n);
    return MCGAL::PointInt(ax, ay, az);
}

/// 二阶曲率预测（论文公式 (2)）：r̂_c = mean_{i ∈ N(c)} r1_i。
/// 邻居 r1_i 来自 vertex 属性 "rd_r1"；属性默认值 (0,0,0) 代表"未被预测过"或 base mesh 顶点。
inline MCGAL::PointInt rdCurvaturePrediction(MCGAL::Facet* face,
                                             int meshId) {
    // ------------------------------------------------------------------
    // MMSE paper predictor stack (Lee et al., ICIP 2011) — decoder replay.
    //
    // When the MMSE read operator has installed a replay round (the residual
    // stream is self-describing: the MMSE blob carries a per-cluster 2-bit
    // mode stream), the predictor correction for this polyFace comes from
    // src/predict/MMSEPredictor.h instead of the RD curvature predictor.
    // The encoder writes `encoded = r1 - correction` and the decoder performs
    // the exact inverse here, so both sides stay bit-symmetric.
    //
    // Default state is disabled => the frozen baselines are byte-identical.
    // ------------------------------------------------------------------
    if (pmsf::mmse_pred::replayState().enabled) {
        return pmsf::mmse_pred::replayCorrection(face);
    }

    auto verts = rdCollectPolygonVerts(face);
    if (verts.empty()) return MCGAL::PointInt(0, 0, 0);
    int64_t sx = 0, sy = 0, sz = 0;
    int n = 0;
    for (MCGAL::Vertex* v : verts) {
        MCGAL::PointInt& r1 =
            MCGAL::contextPool.property<MCGAL::PointInt>(meshId, MCGAL::kVPropRDR1, v);
        sx += r1.x();
        sy += r1.y();
        sz += r1.z();
        n++;
    }
    if (n == 0) return MCGAL::PointInt(0, 0, 0);
    const MCGAL::PointInt pred(static_cast<int>(sx / n),
                               static_cast<int>(sy / n),
                               static_cast<int>(sz / n));
    // P16（2026-09-25）探针：**只统计、不改行为**。判据用 —— 编码端没有做
    // `encoded = r_c - r̂_c` 的扣除，而解码端（PCLTTM reader）已经在算 r̂_c；
    // 若该路径上 r̂_c 恒等于 0，矛盾消解、round-trip 成立；若不为 0，
    // 说明两端语义已分叉。默认路径零影响。
        return pred;
}

/// 把量化整数坐标反量化为浮点 Point（与 readBaseMeshWithQuantization 内部约定一致）
inline MCGAL::Point rdIntToFloatPoint(const MCGAL::PointInt& pi,
                                      const MCGAL::Point& bboxMin,
                                      float quantStep) {
    MCGAL::Point p;
    p[0] = bboxMin.x() + (pi.x() + 0.5f) * quantStep;
    p[1] = bboxMin.y() + (pi.y() + 0.5f) * quantStep;
    p[2] = bboxMin.z() + (pi.z() + 0.5f) * quantStep;
    return p;
}

/// ZigZag 编码：有符号 -> 无符号，保持小幅值→小数
inline uint32_t rdZigZagEncode(int32_t v) {
    return (static_cast<uint32_t>(v) << 1) ^ static_cast<uint32_t>(v >> 31);
}

inline int32_t rdZigZagDecode(uint32_t z) {
    return static_cast<int32_t>((z >> 1) ^ (-(int32_t)(z & 1)));
}

}  // namespace MCGAL

#endif  // RD_PREDICTOR_H
