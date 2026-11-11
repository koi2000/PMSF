/*
 * @file ConquestDecodeLayer.cpp
 * @brief `ConquestDecodeLayer.h` 的实现。
 *
 * **本文件是"已验证解码实体"的唯一落点**（2026-09-22 从验证探针
 * `tools/ConquestLayerRoundTripProbe.cpp` 的 `decodeLayer` 机械搬迁而来）：
 * 判据路径一行未改，只剥掉了 `getenv` 开关与探针专用诊断收集器。
 *
 * 搬迁时删掉的都是**默认运行点下本就不执行**的东西：
 *   - `PMSF_CLR_RECOVER` 那一支（旧导航重放，默认关，未收敛）；
 *   - 真值一环那一支（`inverseRetriangulateWithRing`）—— 解码阶段 `rings` 恒空，
 *     该支从未执行（蓝图 §77）；
 *   - 各 `*DBG` / `*SEQ` 打印。
 *
 * ⚠️ **`inverseRetriangulate` 那一调用被原样保留**，尽管它在当前运行点下是
 * **空操作**（`removeEdges` 已把区域并成多边形，`trueFaceDeg(h) == valence`
 * ⇒ 它的 while 一次都不进）。保留它的理由是**出参 `regionFaces`**：它末尾会
 * 压入"最终留下来的那个多边形面"，而下游的标记循环
 * （`setFFlag(f, kFToBeRemoved)`）正是靠它把区域标记掉。
 * 实测切断这条标记路径会直接把对拍打到 `topoGeomOk=0`（蓝图 §78.2）。
 * 蓝图 §78.1 已直接量过：调用前后真实面度不变 ⇒ 并边侧确实无副作用。
 */
#include "conquest/ConquestDecodeLayer.h"
#include "conquest/ConquestReplayTrace.h"

#include "conquest/ConquestMeshPrimitives.h"
#include "conquest/ConquestTraversal.h"

#include <cmath>
#include <deque>
#include <stdexcept>
#include <limits>

namespace pmsf {
namespace {

/// @brief 绕 `v` 的**有界环**，写出邻居坐标与 id。
///
/// ⚠️ **刻意保留与编码端 `ringNeighbors` 不同的走法**（本函数是纯 ccw：
/// `it = opposite(it)->next()`）。迁移时**没有**把它换成 `ringNeighbors`，
/// 因为这一支是**已被验证过的那一支**，换掉就是改判据。
///
/// 两者在**闭合环**上访问同一组顶点（次序与旋转起点不同，而默认预测子是
/// 算术平均 ⇒ 次序无关）；在**不闭合环**（D17）上二者可能给出不同条数。
/// 编码端 `recordGeometry` 用的是 `ringNeighbors` —— 这处不对称是**已知的
/// 潜在隐患**，已作为待办记在蓝图里；在它被证伪或修正之前，不在这里改。
int ringOf(MCGAL::Vertex* v, double* pos, std::uint32_t* ids, int cap) {
    int n = 0;
    MCGAL::Halfedge* s0 = v ? v->proxyHalfedge() : nullptr;
    if (!s0)
        return 0;
    MCGAL::Halfedge* it = s0;
    for (int i = 0; i < cap; ++i) {
        MCGAL::Vertex* w = it->end_vertex();
        pos[3 * n] = w ? w->x() : 0.0;
        pos[3 * n + 1] = w ? w->y() : 0.0;
        pos[3 * n + 2] = w ? w->z() : 0.0;
        ids[n] = w ? w->poolId() : 0u;
        ++n;
        MCGAL::Halfedge* o = it->opposite();
        if (!o)
            break;
        it = o->next();
        if (!it || it == s0)
            break;
    }
    return n;
}

}  // namespace

ConquestDecodeResult decodeLayer(MCGAL::Mesh* mesh,
                                 ConquestState& st,
                                 MCGAL::Halfedge* seed,
                                 const std::vector<int>& syms,
                                 const std::vector<std::array<int, 3>>& deltas,
                                 const FrontierConquestEngine::QuantGrid& grid,
                                 const EngineHooks& hooks,
                                 std::FILE* predTrace,
                                 std::vector<ConquestDeferredGeometry>* deferredGeometry,
                                 ConquestDecodeControl* control) {
    ConquestDecodeResult res;
    if (deferredGeometry) deferredGeometry->clear();
    if (control && control->trace) control->trace->start(seed);
    if (control && control->feasibleMasks) control->feasibleMasks->clear();
    if (control && (!control->patternsByType || control->patternsByType->size()!=17))
        throw std::runtime_error("conquest controlled decode: missing patch patterns");
    std::deque<MCGAL::Halfedge*> q;

    st.reset();                // 每次进层复位（= 参考实现的 Init()），与编码端一致
    st.setFlag(tgt(seed), 1);  // CONQUERED
    st.setSign(tgt(seed), 1);  // PLUS
    st.setFlag(tgt(opp(seed)), 1);
    st.setSign(tgt(opp(seed)), 2);  // MINUS
    q.push_back(seed);

    const GeometryPredictor& geom = hooks.geometryOrDefault();
    const ResidualQuantizer& quant = hooks.quantizerOrDefault();
    const GatePriorityPolicy& gatePrio = hooks.gatePriorityOrDefault();

    std::size_t si = 0;
    while (!q.empty()) {
        MCGAL::Halfedge* h = popGate(q, gatePrio, st);
        if (!h)
            continue;

        // ⚠️ **不检查 `h->isRemoved()`**：编码端也不检查 —— 它只查
        // `st.fflag(h->face())`，对已移除半边读到的是陈旧但**确定**的值。
        // 在这里加一条比编码端更严的护栏，会让两侧不再同构，
        // 反而把"解码端与编码端逐字一样吗"这个问题搅浑（本轮踩过）。
        // ⚠️ flag 跳过是**必需**的：实测关掉它，第 4 个 gate 就
        // `inverseRetriangulate 失败`（区域不再是那批三角形）。
        if (control && control->trace) control->trace->pop(h,st,q.size());
        const std::uint8_t ff = st.fflag(h->face());
        if (ff == kFConquered || ff == kFToBeRemoved)
            continue;

        if (si >= syms.size()) {
            if(control)res.ok=false; // packed layers retain their trailing nulls
            break;
        }
        auto feasible=[&](int code) {
            const int type=findType(st,h,code+3);
            return type>0&&discoverConquestPatch(h,st,control->patternsByType->at(type));
        };
        if(control && control->feasibleMasks) {
            std::uint8_t mask=0;
            for(int code=0;code<4;++code)if(feasible(code))mask|=static_cast<std::uint8_t>(1u<<code);
            control->feasibleMasks->push_back(mask);
        }
        int s=syms[si];
        if(control && (s<0||s>4)) {res.ok=false;break;}
        if(control && s<4 && !feasible(s)) {
            if(!control->packed) {res.ok=false;break;}
            s=4;++res.inferredNulls; // failed discovery leaves the valence queued
        } else ++si;
        ++res.expandedSymbols;
        if(control && res.expandedSymbols>control->expandedSymbols) {res.ok=false;break;}
        if(control && control->trace)control->trace->symbol(s);
        MCGAL::Vertex* vFront = tgt(nxt(h));

        if (s >= 0 && s <= 3) {
            // ---- 删点分支 ----
            // 编码端此处做的是「erase_center_vertex 后再 retriangulate」。
            // 解码端必须**严格逆序**：先把区域并回多边形，再跑标志/入队循环，
            // 最后才插回中心点 —— 因为编码端那个循环是在**多边形**上跑的
            // （erase 之后、retriangulate 之前）。
            const int valence = s + 3;
            std::vector<MCGAL::Facet*> regionFaces;
            const int gt = findType(st, h, valence);
            if (!removeEdges(mesh, h, gt)) {
                res.ok = false;
                break;
            }
            // sign 写入：与并边同函数（参考的做法），**直接用原 gate h**。
            writeRemoveEdgesSigns(h, gt, [&](MCGAL::Vertex* v, int sg) {
                if (v)
                    st.setSign(v, (std::uint8_t)sg);
            });
            // 见文件头的说明：本调用在当前运行点下是空操作，保留它是为了
            // 出参 `regionFaces`（它承载"最终留下来的那个多边形面"）。
            inverseRetriangulate(mesh, h, valence, &regionFaces);

            MCGAL::Halfedge* gw = h;
            // 编码端此处先标 kFTemp，**随后 `retriangulate` 把它覆盖成
            // kFToBeRemoved**（每个 type 分支都写 h->face()）。解码端不跑
            // retriangulate（三角形已在），必须直接落到**净效果** kFToBeRemoved，
            // 否则这个面不会被后续 gate 跳过 ⇒ 队列比编码端长（实测 107 条）。
            if (h->face())
                st.setFFlag(h->face(), kFToBeRemoved);
            for (int j = 0; j < valence - 1; ++j) {
                gw = nxt(gw);
                if (!gw)
                    break;
                if (tgt(gw))
                    st.setFlag(tgt(gw), 1);
                if (tgt(opp(gw)))
                    st.setFlag(tgt(opp(gw)), 1);
                if (!isBorderEdge(gw))
                    q.push_back(opp(gw));
            }
            // 编码端 retriangulate 标记的 V-2 个面 = 它的**输出** = 解码端的**输入**
            // （就是刚被并回区域的那批面）。逐支标记数完全吻合，见蓝图 §9.5。
            for (MCGAL::Facet* f : regionFaces)
                st.setFFlag(f, kFToBeRemoved);

            std::vector<MCGAL::Facet*> newFaces;
            MCGAL::Vertex* nv = addCenterVertex(mesh, h, &newFaces);
            // 对称假设：后续 gate 看到的是本端的**输出**面（addCenterVertex
            // 新造的 V 个），编码端看到的是它自己的输出（V-2 个），
            // 两者都被标记才可能跳过一致。
            for (MCGAL::Facet* f : newFaces)
                st.setFFlag(f, kFToBeRemoved);
            if (!nv) {
                res.ok = false;
                break;
            }
            if (control && control->trace) control->trace->bind(nv);

            // ---- 几何重建：CRV = BC + Dist（整数域恒等，P13.2 已单独验过）----
            if (deferredGeometry) {
                if (res.inserted >= static_cast<int>(deltas.size())) {
                    res.ok = false;
                    break;
                }
                ConquestDeferredGeometry record;
                record.center = nv;
                record.residual = deltas[res.inserted];
                MCGAL::Halfedge* ring[256];
                const int count = ringNeighbors(nv, ring, 256);
                if (count != valence) ++res.ringMismatch;
                for (int i = 0; i < count; ++i) record.ring.push_back(tgt(ring[i]));
                deferredGeometry->push_back(std::move(record));
            } else if (grid.valid() && res.inserted < (int)deltas.size()) {
                double nb[3 * 256];
                std::uint32_t ids[256];
                // ★ **用与编码端 `recordGeometry` 同一个取环函数**。
                //
                // 原先这里用纯 ccw 的 `ringOf`，编码端用 `ringNeighbors`（先 cw 后 ccw）。
                // 两者给的**集合**相同（RINGDIAG：33237 次插入逐次一致），
                // 但**次序不同** ⇒ 算术平均的**浮点求和次序**不同 ⇒
                // 在恰落在格边界上的点上翻格。实测：bunny 单层两侧 `pred` 从**第 1 个点**
                // 就差一格（1059 vs 1058），整体 5.3% 的重插点差一格。
                // 用同一个函数就消除了这一整类差异。
                MCGAL::Halfedge* hs[256];
                const int hn = ringNeighbors(nv, hs, 256);
                const int n = hn;
                for (int i = 0; i < hn; ++i) {
                    MCGAL::Vertex* w = tgt(hs[i]);
                    nb[3 * i] = w ? w->x() : 0.0;
                    nb[3 * i + 1] = w ? w->y() : 0.0;
                    nb[3 * i + 2] = w ? w->z() : 0.0;
                    ids[i] = w ? w->poolId() : 0u;
                }
                if (n != valence)
                    ++res.ringMismatch;

                // 诊断（零默认影响，只多算不输出）：把编码端那套 `ringNeighbors`
                // 也算一遍，比对**条数**与**算术平均**。见蓝图 §79.5 / §90.4。
                {
                    MCGAL::Halfedge* hs[256];
                    const int m = ringNeighbors(nv, hs, 256);
                    if (m != n) {
                        ++res.ringRuleDiff;
                    } else {
                        double sx = 0, sy = 0, sz = 0, tx = 0, ty = 0, tz = 0;
                        for (int i = 0; i < m; ++i) {
                            MCGAL::Vertex* w = tgt(hs[i]);
                            if (!w)
                                continue;
                            sx += w->x();
                            sy += w->y();
                            sz += w->z();
                        }
                        for (int i = 0; i < n; ++i) {
                            tx += nb[3 * i] / 1.0;
                            ty += nb[3 * i + 1] / 1.0;
                            tz += nb[3 * i + 2] / 1.0;
                        }
                        if (m > 0) {
                            const double dx = std::fabs(sx / m - tx / n);
                            const double dy = std::fabs(sy / m - ty / n);
                            const double dz = std::fabs(sz / m - tz / n);
                            const double d = std::sqrt(dx * dx + dy * dy + dz * dz);
                            if (d > res.ringMeanDiffMax)
                                res.ringMeanDiffMax = d;
                        }
                    }
                }

                PatchView pv;
                pv.neighborPos = nb;
                pv.neighborCount = n;
                pv.quantStep = grid.step;
                pv.bboxMin = grid.bboxMin;
                pv.neighborIds = ids;
                // 本端网格内的中心点编号（刚由 addCenterVertex 分配；不做跨端
                // 相等假设，见 PatchView::centerId 注释）。
                pv.centerId = nv->poolId();

                double pred[3];
                geom.predict(pv, pred);
                if (predTrace) {
                    std::fprintf(predTrace, "%d %d %d\n", quant.toCell(pred[0], grid.bboxMin[0], grid.step),
                                 quant.toCell(pred[1], grid.bboxMin[1], grid.step),
                                 quant.toCell(pred[2], grid.bboxMin[2], grid.step));
                }
                int d[3];
                quant.inverseTransform(deltas[res.inserted].data(), d, pv);
                double p3[3];
                for (int k = 0; k < 3; ++k) {
                    const int cell = quant.toCell(pred[k], grid.bboxMin[k], grid.step) + d[k];
                    // ★ 落到**容器那个世界**：与 base mesh 读回时同一条公式。
                    // base mesh 的顶点是"量化号 → float 公式"得到的；
                    // 若这里用 `quant.toWorld`（double），重插点与存活点就分处**两套末位**
                    // ⇒ 下一层的 pred 由两者混合而成 ⇒ 逐层累积（实测 bunny 未配对
                    // 0→2→11→20→49→70）。这是纯精度对齐，数学上同一个式子。
                    const float bm = (float)grid.bboxMin[k];
                    const float st = (float)grid.step;
                    p3[k] = (double)(bm + ((float)cell + 0.5f) * st);
                }
                // ⚠️ 必须是 `setPoint`。MCGAL 的 Vertex 把坐标存在 `v[0..2]`，
                // `point()`/`setPoint()`/`x()` 读写的是它；而 `setVertexPoint()`
                // 写的是**另一个字段** `vertexPoint_`，`x()` 根本不读 ——
                // 用了它几何就**静默地一点没写进去**（实测：前端点位置序列在
                // 第一次插入处跳到 0，而我自己打印的中间量 p3 却有值）。
                nv->setPoint(MCGAL::Point(p3[0], p3[1], p3[2]));
            }
            st.setFlag(nv, 1);  // 编码端此点已不存在 ⇒ 对解码端等价于"已征服"
            ++res.inserted;
        } else {
            // ---- null-patch 分支 ----
            if (h->face())
                st.setFFlag(h->face(), kFConquered);
            st.setFlag(vFront, 1);
            nullPatchTail(st, h, q);
            ++res.nullPatch;
        }
    }

    res.consumed = si;
    if(control && (si!=syms.size() || res.expandedSymbols!=control->expandedSymbols ||
       res.inserted!=static_cast<int>(deltas.size())))res.ok=false;
    return res;
}

ConquestDecodeResult decodeCleaningLayer(MCGAL::Mesh* mesh, ConquestState& st,
    MCGAL::Halfedge* seed, const std::vector<int>& symbols,
    const std::vector<std::array<int,3>>& residuals,
    std::vector<ConquestDeferredGeometry>& geometry) {
    ConquestDecodeResult result;
    geometry.clear();
    if (!mesh || !seed || !opp(seed)) { result.ok=false; return result; }
    st.reset();
    st.setFlag(tgt(seed),1);st.setFlag(tgt(opp(seed)),1);
    std::deque<MCGAL::Halfedge*> pending{seed};
    while (!pending.empty()) {
        auto* h=pending.front();pending.pop_front();
        if (!h) continue;
        const auto flag=st.fflag(h->face());
        if (flag==kFConquered || flag==kFToBeRemoved) continue;
        if (result.consumed==symbols.size()) {result.ok=false;break;}
        const int symbol=symbols[result.consumed++];
        if (symbol==1) {
            st.setFFlag(h->face(),kFConquered);st.setFlag(tgt(nxt(h)),1);
            if (!isBorderEdge(nxt(h))) pending.push_back(opp(nxt(h)));
            if (!isBorderEdge(prv(h))) pending.push_back(opp(prv(h)));
            ++result.nullPatch;
            continue;
        }
        if (symbol!=0 || trueFaceDeg(h)!=3 || geometry.size()>=residuals.size()) {result.ok=false;break;}
        auto* center=addCenterVertex(mesh,h);
        if (!center) {result.ok=false;break;}
        ConquestDeferredGeometry record;record.center=center;record.residual=residuals[geometry.size()];
        MCGAL::Halfedge* ring[256];const int count=ringNeighbors(center,ring,256);
        if (count!=3) {result.ok=false;++result.ringMismatch;break;}
        for (int i=0;i<count;++i) record.ring.push_back(tgt(ring[i]));
        geometry.push_back(std::move(record));
        st.setFlag(center,1);st.setFFlag(h->face(),kFConquered);
        auto* g=prv(opp(nxt(h)));
        for (int side=0;side<2;++side) {
            if (!g || !opp(g)) {result.ok=false;break;}
            st.setFlag(tgt(opp(g)),1);st.setFFlag(g->face(),kFConquered);
            if (!isBorderEdge(prv(g))) {
                auto* outside=opp(prv(g));
                st.setFFlag(outside->face(),kFConquered);st.setFlag(tgt(nxt(outside)),1);
                if (!isBorderEdge(nxt(outside))) pending.push_back(opp(nxt(outside)));
                if (!isBorderEdge(prv(outside))) pending.push_back(opp(prv(outside)));
            }
            g=prv(opp(g));
        }
        if (!result.ok)break;
        ++result.inserted;
    }
    if (result.consumed!=symbols.size() || geometry.size()!=residuals.size()) result.ok=false;
    return result;
}

ConquestGeometryRestoreResult restoreDeferredConquestGeometry(const std::vector<ConquestDeferredGeometry>& records,
                                     const FrontierConquestEngine::QuantGrid& grid,
                                     const EngineHooks& hooks) {
    if (!grid.valid() || !std::isfinite(grid.step) || !std::isfinite(grid.bboxMin[0]) ||
        !std::isfinite(grid.bboxMin[1]) || !std::isfinite(grid.bboxMin[2])) {
        throw std::runtime_error("deferred geometry: invalid quantization grid");
    }
    const auto& quantizer = hooks.quantizerOrDefault();
    ConquestGeometryRestoreResult result;
    for (const auto& record : records) {
        if (!record.center || record.center->isRemoved()) throw std::runtime_error("deferred geometry: dead/null center");
        std::vector<double> neighbors;
        std::vector<std::uint32_t> ids;
        for (auto* vertex : record.ring) {
            if (!vertex || vertex->isRemoved()) throw std::runtime_error("deferred geometry: dead/null ring vertex");
            neighbors.insert(neighbors.end(), {vertex->x(), vertex->y(), vertex->z()});
            ids.push_back(static_cast<std::uint32_t>(vertex->poolId()));
        }
        PatchView patch;
        patch.neighborPos = neighbors.data();patch.neighborIds = ids.data();
        patch.neighborCount = static_cast<int>(ids.size());patch.centerId = record.center->poolId();
        patch.bboxMin = grid.bboxMin;patch.quantStep = grid.step;
        int predictionCells[3];
        if (!hooks.geometryOrDefault().predictQuantized(patch, predictionCells)) {
            double prediction[3];hooks.geometryOrDefault().predict(patch, prediction);
            for (int axis = 0; axis < 3; ++axis) predictionCells[axis] = quantizer.toCell(prediction[axis], grid.bboxMin[axis], grid.step);
        }
        int residual[3];quantizer.inverseTransform(record.residual.data(), residual, patch);
        double position[3];
        for (int axis = 0; axis < 3; ++axis) {
            const auto wideCell = static_cast<std::int64_t>(predictionCells[axis]) + residual[axis];
            if (wideCell < std::numeric_limits<std::int32_t>::min() || wideCell > std::numeric_limits<std::int32_t>::max()) {
                throw std::runtime_error("deferred geometry: reconstructed cell overflow");
            }
            const auto cell = static_cast<std::int32_t>(wideCell);
            if (predictionCells[axis] < 0) ++result.negativePredictions;
            foldConquestGeometryCell(result.cellSignature, cell);
            position[axis] = static_cast<float>(grid.bboxMin[axis]) +
                (static_cast<float>(cell) + 0.5f) * static_cast<float>(grid.step);
        }
        record.center->setPoint(MCGAL::Point(position[0], position[1], position[2]));
    }
    return result;
}

}  // namespace pmsf
