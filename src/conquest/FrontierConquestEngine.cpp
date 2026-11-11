/*
 * @file FrontierConquestEngine.cpp
 * @brief 前沿式顶点删减引擎的实现（算法无关）。
 *
 * 逐行对照外部参考实现翻译，逐符号对拍通过后迁移进来。本文件里每一处
 * **反直觉**的地方都写了「为什么」，因为那些正是对拍过程中花了最多时间才定位的。
 */

#include "conquest/FrontierConquestEngine.h"

#include "conquest/ConquestTraversal.h"

#include <algorithm>
#include <cmath>
#include <deque>

namespace pmsf {

// ---- 流形性判据 ------------------------------------------------------------
// 判据表（参照参考实现逐 type 读出；**环心是按 type 变的**，这是最容易错的一处）：
//   valence 4: type 5/8  -> circ(target(opp(h))) ∋ P[1]
//              type 6/7  -> circ(target(h))      ∋ P[2]
//   valence 5: type 9/12 -> circ(target(opp(h))) ∋ P[1];
//                           circ(target(next(opp(next(h))))) ∋ P[3]
//              type 10   -> circ(target(h))      ∋ P[3];
//                           circ(target(next(opp(next(h))))) ∋ P[3]
//              type 11   -> circ(target(h))      ∋ P[2]; circ(target(opp(h))) ∋ P[2]
//   valence 6: type 13/16 -> circ(target(opp(h))) ∋ P[1];
//                            circ(target(opp(h))) ∋ P[3];
//                            circ(target(next(opp(next(h))))) ∋ P[3]
//              type 14/15 -> circ(target(h))      ∋ P[2]; circ(target(h)) ∋ P[4];
//                            circ(target(next(opp(prev(h))))) ∋ P[2]
bool FrontierConquestEngine::manifoldViolated(MCGAL::Halfedge* h, int type, int valence) {
    if (valence == 3)
        return false;  // 不插新边，恒安全

    // P[0] 取 **target(h)**（不是 target(next(h))）；绕行起点是 next(h)，只走一步。
    std::vector<int> P(valence, -1);
    MCGAL::Halfedge* g = h;
    if (!g)
        return false;
    P[0] = tgt(g) ? (int)tgt(g)->poolId() : -1;
    g = nxt(g);
    for (int i = 1; i < valence; ++i) {
        if (!g)
            break;
        g = prv(opp(g));
        if (!g)
            break;
        MCGAL::Vertex* src = g->vertex();
        P[i] = src ? (int)src->poolId() : -1;
    }

    // `circ(e) ∋ k`：绕 **target(e)** 一圈，看是否有**邻居**的编号 == P[k]。
    // ⚠️ 必须取邻居的编号，不能取环心自己的 —— 后者会让判据退化成
    // "环心 == P[k]"，几乎恒假 ⇒ **系统性漏判** ⇒ 该发 null patch(4) 的地方
    // 发了删除 ⇒ 后续重三角化切出已存在的对角线。（P13.1 的真实根因之一。）
    auto circHits = [&P](MCGAL::Halfedge* e, int k) -> bool {
        if (!e || k < 0 || k >= (int)P.size() || P[k] < 0)
            return false;
        MCGAL::Vertex* c = tgt(e);
        if (!c)
            return false;
        MCGAL::Halfedge* hs[256];
        const int n = ringNeighbors(c, hs, 256);
        for (int i = 0; i < n; ++i) {
            MCGAL::Vertex* nb = tgt(hs[i]);
            if (nb && (int)nb->poolId() == P[k])
                return true;
        }
        return false;
    };

    if (valence == 4) {
        if (type == 5 || type == 8)
            return circHits(opp(h), 1);
        if (type == 6 || type == 7)
            return circHits(h, 2);
    } else if (valence == 5) {
        if (type == 9 || type == 12)
            return circHits(opp(h), 1) || circHits(nxt(opp(nxt(h))), 3);
        if (type == 10)
            return circHits(h, 3) || circHits(nxt(opp(nxt(h))), 3);
        if (type == 11)
            return circHits(h, 2) || circHits(opp(h), 2);
    } else if (valence == 6) {
        if (type == 13 || type == 16)
            return circHits(opp(h), 1) || circHits(opp(h), 3) || circHits(nxt(opp(nxt(h))), 3);
        if (type == 14 || type == 15)
            return circHits(h, 2) || circHits(h, 4) || circHits(nxt(opp(prv(h))), 2);
    }
    return false;
}

// ---- 重三角化 --------------------------------------------------------------
// 每个 type 的切分导航必须与参考实现**位精确**一致：新边连的是
// `target(h)` 与 `target(g)`。切分后 `g` 的重新取位（`next(next(h))` 等）
// 是这里最容易写错的地方 —— 参考实现在调用重三角化前有一句 `g = h`，
// 把 g 拨回原始 gate；只读调用行、不读紧邻的重新赋值行，就会得到假修正。
void FrontierConquestEngine::retriangulate(MCGAL::Mesh* mesh, ConquestState& st, MCGAL::Halfedge* ch, int valence, int type) {
    MCGAL::Halfedge* h = ch;
    auto split = [&](MCGAL::Halfedge* a2, MCGAL::Halfedge* b2) -> MCGAL::Halfedge* { return mesh->split_facet(a2, b2); };
    MCGAL::Halfedge* g = nxt(h);

    if (type == 1 || type == 2 || type == 4) {
        st.setFFlag(h->face(), kFToBeRemoved);
        st.setSign(tgt(g), 1);
    } else if (type == 3) {
        st.setFFlag(h->face(), kFToBeRemoved);
        st.setSign(tgt(g), 2);
    } else if (type == 5 || type == 8) {
        st.setSign(tgt(g), 1);
        st.setSign(tgt(nxt(g)), 2);
        h = prv(h);
        h = split(h, g);
        st.setFFlag(h->face(), kFToBeRemoved);
        st.setFFlag(opp(h)->face(), kFToBeRemoved);
    } else if (type == 6 || type == 7) {
        st.setSign(tgt(g), 2);
        st.setSign(tgt(nxt(g)), 1);
        g = nxt(g);
        h = split(h, g);
        st.setFFlag(h->face(), kFToBeRemoved);
        st.setFFlag(opp(h)->face(), kFToBeRemoved);
    } else if (type == 9 || type == 12) {
        st.setSign(tgt(g), 1);
        st.setSign(tgt(nxt(g)), 2);
        st.setSign(tgt(nxt(nxt(g))), 1);
        h = prv(h);
        h = split(h, g);
        st.setFFlag(opp(h)->face(), kFToBeRemoved);
        g = nxt(nxt(h));
        h = split(h, g);
        st.setFFlag(h->face(), kFToBeRemoved);
        st.setFFlag(opp(h)->face(), kFToBeRemoved);
    } else if (type == 10) {
        st.setSign(tgt(g), 1);
        st.setSign(tgt(nxt(g)), 2);
        st.setSign(tgt(nxt(nxt(g))), 1);
        g = h;
        h = prv(prv(h));
        h = split(h, g);
        st.setFFlag(opp(h)->face(), kFToBeRemoved);
        g = nxt(h);
        h = prv(h);
        h = split(h, g);
        st.setFFlag(h->face(), kFToBeRemoved);
        st.setFFlag(opp(h)->face(), kFToBeRemoved);
    } else if (type == 11) {
        st.setSign(tgt(g), 2);
        st.setSign(tgt(nxt(g)), 1);
        st.setSign(tgt(nxt(nxt(g))), 2);
        g = nxt(g);
        h = split(h, g);
        st.setFFlag(opp(h)->face(), kFToBeRemoved);
        g = nxt(nxt(h));
        h = split(h, g);
        st.setFFlag(h->face(), kFToBeRemoved);
        st.setFFlag(opp(h)->face(), kFToBeRemoved);
    } else if (type == 13 || type == 16) {
        st.setSign(tgt(g), 1);
        st.setSign(tgt(nxt(g)), 2);
        st.setSign(tgt(nxt(nxt(g))), 1);
        st.setSign(tgt(nxt(nxt(nxt(g)))), 2);
        h = prv(h);
        h = split(h, g);
        st.setFFlag(opp(h)->face(), kFToBeRemoved);
        g = nxt(nxt(h));
        h = split(h, g);
        st.setFFlag(opp(h)->face(), kFToBeRemoved);
        g = nxt(nxt(h));
        h = split(h, g);
        st.setFFlag(h->face(), kFToBeRemoved);
        st.setFFlag(opp(h)->face(), kFToBeRemoved);
    } else if (type == 14 || type == 15) {
        st.setSign(tgt(g), 2);
        st.setSign(tgt(nxt(g)), 1);
        st.setSign(tgt(nxt(nxt(g))), 2);
        st.setSign(tgt(nxt(nxt(nxt(g)))), 1);
        g = nxt(g);
        h = split(h, g);
        st.setFFlag(opp(h)->face(), kFToBeRemoved);
        g = nxt(nxt(h));
        h = split(h, g);
        st.setFFlag(opp(h)->face(), kFToBeRemoved);
        g = nxt(nxt(h));
        h = split(h, g);
        st.setFFlag(h->face(), kFToBeRemoved);
        st.setFFlag(opp(h)->face(), kFToBeRemoved);
    }
    // type == 0：Find_Type 未覆盖的组合。参考实现下不应出现；出现即说明
    // sign 传播与参考不符 —— 记录下来，便于定位。
    else {
        // type == 0：Find_Type 未覆盖的 sign 组合。默认运行点下不应出现；
        // 出现即说明 sign 传播与参考不一致，交由调用方决定怎么处理（引擎不打印）。
        ++unhandledType0_;
    }
}

// ---- 几何 ------------------------------------------------------------------
// `Dist = 量化(被删点) − 量化(预测位置)`，**整数域**相减。解码侧
// `位置 = 预测 + Dist` 精确还原 ⇒ 预测子/变换只影响熵，不影响重建精度。
void FrontierConquestEngine::recordGeometry(MCGAL::Mesh* mesh, MCGAL::Vertex* vdel, GeometrySink* geometry) {
    if (!geometry || !grid_.valid() || !vdel)
        return;

    const ResidualQuantizer& q = hooks().quantizerOrDefault();

    double nb[3 * 256];
    std::uint32_t ids[256];
    MCGAL::Halfedge* hs[256];
    const int n = ringNeighbors(vdel, hs, 256);
    // ★ 邻居坐标取**解码端会拿到的那串**，不是本端手里的那串。
    //
    // 编码端此刻用的是自己网格里的坐标；解码端那一刻用的是 base mesh
    // **写出去又读回来**的坐标。两者在末位上可能不同 ⇒ 算术平均落在格边界时翻格
    // ⇒ 重插点差一格（实测：pred 轨迹从第 1 个点就分开，14.8% 的行不同）。
    // 这里显式做一次同样的往返，使两侧的**输入**逐位一致
    // —— 即 §86「直接用对方那套」从**网格坐标**推进到 **pred 的输入**。
    for (int i = 0; i < n; ++i) {
        MCGAL::Vertex* v = tgt(hs[i]);
        if (v) {
            // ⚠️ **直接用网格里那一串**，不要再套一次 `intPosToFloat`。
            // 实测（§94）：套了以后读数**一字未变** —— 因为网格早已被吸附过；
            // 而吸附改走"解码端那条 float 公式"之后，这里再套 double 版
            // 反而会把两侧又拉开。两侧要一致，靠的是**吸附**那一步对齐公式。
            nb[3 * i + 0] = v->x();
            nb[3 * i + 1] = v->y();
            nb[3 * i + 2] = v->z();
        } else {
            nb[3 * i + 0] = nb[3 * i + 1] = nb[3 * i + 2] = 0.0;
        }
        ids[i] = v ? v->poolId() : 0u;
    }

    PatchView pv;
    pv.neighborPos = nb;
    pv.neighborCount = n;
    pv.quantStep = grid_.step;
    pv.bboxMin = grid_.bboxMin;
    pv.neighborIds = ids;
    // 本端网格内的中心点编号（不做跨端相等假设，见 PatchView::centerId 注释）。
    pv.centerId = vdel->poolId();

    double pred[3];
    hooks().geometryOrDefault().predict(pv, pred);

    if (predTrace_) {
        std::fprintf(predTrace_, "%d %d %d\n", q.toCell(pred[0], grid_.bboxMin[0], grid_.step),
                     q.toCell(pred[1], grid_.bboxMin[1], grid_.step), q.toCell(pred[2], grid_.bboxMin[2], grid_.step));
    }

    const double src[3] = {vdel->x(), vdel->y(), vdel->z()};
    int delta[3];
    for (int k = 0; k < 3; ++k) {
        const int a = q.toCell(src[k], grid_.bboxMin[k], grid_.step);
        const int b = q.toCell(pred[k], grid_.bboxMin[k], grid_.step);
        delta[k] = a - b;
    }
    int out[3];
    q.transform(delta, out, pv);
    geometry->onRemovedVertex(out);
}

// ---- 删减层 ----------------------------------------------------------------
ConquestLayerResult FrontierConquestEngine::decimateLayer(MCGAL::Mesh* mesh,
                                                          ConquestState& st,
                                                          MCGAL::Halfedge* seed,
                                                          std::vector<int>* connOut,
                                                          GeometrySink* geometry) {
    ConquestLayerResult res;
    std::deque<MCGAL::Halfedge*> q;

    st.reset();                // 每次进层复位（= 参考实现的 Init()）。漏掉会让第 2 层起删不动点。
    st.setFlag(tgt(seed), 1);  // CONQUERED
    st.setSign(tgt(seed), 1);  // PLUS
    st.setFlag(tgt(opp(seed)), 1);
    st.setSign(tgt(opp(seed)), 2);  // MINUS
    q.push_back(seed);

    const DecimationPredicate& pred = hooks().decimationOrDefault();
    const GatePriorityPolicy& gatePrio = hooks().gatePriorityOrDefault();
    const SymbolPostProc& post = hooks().symbolPostProcOrDefault();

    auto emit = [&](int sym) {
        if (connOut)
            connOut->push_back(sym);
        post.onSymbol(0, (std::uint32_t)sym);
        ++res.symbolsEmitted;
    };

    while (!q.empty()) {
        MCGAL::Halfedge* h = popGate(q, gatePrio, st);
        if (!h)
            continue;

        const std::uint8_t ff = st.fflag(h->face());
        if (ff == kFConquered || ff == kFToBeRemoved)
            continue;

        MCGAL::Halfedge* nh = nxt(h);
        MCGAL::Vertex* vFront = tgt(nh);
        if (!vFront)
            continue;

        // ⚠️ 必须每次现算：`Vertex::vertex_degree()` 在 MCGAL 里读的是**缓存**，
        // 而 `split_facet` / `erase_center_vertex` 都不维护它（契约把维护责任
        // 交给调用方）。参考实现用的是"每次现算"的 `out_degree`。
        // 依赖该缓存会让**跨层** valence 系统性错误 —— 第 0 层网格干净时看不出来。
        vFront->recompute_degree();

        FrontVertexView view;
        view.valence = vFront->vertex_degree();
        view.isBorder = isBorderVertex(nh);
        view.conquered = (st.flag(vFront) != 0);
        view.id = vFront->poolId();
        
        const size_t qBeforeTrace = q.size();   // 诊断：本 gate 的 push 条数

        if (pred.canDecimate(view)) {
            const int valence = view.valence;
            const int type = findType(st, h, valence);
            if (!manifoldViolated(h, type, valence)) {
                                ++res.removedVertices;
                emit(valence - 3);

                recordGeometry(mesh, vFront, geometry);

                MCGAL::Halfedge* g = h;
                mesh->erase_center_vertex(nxt(g));

                g = h;
                if (g->face())
                    st.setFFlag(g->face(), kFTemp);
                for (int j = 0; j < valence - 1; ++j) {
                    g = nxt(g);
                    if (!g)
                        break;
                    if (tgt(g))
                        st.setFlag(tgt(g), 1);
                    if (tgt(opp(g)))
                        st.setFlag(tgt(opp(g)), 1);
                    if (!isBorderEdge(g))
                        q.push_back(opp(g));
                }
                g = h;
                retriangulate(mesh, st, h, valence, type);
                (void)g;
                if (gateTrace_) {
                    std::fprintf(gateTrace_, "%.17g %.17g %.17g DELETE %zu", vFront->x(), vFront->y(),
                                 vFront->z(), q.size() - qBeforeTrace);
                    for (size_t k = qBeforeTrace; k < q.size(); ++k) {
                        MCGAL::Vertex* w = tgt(nxt(q[k]));
                        std::fprintf(gateTrace_, " (%.17g,%.17g,%.17g)", w ? w->x() : -9.0, w ? w->y() : -9.0,
                                     w ? w->z() : -9.0);
                    }
                    std::fprintf(gateTrace_, "\n");
                }
            } else {
                // null patch：条件不满足，不删点，但仍要扩张前沿。
                                emit(4);
                if (h->face())
                    st.setFFlag(h->face(), kFConquered);
                st.setFlag(vFront, 1);
                // 两路 sign 传播 + **两个**队列 push（参考实现里这条尾部与
                // "最终 else" 分支是复制粘贴的孪生体，逐字相同）。
                MCGAL::Vertex* vA = tgt(h);
                MCGAL::Vertex* vB = tgt(opp(h));
                MCGAL::Vertex* vF = tgt(nxt(h));
                const std::uint8_t sA = st.sign(vA), sB = st.sign(vB);
                if (sA == 1 && sB == 2)
                    st.setSign(vF, 1);
                else if (sA == 2 && sB == 1)
                    st.setSign(vF, 1);
                else if (sA == 1 && sB == 1)
                    st.setSign(vF, 2);
                else if (sA == 2 && sB == 2)
                    st.setSign(vF, 1);
                if (!isBorderEdge(nxt(h)))
                    q.push_back(opp(nxt(h)));
                if (!isBorderEdge(prv(h)))
                    q.push_back(opp(prv(h)));
                if (gateTrace_) {
                    std::fprintf(gateTrace_, "%.17g %.17g %.17g NULL %zu", vFront->x(), vFront->y(),
                                 vFront->z(), q.size() - qBeforeTrace);
                    for (size_t k = qBeforeTrace; k < q.size(); ++k) {
                        MCGAL::Vertex* w = tgt(nxt(q[k]));
                        std::fprintf(gateTrace_, " (%.17g,%.17g,%.17g)", w ? w->x() : -9.0, w ? w->y() : -9.0,
                                     w ? w->z() : -9.0);
                    }
                    std::fprintf(gateTrace_, "\n");
                }
                continue;
            }
        } else {
            // 兜底分支（参考实现里这一支对**带边界**点发 `valence + 2`，
            // 兜底分支。参考实现里这一支对**带边界**点发 `valence + 2`，
            // 但那只在 `Check_border_structure` 为真时；该条件在默认运行点下
            // 落到 false ⇒ 发 4。见 P13_1 文档 §14.2。
                        emit(4);
            if (h->face())
                st.setFFlag(h->face(), kFConquered);
            st.setFlag(vFront, 1);
            // ⚠️ **这里必须做与 null-patch 分支逐字相同的尾部**（4 路 sign 传播
            // + 两个队列 push）。参考实现里"manifold 违规"与"最终 else"两条分支
            // 的尾部是复制粘贴的孪生体，**逐字相同**。
            //
            // 我第一版只写了两个 push、**漏了 sign 传播**，后果是灾难性的：
            // sign 不再传播 ⇒ 后续所有 `findType` 退化成 0 ⇒ 重三角化一个都不做
            // ⇒ 拓扑与验证器全面分岔。而**这一层的符号流看不出问题**
            // （本支符号恒为 4，与 sign 无关），直到下一层 valence 对不上才暴露。
            nullPatchTail(st, h, q);
            if (gateTrace_) {
                std::fprintf(gateTrace_, "%.17g %.17g %.17g NULL %zu", vFront->x(), vFront->y(), vFront->z(),
                             q.size() - qBeforeTrace);
                for (size_t k = qBeforeTrace; k < q.size(); ++k) {
                    MCGAL::Vertex* w = tgt(nxt(q[k]));
                    std::fprintf(gateTrace_, " (%.17g,%.17g,%.17g)", w ? w->x() : -9.0, w ? w->y() : -9.0,
                                 w ? w->z() : -9.0);
                }
                std::fprintf(gateTrace_, "\n");
            }
        }
    }
    return res;
}

// ---- 清理层 ----------------------------------------------------------------
// 只处理 valence == 3 的前端点；**不做可删性判据**（参考实现在这一层的
// 流形判据是被注释掉的：度为 3 的点删除不插新边，本来也不会破坏流形性）。
// 两段式：遍历阶段**只打标记**，真正的删点在清扫阶段按**顶点迭代顺序**做。
ConquestLayerResult FrontierConquestEngine::regulateLayer(MCGAL::Mesh* mesh,
                                                          ConquestState& st,
                                                          MCGAL::Halfedge* seed,
                                                          std::vector<int>* connOut,
                                                          GeometrySink* geometry) {
    ConquestLayerResult res;
    std::deque<MCGAL::Halfedge*> q;

    st.reset();
    st.setFlag(tgt(seed), 1);
    st.setFlag(tgt(opp(seed)), 1);
    q.push_back(seed);

    const SymbolPostProc& post = hooks().symbolPostProcOrDefault();
    auto emit = [&](int sym) {
        if (connOut)
            connOut->push_back(sym);
        post.onSymbol(0, (std::uint32_t)sym);
        ++res.symbolsEmitted;
    };

    // ---- Phase A：遍历 + 只打标记 ----
    while (!q.empty()) {
        MCGAL::Halfedge* h = q.front();
        q.pop_front();
        if (!h)
            continue;
        MCGAL::Halfedge* nh = nxt(h);
        MCGAL::Vertex* vFront = tgt(nh);
        if (!vFront)
            continue;
        vFront->recompute_degree();
        const int valence = vFront->vertex_degree();

        const std::uint8_t ff = st.fflag(h->face());
        if (ff == kFConquered || ff == kFToBeRemoved)
            continue;

        FrontVertexView view;
        view.valence=valence;view.isBorder=isBorderVertex(nh);view.conquered=st.flag(vFront)!=0;view.id=vFront->poolId();
        if (!view.conquered && valence == 3 && !view.isBorder && hooks().decimationOrDefault().canRegulate(view)) {
            ++res.removedVertices;
            emit(0);  // 符号 0 = 删除（清理层是**二元**字母表）

            // 几何在 **Phase A** 记录：此时网格还是本轮清扫之前的状态。
            // 若挪到 Phase B，前面的 victim 已被删 ⇒ 预测子会不一致。
            recordGeometry(mesh, vFront, geometry);

            MCGAL::Halfedge* g = nh;
            st.setFlag(tgt(g), kVMoved);
            if (g->face())
                st.setFFlag(g->face(), kFToBeRemoved);

            g = prv(opp(g));
            st.setFlag(tgt(opp(g)), 1);
            if (g->face())
                st.setFFlag(g->face(), kFToBeRemoved);
            if (!isBorderEdge(prv(g))) {
                MCGAL::Halfedge* h1 = opp(prv(g));
                if (h1->face())
                    st.setFFlag(h1->face(), kFConquered);
                st.setFlag(tgt(nxt(h1)), 1);
                if (!isBorderEdge(nxt(h1)))
                    q.push_back(opp(nxt(h1)));
                if (!isBorderEdge(prv(h1)))
                    q.push_back(opp(prv(h1)));
            }
            g = prv(opp(g));
            st.setFlag(tgt(opp(g)), 1);
            if (g->face())
                st.setFFlag(g->face(), kFToBeRemoved);
            if (!isBorderEdge(prv(g))) {
                MCGAL::Halfedge* h2 = opp(prv(g));
                if (h2->face())
                    st.setFFlag(h2->face(), kFConquered);
                st.setFlag(tgt(nxt(h2)), 1);
                if (!isBorderEdge(nxt(h2)))
                    q.push_back(opp(nxt(h2)));
                if (!isBorderEdge(prv(h2)))
                    q.push_back(opp(prv(h2)));
            }
        } else {
            emit(1);  // 符号 1 = null
            if (h->face())
                st.setFFlag(h->face(), kFConquered);
            st.setFlag(vFront, 1);
            if (!isBorderEdge(nh))
                q.push_back(opp(nh));
            if (!isBorderEdge(prv(h)))
                q.push_back(opp(prv(h)));
        }
    }

    // ---- Phase B：按**顶点迭代顺序**删掉所有标记 ----
    std::vector<MCGAL::Vertex*> victims;
    for (MCGAL::Vertex* v : mesh->vertices())
        if (v && !v->isRemoved() && st.flag(v) == kVMoved)
            victims.push_back(v);
    for (MCGAL::Vertex* v : victims) {
        if (v->isRemoved())
            continue;
        // 需要一条 **target == v** 的入边。`boundedRing` 式的出边圈取 opposite 即可。
        MCGAL::Halfedge* start = v->proxyHalfedge();
        if (!start)
            continue;
        MCGAL::Halfedge* in = nullptr;
        MCGAL::Halfedge* it = start;
        for (int i = 0; i < 256; ++i) {
            MCGAL::Halfedge* o = it->opposite();
            if (o && tgt(o) == v) {
                in = o;
                break;
            }
            MCGAL::Halfedge* nx = it->opposite();
            if (!nx)
                break;
            it = nx->next();
            if (!it || it == start)
                break;
        }
        if (in)
            mesh->erase_center_vertex(in);
    }
    return res;
}

}  // namespace pmsf
