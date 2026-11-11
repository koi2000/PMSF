/*
 * @file POMARCustomizer.h
 * @brief POMAR 论文 "最远 v2 + 法向 OK 检查" 的 entry customizer，被
 *        POMARCompress.cpp / POMARV2Compress.cpp / POMARColumnarCompress.cpp 共享。
 */
#pragma once

#include "Facet.h"
#include "common/BoundedRingTraversal.h"
#include "Halfedge.h"
#include "MeshUtils.h"
#include "Vertex.h"
#include "common/HalfedgeErrorEntry.h"
#include "selection/SelectOperator.h"

namespace pomar_impl {

inline bool pomarEntryCustomizer(HalfedgeErrorEntry& entry, MCGAL::Halfedge* edge) {
    MCGAL::Vertex* vh_from = edge->vertex();
    MCGAL::Vertex* vh_to = edge->end_vertex();
    unsigned i_nbFaces = vh_from->vertex_degree();
    MCGAL::Vertex* faceVertices[i_nbFaces][3];
    MCGAL::Vector3 faceNormals[i_nbFaces];
    // ── PHASE64：**边界扇保护**（修 dragon 上的段错误） ────────────────────
    //
    // 实测（附 85）：dragon dec/enc 上 POMAR 在下面 `hehIt2->next()` 处 SIGSEGV，
    // core 里 `hehIt = 0x0, i = 2` —— 即第 3 次扇步时 `find_prev()->opposite()`
    // 已经返回空。根因是**表示差异**：
    //   - 参考实现（OpenMesh，meshDecimaterCost.cpp:109-127）用 `prev/opposite`
    //     两步绕顶点，OpenMesh 的边界是**真实半边环** ⇒ 永不返回 invalid；
    //   - MCGAL 用 `opposite_ == nullptr` 表示边界 ⇒ 同一趟走法在边界处会拿到 null。
    //
    // 计数也吻合：边界顶点**度 = 面数 + 1**，而 `i_nbFaces = vertex_degree()`
    // 取的是**度** ⇒ 循环比真实扇面**多跑一次**，那一次正落在边界上。
    //
    // ⚠️ 为什么门禁一直没抓到：三个门禁网格（bunny / LetterA / fandisk）**全是
    // 水密网格**（实测边界边 = 0 / 0 / 0），边界路径从未被走到；dragon 有
    // 4,474 条边界边、3,693 个度≥3 的边界顶点 —— 这正是失败随规模才显形的原因。
    //
    // 语义：走到空（或空面）= 扇面已尽 ⇒ **停**。记下**真实走过的面数**，
    // 供后面那趟法向检查使用。
    //
    // ⚠️ 字节中性（可证）：水密网格上永不为空 ⇒ `nbFacesSeen == i_nbFaces`
    // 恒成立，下面两个循环与改动前**逐次迭代一一对应**。
    MCGAL::Halfedge* hehIt = edge;
    unsigned nbFacesSeen = 0;
    for (unsigned i = 0; i < i_nbFaces; ++i) {
        if (hehIt == nullptr || hehIt->face() == nullptr)
            break;  // 到边界，扇面已尽
        MCGAL::Halfedge* hehIt2 = hehIt;
        for (unsigned j = 0; j < 3; ++j) {
            faceVertices[i][j] = hehIt2->end_vertex();
            hehIt2 = hehIt2->next();
        }
        MCGAL::Facet* fh = hehIt->face();
        faceNormals[i] = fh->normal();
        ++nbFacesSeen;
        hehIt = hehIt->find_prev();
        hehIt = hehIt->opposite();
    }
    // `i + 1 < nbFacesSeen` 与原文 `i < i_nbFaces - 1` 在水密时等价；
    // 且当 nbFacesSeen == 0/1 时**无下溢**（原文的无符号下溢会变成死循环）。
    for (unsigned i = 1; i + 1 < nbFacesSeen; ++i) {
        faceVertices[i][2] = vh_to;
        MCGAL::Vector3 normal = MCGAL::calculateFaceNormal(
            faceVertices[i][0], faceVertices[i][1], faceVertices[i][2]);
        if (!areNormalOk(normal, faceNormals[i])) {
            entry.edge->b_collapseOk = false;
            continue;
        }
    }
    MCGAL::Vertex* v2 = nullptr;
    float f_maxLen = 0;
    for (MCGAL::Halfedge* h_it : pmsf::boundedRing(vh_from)) {
        MCGAL::Vertex* vv_it = h_it->end_vertex();
        float f_len = (vh_to->point() - vv_it->point()).length();
        if (vv_it != vh_to && !vv_it->isConnected(vh_to) &&
            (!(v2 != nullptr) || (f_len > f_maxLen))) {
            if (f_len > f_maxLen) {
                v2 = vv_it;
                f_maxLen = f_len;
            }
        }
    }
    if (v2 == nullptr) { entry.edge->b_collapseOk = false; return false; }
    entry.error = f_maxLen;
    entry.customData = v2;
    if (!MCGAL::new_is_collapse_ok(edge)) { entry.edge->b_collapseOk = false; return false; }
    entry.edge->b_collapseOk = true;
    return true;
}

}  // namespace pomar_impl
