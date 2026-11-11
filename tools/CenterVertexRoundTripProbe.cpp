// SPDX-License-Identifier: Apache-2.0
//
// tools/CenterVertexRoundTripProbe.cpp —— P13.4 (C) 第一步的探针。
//
// 目标：为 `addCenterVertex`（MCGAL 没有，要自建）建立一个**互逆性测试**。
// 现状：探针已能跑到 erase 侧并给出自洽的 V/E/F（见下），add 侧待实现。
//
// 已验证（cow_simp）：目标 v4 度 4，环 {33,34,35,111}；
//   erase 前 V=148 F=292（E=438）；erase 后 V=147 F=289（E=434）。
//   F 减 3 = 4 个三角形塌成 1 个多边形；E 减 4 = 去掉 4 条邻边。Euler 自洽。
//
// 待做：实现 addCenterVertex，然后断言环逐顶点还原（排序后相同）、
//       V/E/F 与 χ 回到原值。
//
// 候选逆公式（由 remove_tip/insert_tip 的函数体手推，**未经测试**）：
//   remove_tip(h): h.next = X.opposite.next，其中 X = h.next()（remove 前）
//   逆:  X.opposite.setNext(h.next());  h.setNext(X);
//   —— 注意 insert_tip 的两个赋值对象都不对，所以必须自己写两级 setNext。
#include "conquest/ConquestMeshPrimitives.h"
#include "Facet.h"
#include "Global.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Point.h"
#include "Vertex.h"
#include "core/MeshRuntime.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <vector>

using pmsf::addCenterVertex;
using pmsf::edgeExists;
using pmsf::inverseRetriangulate;
using pmsf::trueFaceDeg;

int livingV(MCGAL::Mesh* m) {
    int n = 0;
    for (MCGAL::Vertex* v : m->vertices())
        if (v && !v->isRemoved())
            ++n;
    return n;
}
int livingF(MCGAL::Mesh* m) {
    int n = 0;
    for (MCGAL::Facet* f : m->faces())
        if (f && !f->isRemoved())
            ++n;
    return n;
}
// 绕 v 的出边环（有界），返回邻居顶点号
std::vector<unsigned> ring(MCGAL::Vertex* v) {
    std::vector<unsigned> out;
    MCGAL::Halfedge* s = v ? v->proxyHalfedge() : nullptr;
    if (!s)
        return out;
    MCGAL::Halfedge* it = s;
    for (int i = 0; i < 256; ++i) {
        out.push_back(it->end_vertex() ? it->end_vertex()->poolId() : 0u);
        MCGAL::Halfedge* o = it->opposite();
        if (!o)
            break;
        it = o->next();
        if (!it || it == s)
            break;
    }
    std::sort(out.begin(), out.end());
    return out;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "用法: %s <mesh.off>\n", argv[0]);
        return 1;
    }
    pmsf::ensureMeshRuntimeInitialized();
    auto mesh = std::make_shared<MCGAL::Mesh>(MCGAL::Mesh(std::string(argv[1])));
    std::fprintf(stderr, "[RT] V=%d F=%d\n", livingV(mesh.get()), livingF(mesh.get()));

    // 取第 K 个度∈[4,6] 的点做实验（K 由 PMSF_RT_K 指定；每点一个进程 ⇒ 全新 poolId 池）
    int skipLeft = 0;
    if (const char* e = std::getenv("PMSF_RT_K"))
        skipLeft = std::atoi(e);
    const int kIdx = skipLeft;
    MCGAL::Vertex* target = nullptr;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved())
            continue;
        v->recompute_degree();
        const int dg = v->vertex_degree();
        if (dg >= 4 && dg <= 6) {
            if (skipLeft > 0) {
                --skipLeft;
                continue;
            }
            target = v;
            break;
        }
    }
    if (!target) {
        std::fprintf(stderr, "[RT] 没找到可用点\n");
        return 1;
    }
    std::fprintf(stderr, "[RT] VID=%u VAL=%d\n", target->poolId(), target->vertex_degree());
    const int vPre = livingV(mesh.get()), fPre = livingF(mesh.get());
    std::fprintf(stderr, "[RT] 目标 v%u 度=%d\n", target->poolId(), target->vertex_degree());
    auto before = ring(target);
    std::fprintf(stderr, "[RT] 前环:");
    for (auto x : before)
        std::fprintf(stderr, " %u", x);
    std::fprintf(stderr, "\n");

    // 入边：target==v 的一条
    MCGAL::Halfedge* in = nullptr;
    {
        MCGAL::Halfedge* s = target->proxyHalfedge();
        MCGAL::Halfedge* it = s;
        for (int i = 0; i < 256; ++i) {
            MCGAL::Halfedge* o = it->opposite();
            if (o && o->end_vertex() == target) {
                in = o;
                break;
            }
            MCGAL::Halfedge* nx = it->opposite();
            if (!nx)
                break;
            it = nx->next();
            if (!it || it == s)
                break;
        }
    }
    if (!in) {
        std::fprintf(stderr, "[RT] 找不到入边\n");
        return 1;
    }

    // 记录 erase 前的环半边（用于逆）
    std::vector<MCGAL::Halfedge*> ringHe;
    {
        MCGAL::Halfedge* s = target->proxyHalfedge();
        MCGAL::Halfedge* it = s;
        for (int i = 0; i < 256; ++i) {
            ringHe.push_back(it);
            MCGAL::Halfedge* o = it->opposite();
            if (!o)
                break;
            it = o->next();
            if (!it || it == s)
                break;
        }
    }
    std::fprintf(stderr, "[RT] ringHe 条数=%zu\n", ringHe.size());

    mesh->erase_center_vertex(in);
    const int vErase = livingV(mesh.get()), fErase = livingF(mesh.get());  // 切分前的基准
    std::fprintf(stderr, "[RT] erase 后 V=%d F=%d  v%u removed=%d\n", livingV(mesh.get()), livingF(mesh.get()), target->poolId(),
                 (int)target->isRemoved());

    // ---- 链条：polygon → split（编码端重三角化）→ join（解码端逆）→ add ----
    {
        MCGAL::Halfedge* poly = nullptr;
        for (MCGAL::Facet* f : mesh->faces())
            if (f && !f->isRemoved() && trueFaceDeg(f->proxyHalfedge()) != 3) {
                poly = f->proxyHalfedge();
                break;
            }
        if (!poly) {
            std::fprintf(stderr, "[RT] 找不到多边形面\n");
            return 1;
        }
        const int deg = trueFaceDeg(poly);
        std::fprintf(stderr, "[RT] 多边形度=%d\n", deg);

        // 记录多边形上各边的 poolId（**切分前**）
        std::vector<int> polyIds;
        {
            MCGAL::Halfedge* it = poly;
            for (int i = 0; i < 256; ++i) {
                polyIds.push_back((int)it->poolId());
                it = it->next();
                if (!it || it == poly)
                    break;
            }
        }
        // 同时记录全局最大 poolId（切分前）
        long long maxIdBefore = 0;
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved())
                continue;
            MCGAL::Halfedge* it = f->proxyHalfedge();
            for (int i = 0; i < 256 && it; ++i) {
                maxIdBefore = std::max(maxIdBefore, (long long)it->poolId());
                it = it->next();
                if (!it || it == f->proxyHalfedge())
                    break;
            }
        }

        // 编码端：切 deg-3 刀（quadr=1 / penta=2 / hexa=3），每刀削掉一个三角形
        MCGAL::Halfedge* newEdge = nullptr;
        for (int k = 0; k < deg - 3; ++k) {
            MCGAL::Halfedge* cur = nullptr;
            for (MCGAL::Facet* f : mesh->faces())
                if (f && !f->isRemoved() && trueFaceDeg(f->proxyHalfedge()) != 3) {
                    cur = f->proxyHalfedge();
                    break;
                }
            if (!cur)
                break;
            const int d = trueFaceDeg(cur);
            // 沿面旋转起点，取第一条**不构成已存在弦**的对角线（v_i, v_{i+2}）。
            // 不这么做会在有弦的多边形上撞 `Halfedge::reset` 的自动配对断言
            // （实测 K=7 / VID=14 度 4 必崩）。真实编码器的 retriangulate 有
            // manifoldViolated 把关做同一件事。
            MCGAL::Halfedge* start = cur;
            bool okChord = false;
            for (int t = 0; t < d; ++t) {
                MCGAL::Vertex* a = start->vertex();
                MCGAL::Vertex* b = start->next() ? start->next()->next() ? start->next()->next()->vertex() : nullptr : nullptr;
                if (a && b && !edgeExists(a, b)) {
                    okChord = true;
                    break;
                }
                start = start->next();
                if (!start)
                    break;
            }
            if (!okChord) {
                std::fprintf(stderr, "[RT] 多边形上找不到非弦对角线\n");
                break;
            }
            MCGAL::Halfedge* hh = start;
            for (int i = 0; i < d - 1; ++i)
                hh = hh->next();  // prev(start)
            MCGAL::Halfedge* gg = start->next();
            newEdge = mesh->split_facet(hh, gg);
            if (!newEdge)
                break;
        }
        // 切分后多边形仍在（度数降为 3 说明切够了）：把 gate 重新指到它
        {
            MCGAL::Halfedge* cur = nullptr;
            for (MCGAL::Facet* f : mesh->faces())
                if (f && !f->isRemoved() && trueFaceDeg(f->proxyHalfedge()) != 3) {
                    cur = f->proxyHalfedge();
                    break;
                }
            if (cur)
                poly = cur;
        }
        const int vS = livingV(mesh.get()), fS = livingF(mesh.get());
        std::fprintf(stderr, "[RT] split 后 V=%d F=%d 新边=%s\n", vS, fS, newEdge ? "有" : "无");
        if (newEdge) {
            std::fprintf(stderr, "[RT] **高水位检验**: 新边 halfedge poolId=%d ; 切分前全局最大=%lld ; 多边形各边=%d", (int)newEdge->poolId(),
                         maxIdBefore, polyIds.empty() ? -1 : polyIds[0]);
            for (size_t i = 1; i < polyIds.size(); ++i)
                std::fprintf(stderr, ",%d", polyIds[i]);
            std::fprintf(stderr, "\n[RT]   新边 id > 全局最大? %s  ⇒ 高水位可判内边? %s\n",
                         ((long long)newEdge->poolId() > maxIdBefore) ? "**是**" : "否（假设不成立）",
                         ((long long)newEdge->poolId() > maxIdBefore) ? "倾向可用" : "**证伪**");
        }
        if (!newEdge) {
            std::fprintf(stderr, "[RT] split 没返回边\n");
            return 1;
        }

        // 解码端：逆切分。**用判据版**，不依赖 split 的返回值 ——
        // 因为真实解码端拿不到那条指针（它只握着 gate），
        // 而 type 13/16 的多次切分让"记住指针"这条路走不通。
        (void)newEdge;
        const bool okInv = inverseRetriangulate(mesh.get(), poly, deg);
        const int vJ = livingV(mesh.get()), fJ = livingF(mesh.get());
        MCGAL::Halfedge* jr = poly;
        std::fprintf(stderr, "[RT] inverseRetriangulate = %s\n", okInv ? "成功" : "**失败**");
        std::fprintf(stderr, "[RT] join  后 V=%d F=%d (切分前基准 V=%d F=%d, 切了%d刀) 还原=%s\n", vJ, fJ, vErase, fErase, deg - 3,
                     (vJ == vErase && fJ == fErase) ? "是" : "**否**");
        std::fprintf(stderr, "[RT] join 后真实面度=%d (期望 %d) [存储计数=%d]\n", trueFaceDeg(jr), deg, jr->face() ? jr->face()->facet_degree() : -1);

        // ⚠️ 必须在 add 之前取：addCenterVertex 会复用 jr 所在的面当 T_0，
        // 之后 trueFaceDeg(jr) 恒为 3（踩过：这里错一度得出假的 degOk=0）
        const bool degOk = (trueFaceDeg(jr) == deg);

        // 再插回中心点
        MCGAL::Vertex* nc = addCenterVertex(mesh.get(), jr);
        if (!nc) {
            std::fprintf(stderr, "[RT] addCenterVertex 返回空\n");
            return 1;
        }
        const int vA = livingV(mesh.get()), fA = livingF(mesh.get());
        std::fprintf(stderr, "[RT] add   后 V=%d F=%d  新点度=%d\n", vA, fA, nc->vertex_degree());
        auto af = ring(nc);
        std::fprintf(stderr, "[RT] 后环:");
        for (auto x : af)
            std::fprintf(stderr, " %u", x);
        std::fprintf(stderr, "\n");
        const bool ringOk = (af == before), vfOk = (vA == vPre && fA == fPre);
        std::fprintf(stderr, "[RT] 环还原=%s  V/F 还原=%s (基准 V=%d F=%d)\n", ringOk ? "是" : "**否**", vfOk ? "是" : "**否**", vPre, fPre);
        // 机读结论（stdout，供批量扫描）
        std::printf("RESULT K=%d VID=%u VAL=%d okInv=%d degOk=%d ringOk=%d vfOk=%d\n", kIdx, target->poolId(), deg, (int)okInv, (int)degOk,
                    (int)ringOk, (int)vfOk);
    }
    return 0;
}