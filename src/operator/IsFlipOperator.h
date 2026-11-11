#pragma once

#include "Facet.h"
#include "Halfedge.h"
#include "Point.h"
#include "Vertex.h"
#include "common/HalfedgeErrorEntry.h"
#include "operator/IsRemovableOperator.h"

class IsFlipOperator : public IsRemovableOperator {
  public:
    enum class TargetMode {
        EntryPoint,
        StartVertex,
        EndVertex,
    };

    explicit IsFlipOperator(TargetMode mode = TargetMode::EntryPoint)
        : targetMode_(mode) {}

    void init(std::vector<MCGAL::Vertex*> v,
              std::vector<MCGAL::Halfedge*> h,
              std::vector<MCGAL::Facet*> f) override {
        (void)v;
        (void)f;
        seeds = h;
    }

    bool check(MCGAL::Vertex* v0, MCGAL::Vertex* v1, MCGAL::Point ptTarget) const {
        if (v0 == nullptr || v1 == nullptr || v0->isRemoved() || v1->isRemoved()) {
            return true;
        }

        const float kCosMax = 0.999f;
        const float kCosMaxSq = kCosMax * kCosMax;
        // 单个 1-ring 面的判定；返回 0=继续，1=拒绝（flip/异常）。
        // 与旧实现逐项相同；提取成 lambda 以便用 O(1) 环走（原 `*v0` 迭代器
        // 每步要 find_prev，非三角面 O(deg)）。环集合相同、顺序相反；
        // 本函数是「任一违例即 true」的 OR 语义 ⇒ 结果与顺序无关、字节等价。
        auto checkOne = [&](MCGAL::Halfedge* hIt) -> int {
            if (hIt == nullptr || hIt->face() == nullptr) return 0;
            MCGAL::Facet* fIt = hIt->face();
            MCGAL::Halfedge* ph = fIt->proxyHalfedge();
            if (fIt->isRemoved() || ph == nullptr || ph->next() == nullptr ||
                ph->next()->next() == nullptr) {
                return 0;
            }

            MCGAL::Halfedge* ph1 = ph->next();
            MCGAL::Halfedge* ph2 = ph1->next();
            MCGAL::Vertex* fv[3];
            fv[0] = ph->vertex();
            fv[1] = ph1->vertex();
            fv[2] = ph2->vertex();

            if (fv[0] == nullptr || fv[1] == nullptr || fv[2] == nullptr) return 1;
            if (fv[0] == v1 || fv[1] == v1 || fv[2] == v1) return 0;

            int idxV0 = (fv[0] == v0) ? 0 : (fv[1] == v0) ? 1 : 2;
            if (fv[idxV0] != v0) return 0;

            const MCGAL::Point& pt0 = fv[idxV0]->point();
            const MCGAL::Point& pt1 = fv[(idxV0 + 1) % 3]->point();
            const MCGAL::Point& pt2 = fv[(idxV0 + 2) % 3]->point();

            MCGAL::Vector3 dir1 = pt1 - ptTarget;
            MCGAL::Vector3 dir2 = pt2 - ptTarget;
            float len1Sq = dir1.dot(dir1);
            float len2Sq = dir2.dot(dir2);
            float lenSqProd = len1Sq * len2Sq;
            if (lenSqProd <= 0.0f) return 0;
            float dotv = dir1.dot(dir2);
            if (dotv * dotv > kCosMaxSq * lenSqProd) return 1;

            MCGAL::Vector3 norm = dir1.cross(dir2);
            if (norm.dot(norm) <= 0.0f) return 1;
            norm = norm.normalize();

            MCGAL::Vector3 e1(pt1.x() - pt0.x(), pt1.y() - pt0.y(), pt1.z() - pt0.z());
            MCGAL::Vector3 e2(pt2.x() - pt0.x(), pt2.y() - pt0.y(), pt2.z() - pt0.z());
            MCGAL::Vector3 oldNormRaw = e1.cross(e2);
            if (oldNormRaw.dot(oldNormRaw) <= 0.0f) return 1;
            MCGAL::Vector3 normold = oldNormRaw.normalize();

            // Match the reference QEM implementation's inversion guard:
            // reject when the old/new face normals diverge past dot <= 0.25.
            if (normold.dot(norm) <= 0.25f) return 1;
            return 0;
        };
        // 快速环走：h = h->opposite()->next()，起点取 proxyHalfedge。
        // 未在 4096 步内闭合（异常环）时退回原 *v0 迭代器路径，行为不变。
        MCGAL::Halfedge* hStart = v0->proxyHalfedge();
        if (hStart != nullptr) {
            MCGAL::Halfedge* hIt = hStart;
            int guard = 0;
            for (;;) {
                if (checkOne(hIt) != 0) return true;
                MCGAL::Halfedge* opp = hIt->opposite();
                if (opp == nullptr) break;
                hIt = opp->next();
                if (hIt == nullptr || ++guard > 4096) break;
                if (hIt == hStart) return false;
            }
        }
        for (MCGAL::Halfedge* hIt : *v0) {
            if (checkOne(hIt) != 0) return true;
        }
        return false;
    }

    bool isRemovable(HalfedgeErrorEntry& entry) override {
        MCGAL::Halfedge* halfedge = entry.edge;
        if (halfedge == nullptr || halfedge->vertex() == nullptr ||
            halfedge->end_vertex() == nullptr) {
            return false;
        }
        MCGAL::Point target = entry.p;
        if (targetMode_ == TargetMode::StartVertex) {
            target = halfedge->vertex()->point();
        } else if (targetMode_ == TargetMode::EndVertex) {
            target = halfedge->end_vertex()->point();
        }
        return !check(halfedge->vertex(), halfedge->end_vertex(), target) &&
               !check(halfedge->end_vertex(), halfedge->vertex(), target);
    }

    std::vector<MCGAL::Halfedge*> seeds;

  private:
    TargetMode targetMode_ = TargetMode::EntryPoint;
};
