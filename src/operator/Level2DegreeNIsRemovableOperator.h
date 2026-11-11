/*
 * @file Level2DegreeNIsRemovableOperator.h
 * @brief Phase 2 重构：把 impl/{MMSE,RD,VQMMC}/Degree{Three,Six}IsRemovableOperator.h
 *        原本 3×2=6 份副本中**字节一致**的那一组统一到这里，作为框架公共资产。
 *
 * 语义（与 MMSE/RD/VQMMC 旧副本完全一致）：
 *   - 双 pass 简化：level%2==0 是 decimating pass，level%2==1 是 cleaning pass。
 *   - DegreeThree：decimating pass（偶数 level）一律放行，cleaning pass 仅放行
 *     端点度 ==3 的半边。
 *   - DegreeSix   ：cleaning pass（奇数 level）一律放行，decimating pass 仅放行
 *     端点度 ∈ (3, 6]（即 4/5/6）的半边。这是 MMSE/RD/VQMMC 的"宽松"判据，
 *     与 APTM 修过的 [2, 6] 判据不同 —— APTM 的版本留在 impl/APTM/ 下。
 *
 * 历史差异不合并到本文件：
 *   - APTM: deg ∈ [2, 6] 的修正版  → impl/APTM/DegreeSixIsRemovableOperator.h
 *   - PCATM: 带 null-check 与自定义判据 → impl/PCATM/DegreeSixIsRemovableOperator.h
 *   - PCLTTM: cached-degree 性能优化版 → impl/PCLTTM/Degree{Three,Six}IsRemovableOperator.h
 */
#pragma once

#include "operator/IsRemovableOperator.h"

#include "ContextPool.h"
#include "Global.h"
#include "Halfedge.h"
#include "Vertex.h"
#include "common/HalfedgeErrorEntry.h"

namespace pmsf {

/// @brief 双 pass decimating/cleaning 方案中的 DegreeThree 过滤算子。
///        decimating pass（level%2==0）一律放行；cleaning pass 仅放行度 ==3。
class Level2DegreeThreeIsRemovableOperator : public IsRemovableOperator {
  public:
    Level2DegreeThreeIsRemovableOperator() = default;

    void init(std::vector<MCGAL::Vertex*> v,
              std::vector<MCGAL::Halfedge*> h,
              std::vector<MCGAL::Facet*> f) override {
        (void)v; (void)f;
        seeds = h;
    }

    bool isRemovable(HalfedgeErrorEntry& entry) override {
        // ⚠️ 2026-05-04 性能优化：走无锁 levelFast() 而非 getMetadataOrDefault。
        //   旧路径每调用一次要 mutex.lock + hashmap.find + string hash + ContextValue
        //   拷贝；该函数被 PriorityErrorSelector::pushOrUpdate / update 每候选调用 1–2 次，
        //   MMSE/APTM/RD/VQMMC 上 bunny 30 rounds ≈ 百万量级。
        if (MCGAL::contextPool.levelFast() % 2 == 0) {
            return true;
        }
        if (entry.edge && entry.edge->end_vertex()
            && entry.edge->end_vertex()->vertex_degree() == 3) {
            return true;
        }
        return false;
    }

    std::vector<MCGAL::Halfedge*> seeds;
};

/// @brief 双 pass decimating/cleaning 方案中的 DegreeSix 过滤算子（MMSE/RD/VQMMC 版）。
///        cleaning pass（level%2==1）一律放行；decimating pass 仅放行度 ∈ (3, 6]。
class Level2DegreeSixIsRemovableOperator : public IsRemovableOperator {
  public:
    Level2DegreeSixIsRemovableOperator() = default;

    void init(std::vector<MCGAL::Vertex*> v,
              std::vector<MCGAL::Halfedge*> h,
              std::vector<MCGAL::Facet*> f) override {
        (void)v; (void)f;
        seeds = h;
    }

    bool isRemovable(HalfedgeErrorEntry& entry) override {
        // ⚠️ 2026-05-04 性能优化：levelFast 无锁路径。
        if (MCGAL::contextPool.levelFast() % 2 == 1) {
            return true;
        }
        if (entry.edge && entry.edge->end_vertex()) {
            int d = entry.edge->end_vertex()->vertex_degree();
            if (d > 3 && d <= 6) return true;
        }
        return false;
    }

    std::vector<MCGAL::Halfedge*> seeds;
};

}  // namespace pmsf
