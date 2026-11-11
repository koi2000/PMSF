/*
 * @Author: koi
 * @Date: 2025-09-20 11:51:21
 * @Description:
 */
/*
 * @Author: koi
 * @Date: 2025-09-03 21:29:50
 * @Description:
 */
/*
 * @Author: koi
 * @Date: 2025-08-25 22:29:32
 * @Description:
 */
#ifndef BOUNDARY_ISREOVABLEOPERATOR_H
#define BOUNDARY_ISREOVABLEOPERATOR_H

#include "Facet.h"
#include "Halfedge.h"
#include "Vertex.h"
#include "common/BoundedRingTraversal.h"
#include "operator/IsRemovableOperator.h"
#include <cstddef>
#include <cstdlib>
#include <set>
#include <unordered_set>
#include <vector>

namespace pmsf {

// perf（2026-11-09 QEM 实测）：PMSF_RELAX_BOUNDARY_PROTECT 原先每个候选判定
// 一次 getenv（单次压缩 3.4 万次）。开关进程内不变，缓存一次；语义不变。
inline const char* relaxBoundaryProtectEnvCached() {
    static const char* v = std::getenv("PMSF_RELAX_BOUNDARY_PROTECT");
    return v;
}
/// Encoder-only freeze set for shared-mesh boundary compression patches.
/// Empty by default => zero behavioral change for all existing paths.
inline std::unordered_set<MCGAL::Vertex*>& sharedBoundaryFrozenVertices() {
    static std::unordered_set<MCGAL::Vertex*> s;
    return s;
}

/// Encoder-only anchor guard set.  Unlike `sharedBoundaryFrozenVertices`, this
/// is consulted only by the eliminate operator (not by BoundaryIsRemovable),
/// so protecting anchors does not change which vertices pass the removable
/// predicate and therefore cannot corrupt the coarse-mesh triangulation.
inline std::unordered_set<MCGAL::Vertex*>& anchorGuardedVertices() {
    static std::unordered_set<MCGAL::Vertex*> s;
    return s;
}
}  // namespace pmsf

class BoundaryIsRemovableOperator : public IsRemovableOperator {
  public:
    BoundaryIsRemovableOperator() = default;

    void init(std::vector<MCGAL::Vertex*> v, std::vector<MCGAL::Halfedge*> h, std::vector<MCGAL::Facet*> f){};

    bool isRemovable(HalfedgeErrorEntry& entry) {
        MCGAL::Halfedge* halfedge = entry.edge;
        if (halfedge) {
            auto& frozen = pmsf::sharedBoundaryFrozenVertices();
            if (!frozen.empty() &&
                (frozen.count(halfedge->vertex()) || frozen.count(halfedge->end_vertex()))) {
                return false;
            }
        }
        // [PHASE61 物品1① **测量开关**，默认关闭]
        // 边界保护是**充分**条件，不是**必要**条件：真正保证「消除 → 再插入」对
        // groupId 可逆的充要条件是「被消顶点（end_vertex）合面前那个环的**面 gid
        // 一致**」—— 解码端 insert 只用**一个** gid 重建整个环，环内 gid 不一致时
        // 编码端这一步就**销毁**了信息（实测：N=2 关保护 ⇒ 555 处、1115 个面）。
        // 打开本开关只对「环面 gid 一致」的候选**放行**，其余仍走原保护。
        // ⚠️ 这是为**测量**该杠杆能省多少字节；不是在承诺一个可交付口径。
        // [PHASE61 物品1① **第二级测量开关**，默认关闭]
        // level>=2 再放一档：只要被消顶点的扇区**闭合**就放行，**不看 gid 是否一致**。
        // 这不是可交付口径 —— 环内 gid 不一致时编码端会销毁 gid（附 34 实测），
        // 它唯一的作用是**量出「gid 一致性」这一个约束还挡着多少顶点**
        // （= 附 34 的 relax 与 no-protect 之间那段距离里，哪些是拓扑不可逆、
        //  哪些只是 gid 不可逆）。配套计数器见 `remove_point` 的 `PMSF_ELIM_FAN_DIAG`。
        // PHASE61 物品 1 ①：**分区路径默认开启**（level 1）。
        //
        // 判据 = 该面属于某个分区（`groupId >= 0`）。**非分区路径 `groupId` 恒为 -1
        // 且从不为 0**，故此分支在默认路径上**不可达** ⇒ 默认产物逐字节不变。
        //
        // 为什么默认开：level 1 放松到的**恰好是编码端/解码端互为逆运算的充要条件**
        // ——「被消顶点的扇区 gid 均匀」。附 34 实测：不放松时边界保护间接阻止了
        // 本可安全进行的消除，N=64 因此 +20.8% 出带；放松后 47.31（+17.7%，**进带**），
        // 且八档全部精确回环 34834/69664、MeshCompare 三项全 true、nn max 3.24e-05 ≪ tol。
        // ⇒ 它不是「以质量换 bpv」，是把保护**收紧到必要的那一部分**。
        //
        // `PMSF_RELAX_BOUNDARY_PROTECT=0` 可显式退回旧行为（对照用）。
        // level >= 2 仍是**纯测量开关**（不看 gid 一致性 ⇒ 编码端会销毁 gid，
        // 见附 34），**只允许 env 显式开启**，不随默认生效。
        const char* relaxEnv = pmsf::relaxBoundaryProtectEnvCached();
        if (relaxEnv != nullptr && std::atoi(relaxEnv) >= 2) {
            if (fanClosed(halfedge)) {
                return true;
            }
        } else {
            const bool partitioned =
                halfedge->face() != nullptr && halfedge->face()->groupId() >= 0;
            const bool relaxLevel1 =
                partitioned ? (relaxEnv == nullptr || std::atoi(relaxEnv) != 0)
                            : (relaxEnv != nullptr);
            if (relaxLevel1 && fanGroupUniform(halfedge)) {
                return true;
            }
        }
        if (halfedge->isBoundary()) {
            return false;
        }

        for (MCGAL::Halfedge* hit : *halfedge->vertex()) {
            if (hit->isBoundary()) {
                return false;
            }
        }
        for (MCGAL::Halfedge* hit : *halfedge->end_vertex()) {
            if (hit->isBoundary()) {
                return false;
            }
        }
        return true;
        // if (!halfedge->isBoundary()) {
        //     return true;
        // }
        // return check(halfedge) && check(halfedge->next_boundary());
    };

    // 被消顶点（`end_vertex`）的扇区是否**闭合**（每个半edge都有面），不看 gid。
    bool fanClosed(MCGAL::Halfedge* halfedge) const {
        if (!halfedge) {
            return false;
        }
        MCGAL::Vertex* removedVertex = halfedge->end_vertex();
        if (!removedVertex) {
            return false;
        }
        bool any = false;
        for (MCGAL::Halfedge* hit : pmsf::boundedRing(removedVertex)) {
            if (!hit || !hit->face()) {
                return false;
            }
            any = true;
        }
        return any;
    }

    // 被消顶点（`end_vertex`）的环面 gid 是否一致；环闭合且每个半edge都有面才算。
    bool fanGroupUniform(MCGAL::Halfedge* halfedge) const {
        if (!halfedge) {
            return false;
        }
        MCGAL::Vertex* removedVertex = halfedge->end_vertex();
        if (!removedVertex) {
            return false;
        }
        int gid = -1;
        bool any = false;
        for (MCGAL::Halfedge* hit : pmsf::boundedRing(removedVertex)) {
            if (!hit || !hit->face()) {
                return false;
            }
            if (!any) {
                gid = hit->face()->groupId();
                any = true;
            } else if (hit->face()->groupId() != gid) {
                return false;
            }
        }
        return any;
    }

    bool check(MCGAL::Halfedge* halfedge) {
        if (!halfedge) {
            return false;
        }
        std::set<int> cnt;
        for (MCGAL::Halfedge* vhit : *halfedge->vertex()) {
            if (vhit->face()->groupId()) {
                cnt.insert(vhit->face()->groupId());
            }
        }
        if (cnt.size() >= 3) {
            return false;
        }
        cnt.clear();
        for (MCGAL::Halfedge* vhit : *halfedge->end_vertex()) {
            if (vhit->face()->groupId()) {
                cnt.insert(vhit->face()->groupId());
            }
        }
        if (cnt.size() >= 3) {
            return false;
        }

        return true;
    }
};

#endif