/*
 * @file QEMMetricHook.cpp
 * @brief Phase A/B: 示例 QEM 外部误差度量 hook。
 *
 * 实现一个“边长”metric：折叠代价 = 边两端点距离的平方，目标位置 = 两端点中点。
 * 仅使用 MCGAL 头文件中的 inline 访问器，不链接 host 符号，完全通过 C-ABI 交互。
 */
#include "hook/PMSFHookAPI.h"

#include "Halfedge.h"
#include "Point.h"
#include "Vertex.h"

#include <cmath>
#include <new>

namespace {

struct State {
    // 未来可存放 per-mesh 状态；本示例无状态。
};

float edgeLengthCost(void* userdata, void* halfedge, float* targetPoint) {
    (void)userdata;
    if (!halfedge || !targetPoint) return 1e30f;
    auto* h = static_cast<MCGAL::Halfedge*>(halfedge);
    MCGAL::Vertex* v0 = h->vertex();
    MCGAL::Vertex* v1 = h->end_vertex();
    if (!v0 || !v1) return 1e30f;
    MCGAL::Point p0 = v0->point();
    MCGAL::Point p1 = v1->point();
    float dx = p0.x() - p1.x();
    float dy = p0.y() - p1.y();
    float dz = p0.z() - p1.z();
    targetPoint[0] = (p0.x() + p1.x()) * 0.5f;
    targetPoint[1] = (p0.y() + p1.y()) * 0.5f;
    targetPoint[2] = (p0.z() + p1.z()) * 0.5f;
    return dx * dx + dy * dy + dz * dz;
}

void afterEliminate(void* userdata, void* halfedge) {
    (void)userdata;
    (void)halfedge;
}

void destroy(void* userdata) {
    State* s = static_cast<State*>(userdata);
    delete s;
}

}  // namespace

extern "C" {

PMSFErrorMetricV1* pmsf_create_error_metric_v1() {
    PMSFErrorMetricV1* m = new (std::nothrow) PMSFErrorMetricV1();
    if (!m) return nullptr;
    m->userdata = new (std::nothrow) State();
    if (!m->userdata) {
        delete m;
        return nullptr;
    }
    m->cost = edgeLengthCost;
    m->on_after_eliminate = afterEliminate;
    m->destroy = destroy;
    return m;
}

}  // extern "C"
