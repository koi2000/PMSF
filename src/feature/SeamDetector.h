/*
 * @file SeamDetector.h
 * @brief PCLTTM UV/material 缝检测。
 *
 * 论文 PCLTTM (Lee et al. 2014) 的 key trick：贴图/材质缝两侧应当 clamp
 * elimination —— 缝上的顶点不可塌缩到任一侧，否则 UV 展开会撕裂。
 *
 * 框架化实现：以 mesh 的 face 属性为信号，如果相邻面的 material_id 不同，
 * 则两面共享的那条边两端顶点是 seam anchor。
 *
 * 由于当前 MCGAL::Facet 没有原生 material_id 字段，这里通过 ContextPool
 * 的通用 property 查询（默认属性名 "material_id"，int 类型）。若 mesh 没有
 * 写入该属性，detect() 返回空集（语义：无缝、不约束任何顶点）。
 */
#ifndef PMSF_FEATURE_SEAM_DETECTOR_H
#define PMSF_FEATURE_SEAM_DETECTOR_H

#include "FeatureDetector.h"
#include "ContextPool.h"
#include "Facet.h"
#include "Global.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Vertex.h"

#include <cstdint>
#include <memory>
#include <string>

namespace pmsf {

class SeamDetector final : public FeatureDetector {
public:
    explicit SeamDetector(std::string property_name = "material_id")
        : prop_(std::move(property_name)) {}

    FeatureSet detect(std::shared_ptr<MCGAL::Mesh> mesh) override {
        FeatureSet out;
        if (!mesh) return out;

        const int meshId = mesh->meshId();
        for (MCGAL::Vertex* v : mesh->vertices()) {
            if (v == nullptr || v->isRemoved()) continue;
            int firstMat = -1;
            bool seam = false;
            for (MCGAL::Halfedge* h : v->halfedges()) {
                if (h == nullptr || h->face() == nullptr || h->face()->isRemoved()) continue;
                const int mat =
                    MCGAL::contextPool.property<int>(meshId, prop_.c_str(), h->face());
                if (firstMat < 0) { firstMat = mat; continue; }
                if (mat != firstMat) { seam = true; break; }
            }
            if (seam) {
                out.insert(static_cast<std::uint32_t>(v->poolId()));
            }
        }
        return out;
    }

private:
    std::string prop_;
};

}  // namespace pmsf

#endif  // PMSF_FEATURE_SEAM_DETECTOR_H
