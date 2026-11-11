/*
 * @Author: koi
 * @Date: 2025-09-02 15:45:23
 * @Description:
 */
#ifndef Reconstruct_OPERATOR_H
#define Reconstruct_OPERATOR_H

#include "Halfedge.h"
#include "Mesh.h"
#include "Vertex.h"
#include <memory>
#include <vector>

class ReconstructOperator {
  public:
    ReconstructOperator() = default;

    ~ReconstructOperator() = default;


    void init(std::shared_ptr<MCGAL::Mesh> mesh) {
      this->mesh_ = mesh;
    };

    /// 由 DeserializeOperator 调用，开启后 insert_point 会将 face 上保存的
    /// Vector3i 残差通过 "残差 + barycenter(量化整数) → 反量化" 的方式还原出顶点坐标。
    void setEnablePrediction(bool v) {
      enablePrediction_ = v;
    }

    virtual void reconstruct(MCGAL::Halfedge* halfedge) = 0;

    virtual void reconstruct(std::vector<MCGAL::Vertex*>& vertices, std::vector<MCGAL::Halfedge*>& halfedge, std::vector<MCGAL::Facet*>& facets) = 0;

    std::shared_ptr<MCGAL::Mesh> mesh_;
    bool enablePrediction_ = false;
};

#endif