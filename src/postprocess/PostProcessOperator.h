#pragma once

#include "Mesh.h"
#include "../common/Graph.h"
#include <memory>
class PostProcessOperator {
  public:
    PostProcessOperator(/* args */) = default;
    ~PostProcessOperator() = default;

    virtual void process(std::shared_ptr<MCGAL::Mesh> mesh) = 0;
    virtual void processWithGraph(std::shared_ptr<MCGAL::Mesh> mesh, std::shared_ptr<Graph> graph) = 0;
};
