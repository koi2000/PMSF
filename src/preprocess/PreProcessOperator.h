#pragma once

#include "Mesh.h"
#include <memory>
class PreProcessOperator {
  public:
    PreProcessOperator(/* args */) = default;
    ~PreProcessOperator() = default;

    virtual void process(std::shared_ptr<MCGAL::Mesh> mesh) = 0;
};
