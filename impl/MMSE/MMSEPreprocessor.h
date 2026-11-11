#pragma once

#include "preprocess/PreProcessOperator.h"
#include <iostream>

class MMSEPreprocessor : public PreProcessOperator {
  public:
    MMSEPreprocessor() = default;
    ~MMSEPreprocessor() = default;

    void process(std::shared_ptr<MCGAL::Mesh> mesh) override {
        mesh->quantizeVertexPositions();
        std::cout << "[MMSEPreprocessor] Quantized vertex positions, nbQuantBits="
                  << mesh->i_nbQuantBits << ", quantStep=" << mesh->f_quantStep << std::endl;
    }
};
