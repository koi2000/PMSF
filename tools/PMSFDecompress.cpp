/*
 * @file PMSFDecompress.cpp
 * @brief Phase A/B: 配置驱动的统一解压入口。
 *
 * 用法: PMSFDecompress <config.json>
 */
#include "config/PMSFConfigParser.h"
#include "algorithm/SimplificationAlgorithmRegistry.h"
#include "boundary/BoundaryCompressionModule.h"
#include "common/TimeRecorder.h"
#include <vector>

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "用法: " << argv[0] << " <config.json>\n";
        return 1;
    }

    try {
        pmsf::PMSFConfig cfg = pmsf::parseConfigFromFile(argv[1]);
        if (cfg.input.empty() || cfg.output.empty()) {
            std::cerr << "[PMSFDecompress] config 必须包含 input 与 output 字段\n";
            return 1;
        }
        for (const auto& kv : cfg.extra) {
            if (!kv.first.empty()) {
                setenv(kv.first.c_str(), kv.second.c_str(), 1);
            }
        }
        TimeRecorder recorder("PMSFDecompress total");
        pmsf::dispatchDecompressByName(cfg);
        if (cfg.boundaryCompression) {
            std::vector<int> payload;
            const std::string bnd =
                pmsf::BoundaryCompressionModule::sectionPathForArtifact(cfg.input);
            if (pmsf::BoundaryCompressionModule::readPayloadSection(bnd, payload)) {
                std::fprintf(stderr, "[boundary-module] read payload=%zu ints from %s\n",
                             payload.size(), bnd.c_str());
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "[PMSFDecompress] error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
