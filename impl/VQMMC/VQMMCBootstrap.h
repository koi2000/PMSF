/*
 * @file VQMMCBootstrap.h
 * @brief CLI parsing + property-pool bootstrap helper for VQMMC (Phase 13c).
 *
 * VQMMC has a legitimately custom compress loop (custom SymbolCollectOperator
 * + PostProcessor running every round + BasicSerializeOperator driven
 * manually). Migrating it to CompressPipeline::run would either need a new
 * AlgoPreset case + pipeline-config boolean or would risk breaking
 * byte-identity. Phase 13c-VQMMC therefore stays with the custom loop but
 * extracts the boilerplate (CLI, property pool sizing, header building)
 * to shrink the main to <80 LOC.
 */
#ifndef PMSF_IMPL_VQMMC_VQMMC_BOOTSTRAP_H
#define PMSF_IMPL_VQMMC_VQMMC_BOOTSTRAP_H

#include "ContextPool.h"
#include "Global.h"
#include "Mesh.h"
#include "header/PMSFHeader.h"

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

namespace vqmmc_impl {

struct Args {
    std::string input;
    std::string output;
    int rounds = 30;
    int codebookSize = 256;
};

inline Args parseArgs(int argc, char* argv[]) {
    Args a;
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0]
                  << " <input.off> <output.bin> [rounds=30] [codebookSize=256]\n";
        std::exit(1);
    }
    a.input = argv[1];
    a.output = argv[2];
    if (argc >= 4) a.rounds = std::atoi(argv[3]);
    if (argc >= 5) a.codebookSize = std::atoi(argv[4]);
    return a;
}

inline std::shared_ptr<MCGAL::Mesh> bootstrapMesh(const std::string& path) {
    MCGAL::contextPool.initPoolSize(1);
    MCGAL::contextPool.registerPool(
        MCGAL::ContextPool::poolSizeFromEnv("VQMMC_POOL_V", 500000),
        MCGAL::ContextPool::poolSizeFromEnv("VQMMC_POOL_H", 10000000),
        MCGAL::ContextPool::poolSizeFromEnv("VQMMC_POOL_F", 500000));
    MCGAL::contextPool.setMetadata("level", MCGAL::ContextValue(0));
    return std::make_shared<MCGAL::Mesh>(MCGAL::Mesh(path));
}

inline PMSFHeader buildHeader(int rounds) {
    return PMSFHeader::Builder()
        .setRound(rounds)
        .setEnableCompress(false)
        .setEnableQuantization(true)
        .setEnableSegmentation(false)
        .setEnableAdjustMent(false)
        .build();
}

}  // namespace vqmmc_impl

#endif  // PMSF_IMPL_VQMMC_VQMMC_BOOTSTRAP_H
