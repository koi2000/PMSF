#include "core/MeshRuntime.h"  // pmsf::ensureMeshRuntimeInitialized
#include "core/CompressPipeline.h"  // pmsf::runDecompress
#include "common/TimeRecorder.h"
#include "compress/BasicSerializeOperator.h"
#include "core/CompressPipeline.h"
#include "elimination/VertexRemovalEliminateOperator.h"
#include "operator/BasicIsRemovableOperator.h"
#include "operator/UnifiedManifoldIsRemovableOperator.h"
#include "options/CompressOptions.h"
#include "options/SelectOptions.h"
#include "selection/ErrorSource.h"
#include "selection/PriorityErrorSelector.h"
#include "storage/ColumnarBlobOperators.h"

#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>

namespace {

bool parseBoolOption(const std::string& value, bool fallback) {
    if (value == "1" || value == "true" || value == "TRUE" ||
        value == "on" || value == "ON" || value == "yes" || value == "YES") {
        return true;
    }
    if (value == "0" || value == "false" || value == "FALSE" ||
        value == "off" || value == "OFF" || value == "no" || value == "NO") {
        return false;
    }
    return fallback;
}

bool defaultQuantizationEnabled() {
    const char* env = std::getenv("PMSF_DEFAULT_ENABLE_QUANTIZATION");
    if (!env || !*env) env = std::getenv("PMSF_DEFAULT_QUANTIZATION");
    return env && *env ? parseBoolOption(env, true) : true;
}

}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "用法: " << argv[0]
                  << " <输入网格文件> <输出ORC文件> [--rounds=N] [--quantization=0|1]\n";
        return 1;
    }

    std::string inputPath = argv[1];
    std::string outputPath = argv[2];
    int rounds = 30;
    bool enableQuantization = defaultQuantizationEnabled();

    for (int i = 3; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg.rfind("--rounds=", 0) == 0) {
            rounds = std::atoi(arg.substr(9).c_str());
            if (rounds <= 0) rounds = 30;
        } else if (arg.rfind("--quantization=", 0) == 0) {
            enableQuantization = parseBoolOption(arg.substr(15), enableQuantization);
        } else if (arg == "--no-quantization") {
            enableQuantization = false;
        } else if (arg == "--quantization") {
            enableQuantization = true;
        }
    }

    auto brmop = std::make_shared<BasicIsRemovableOperator>();
    auto mrmop = std::make_shared<UnifiedManifoldIsRemovableOperator>();
    auto soptions = SelectOptions::Builder()
                        .withErrorSource(ErrorSource::Halfedge)
                        .withIsRemovableOperator(brmop)
                        .withIsRemovableOperator(mrmop)
                        .build();
    auto selectOp = std::make_shared<PriorityErrorSelector>(*soptions);
    auto eliminateOp = std::make_shared<VertexRemovalEliminateOperator>();

    auto options = CompressOptions::Builder()
                       .setSelect(selectOp)
                       .setRound(rounds)
                       .setEliminate(eliminateOp)
                       .setEnableCompress(true)
                       .setEnablePrediction(true)
                       .setEnableQuantization(enableQuantization)
                       .setPath(inputPath)
                       .setOutputPath(outputPath)
                       .build();

    auto cfg = pmsf::CompressPipelineConfig::defaultBinary();
    cfg.headerCarryPrediction = true;
    cfg.serializerFactory = [](std::shared_ptr<SymbolCollectOperator> collector,
                               const CompressOptions& opts) -> std::shared_ptr<SerializeOperator> {
        auto inner = std::make_shared<BasicSerializeOperator>(
            collector, opts.isEnablePrediction(), opts.isEnableQuantization(), opts.isEnableCompress());
        return std::make_shared<pmsf::ORCBlobSerializeOperator>(inner);
    };

    TimeRecorder recorder("orc_compress");
    pmsf::ensureMeshRuntimeInitialized();
    pmsf::runCompress(options, cfg);
    return 0;
}
