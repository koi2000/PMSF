#include "core/MeshRuntime.h"  // pmsf::ensureMeshRuntimeInitialized
#include "core/CompressPipeline.h"  // pmsf::runDecompress
#include "common/TimeRecorder.h"
#include "decompress/DeserializeOperator.h"
#include "decompress/OptimizedBasicDeserializeOperator.h"
#include "decompress/OptimizedSymbolReadOperator.h"
#include "options/DecompressOptions.h"
#include "reconstruct/BasicReconstructOperator.h"
#include "storage/ColumnarBlobOperators.h"

#include <iostream>
#include <memory>
#include <string>

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "用法: " << argv[0] << " <输入ORC文件> <输出OFF文件>\n";
        return 1;
    }

    std::string inputPath = argv[1];
    std::string outputPath = argv[2];

    auto reader = std::make_shared<OptimizedSymbolReadOperator>();
    auto reconstructor = std::make_shared<BasicReconstructOperator>();
    auto inner = std::make_shared<OptimizedBasicDeserializeOperator>(reader, reconstructor);
    std::shared_ptr<DeserializeOperator> deserializer =
        std::make_shared<pmsf::ORCBlobDeserializeOperator>(inner);

    auto options = DecompressOptions::Builder()
                       .setDeserializeOperator(deserializer)
                       .setPath(inputPath)
                       .setOutputPath(outputPath)
                       .build();

    pmsf::ensureMeshRuntimeInitialized();
    TimeRecorder recorder("orc_decompress");
    (void)pmsf::runDecompress(options);
    return 0;
}
