/*
 * @Author: koi
 * @Description: Default版本 Parquet格式解压缩 Demo
 * 用法: DefaultParquetDecompress <输入Parquet文件> <输出网格文件.off>
 */
#include "Options.h"
#include "core/MeshRuntime.h"  // pmsf::ensureMeshRuntimeInitialized
#include "core/CompressPipeline.h"  // pmsf::runDecompress
#include "common/TimeRecorder.h"
#include "decompress/ParquetDeserializeOperator.h"
#include "decompress/OptimizedSymbolReadOperator.h"
#include "reconstruct/BasicReconstructOperator.h"
#include <iostream>
#include <memory>

int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "用法: " << argv[0] << " <输入Parquet文件> <输出网格文件>\n";
        std::cerr << "示例: " << argv[0] << " output.parquet decoded.off\n";
        return 1;
    }
    std::string inputPath = argv[1];
    std::string outputPath = argv[2];

    pmsf::ensureMeshRuntimeInitialized();
    std::shared_ptr<SymbolReadOperator> symbolReadOperator = std::make_shared<OptimizedSymbolReadOperator>();
    std::shared_ptr<ReconstructOperator> reconstructOperator = std::make_shared<BasicReconstructOperator>();
    std::shared_ptr<DeserializeOperator> deserializeOperator =
        std::make_shared<ParquetDeserializeOperator>(symbolReadOperator, reconstructOperator);

    DecompressOptions options = DecompressOptions::Builder()
                                    .setDeserializeOperator(deserializeOperator)
                                    .setPath(inputPath)
                                    .setOutputPath(outputPath)
                                    .build();

    TimeRecorder recorder("parquet_decompress");
    (void)pmsf::runDecompress(options);

    std::cout << "\n[完成] Parquet格式解压缩输出: " << outputPath << std::endl;
    return 0;
}
