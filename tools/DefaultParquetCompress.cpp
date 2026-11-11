/*
 * @Author: koi
 * @Description: Default版本 Parquet格式压缩 Demo
 * 用法: DefaultParquetCompress <输入网格文件.off> <输出文件.parquet>
 */
#include "Options.h"
#include "core/MeshRuntime.h"  // pmsf::ensureMeshRuntimeInitialized
#include "core/CompressPipeline.h"  // pmsf::runDecompress
#include "common/TimeRecorder.h"
#include "compress/ParquetSerializeOperator.h"
#include "compress/OptimizedSymbolCollectOperator.h"
#include "elimination/EliminateOperator.h"
#include "elimination/VertexRemovalEliminateOperator.h"
#include "operator/BasicIsRemovableOperator.h"
#include "operator/IsRemovableOperator.h"
#include "operator/ManifoldIsRemovableOperator.h"
#include "operator/UnifiedManifoldIsRemovableOperator.h"
#include "operator/SeedIsRemovableOperator.h"
#include "options/SelectOptions.h"
#include "selection/ErrorSource.h"
#include "selection/GreedySelector.h"
#include "selection/SelectOperator.h"
#include "header/PMSFHeader.h"
#include "Mesh.h"
#include "common/HalfedgeErrorEntry.h"
#include <iostream>
#include <memory>
#include <string>

/**
 * @brief 使用Parquet格式输出的压缩流程
 * @details 与DefaultCompress流程一致，但使用ParquetSerializeOperator替代BasicSerializeOperator，
 * 输出为Apache Parquet列式存储格式。
 */
int main(int argc, char* argv[]) {
    if (argc < 3) {
        std::cerr << "用法: " << argv[0] << " <输入网格文件> <输出Parquet文件>\n";
        std::cerr << "示例: " << argv[0] << " datasets/Stanford/bunny.off output.parquet\n";
        return 1;
    }
    std::string inputPath = argv[1];
    std::string outputPath = argv[2];

    // 配置简化算子
    std::shared_ptr<IsRemovableOperator> brmop = std::make_shared<BasicIsRemovableOperator>();
    std::shared_ptr<IsRemovableOperator> mrmop = std::make_shared<UnifiedManifoldIsRemovableOperator>();
    std::shared_ptr<SelectOptions> soptions = SelectOptions::Builder()
                                                  .withErrorSource(ErrorSource::Halfedge)
                                                  .withHalfedgeErrorAccessor(nullptr)
                                                  .withVertexErrorAccessor(nullptr)
                                                  .withIsRemovableOperator(brmop)
                                                  .withIsRemovableOperator(mrmop)
                                                  .build();

    std::shared_ptr<SelectOperator> selectOp = std::make_shared<GreedySelector>(*soptions);
    std::shared_ptr<EliminateOperator> eliminateOp = std::make_shared<VertexRemovalEliminateOperator>();

    // 构建压缩选项
    int round = 30;
    bool enablePrediction = true;
    bool enableQuantization = false;
    bool enableCompress = false;

    PMSFHeader header = PMSFHeader::Builder()
                            .setRound(round)
                            .setEnableCompress(enableCompress)
                            .setEnableQuantization(enableQuantization)
                            .setEnableSegmentation(false)
                            .setEnableAdjustMent(false)
                            .build();

    // 加载网格
    std::shared_ptr<MCGAL::Mesh> mesh = std::make_shared<MCGAL::Mesh>(MCGAL::Mesh(inputPath));

    // 创建符号收集器和Parquet序列化器
    std::shared_ptr<SymbolCollectOperator> collector = std::make_shared<OptimizedSymbolCollectOperator>();
    std::shared_ptr<SerializeOperator> serializer =
        std::make_shared<ParquetSerializeOperator>(collector, enablePrediction, enableQuantization, enableCompress);

    std::shared_ptr<IsRemovableOperator> seedIsRemovableOperator = std::make_shared<SeedIsRemovableOperator>();

    // 选择seed
    MCGAL::Halfedge* seed = nullptr;
    MCGAL::Point p;
    selectOp->init(mesh);
    selectOp->selectSeed(seed);
    eliminateOp->init(mesh, header);

    HalfedgeErrorEntry entry(seed, p, 0, 0);
    collector->init(mesh, {seed}, header);
    seedIsRemovableOperator->init({}, {seed}, {});
    selectOp->addIsRemovableOperator(seedIsRemovableOperator);

    // 执行压缩轮次
    TimeRecorder recorder("parquet_compress");
    for (int i = 0; i < round; i++) {
        while (selectOp->select(entry)) {
            eliminateOp->eliminate(entry);
        }
        collector->collect(seed);
        selectOp->reset();
    }

    // 序列化为Parquet格式
    auto headerPtr = std::make_shared<PMSFHeader>(header);
    serializer->setHeader(headerPtr);
    serializer->init();
    serializer->serializeHeader(header);
    serializer->serializeBaseMeshWithSeed(mesh, {seed});
    serializer->serialize(outputPath);

    std::cout << "\n[完成] Parquet格式压缩输出: " << outputPath << std::endl;
    return 0;
}
