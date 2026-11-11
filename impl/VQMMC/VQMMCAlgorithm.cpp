/*
 * @file VQMMCAlgorithm.cpp
 * @brief VQMMC (Li et al., IEEE SPL 2006, "Vector Quantization in Multiresolution
 *        Mesh Compression") 接入 ISimplificationAlgorithm 契约。
 *
 * 论文 Tbl. tab:hook_loc 里 VQMMC 重写的是 H1（12 行）、H4（409 行）、H5（13 行）
 * 与码本子系统（191 行）。
 *
 * ## 为什么这个插件覆写了 compress()/decompress()
 *
 * VQMMC 是设计文档 §2.1.3 的 **码本子系统**案例：几何不走残差流，而是每层
 * 训练一个 VQ 码本、只写码本索引（`VQMMCPostProcessor` + `VectorQuantizer`），
 * 再配 `SeedIsRemovableOperator` 把种子钉死、由单一种子驱动整棵树重建。
 * 这条流水线与框架的 `CompressPipelineConfig` 形状不同；设计文档把
 * 「VQMMC 专用路径迁入统一管线」排在 Phase 4。
 *
 * 本轮的接入形态：算法代码全部回到 impl/VQMMC/ 自己的 cpp，由契约的
 * compress()/decompress() 承载，解压侧零入口文件。
 *
 * 字节核实：scripts/tests/verify_algo_migration.sh vqmmc（同二进制 A/B）。
 */
#include "algorithm/SimplificationAlgorithmRegistry.h"
#include "core/MeshRuntime.h"
#include "DegreeSixIsRemovableOperator.h"
#include "DegreeThreeIsRemovableOperator.h"
#include "VQMMCBootstrap.h"
#include "VQMMCColumnarDeserializeOperator.h"
#include "VQMMCColumnarSerializeOperator.h"
#include "VQMMCDeserializeOperator.h"
#include "VQMMCPostProcessor.h"
#include "VQMMCReconstructOperator.h"
#include "VQMMCSymbolCollectOperator.h"
#include "VQMMCSymbolReadOperator.h"
#include "VQMMCStrictSymbolCollectOperator.h"
#include "VQMMCStrictSymbolReadOperator.h"
#include "VQMMCPaperDeserializeOperator.h"
#include "VQMMCPaperOpaqueIO.h"
#if PMSF_VQMMC_HAS_OPAQUE_PAPER
#include "storage/PaperStreamOpaquePayloadV2.h"
#include "storage/PaperStreamWriterV2.h"
#include "compress/ConnectivityBlobContainer.h"
#endif
#include "VQMMCPaperRegistration.h"
#include "common/HalfedgeErrorEntry.h"
#include "common/TimeRecorder.h"
#include "compress/BasicSerializeOperator.h"
#include "compress/SymbolCollectOperator.h"
#include "core/Backend.h"
#include "elimination/VertexRemovalEliminateOperator.h"
#include "header/PMSFHeader.h"
#include "operator/BasicIsRemovableOperator.h"
#include "operator/IsFlipOperator.h"
#include "operator/SeedIsRemovableOperator.h"
#include "operator/UnifiedManifoldIsRemovableOperator.h"
#include "operator/VisualTopologyIsRemovableOperator.h"
#include "options/SelectOptions.h"
#include "selection/ErrorSource.h"
#include "selection/PriorityErrorSelector.h"
#include "ContextPool.h"
#include "Mesh.h"
#include <cstdio>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>


namespace pmsf {

namespace {

    /// Phase 1 校验用：把注册名临时改成 "<name>_probe"，就能在同一份二进制里
    /// 让 `algorithm` 字段分别命中 legacy 过渡桥与插件路径，做真正的 A/B。
    /// Phase 4 删除 legacy 桥后本开关一并移除。
    static std::string registeredName(const char* real, const char* probeEnv,
                                      const char* probeName) {
        const char* v = std::getenv(probeEnv);
        return (v && *v) ? std::string(probeName) : std::string(real);
    }

bool envOn(const char* name) {
    const char* v = std::getenv(name);
    return v && *v && std::string(v) != "0";
}

class VQMMCTraceEliminateOperator final
    : public VertexRemovalEliminateOperator {
  public:
    explicit VQMMCTraceEliminateOperator(
        const std::shared_ptr<VQMMCPostProcessor>& postProcessor)
        : postProcessor_(postProcessor) {}

    bool remove_point(MCGAL::Halfedge* h) override {
        if (!h || !h->end_vertex()) return false;
        const int removedVertexId = h->end_vertex()->globalVertexId();
        const bool removed = VertexRemovalEliminateOperator::remove_point(h);
        if (removed && postProcessor_ && h->face()) {
            postProcessor_->recordRemovedVertex(h->face(), removedVertexId);
        }
        return removed;
    }

  private:
    std::shared_ptr<VQMMCPostProcessor> postProcessor_;
};

}  // namespace

namespace {

/// @brief M5-5：把 canonical VQMMC binary artifact 序列化到指定路径。
///
/// 原 binary backend 的序列化体原样抽出：paper 分支复用它把 canonical 字节
/// 写到临时文件后逐字节捕获（不重编码），保证 paper opaque payload 与
/// canonical 64,510 B 完全一致。默认 binary 路径仍以 cfg.output 调用，字节不变。
struct VQMMCCanonicalBinaryInfo {
    int totalVertices = 0;
    int totalFaces = 0;
    int baseVertices = 0;
    int baseFaces = 0;
    int quantBits = 0;
    float bboxMin[3] = {};
    float quantStep = 0.f;
    std::vector<std::uint8_t> basePrefix;
};

VQMMCCanonicalBinaryInfo serializeVQMMCCanonicalBinary(const PMSFConfig& cfg,
                                                       const std::string& outputPath) {
    // Binary 路径
    TimeRecorder recorder("VQMMC compress");
    struct Args {
        std::string input;
        std::string output;
        int rounds;
        int codebookSize;
    } args{cfg.input, outputPath, cfg.effectiveRounds(), cfg.effectiveCodebookSize()};
    auto mesh = vqmmc_impl::bootstrapMesh(args.input);
    VQMMCCanonicalBinaryInfo info;
    if (mesh) {
        info.totalVertices = static_cast<int>(mesh->vertices().size());
        info.totalFaces = static_cast<int>(mesh->faces().size());
    }
    auto header = vqmmc_impl::buildHeader(args.rounds);

    auto soptions = SelectOptions::Builder()
        .withErrorSource(ErrorSource::Halfedge)
        .withHalfedgeErrorAccessor(nullptr).withVertexErrorAccessor(nullptr)
        .withIsRemovableOperator(std::make_shared<BasicIsRemovableOperator>())
        .withIsRemovableOperator(std::make_shared<UnifiedManifoldIsRemovableOperator>())
        .withIsRemovableOperator(std::make_shared<DegreeThreeIsRemovableOperator>())
        .withIsRemovableOperator(std::make_shared<DegreeSixIsRemovableOperator>())
        .build();

    auto select = std::make_shared<PriorityErrorSelector>(*soptions);
    auto postProcessor = std::make_shared<VQMMCPostProcessor>(args.codebookSize);
    std::shared_ptr<VertexRemovalEliminateOperator> eliminate =
        std::make_shared<VQMMCTraceEliminateOperator>(postProcessor);
    std::shared_ptr<SymbolCollectOperator> collector;
    const char* envStrict = std::getenv("CODEC_030");
    const char* envMepp2 = std::getenv("VQMMC_MEPP2");
    const char* envPaperConn = std::getenv("VQMMC_PAPER_CONN");
    if ((envStrict && envStrict[0] == '1') ||
        (envMepp2 && envMepp2[0] != '0') ||
        (envPaperConn && envPaperConn[0] != '0')) {
        collector = std::make_shared<VQMMCStrictSymbolCollectOperator>(postProcessor);
        std::cout << "[VQMMC] Using STRICT collector (Range Coder, BlobLayout="
                  << kVQMMCStrictBlobLayout << ")" << std::endl;
    } else {
        collector = std::make_shared<VQMMCSymbolCollectOperator>(postProcessor);
    }
    std::shared_ptr<SerializeOperator> serializer =
        std::make_shared<BasicSerializeOperator>(collector, false, true, false);

    if (const char* guard = std::getenv("PMSF_VISUAL_TOPOLOGY_GUARD");
        (guard && guard[0] != '\0' && guard[0] != '0') ||
        (std::getenv("PMSF_VISUAL_ISFLIP_GUARD") &&
         std::getenv("PMSF_VISUAL_ISFLIP_GUARD")[0] != '\0' &&
         std::getenv("PMSF_VISUAL_ISFLIP_GUARD")[0] != '0')) {
        select->addIsRemovableOperator(
            std::make_shared<VisualTopologyIsRemovableOperator>());
    }
    if (const char* guard = std::getenv("PMSF_VISUAL_ISFLIP_GUARD");
        guard && guard[0] != '\0' && guard[0] != '0') {
        select->addIsRemovableOperator(std::make_shared<IsFlipOperator>());
    }
    if (select->options_.getRegisterFunc()) select->options_.getRegisterFunc()(mesh);
    select->init(mesh);
    eliminate->init(mesh, header);

    MCGAL::Halfedge* seed = nullptr;
    MCGAL::Point p;
    HalfedgeErrorEntry entry(seed, p, 0, 0);
    select->select(entry);
    seed = entry.edge;
    collector->init(mesh, {seed}, header);
    auto seedRm = std::make_shared<SeedIsRemovableOperator>();
    seedRm->init({}, {seed}, {});
    select->addIsRemovableOperator(seedRm);

    for (int i = 0; i < args.rounds; i++) {
        MCGAL::contextPool.setMetadata("level", MCGAL::ContextValue(i));
        while (select->select(entry)) eliminate->eliminate(entry);
        postProcessor->process(mesh);
        collector->collect(seed);
        select->reset();
    }
    postProcessor->finalizeDecoderEquivalentGeometry(mesh);
    std::dynamic_pointer_cast<VQMMCSymbolCollectOperator>(collector)
        ->refreshGeometryFromPostProcessor();

    serializer->init();
    serializer->serializeHeader(header);
    serializer->serializeBaseMeshWithSeed(mesh, {seed});
    info.baseVertices = static_cast<int>(mesh->vertices().size());
    info.baseFaces = static_cast<int>(mesh->faces().size());
    info.quantBits = static_cast<int>(mesh->i_nbQuantBits);
    info.quantStep = mesh->f_quantStep;
    for (int axis = 0; axis < 3; ++axis) info.bboxMin[axis] = mesh->bboxMin[axis];
    info.basePrefix.assign(reinterpret_cast<const std::uint8_t*>(serializer->buffer_),
                            reinterpret_cast<const std::uint8_t*>(serializer->buffer_) + serializer->dataOffset_);
    serializer->serialize(args.output);
    return info;
}

#if PMSF_VQMMC_HAS_OPAQUE_PAPER
void writeVQMMCPaperRelations(const PMSFConfig& cfg, const std::string& path,
                              const std::vector<std::uint8_t>& payload,
                              const VQMMCCanonicalBinaryInfo& info) {
    if (payload.size() < 4 + info.basePrefix.size() || info.basePrefix.empty()) {
        throw std::runtime_error("VQMMC paper relations: canonical base prefix is missing");
    }
    std::uint32_t contentLength = 0;
    std::memcpy(&contentLength, payload.data(), 4);
    if (contentLength + 4ull != payload.size() ||
        !std::equal(info.basePrefix.begin(), info.basePrefix.end(), payload.begin() + 4)) {
        throw std::runtime_error("VQMMC paper relations: canonical length/base prefix mismatch");
    }
    std::vector<std::uint8_t> blob(payload.begin() + 4 + info.basePrefix.size(), payload.end());
    pmsf::ConnectivityBlobReader native(blob);
    if (native.numStreams() != 4 || native.numRounds() != cfg.effectiveRounds() ||
        native.header().blob_layout != kVQMMCStrictBlobLayout) {
        throw std::runtime_error("VQMMC paper relations: expected the strict four-stream CONN layout");
    }
    pmsf::PaperStreamTable table;
    auto& object = table.object;
    object.formatVersion = pmsf::kPaperStreamFormatVersionV2;
    object.algorithm = "vqmmc";
    object.schemaMode = pmsf::PaperSchemaMode::Typed;
    object.payloadMode = pmsf::PaperPayloadMode::Typed;
    object.numPartitions = 1;
    object.numRounds = native.numRounds();
    object.totalVertices = info.totalVertices; object.totalFaces = info.totalFaces;
    object.enableQuantization = true; object.quantBits = info.quantBits;
    object.quantStep = info.quantStep;
    for (int axis = 0; axis < 3; ++axis) object.bboxMin[axis] = info.bboxMin[axis];
    object.blobLayout = kVQMMCStrictBlobLayout;
    object.schemeVersion = native.header().scheme_version;
    object.codecVersion = 4;
    object.baseMeshCodec = "vqmmc.binary-prefix";
    object.boundaryCodec.clear(); object.raPrefixUnit = "lod_block";
    for (const char* role : {"F", "Et", "Ef", "V"}) {
        object.streams.push_back({role, pmsf::PaperCellType::Bytes, pmsf::PaperStreamEncoding::Plain,
            "vqmmc.native", 0, true, pmsf::PaperCellShape::Scalar,
            pmsf::PaperStreamRelation::Connectivity, pmsf::PaperStreamSource::Payload});
    }
    for (const char* role : {"Cf", "Cv", "CfQ", "CvQ"}) {
        const bool quantized = std::string(role).back() == 'Q';
        object.streams.push_back({role, quantized ? pmsf::PaperCellType::Int32 : pmsf::PaperCellType::Float,
            pmsf::PaperStreamEncoding::Plain, "vqmmc.indices+codebook", 0, true,
            pmsf::PaperCellShape::Vec3, pmsf::PaperStreamRelation::Geometry,
            pmsf::PaperStreamSource::Payload});
    }
    for (const char* role : {"vqmmc_indices", "vqmmc_codebook"}) {
        object.streams.push_back({role, pmsf::PaperCellType::Bytes, pmsf::PaperStreamEncoding::Plain,
            "vqmmc.native", 0, true, pmsf::PaperCellShape::Scalar,
            pmsf::PaperStreamRelation::Geometry, pmsf::PaperStreamSource::Payload});
    }
    pmsf::PaperPartitionRow partition;
    partition.numLods = native.numRounds();
    partition.numVertices = info.baseVertices; partition.numFaces = info.baseFaces;
    partition.baseMesh = info.basePrefix; partition.baseMeshCodec = object.baseMeshCodec;
    partition.enableQuantization = true; partition.quantBits = info.quantBits;
    partition.quantStep = info.quantStep;
    for (int axis = 0; axis < 3; ++axis) partition.bboxMin[axis] = info.bboxMin[axis];
    table.partitions.push_back(std::move(partition));
    std::uint64_t offset = 0;
    for (int lod = 0; lod < native.numRounds(); ++lod) {
        pmsf::PaperLodRow row; row.lodId = lod;
        auto bytes = [&](int stream) {
            const auto span = native.getStream(lod, stream);
            return std::vector<std::uint8_t>(span.begin(), span.end());
        };
        row.F = bytes(0); row.Et = bytes(1);
        if (row.F.size() >= 4) {
            std::int32_t count = 0; std::memcpy(&count, row.F.data(), 4);
            if (count < 0) throw std::runtime_error("VQMMC paper: negative facet stream count");
            row.countF = count;
        }
        row.extension["vqmmc_indices"] = bytes(2);
        row.extension["vqmmc_codebook"] = bytes(3);
        row.payloadByteOffset = offset;
        row.payloadByteLength = native.roundTotalBytes(lod);
        offset += row.payloadByteLength;
        table.lods.push_back(std::move(row));
    }
    pmsf::PaperWriterV2Options options; options.lodsPerRowGroup = 8;
    pmsf::PaperStreamWriterV2().write(table, path, options);
}
#endif

/// @brief M5-5 (task-35): VQMMC paper backend.
///
/// Encode: reuse the canonical binary serialization verbatim (no re-encoding),
/// then wrap the bytes in the M5-5 opaque-payload P2BL v2 container. Enabled by
/// `PMSF_VQMMC_PAPER_ARTIFACT=1`; without the env (or without the storage
/// helper in this build) the historical fail-fast is preserved.
void runVQMMCCompressPaper(const PMSFConfig& cfg) {
    if (!envOn("PMSF_VQMMC_PAPER_ARTIFACT") && !envOn("PMSF_PAPERSTREAM_V2")) {
        throw std::runtime_error(
            "[VQMMC] Paper backend is not implemented; VQMMC still writes its "
            "own parquet/compact artifact. Use columnar/binary, or keep it out "
            "of the paper coverage matrix. (M5-5: set "
            "PMSF_VQMMC_PAPER_ARTIFACT=1 to write the opaque paper artifact.)");
    }
#if PMSF_VQMMC_HAS_OPAQUE_PAPER
    TimeRecorder recorder("VQMMC paper compress");
    if (cfg.output.empty()) {
        throw std::runtime_error("[VQMMC] paper backend requires an output path");
    }
    const std::string tmpPath = cfg.output + ".vqmmc_canonical.tmp";
    const VQMMCCanonicalBinaryInfo info =
        serializeVQMMCCanonicalBinary(cfg, tmpPath);
    std::vector<std::uint8_t> payload;
    {
        std::ifstream in(tmpPath, std::ios::binary);
        if (!in.is_open()) {
            throw std::runtime_error(
                "[VQMMC] paper backend cannot reopen canonical temp artifact: " +
                tmpPath);
        }
        payload.assign(std::istreambuf_iterator<char>(in),
                       std::istreambuf_iterator<char>());
    }
    std::remove(tmpPath.c_str());
    if (payload.size() < 4) {
        throw std::runtime_error(
            "[VQMMC] paper backend captured an empty canonical artifact");
    }
    pmsf::paper_opaque::OpaquePaperArtifactSpec spec;
    spec.algorithm = "vqmmc";
    spec.blobLayout = kVQMMCStrictBlobLayout;
    spec.codecVersion = 4;
    spec.schemeVersion = 6;  // canonical CONN container scheme_version
    spec.rounds = cfg.effectiveRounds();
    spec.totalVertices = info.totalVertices;
    spec.totalFaces = info.totalFaces;
    spec.baseVertices = 0;   // base/quant header lives inside the opaque payload
    spec.baseFaces = 0;
    spec.boundaryCodec = "";
    std::string error;
    std::string paperStem = cfg.output;
    const std::string::size_type slash = paperStem.find_last_of("/\\");
    const std::string::size_type dot = paperStem.find_last_of('.');
    if (dot != std::string::npos &&
        (slash == std::string::npos || dot > slash)) {
        paperStem.erase(dot);
    }
    const std::string paperPath = paperStem + "_paper.parquet";
    if (const char* schema = std::getenv("PMSF_PAPER_SCHEMA"); schema && std::string(schema) == "typed") {
        writeVQMMCPaperRelations(cfg, paperPath, payload, info);
        return;
    }
    if (!pmsf::paper_opaque::writePaperStreamV2OpaquePayloadArtifact(
            paperPath, payload, spec, pmsf::paper_multistream::PayloadCodec::ZlibRaw, &error)) {
        throw std::runtime_error(
            "[VQMMC] paper artifact write failed: " + error);
    }
#else
    throw std::runtime_error(
        "[VQMMC] Paper artifact encode requires the "
        "PaperStreamOpaquePayloadV2 storage helper (not built in this "
        "configuration); this is an M5-5 pending build.");
#endif
}

/// @brief M5-5 paper decode: read the opaque payload and run the unchanged
///        strict VQMMC decode path (strict reader + reconstructor + postproc).
void runVQMMCDecompressPaper(const PMSFConfig& cfg) {
#if PMSF_VQMMC_HAS_OPAQUE_PAPER
    TimeRecorder recorder("VQMMC paper decompress");
    MCGAL::contextPool.initPoolSize(1);
    MCGAL::contextPool.registerPool(
        MCGAL::ContextPool::poolSizeFromEnv("VQMMC_POOL_V", 500000),
        MCGAL::ContextPool::poolSizeFromEnv("VQMMC_POOL_H", 10000000),
        MCGAL::ContextPool::poolSizeFromEnv("VQMMC_POOL_F", 500000));
    auto des = std::make_shared<VQMMCPaperDeserializeOperator>(
        std::make_shared<VQMMCStrictSymbolReadOperator>(),
        std::make_shared<VQMMCReconstructOperator>(),
        std::make_shared<VQMMCPostProcessor>(cfg.effectiveCodebookSize()));
    des->setQueryMaxLod(cfg.maxLod);
    des->init(cfg.input);
    PMSFHeader header;
    des->deserializeHeader(header);
    auto mesh = des->deserializeBaseMesh();
    int round = header.getRound();
    while (round--) {
        mesh->resetState();
        des->deserializeOneRound();
    }
    if (!cfg.output.empty()) {
        des->exportMesh()->dumpto_oldtype(cfg.output);
    }
#else
    throw std::runtime_error(
        "[VQMMC] Paper artifact decode requires the "
        "PaperStreamOpaquePayloadV2 storage helper (not built in this "
        "configuration); this is an M5-5 pending build.");
#endif
}

}  // namespace

void runVQMMCCompress(const PMSFConfig& cfg) {
    if (cfg.backend == Backend::Paper) {
        runVQMMCCompressPaper(cfg);
        return;
    }
    if (cfg.backend == Backend::Columnar) {
        TimeRecorder recorder("vqmmc_columnar_compress");
        MCGAL::contextPool.initPoolSize(1);
        MCGAL::contextPool.registerPool(
            MCGAL::ContextPool::poolSizeFromEnv("VQMMC_POOL_V", 500000),
            MCGAL::ContextPool::poolSizeFromEnv("VQMMC_POOL_H", 10000000),
            MCGAL::ContextPool::poolSizeFromEnv("VQMMC_POOL_F", 500000));
        MCGAL::contextPool.setMetadata("level", MCGAL::ContextValue(0));

        int rounds = cfg.effectiveRounds();
        int codebookSize = cfg.effectiveCodebookSize();

        PMSFHeader header = PMSFHeader::Builder()
                                .setRound(rounds)
                                .setEnableCompress(false)
                                .setEnableQuantization(true)
                                .setEnableSegmentation(false)
                                .setEnableAdjustMent(false)
                                .build();

        auto mesh = std::make_shared<MCGAL::Mesh>(MCGAL::Mesh(cfg.input));

        auto soptions = SelectOptions::Builder()
                            .withErrorSource(ErrorSource::Halfedge)
                            .withHalfedgeErrorAccessor(nullptr)
                            .withVertexErrorAccessor(nullptr)
                            .withIsRemovableOperator(std::make_shared<BasicIsRemovableOperator>())
                            .withIsRemovableOperator(std::make_shared<UnifiedManifoldIsRemovableOperator>())
                            .withIsRemovableOperator(std::make_shared<DegreeThreeIsRemovableOperator>())
                            .withIsRemovableOperator(std::make_shared<DegreeSixIsRemovableOperator>())
                            .build();

        auto select = std::make_shared<PriorityErrorSelector>(*soptions);
        auto postProcessor = std::make_shared<VQMMCPostProcessor>(codebookSize);
        std::shared_ptr<VertexRemovalEliminateOperator> eliminate =
            std::make_shared<VQMMCTraceEliminateOperator>(postProcessor);
        std::shared_ptr<SymbolCollectOperator> collector =
            std::make_shared<VQMMCSymbolCollectOperator>(postProcessor);
        auto serializer = std::make_shared<VQMMCColumnarSerializeOperator>(
            collector, /*enablePrediction*/false, /*enableQuantization*/true, /*enableCompress*/false);
        auto seedIsRemovableOperator = std::make_shared<SeedIsRemovableOperator>();

        if (const char* guard = std::getenv("PMSF_VISUAL_TOPOLOGY_GUARD");
            (guard && guard[0] != '\0' && guard[0] != '0') ||
            (std::getenv("PMSF_VISUAL_ISFLIP_GUARD") &&
             std::getenv("PMSF_VISUAL_ISFLIP_GUARD")[0] != '\0' &&
             std::getenv("PMSF_VISUAL_ISFLIP_GUARD")[0] != '0')) {
            select->addIsRemovableOperator(
                std::make_shared<VisualTopologyIsRemovableOperator>());
        }
        if (const char* guard = std::getenv("PMSF_VISUAL_ISFLIP_GUARD");
            guard && guard[0] != '\0' && guard[0] != '0') {
            select->addIsRemovableOperator(std::make_shared<IsFlipOperator>());
        }
        if (select->options_.getRegisterFunc()) select->options_.getRegisterFunc()(mesh);
        select->init(mesh);
        eliminate->init(mesh, header);

        MCGAL::Halfedge* seed = nullptr;
        MCGAL::Point p;
        HalfedgeErrorEntry entry(seed, p, 0, 0);
        select->select(entry);
        seed = entry.edge;

        collector->init(mesh, {seed}, header);
        seedIsRemovableOperator->init({}, {seed}, {});
        select->addIsRemovableOperator(seedIsRemovableOperator);

        for (int i = 0; i < rounds; i++) {
            MCGAL::contextPool.setMetadata("level", MCGAL::ContextValue(i));
            while (select->select(entry)) eliminate->eliminate(entry);
            postProcessor->process(mesh);
            collector->collect(seed);
            select->reset();
        }
        postProcessor->finalizeDecoderEquivalentGeometry(mesh);
        std::dynamic_pointer_cast<VQMMCSymbolCollectOperator>(collector)
            ->refreshGeometryFromPostProcessor();

        serializer->init();
        serializer->setHeader(std::make_shared<PMSFHeader>(header));
        serializer->serializeBaseMeshWithSeed(mesh, {seed});
        serializer->serialize(cfg.output);
        return;
    }

    (void)serializeVQMMCCanonicalBinary(cfg, cfg.output);
}

void runVQMMCDecompress(const PMSFConfig& cfg) {
    if (cfg.backend == Backend::Paper) {
        runVQMMCDecompressPaper(cfg);
        return;
    }
    if (cfg.backend == Backend::Columnar) {
        TimeRecorder recorder("vqmmc_columnar_decompress");
        MCGAL::contextPool.initPoolSize(1);
        MCGAL::contextPool.registerPool(
            MCGAL::ContextPool::poolSizeFromEnv("VQMMC_POOL_V", 500000),
            MCGAL::ContextPool::poolSizeFromEnv("VQMMC_POOL_H", 10000000),
            MCGAL::ContextPool::poolSizeFromEnv("VQMMC_POOL_F", 500000));

        auto innerDeserializer = std::make_shared<VQMMCDeserializeOperator>(
            std::make_shared<VQMMCSymbolReadOperator>(),
            std::make_shared<VQMMCReconstructOperator>(),
            std::make_shared<VQMMCPostProcessor>());
        auto deserializer = std::make_shared<VQMMCColumnarDeserializeOperator>(innerDeserializer);

        deserializer->init(cfg.input);
        PMSFHeader header;
        deserializer->deserializeHeader(header);
        auto mesh = deserializer->deserializeBaseMesh();

        int round = cfg.maxLod >= 0 ? std::min(cfg.maxLod, header.getRound()) : header.getRound();
        while (round--) {
            mesh->resetState();
            deserializer->deserializeOneRound();
        }
        if (!cfg.output.empty()) {
            deserializer->exportMesh()->dumpto_oldtype(cfg.output);
        }
        return;
    }

    // Binary 路径
    TimeRecorder recorder("VQMMC decompress");
    MCGAL::contextPool.initPoolSize(1);
    MCGAL::contextPool.registerPool(
        MCGAL::ContextPool::poolSizeFromEnv("VQMMC_POOL_V", 500000),
        MCGAL::ContextPool::poolSizeFromEnv("VQMMC_POOL_H", 10000000),
        MCGAL::ContextPool::poolSizeFromEnv("VQMMC_POOL_F", 500000));

    const char* envStrict = std::getenv("CODEC_030");
    const char* envMepp2 = std::getenv("VQMMC_MEPP2");
    const char* envPaperConn = std::getenv("VQMMC_PAPER_CONN");
    bool useStrict = (envStrict && envStrict[0] == '1') ||
                     (envMepp2 && envMepp2[0] != '0') ||
                     (envPaperConn && envPaperConn[0] != '0');

    std::shared_ptr<VQMMCDeserializeOperator> des;
    if (useStrict) {
        std::cout << "[VQMMC] Using STRICT reader (Range Coder, BlobLayout="
                  << kVQMMCStrictBlobLayout << ")" << std::endl;
        des = std::make_shared<VQMMCDeserializeOperator>(
            std::make_shared<VQMMCStrictSymbolReadOperator>(),
            std::make_shared<VQMMCReconstructOperator>(),
            std::make_shared<VQMMCPostProcessor>());
        des->setStrictMode(true);
    } else {
        des = std::make_shared<VQMMCDeserializeOperator>(
            std::make_shared<VQMMCSymbolReadOperator>(),
            std::make_shared<VQMMCReconstructOperator>(),
            std::make_shared<VQMMCPostProcessor>());
    }
    des->init(cfg.input);
    PMSFHeader header;
    des->deserializeHeader(header);
    auto mesh = des->deserializeBaseMesh();
    int round = cfg.maxLod >= 0 ? std::min(cfg.maxLod, header.getRound()) : header.getRound();
    while (round--) { mesh->resetState(); des->deserializeOneRound(); }
    if (!cfg.output.empty()) des->exportMesh()->dumpto_oldtype(cfg.output);
}

namespace {
/// 契约实现：VQMMC 的码本子系统与单种子重建由算法自己驱动（L2，Phase 4 收敛）。
class VQMMCAlgorithm final : public ISimplificationAlgorithm {
  public:
    AlgorithmInfo info() const override {
        return AlgorithmInfo{
            .name = registeredName("vqmmc", "PMSF_PROBE_VQMMC", "vqmmc_probe"),
            .version = 1,
            .primitive = PrimitiveKind::Vertex,
            .selection = SelectionKind::GeometryDriven,
        };
    }
    void compress(const PMSFConfig& cfg) override { runVQMMCCompress(cfg); }
    /// 解压选项：VQMMC 的符号流只有配套 reader 读得懂；随机访问查询
    /// (runQuery) 走「框架循环 + 本函数给出的解压器」这条路，必须用同一个。
    DecompressOptions makeDecompressOptions(Backend backend, const std::string& input,
                                            const std::string& output) const override {
        if (backend == Backend::Paper) {
#if PMSF_VQMMC_HAS_OPAQUE_PAPER
            return DecompressOptions::Builder()
                .setDeserializeOperator(pmsf::makeVQMMCPaperTriple().deserializer)
                .setPath(input)
                .setOutputPath(output)
                .build();
#else
            throw std::runtime_error(
                "[VQMMC] Paper artifact decode requires the "
                "PaperStreamOpaquePayloadV2 storage helper (not built in this "
                "configuration); this is an M5-5 pending build.");
#endif
        }
        auto inner = std::make_shared<VQMMCDeserializeOperator>(
            std::make_shared<VQMMCSymbolReadOperator>(),
            std::make_shared<VQMMCReconstructOperator>(),
            std::make_shared<VQMMCPostProcessor>());
        auto deserializer = std::make_shared<VQMMCColumnarDeserializeOperator>(inner);
        return DecompressOptions::Builder()
            .setDeserializeOperator(deserializer)
            .setPath(input)
            .setOutputPath(output)
            .build();
    }

    void decompress(const PMSFConfig& cfg) override { runVQMMCDecompress(cfg); }

    /// H1 / `SelectionPolicy`：与 MMSE 同一策略（论文 §4.2 的 PGS 第一刀近似）。
    ///
    /// **默认关**（`PMSF_VQMMC_PGS=1` 开启）—— 与 MMSE 那边不同，这里先做
    /// 「先量后改」的对照实验，量到收益再决定是否翻默认 + 重冻基线。
    /// 默认返回 `{}` ⇒ 框架默认策略 ⇒ **与历史行为逐字节一致**。
    SelectionPolicy selectionPolicy() const override {
        if (!envOn("PMSF_VQMMC_PGS")) {
            return {};
        }
        SelectionPolicy policy;
        policy.errorOverride = [](MCGAL::Halfedge* e, MCGAL::Point& /*p*/) -> float {
            if (e == nullptr) return 0.0f;
            MCGAL::Vertex* a = e->vertex();
            MCGAL::Vertex* b = e->end_vertex();
            const float va = (a != nullptr) ? static_cast<float>(a->vertex_degree()) : 0.0f;
            const float vb = (b != nullptr) ? static_cast<float>(b->vertex_degree()) : 0.0f;
            return va + vb;
        };
        return policy;
    }
};

}  // namespace
}  // namespace pmsf

PMSF_REGISTER_SIMPLIFICATION_ALGORITHM(pmsf::VQMMCAlgorithm);
