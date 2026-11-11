#include "RD/compress/RDStrictSymbolCollectOperator.h"
#include "common/patch/RDPatchPipeline.h"
#include "core/MeshRuntime.h"
#include "header/PMSFHeader.h"

#include "Mesh.h"
#include "predict/RDPredictor.h"
#include <nlohmann/json.hpp>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: RDStrictCompress <input.off> <output-prefix> [qbits] [target-base]\n";
        return 2;
    }
    const std::string input = argv[1];
    const std::string prefix = argv[2];
    const int qbits = argc > 3 ? std::atoi(argv[3]) : 12;
    const int targetBase = argc > 4 ? std::atoi(argv[4]) : 50;

    setenv("PMSF_POOL_V", "1000000", 0);
    setenv("PMSF_POOL_H", "10000000", 0);
    setenv("PMSF_POOL_F", "4000000", 0);
    pmsf::ensureMeshRuntimeInitializedForInputFile(input);
    MCGAL::contextPool.attachVertexProperty<MCGAL::PointInt>(
        DEFAULT_MESH_ID, MCGAL::kVPropRDR1);
    auto mesh = std::make_shared<MCGAL::Mesh>(input);
    mesh->i_nbQuantBits = static_cast<unsigned>(qbits);
    mesh->quantizeVertexPositions();
    const MCGAL::Point inputBBoxMin = mesh->bboxMin;
    const float inputQuantStep = mesh->f_quantStep;
    const int inputV = mesh->size_of_vertices();
    const int inputF = mesh->size_of_facets();

    pmsf::RDPatchPipelineConfig cfg;
    cfg.strictRDConquest = true;
    cfg.usePMSFBoundaryGates = true;
    cfg.maxCycles = 1;
    if (const char* mr = std::getenv("PMSF_RDSTRICT_MAX_REMOVALS_PER_RUN")) {
        cfg.maxRemovalsPerRun = std::atoi(mr);
    }
    if (const char* mc = std::getenv("PMSF_RDSTRICT_MAX_CYCLES")) {
        const int v = std::atoi(mc);
        if (v > 0) cfg.maxCycles = v;
    }
    cfg.useRDTagEliminate = true;
    cfg.maxStepsMultiplier = 50;

    pmsf::RDStrictSymbolCollectOperator collector;
    collector.setPipelineConfig(cfg);
    PMSFHeader header;
    collector.init(mesh, {}, header);
    collector.collect(nullptr);
    const auto& result = collector.lastResult();

    std::vector<char> blob(256u * 1024u * 1024u);
    const int blobBytes = collector.exportToBufferWithRangeCoder(blob.data(), true);
    if (blobBytes <= 0) {
        std::cerr << "RDStrictCompress: empty blob\n";
        return 1;
    }
    blob.resize(static_cast<size_t>(blobBytes));

    const std::string basePath = prefix + "_base.off";
    const std::string blobPath = prefix + "_rdstrict.blob";
    const std::string metaPath = prefix + "_meta.json";
    if (!result.mesh) {
        std::cerr << "RDStrictCompress: missing base mesh\n";
        return 1;
    }
    result.mesh->dumpto_oldtype(basePath);
    {
        std::ofstream os(blobPath, std::ios::binary);
        os.write(blob.data(), static_cast<std::streamsize>(blob.size()));
    }

    nlohmann::json meta;
    meta["layout"] = 29;
    meta["scheme_version"] = 1;
    meta["streams"] = 3;
    meta["quant_bits"] = qbits;
    meta["quant_step"] = inputQuantStep;
    meta["bbox_min"] = nlohmann::json::array({inputBBoxMin.x(), inputBBoxMin.y(), inputBBoxMin.z()});
    meta["vertices_in"] = inputV;
    meta["faces_in"] = inputF;
    meta["vertices_base"] = result.numVerticesOut;
    meta["faces_base"] = result.mesh->size_of_facets();
    meta["removed"] = result.numDecimatedTotal;
    meta["null_patches"] = result.numNullPatches;
    meta["blob_bytes"] = blob.size();
    meta["connectivity_bytes"] = collector.lastConnectivityBytes();
    meta["geometry_bytes"] = collector.lastGeometryBytes();
    meta["sideinfo_bytes"] = collector.lastSideInfoBytes();
    meta["container_overhead_bytes"] = collector.lastContainerOverheadBytes();
    meta["base_bytes"] = 0;
    meta["target_base"] = targetBase;
    std::ofstream(metaPath) << meta.dump(2) << "\n";

    std::cout << "RDStrictCompress layout=29 blob=" << blob.size()
              << " V_in=" << result.numVerticesIn
              << " V_base=" << result.mesh->size_of_vertices()
              << " removed=" << result.numDecimatedTotal << "\n";
    return 0;
}
