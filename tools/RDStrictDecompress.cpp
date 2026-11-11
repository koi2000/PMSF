#include "RD/decompress/RDStrictSymbolReadOperator.h"

#include "Mesh.h"
#include "core/MeshRuntime.h"
#include "predict/RDPredictor.h"
#include <nlohmann/json.hpp>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: RDStrictDecompress <output-prefix> <out.off>\n";
        return 2;
    }
    const std::string prefix = argv[1];
    const std::string outPath = argv[2];
    std::ifstream metaIn(prefix + "_meta.json");
    if (!metaIn) {
        std::cerr << "RDStrictDecompress: missing meta\n";
        return 1;
    }
    nlohmann::json meta;
    metaIn >> meta;

    setenv("PMSF_POOL_V", "1000000", 0);
    setenv("PMSF_POOL_H", "10000000", 0);
    setenv("PMSF_POOL_F", "4000000", 0);
    const int vin = meta.value("vertices_in", 0);
    const int fin = meta.value("faces_in", 0);
    if (vin > 0 && fin > 0) pmsf::ensureMeshRuntimeInitializedForMeshSize(vin, fin);
    else pmsf::ensureMeshRuntimeInitialized();
    auto mesh = std::make_shared<MCGAL::Mesh>(prefix + "_base.off");
    mesh->i_nbQuantBits = meta.value("quant_bits", 12);
    mesh->f_quantStep = meta.value("quant_step", 1.0f);
    auto bmin = meta["bbox_min"];
    mesh->bboxMin = MCGAL::Point(bmin[0].get<float>(), bmin[1].get<float>(), bmin[2].get<float>());
    // base OFF already carries encoder-quantized float positions; do not re-quantize.

    std::ifstream blobIn(prefix + "_rdstrict.blob", std::ios::binary);
    std::vector<char> blob((std::istreambuf_iterator<char>(blobIn)),
                           std::istreambuf_iterator<char>());
    if (blob.empty()) {
        std::cerr << "RDStrictDecompress: empty blob\n";
        return 1;
    }

    pmsf::RDStrictSymbolReadOperator reader;
    reader.setMesh(mesh);
    size_t offset = 0;
    reader.importFromRangeCodedBuffer(blob.data(), offset, meta.value("quant_bits", 12));
    std::vector<MCGAL::Vertex*> vertices;
    std::vector<MCGAL::Halfedge*> halfedges;
    std::vector<MCGAL::Facet*> facets;
    reader.collectFromDecoded(nullptr, vertices, halfedges, facets);
    auto out = reader.mesh();
    if (!out) {
        std::cerr << "RDStrictDecompress: no mesh\n";
        return 1;
    }
    out->dumpto_oldtype(outPath);
    std::cout << "RDStrictDecompress V=" << out->size_of_vertices()
              << " F=" << out->size_of_facets() << "\n";
    return 0;
}
