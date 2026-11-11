/*
 * @Author: koi
 * @Date: 2025-09-01 20:09:27
 * @Description:
 */

#include "PPMC/compress/PPMCSerializeOperator.h"

#include "PPMC/PPMCFormatConstants.h"
#include "BufferUtils.h"
#include "Configuration.h"
#include "EntropyCodec.h"
#include "Facet.h"
#include "Halfedge.h"
#include "MeshUtils.h"
#include "Point.h"
#include "Vertex.h"
#include "common/DebugTool.h"
#include "PPMC/PPMCHeader.h"
#include "storage/PMSFUnifiedArtifactWriter.h"
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

void PPMCSerializeOperator::init() {
    buffer_ = new char[BUFFER_SIZE];
}

void PPMCSerializeOperator::serializeInt(int val) {
    writeInt(buffer_, dataOffset_, val);
}

void PPMCSerializeOperator::serializeBaseMesh(std::shared_ptr<MCGAL::Mesh> mesh) {}

void PPMCSerializeOperator::serializeBaseMeshWithSeed(std::shared_ptr<MCGAL::Mesh> mesh, std::vector<MCGAL::Halfedge*> seeds) {
    // PPMCHeader* pheader = static_cast<PPMCHeader*>(header_.get());
    mesh->garbage_collection();
    // Write the bounding box min coordinate.

    writePoint(buffer_, dataOffset_, mesh->bboxMin);
    // Write the quantization step.
    writeFloat(buffer_, dataOffset_, mesh->f_quantStep);

    unsigned i_nbVerticesBaseMesh = mesh->size_of_vertices();
    unsigned i_nbFacesBaseMesh = mesh->size_of_facets();
    unsigned i_nbBitsPerVertex = ceil(log(i_nbVerticesBaseMesh) / log(2));

    unsigned i_bitOffset = 0;
    dataOffset_++;

    // Write the codec option status.
    // write header instead
    // writeBits(b_useAdaptiveQuantization, 1, p_data + dataOffset - 1, i_bitOffset, dataOffset);
    // writeBits(b_useLiftingScheme, 1, p_data + dataOffset - 1, i_bitOffset, dataOffset);
    // writeBits(b_useCurvaturePrediction, 1, p_data + dataOffset - 1, i_bitOffset, dataOffset);
    // writeBits(b_useConnectivityPredictionFaces, 1, p_data + dataOffset - 1, i_bitOffset, dataOffset);
    // writeBits(b_useConnectivityPredictionEdges, 1, p_data + dataOffset - 1, i_bitOffset, dataOffset);
    // writeBits(b_useTriangleMeshConnectivityPredictionFaces, 1, p_data + dataOffset - 1, i_bitOffset, dataOffset);

    // Write the geometry quantization of the mesh.
    assert(mesh->i_nbQuantBits - 1 < 1 << 4);
    writeBits(mesh->i_nbQuantBits - 1, 4, buffer_+ dataOffset_ - 1, i_bitOffset, dataOffset_);

    // Write the number of level of decimations.
    // assert(i_nbDecimations < 1 << 6);
    // writeBits(i_nbDecimations, 6, p_data + dataOffset - 1, i_bitOffset, dataOffset);

    // Write the number of adaptive quantizations.
    // assert(i_nbQuantizations < 1 << 6);
    // writeBits(i_nbQuantizations, 6, p_data + dataOffset - 1, i_bitOffset, dataOffset);

    // Write the number of non-convex level of details.
    // writeBits(i_nbDecimations - i_levelNotConvexId, 6, p_data + dataOffset - 1, i_bitOffset, dataOffset);

    // Write the number of vertices and faces on 16 bits.
    writeBits(i_nbVerticesBaseMesh, 16, buffer_+ dataOffset_ - 1, i_bitOffset, dataOffset_);
    writeBits(i_nbFacesBaseMesh, 16, buffer_+ dataOffset_ - 1, i_bitOffset, dataOffset_);

    // Write the base mesh vertex coordinates.
    // P3.e 对齐：lifting scheme 启用时 base mesh 多 1 bit 量化精度 (LIFTING_NB_ADDITIONAL_BITS_GEOMETRY=1)
    unsigned i_nbAdditionalBitsGeometry = 1;

    // Write the vertices of the edge that is the departure of the coding conquests.

    MCGAL::PointInt p = mesh->floatPosToInt(seeds[0]->vertex());
    for (unsigned i = 0; i < 3; ++i) {
        assert(p[i] < 1 << (mesh->i_nbQuantBits - i_curQuantizationId + i_nbAdditionalBitsGeometry));
        writeBits(p[i], mesh->i_nbQuantBits - i_curQuantizationId + i_nbAdditionalBitsGeometry, buffer_+ dataOffset_ - 1, i_bitOffset, dataOffset_);
    }
    seeds[0]->vertex()->setVid(0);
    p = mesh->floatPosToInt(seeds[0]->end_vertex());
    for (unsigned i = 0; i < 3; ++i) {
        assert(p[i] < 1 << (mesh->i_nbQuantBits - i_curQuantizationId + i_nbAdditionalBitsGeometry));
        writeBits(p[i], mesh->i_nbQuantBits - i_curQuantizationId + i_nbAdditionalBitsGeometry, buffer_+ dataOffset_ - 1, i_bitOffset, dataOffset_);
    }
    seeds[0]->end_vertex()->setVid(1);

    // Write the other vertices.
    size_t id = 2;
    for (MCGAL::Vertex* vit : mesh->vertices()) {
        if (vit == seeds[0]->vertex() || vit == seeds[0]->end_vertex())
            continue;

        MCGAL::PointInt p = mesh->floatPosToInt(vit->point());

        // Write the coordinates.
        for (unsigned i = 0; i < 3; ++i) {
            // assert(p[i] < 1 << (mesh->i_nbQuantBits - i_curQuantizationId + i_nbAdditionalBitsGeometry));
            if(p[i] >= 1 << (mesh->i_nbQuantBits - i_curQuantizationId + i_nbAdditionalBitsGeometry)) {
                // printf("p[%d] = %d\n", i, p[i]);
                p[i] = 1 << (mesh->i_nbQuantBits - i_curQuantizationId + i_nbAdditionalBitsGeometry);
                p[i]--;
            }
            writeBits(p[i], (mesh->i_nbQuantBits - i_curQuantizationId + i_nbAdditionalBitsGeometry), buffer_+ dataOffset_ - 1, i_bitOffset, dataOffset_);
        }
        // Set an id to the vertex.
        vit->setVid(id++);
    }
    // Write the base mesh face vertex indices.
    std::vector<int> fids;
    for (MCGAL::Facet* fit : mesh->faces()) {
        unsigned i_faceDegree = fit->facet_degree();
        unsigned i_code = i_faceDegree - 3;
        // 位宽与解码端 PPMCDeserializeOperator 的 NB_BITS_FACE_DEGREE_BASE_MESH
        // 必须一致。3 位（degree ≤ 10）承载不了 LetterA 这类含高次面的基础网格，
        // 加宽到 4 位（degree ≤ 18）。
        static_assert(NB_BITS_FACE_DEGREE_BASE_MESH == 4,
                      "位宽改了要同步改这里的 assert 与 writeBits 的字面量");
        assert(i_code < 1u << NB_BITS_FACE_DEGREE_BASE_MESH);
        writeBits(i_code, NB_BITS_FACE_DEGREE_BASE_MESH, buffer_+ dataOffset_ - 1, i_bitOffset, dataOffset_);

        MCGAL::Halfedge* hit = fit->proxyHalfedge();
        MCGAL::Halfedge* end = hit;
        do {
            // Write the current vertex id.
            fids.push_back(hit->vertex()->vid());
            writeBits(hit->vertex()->vid(), i_nbBitsPerVertex, buffer_+ dataOffset_ - 1, i_bitOffset, dataOffset_);
            hit = hit->next();
        } while (hit != end);
    }

    if (i_bitOffset == 0)
        dataOffset_--;
}

void PPMCSerializeOperator::serialize(std::string path) {
    int size = collector_->exportToBuffer(buffer_ + dataOffset_, enableQuantization_, enableCompress_);
    char* outBlock = buffer_;
    int outBlockSize = dataOffset_ + size;
    // if (enableCompress_) {
    //     serializeCharPointer(buffer_, dataOffset_ + size, outBlock, outBlockSize);
    // }

    writeLengthPrefixedLegacySingleFile(path, outBlock,
                                        static_cast<std::size_t>(outBlockSize),
                                        0);
}
