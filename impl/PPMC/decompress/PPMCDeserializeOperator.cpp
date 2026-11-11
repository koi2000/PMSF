#include "PPMC/decompress/PPMCDeserializeOperator.h"
#include "common/BoundedRingTraversal.h"

#include "PPMC/PPMCFormatConstants.h"
#include "PPMC/decompress/PPMCSymbolReadOperator.h"
#include "BufferUtils.h"
#include "Configuration.h"
#include "MeshUtils.h"
#include "Point.h"
#include "Vertex.h"

#include <cstdio>
#include <cstdlib>


std::shared_ptr<MCGAL::Mesh> PPMCDeserializeOperator::deserializeBaseMesh() {
    // printf("Reading the base mesh.\n");
    MCGAL::Point bboxMin;
    float f_quantStep;
    int i_nbQuantBits;
    // Read the bounding box min coordinate.
    float coord[3];
    for (unsigned i = 0; i < 3; ++i) {
        coord[i] = readFloat(buffer_, dataOffset_);
    }
    bboxMin = MCGAL::Point(coord[0], coord[1], coord[2]);

    // Read the quantization step.
    f_quantStep = readFloat(buffer_, dataOffset_);

    unsigned i_bitOffset = 0;
    dataOffset_++;

    // TODO: 暂时先不在header中存储这些东西
    // b_useAdaptiveQuantization = readBits(1, p_data + dataOffset - 1, i_bitOffset, dataOffset);
    // b_useLiftingScheme = readBits(1, p_data + dataOffset - 1, i_bitOffset, dataOffset);
    // b_useCurvaturePrediction = readBits(1, p_data + dataOffset - 1, i_bitOffset, dataOffset);
    // b_useConnectivityPredictionFaces = readBits(1, p_data + dataOffset - 1, i_bitOffset, dataOffset);
    // b_useConnectivityPredictionEdges = readBits(1, p_data + dataOffset - 1, i_bitOffset, dataOffset);
    // b_useTriangleMeshConnectivityPredictionFaces = readBits(1, p_data + dataOffset - 1, i_bitOffset, dataOffset);
    // 暂时先移除AdaptiveQuantization
    i_nbQuantBits = readBits(4, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_) + 1;
    // i_quantBits = readBits(4, p_data + dataOffset - 1, i_bitOffset, dataOffset) + 1;
    // i_nbDecimations = readBits(6, p_data + dataOffset - 1, i_bitOffset, dataOffset);
    // i_nbQuantizations = i_curQuantizationId = readBits(6, p_data + dataOffset - 1, i_bitOffset, dataOffset);
    // i_levelNotConvexId = readBits(6, p_data + dataOffset - 1, i_bitOffset, dataOffset);

    // Set the mesh bounding box.
    // unsigned i_nbQuantStep = 1 << mesh_->i_nbQuantBits;
    // bbMax = bbMin + Vector(i_nbQuantStep * f_quantStep,
    //                        i_nbQuantStep * f_quantStep,
    //                        i_nbQuantStep * f_quantStep);

    unsigned i_nbVerticesBaseMesh = readBits(16, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
    unsigned i_nbFacesBaseMesh = readBits(16, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
    unsigned i_nbBitsPerVertex = ceil(log(i_nbVerticesBaseMesh) / log(2));

    std::deque<MCGAL::Point>* p_pointDeque = new std::deque<MCGAL::Point>();
    std::deque<uint32_t*>* p_faceDeque = new std::deque<uint32_t*>();
    // P3.e 对齐：lifting scheme 启用时 base mesh 多 1 bit 量化精度
    unsigned i_nbAdditionalBitsGeometry = 1;

    // Read the vertex positions (with extra bit for lifting scheme precision).
    unsigned i_nbBitsPerCoord = i_nbQuantBits + i_nbAdditionalBitsGeometry;
    for (unsigned i = 0; i < i_nbVerticesBaseMesh; ++i) {
        uint32_t p[3];
        for (unsigned j = 0; j < 3; ++j)
            p[j] = readBits(i_nbBitsPerCoord, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
        MCGAL::PointInt posInt(p[0], p[1], p[2]);
        MCGAL::Point pos = MCGAL::intPosToFloat(posInt, bboxMin, f_quantStep);
        p_pointDeque->push_back(pos);
    }
    std::vector<int> fids;
    // Read the face vertex indices.
    for (unsigned i = 0; i < i_nbFacesBaseMesh; ++i) {
        uint32_t* f = new uint32_t[(1 << NB_BITS_FACE_DEGREE_BASE_MESH) + 3];

        // Write in the first cell of the array the face degree.
        f[0] = readBits(NB_BITS_FACE_DEGREE_BASE_MESH, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_) + 3;

        for (unsigned j = 1; j < f[0] + 1; ++j) {
            f[j] = readBits(i_nbBitsPerVertex, buffer_ + dataOffset_ - 1, i_bitOffset, dataOffset_);
            fids.push_back(f[j]);
        }
        p_faceDeque->push_back(f);
    }
    mesh_ = MCGAL::buildFromBufferNonTri(DEFAULT_MESH_ID, p_pointDeque, p_faceDeque);
    mesh_->bboxMin = bboxMin;
    mesh_->f_quantStep = f_quantStep;
    mesh_->i_nbQuantBits = i_nbQuantBits;
    MCGAL::Vertex* v0 = mesh_->vertices()[0];
    MCGAL::Vertex* v1 = mesh_->vertices()[1];
    for (MCGAL::Halfedge* hit : pmsf::boundedRing(v0)) {
        if (hit->end_vertex() == v1) {
            seed = hit;
        }
    }
    // P3 对齐（留待后续 P3.b 落地）：记录 seed 两端点，本意是每轮重新查找 v0→v1
    //   halfedge 以绕过 MCGAL 的 stale halfedge 列表问题。但实测 reconstruct 后
    //   v0→v1 halfedge 会被 `join_face` 删除（setRemoved），说明根因在符号流对称性
    //   而不仅仅是 halfedge 指针失效。暂时保留这两个字段但不使用；后续 P3.b 解决
    //   符号流对称性后再启用。
    seedV0 = v0;
    seedV1 = v1;
    if (std::getenv("PMSF_DBG_PPMC")) {
        std::fprintf(stderr,
                     "[DBG-ppmc-base] verts=%zu faces=%zu seed=%p v0=%p v1=%p quantBits=%u\n",
                     mesh_->vertices().size(), mesh_->faces().size(), (void*)seed, (void*)v0,
                     (void*)v1, i_nbQuantBits);
    }
    // Free the memory.
    for (unsigned i = 0; i < p_faceDeque->size(); ++i)
        delete[] p_faceDeque->at(i);
    delete p_faceDeque;
    delete p_pointDeque;

    if (i_bitOffset == 0)
        dataOffset_--;
    reconstructOperator_->init(mesh_);
    return mesh_;
}

void PPMCDeserializeOperator::deserializeOneRound() {
    const bool tprof = [] {
        const char* e = std::getenv("PMSF_PPMC_TPROF");
        return e && *e && *e != '0';
    }();
    auto tnow = [] { return std::chrono::steady_clock::now(); };
    auto tms = [](const std::chrono::steady_clock::time_point& a,
                  const std::chrono::steady_clock::time_point& b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
    };
    const auto td0 = tnow();
    std::vector<MCGAL::Vertex*> vertices;
    std::vector<MCGAL::Halfedge*> halfedges;
    std::vector<MCGAL::Facet*> facets;
    if (header_.isEnableQuantization()) {
        symbolReadOperator_->enableQuantization(mesh_->bboxMin, mesh_->f_quantStep);
    }

    // ⚠️ P3.e 对齐（2026-05-14）：每轮解码前重置所有 face 和 halfedge 状态标志。
    //   原版 PPMC `beginUndecimationConquest()` 对全网格调用 hit->resetState() 和
    //   fit->resetState()。若跳过这一步，上一轮 undecimationStep 标记的
    //   Splittable/Unsplittable/Processed 标志会残留，导致 isConquered() / isProcessed()
    //   提前返回 true，BFS 无法遍历到这些面/边，从第二轮起即出现"边合并=0 面片分裂=0"。
    mesh_->resetState();
    const auto td1 = tnow();

    // ⚠️ P3.c 对齐（2026-05-01）：每轮 seed 刷新（原版 PPMC pushHehInit 等价）。
    //   OpenMesh 的顺序是 resetState() 后再 pushHehInit()；refresh 不能读取上一轮
    //   残留的 Splittable/Unsplittable 状态。
    if (seedV0 && seedV1) {
        auto findLiveHalfedge = [this](MCGAL::Vertex* from,
                                       MCGAL::Vertex* to) -> MCGAL::Halfedge* {
            if (!mesh_ || !from) return nullptr;
            for (MCGAL::Facet* f : mesh_->faces()) {
                if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                MCGAL::Halfedge* h = f->proxyHalfedge();
                MCGAL::Halfedge* start = h;
                int guard = 0;
                do {
                    if (h && !h->isRemoved() && h->vertex() == from &&
                        (to == nullptr || h->end_vertex() == to)) {
                        return h;
                    }
                    h = h ? h->next() : nullptr;
                } while (h && h != start && ++guard <= 2048);
            }
            return nullptr;
        };

        if (MCGAL::Halfedge* fresh = findLiveHalfedge(seedV0, seedV1)) {
            seed = fresh;
        } else if (MCGAL::Halfedge* fallback = findLiveHalfedge(seedV0, nullptr)) {
            seed = fallback;
        } else if (MCGAL::Halfedge* fallback = findLiveHalfedge(seedV1, nullptr)) {
            seed = fallback;
        }
        // 找不到 fresh 时保留上一轮 seed；collect 会在无效输入上尽早暴露错误。
    }

    if (auto* ppmcRdr = dynamic_cast<PPMCSymbolReadOperator*>(symbolReadOperator_.get())) {
        ppmcRdr->setMesh(mesh_);
    }
    const auto td2 = tnow();
    symbolReadOperator_->collect(seed, buffer_, dataOffset_, vertices, halfedges, facets);
    const auto td3 = tnow();

    reconstructOperator_->reconstruct(vertices, halfedges, facets);
    const auto td4 = tnow();
    if (tprof) {
        std::fprintf(stderr, "[PPMC_DTPROF] reset=%.1f seed=%.1f collect=%.1f reconstruct=%.1f total=%.1f\n",
                     tms(td0, td1), tms(td1, td2), tms(td2, td3), tms(td3, td4), tms(td0, td4));
    }
}
