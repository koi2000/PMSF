/*
 * @Author: koi
 * @Description: Parquet V2 多表方案序列化操作符实现
 *
 * 4 个独立 Parquet 文件：
 *   - <basename>_metadata.parquet   : 1 行，PMSFHeader 元数据
 *   - <basename>_base_mesh.parquet  : N 行（N=group数），每 group 一行
 *   - <basename>_symbols.parquet    : N*R 行（N=group数, R=round数），每 (group,round) 一行
 *   - <basename>_boundary.parquet   : 边界图（可选）
 */
#include "ColumnarSerializeOperator.h"
#include "CanonicalArrowTableBuilder.h"
#include "PMSFCollectorLayoutRegistry.h"
#include "core/CanonicalSymbolCollector.h"
#include "storage/PMSFSymbolStorageAdapter.h"
#include "decompress/OptimizedSymbolReadOperator.h"
#include "storage/PMSFStorageContextSideChannel.h"
#include "CollectorSymbolsTableBuilder.h"
#include "compress/ConnectivityBlobContainer.h"
#include "Configuration.h"   // BUFFER_SIZE
#include "Facet.h"
#include "Halfedge.h"
#include "Point.h"
#include "Vertex.h"
#include "storage/PMSFUnifiedArtifactWriter.h"
#include "PMSFPayloadAdapterRegistry.h"
#include "scheme/StorageSchemeRegistry.h"
#include "storage/PMSFCompactBinEnvelope.h"
#include "EdgeBreakerMeshAdapter.h"
#include <cstdint>
#include <cstring>
#include <array>
#include <algorithm>
#include <climits>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include <sstream>
#include <set>
#include <string>
#include <system_error>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <zlib.h>
#ifdef PMSF_HAS_ZSTD
#include <zstd.h>
#endif

#include <arrow/api.h>
#include <arrow/builder.h>
#include <arrow/io/api.h>
#include <parquet/arrow/writer.h>
#include <parquet/properties.h>

// ============================================================================
// 初始化
// ============================================================================

void ColumnarSerializeOperator::init() {
    buffer_ = nullptr;
    dataOffset_ = 0;
    groupBaseMeshData_.clear();
    sharedVertexX_.clear();
    sharedVertexY_.clear();
    sharedVertexZ_.clear();
    sharedVertexQX_.clear();
    sharedVertexQY_.clear();
    sharedVertexQZ_.clear();
    totalVertices_ = 0;
    totalFaces_ = 0;
    originalInputVertices_ = 0;
    originalInputFaces_ = 0;
}

void ColumnarSerializeOperator::serializeInt(int val) {
    (void)val;
}

void ColumnarSerializeOperator::serializeBaseMesh(std::shared_ptr<MCGAL::Mesh> mesh) {
    (void)mesh;
}

// ============================================================================
// 基础网格序列化：提取结构化数据（与之前相同）
// ============================================================================

void ColumnarSerializeOperator::serializeBaseMeshWithSeed(
    std::shared_ptr<MCGAL::Mesh> mesh, std::vector<MCGAL::Halfedge*> seeds) {

    groupBaseMeshData_.clear();
    sharedVertexX_.clear();
    sharedVertexY_.clear();
    sharedVertexZ_.clear();
    sharedVertexQX_.clear();
    sharedVertexQY_.clear();
    sharedVertexQZ_.clear();
    if (seeds.empty()) {
        std::cerr << "[Columnar] serializeBaseMeshWithSeed: seeds 为空，无法序列化基础网格" << std::endl;
        return;
    }

    if (getBlobLayout() == pmsf::kPaperBlobLayoutCodec025) {
        totalVertices_ = 0;
        totalFaces_ = 0;
        for (MCGAL::Vertex* v : mesh->vertices()) {
            if (v && !v->isRemoved()) ++totalVertices_;
        }
        for (MCGAL::Facet* f : mesh->faces()) {
            if (f && !f->isRemoved()) ++totalFaces_;
        }
    } else {
        mesh->garbage_collection();
        totalVertices_ = mesh->size_of_vertices();
        totalFaces_ = mesh->size_of_facets();
    }


    bool isSegmented = header_->isEnableSegmentation();
    int groupNumber = isSegmented ? header_->getGroupNumber() : 1;
    if (groupNumber <= 0) groupNumber = 1;
    if (isSegmented && groupNumber > static_cast<int>(seeds.size())) {
        std::cerr << "[Columnar] 分区数量(" << groupNumber
                  << ") 大于 seeds 数量(" << seeds.size()
                  << ")，按 seeds 数量截断" << std::endl;
        groupNumber = static_cast<int>(seeds.size());
        header_->setGroupNumber(groupNumber);
    }
    if (groupNumber <= 1) {
        isSegmented = false;
        header_->setGroupNumber(1);
    }
    const bool preservesLiftedBaseMesh = collector_->preservesLiftedBaseMesh();
    const int baseMeshQuantBits = static_cast<int>(mesh->i_nbQuantBits) +
                                  (preservesLiftedBaseMesh ? 1 : 0);
    auto clampBaseMeshQuantCoord = [&](int32_t v) -> int32_t {
        if (v < 0) return 0;
        if (baseMeshQuantBits > 0 && baseMeshQuantBits < 31) {
            const int32_t maxV = (1 << baseMeshQuantBits) - 1;
            if (v > maxV) return maxV;
        }
        return v;
    };

    if (isSegmented && groupNumber > 1) {
        // ========== 分区模式 ==========
        const bool sharedCoords = sharedBaseMeshCoordinates_;
        const bool faceCoordIds = sharedCoords && sharedBaseMeshFaceCoordIds_;
        std::map<std::array<int32_t, 3>, int32_t> sharedQCoordIds;
        std::map<std::array<float, 3>, int32_t> sharedFloatCoordIds;
        std::map<int32_t, int32_t> sharedGlobalVertexIds;
        auto sharedCoordId = [&](const MCGAL::Point& p) -> int32_t {
            if (enableQuantization_) {
                MCGAL::PointInt pi = mesh->floatPosToInt(p);
                for (int i = 0; i < 3; i++) {
                    pi[i] = clampBaseMeshQuantCoord(pi[i]);
                }
                std::array<int32_t, 3> key{pi[0], pi[1], pi[2]};
                auto it = sharedQCoordIds.find(key);
                if (it != sharedQCoordIds.end()) return it->second;
                int32_t id = static_cast<int32_t>(sharedVertexQX_.size());
                sharedQCoordIds.emplace(key, id);
                sharedVertexQX_.push_back(key[0]);
                sharedVertexQY_.push_back(key[1]);
                sharedVertexQZ_.push_back(key[2]);
                return id;
            }
            std::array<float, 3> key{static_cast<float>(p.x()),
                                     static_cast<float>(p.y()),
                                     static_cast<float>(p.z())};
            auto it = sharedFloatCoordIds.find(key);
            if (it != sharedFloatCoordIds.end()) return it->second;
            int32_t id = static_cast<int32_t>(sharedVertexX_.size());
            sharedFloatCoordIds.emplace(key, id);
            sharedVertexX_.push_back(key[0]);
            sharedVertexY_.push_back(key[1]);
            sharedVertexZ_.push_back(key[2]);
            return id;
        };
        auto sharedCoordIdForVertex = [&](MCGAL::Vertex* v) -> int32_t {
            if (!faceCoordIds) {
                return sharedCoordId(v->point());
            }
            const int32_t globalId = v->globalVertexId();
            auto it = sharedGlobalVertexIds.find(globalId);
            if (it != sharedGlobalVertexIds.end()) return it->second;
            const int32_t id = enableQuantization_
                                   ? static_cast<int32_t>(sharedVertexQX_.size())
                                   : static_cast<int32_t>(sharedVertexX_.size());
            sharedGlobalVertexIds.emplace(globalId, id);
            if (enableQuantization_) {
                MCGAL::PointInt pi = mesh->floatPosToInt(v->point());
                for (int i = 0; i < 3; i++) {
                    pi[i] = clampBaseMeshQuantCoord(pi[i]);
                }
                sharedVertexQX_.push_back(pi[0]);
                sharedVertexQY_.push_back(pi[1]);
                sharedVertexQZ_.push_back(pi[2]);
            } else {
                MCGAL::Point p = v->point();
                sharedVertexX_.push_back(static_cast<float>(p.x()));
                sharedVertexY_.push_back(static_cast<float>(p.y()));
                sharedVertexZ_.push_back(static_cast<float>(p.z()));
            }
            return id;
        };

        std::set<MCGAL::Vertex*> seedVertexSet;
        for (auto* seed : seeds) {
            seedVertexSet.insert(seed->vertex());
            seedVertexSet.insert(seed->end_vertex());
        }

        int globalVid = seeds.size() * 2;

        for (int gid = 0; gid < groupNumber; gid++) {
            GroupBaseMeshData data;
            data.groupId = gid;
            data.faceVerticesAreCoordIds = faceCoordIds;

            MCGAL::Halfedge* seed = seeds[gid];
            MCGAL::Point p0 = seed->vertex()->point();
            MCGAL::Point p1 = seed->end_vertex()->point();
            data.seedV0X = p0.x(); data.seedV0Y = p0.y(); data.seedV0Z = p0.z();
            data.seedV1X = p1.x(); data.seedV1Y = p1.y(); data.seedV1Z = p1.z();
            if (sharedCoords) {
                data.seedV0CoordId = sharedCoordIdForVertex(seed->vertex());
                data.seedV1CoordId = sharedCoordIdForVertex(seed->end_vertex());
            }

            if (enableQuantization_) {
                MCGAL::PointInt pi0 = mesh->floatPosToInt(p0);
                MCGAL::PointInt pi1 = mesh->floatPosToInt(p1);
                data.seedV0QX = pi0[0]; data.seedV0QY = pi0[1]; data.seedV0QZ = pi0[2];
                data.seedV1QX = pi1[0]; data.seedV1QY = pi1[1]; data.seedV1QZ = pi1[2];
            }

            seed->vertex()->setVid(sharedCoords ? 0 : gid * 2);
            seed->end_vertex()->setVid(sharedCoords ? 1 : gid * 2 + 1);

            for (MCGAL::Vertex* vit : mesh->vertices()) {
                if (vit->isRemoved()) continue;
                if (sharedCoords) {
                    if (vit == seed->vertex() || vit == seed->end_vertex()) continue;
                } else if (seedVertexSet.count(vit)) {
                    continue;
                }
                if (vit->groupId() != gid) continue;

                MCGAL::Point point = vit->point();
                if (sharedCoords && !faceCoordIds) {
                    data.vertexCoordIds.push_back(sharedCoordIdForVertex(vit));
                } else if (!sharedCoords) {
                    data.vertexX.push_back(point.x());
                    data.vertexY.push_back(point.y());
                    data.vertexZ.push_back(point.z());
                }

                if (enableQuantization_) {
                    MCGAL::PointInt pi = mesh->floatPosToInt(point);
                    for (int i = 0; i < 3; i++) {
                        pi[i] = clampBaseMeshQuantCoord(pi[i]);
                    }
                    if (!sharedCoords) {
                        data.vertexQX.push_back(pi[0]);
                        data.vertexQY.push_back(pi[1]);
                        data.vertexQZ.push_back(pi[2]);
                    }
                }
                if (!faceCoordIds) {
                    vit->setVid(sharedCoords ? static_cast<int>(data.vertexCoordIds.size() + 1)
                                             : globalVid++);
                }
            }
            data.numVertices = sharedCoords ? static_cast<int>(data.vertexCoordIds.size())
                                            : static_cast<int>(data.vertexX.size());
            groupBaseMeshData_.push_back(std::move(data));
        }

        std::vector<std::unordered_map<int32_t, int32_t>> sharedLocalVid;
        if (sharedBaseMeshCoordinates_ && !sharedBaseMeshFaceCoordIds_) {
            sharedLocalVid.resize(groupBaseMeshData_.size());
            for (size_t gid = 0; gid < groupBaseMeshData_.size(); ++gid) {
                auto& data = groupBaseMeshData_[gid];
                sharedLocalVid[gid][data.seedV0CoordId] = 0;
                sharedLocalVid[gid][data.seedV1CoordId] = 1;
                for (size_t i = 0; i < data.vertexCoordIds.size(); ++i) {
                    sharedLocalVid[gid][data.vertexCoordIds[i]] = static_cast<int32_t>(i + 2);
                }
            }
        }
        auto sharedLocalIdForVertex = [&](int gid, MCGAL::Vertex* v) -> int32_t {
            int32_t cid = sharedCoordIdForVertex(v);
            auto& mp = sharedLocalVid[gid];
            auto it = mp.find(cid);
            if (it != mp.end()) return it->second;
            auto& data = groupBaseMeshData_[gid];
            int32_t localId = static_cast<int32_t>(data.vertexCoordIds.size() + 2);
            data.vertexCoordIds.push_back(cid);
            data.numVertices = static_cast<int>(data.vertexCoordIds.size());
            mp[cid] = localId;
            return localId;
        };

        // 收集面片
        for (MCGAL::Facet* fit : mesh->faces()) {
            if (fit->isRemoved()) continue;
            MCGAL::Halfedge* st = fit->proxyHalfedge();
            MCGAL::Halfedge* ed = st;
            std::vector<int32_t> faceVids;
            int faceGroupId = -1;
            bool openFace = false;
            while (st) {
                if (faceGroupId == -1) faceGroupId = st->vertex()->groupId();
                if (sharedBaseMeshCoordinates_ && faceGroupId >= 0 && faceGroupId < groupNumber) {
                    if (sharedBaseMeshFaceCoordIds_) {
                        faceVids.push_back(sharedCoordIdForVertex(st->vertex()));
                    } else {
                        faceVids.push_back(st->vertex() ? st->vertex()->vid() : -1);
                    }
                } else {
                    faceVids.push_back(st->vertex()->vid());
                }
                if (!st->next()) {
                    openFace = true;
                    MCGAL::Vertex* endVertex = st->end_vertex();
                    if (endVertex) {
                        if (sharedBaseMeshCoordinates_ && faceGroupId >= 0 && faceGroupId < groupNumber) {
                            if (sharedBaseMeshFaceCoordIds_) {
                                faceVids.push_back(sharedCoordIdForVertex(endVertex));
                            } else {
                                faceVids.push_back(endVertex->vid());
                            }
                        } else {
                            faceVids.push_back(endVertex->vid());
                        }
                    }
                    break;
                }
                st = st->next();
                if (st == ed) break;
            }

            const bool validFaceIds =
                std::all_of(faceVids.begin(), faceVids.end(),
                            [](int32_t vid) { return vid >= 0; });
            if (validFaceIds && faceGroupId >= 0 && faceGroupId < groupNumber &&
                faceVids.size() >= 3) {
                for (int32_t vid : faceVids) {
                    groupBaseMeshData_[faceGroupId].faceVertices.push_back(vid);
                }
                int32_t degree = static_cast<int32_t>(faceVids.size());
                groupBaseMeshData_[faceGroupId].faceDegrees.push_back(openFace ? -degree : degree);
            }
        }
        for (auto& data : groupBaseMeshData_) {
            if (sharedBaseMeshCoordinates_) {
                if (sharedBaseMeshFaceCoordIds_) {
                    std::set<int32_t> coords(data.faceVertices.begin(), data.faceVertices.end());
                    coords.erase(data.seedV0CoordId);
                    coords.erase(data.seedV1CoordId);
                    data.vertexCoordIds.clear();
                    data.numVertices = static_cast<int>(coords.size());
                } else {
                    data.numVertices = static_cast<int>(data.vertexCoordIds.size());
                }
            }
            data.numFaces = data.faceDegrees.size();
        }

    } else {
        // ========== 非分区模式 ==========
        GroupBaseMeshData data;
        data.groupId = 0;

        MCGAL::Halfedge* seed = seeds[0];
        int id = 0;
        std::unordered_map<MCGAL::Vertex*, int> vertexToLocalId;

        MCGAL::Point p0 = seed->vertex()->point();
        MCGAL::Point p1 = seed->end_vertex()->point();
        data.seedV0X = p0.x(); data.seedV0Y = p0.y(); data.seedV0Z = p0.z();
        data.seedV1X = p1.x(); data.seedV1Y = p1.y(); data.seedV1Z = p1.z();

        if (enableQuantization_) {
            MCGAL::PointInt pi0 = mesh->floatPosToInt(p0);
            MCGAL::PointInt pi1 = mesh->floatPosToInt(p1);
            data.seedV0QX = pi0[0]; data.seedV0QY = pi0[1]; data.seedV0QZ = pi0[2];
            data.seedV1QX = pi1[0]; data.seedV1QY = pi1[1]; data.seedV1QZ = pi1[2];
        }

        seed->vertex()->setVid(id);
        vertexToLocalId[seed->vertex()] = id++;
        seed->end_vertex()->setVid(id);
        vertexToLocalId[seed->end_vertex()] = id++;

        const bool legacySharedSingleGroupOrder =
            (sharedBaseMeshCoordinates_ && sharedBaseMeshFaceCoordIds_) ||
            preservesLiftedBaseMesh;

        auto appendVertex = [&](MCGAL::Vertex* vit) -> int {
            if (vit == nullptr) return -1;
            MCGAL::Point point = vit->point();
            data.vertexX.push_back(point.x());
            data.vertexY.push_back(point.y());
            data.vertexZ.push_back(point.z());

            if (enableQuantization_) {
                MCGAL::PointInt pi = mesh->floatPosToInt(point);
                for (int i = 0; i < 3; i++) {
                    pi[i] = clampBaseMeshQuantCoord(pi[i]);
                }
                data.vertexQX.push_back(pi[0]);
                data.vertexQY.push_back(pi[1]);
                data.vertexQZ.push_back(pi[2]);
            }
            const int assigned = id++;
            vit->setVid(assigned);
            vertexToLocalId[vit] = assigned;
            return assigned;
        };

        // serializeBaseMeshWithSeed precedes payload export: blobLayout_ is
        // still UNKNOWN here. Query the registered collector instead.
        const auto baseCollectorLayout =
            pmsf::paperCollectorLayoutChoice(collector_.get(), enablePrediction_);
        const bool segmentedSharedBase =
            baseCollectorLayout.valid &&
            baseCollectorLayout.blobLayout == pmsf::kPaperBlobLayoutSegmentedRangeCodedPred;
        const char* vertexOrderEnv = std::getenv("PMSF_BASE_VERTEX_ORDER");
        const std::string vertexOrder =
            vertexOrderEnv && *vertexOrderEnv
                ? vertexOrderEnv
                : (segmentedSharedBase ? "morton" : "off");
        const bool mortonVertexOrder =
            !legacySharedSingleGroupOrder && vertexOrder == "morton" &&
            enableQuantization_;

        if (mortonVertexOrder) {
            // Assign source vids BEFORE collector export writes group seeds.
            // Keep seed 0/1, face/corner order, coordinates and ownership intact.
            // The mesh containers themselves are never sorted.
            struct OrderedVertex {
                MCGAL::Vertex* vertex;
                std::size_t ordinal;
                std::array<int32_t, 3> q;
                std::uint64_t morton;
            };
            auto spreadMortonBits = [](std::uint32_t value) {
                std::uint64_t result = 0;
                for (unsigned bit = 0; bit < 21; ++bit) {
                    result |= static_cast<std::uint64_t>((value >> bit) & 1u)
                              << (3u * bit);
                }
                return result;
            };
            std::vector<OrderedVertex> ordered;
            ordered.reserve(mesh->vertices().size());
            for (std::size_t ordinal = 0; ordinal < mesh->vertices().size(); ++ordinal) {
                MCGAL::Vertex* vit = mesh->vertices()[ordinal];
                if (!vit || vit->isRemoved() ||
                    vit == seed->vertex() || vit == seed->end_vertex()) {
                    continue;
                }
                MCGAL::PointInt pi = mesh->floatPosToInt(vit->point());
                for (int axis = 0; axis < 3; ++axis) {
                    pi[axis] = clampBaseMeshQuantCoord(pi[axis]);
                }
                const std::array<int32_t, 3> q{pi[0], pi[1], pi[2]};
                const std::uint64_t morton =
                    spreadMortonBits(static_cast<std::uint32_t>(q[0])) |
                    (spreadMortonBits(static_cast<std::uint32_t>(q[1])) << 1) |
                    (spreadMortonBits(static_cast<std::uint32_t>(q[2])) << 2);
                ordered.push_back({vit, ordinal, q, morton});
            }
            std::stable_sort(ordered.begin(), ordered.end(),
                             [](const OrderedVertex& a, const OrderedVertex& b) {
                                 if (a.morton != b.morton)
                                     return a.morton < b.morton;
                                 if (a.q != b.q)
                                     return a.q < b.q;
                                 return a.ordinal < b.ordinal;
                             });
            for (const OrderedVertex& item : ordered) {
                appendVertex(item.vertex);
            }
            std::cout << "[Columnar] base vertex order: morton nonseed="
                      << ordered.size() << std::endl;
        } else if (legacySharedSingleGroupOrder) {
            for (MCGAL::Vertex* vit : mesh->vertices()) {
                if (vit->isRemoved()) continue;
                if (vit->poolId() == seed->vertex()->poolId() ||
                    vit->poolId() == seed->end_vertex()->poolId()) {
                    continue;
                }
                appendVertex(vit);
            }
        }

        auto assignVertexId = [&](MCGAL::Vertex* vit) -> int {
            if (vit == nullptr) return -1;
            auto it = vertexToLocalId.find(vit);
            if (it != vertexToLocalId.end()) return it->second;
            return appendVertex(vit);
        };

        // Lever A（分区 BPV）：可选按 groupId 稳定排序基网 facet 落点，使
        // facet→partition 通道退化为 ~G 段。必须与 SegmentedOptimizedSymbolCollect
        // Operator 的通道写出用**同一把**排序键（groupId，stable）。解码端按
        // base_mesh 表行序建面，故表序 == 通道序 ⇒ 回环正确。默认关闭。
        std::vector<MCGAL::Facet*> orderedBaseFacets;
        for (MCGAL::Facet* fit : mesh->faces()) {
            if (fit->isRemoved()) continue;
            orderedBaseFacets.push_back(fit);
        }
        {
            const char* sortEnv = std::getenv("PMSF_SEG_BASE_FACE_SORT");
            const bool sortByGroup = sortEnv && *sortEnv &&
                std::strcmp(sortEnv, "0") != 0 &&
                std::strcmp(sortEnv, "false") != 0 &&
                std::strcmp(sortEnv, "FALSE") != 0;
            if (sortByGroup) {
                std::stable_sort(orderedBaseFacets.begin(), orderedBaseFacets.end(),
                                 [](MCGAL::Facet* a, MCGAL::Facet* b) {
                                     return a->groupId() < b->groupId();
                                 });
            }
        }
        for (MCGAL::Facet* fit : orderedBaseFacets) {
            MCGAL::Halfedge* st = fit->proxyHalfedge();
            MCGAL::Halfedge* ed = st;
            std::vector<int32_t> faceVids;
            bool validFace = true;
            do {
                const int vid = assignVertexId(st->vertex());
                if (vid < 0) {
                    validFace = false;
                    break;
                }
                faceVids.push_back(vid);
                st = st->next();
            } while (st != ed);
            if (validFace && faceVids.size() >= 3) {
                for (int32_t vid : faceVids) {
                    data.faceVertices.push_back(vid);
                }
                data.faceDegrees.push_back((int32_t)faceVids.size());
            }
        }
        data.numVertices = data.vertexX.size();
        data.numFaces = data.faceDegrees.size();
        totalVertices_ = 2 + data.numVertices;
        totalFaces_ = data.numFaces;
        groupBaseMeshData_.push_back(std::move(data));
    }

    // [PHASE61 后续] base face id-space 诊断（env 门控，默认关闭，只读不改字节）。
    
}

// ============================================================================
// Schema 构建（4 个独立 Schema）
// ============================================================================

std::shared_ptr<arrow::Schema> ColumnarSerializeOperator::buildMetadataSchema() {
    return arrow::schema({
        arrow::field(pmsf::kParquetNumRoundsColumn,          arrow::int32(),    false),
        arrow::field(pmsf::kParquetGroupNumberColumn,        arrow::int32(),    false),
        arrow::field(pmsf::kParquetEnableCompressColumn,     arrow::boolean(),  false),
        arrow::field(pmsf::kParquetEnableQuantizationColumn, arrow::boolean(),  false),
        arrow::field(pmsf::kParquetEnableSegmentationColumn, arrow::boolean(),  false),
        arrow::field(pmsf::kParquetEnableAdjustmentColumn,   arrow::boolean(),  false),
        arrow::field(pmsf::kParquetQuantBitsColumn,          arrow::int32(),    true),
        arrow::field(pmsf::kParquetBBoxMinXColumn,          arrow::float32(),  true),
        arrow::field(pmsf::kParquetBBoxMinYColumn,          arrow::float32(),  true),
        arrow::field(pmsf::kParquetBBoxMinZColumn,          arrow::float32(),  true),
        arrow::field(pmsf::kParquetQuantStepColumn,          arrow::float32(),  true),
        arrow::field(pmsf::kParquetTotalVerticesColumn,      arrow::int32(),    false),
        arrow::field(pmsf::kParquetTotalFacesColumn,         arrow::int32(),    false),
        // V2 BPV 优化：codec_version=2 表示 symbols 表使用 rc_blob 单列；
        // 缺失该字段等价于 codec_version=1（旧 bool-list 多列格式），保持向后兼容。
        arrow::field(pmsf::kParquetCodecVersionColumn,       arrow::int32(),    true),
        arrow::field(pmsf::kParquetEnablePredictionColumn,   arrow::boolean(),  true),
        // V2 blob layout：解码端用它选择对称的解码入口（见 BlobLayout 枚举）
        arrow::field(pmsf::kParquetBlobLayoutColumn,         arrow::int32(),    true),
        // 新增：canonical 包装路径下记录原始算法-native layout，供 decoder 选择
        // 正确的 canonical spec 与 reassembly adapter。
        arrow::field(pmsf::kParquetCanonicalSourceLayoutColumn, arrow::int32(), true),
        // 新增：是否对 simplify 后的 polygon face 做 fan-triangulation。
        //   - true（默认）= 历史行为（VertexRemoval 全部切三角，Optimized v3 per-edge 规则）
        //   - false        = PPMC 风格 polygon 保留 + Optimized "second-pop + skip both
        //                    unsplittable" edge-sym 发射规则
        // 该列缺失（老文件）等价于 true，保持向后兼容。
        arrow::field(pmsf::kParquetEnableTriangulationColumn, arrow::boolean(), true),
        // 新增：3 个 PPMC 风格预测开关（论文 §3.3）。缺失列等价于 false。
        // - enable_conn_pred_faces：face connectivity prediction（用面积均值预测 facet sym）
        // - enable_conn_pred_edges：edge connectivity prediction（用边长均值预测 edge sym）
        // - enable_tri_balance：triangle-mesh balance prediction（仅纯三角网格生效）
        arrow::field(pmsf::kParquetEnableConnPredFacesColumn, arrow::boolean(), true),
        arrow::field(pmsf::kParquetEnableConnPredEdgesColumn, arrow::boolean(), true),
        arrow::field(pmsf::kParquetEnableTriBalanceColumn,     arrow::boolean(), true),
    });
}

std::shared_ptr<arrow::Schema> ColumnarSerializeOperator::buildBaseMeshSchema() {
    // 量化模式下：只存量化坐标（int16），不存浮点坐标（消除 4 byte/coord 的冗余 + 强制
    // 解码端用量化坐标 snap 反量化，与 PPMC/QEM 等"量化域 BFS"路径**字节级一致**）。
    // 非量化模式下：只存浮点坐标，不存量化坐标。
    auto qCoordType = arrow::int16();
    std::vector<std::shared_ptr<arrow::Field>> fields = {
        arrow::field(pmsf::kParquetGroupIdColumn,    arrow::int32(),    false),
        // 顶点/面片计数
        arrow::field(pmsf::kParquetNumVerticesColumn, arrow::int32(),   false),
        arrow::field(pmsf::kParquetNumFacesColumn,    arrow::int32(),   false),
    };
    const bool sharedCoords = sharedBaseMeshCoordinates_ && header_ &&
                              header_->isEnableSegmentation() &&
                              groupBaseMeshData_.size() > 1;
    if (sharedCoords) {
        fields.push_back(arrow::field(pmsf::kParquetSeedV0CoordIdColumn, arrow::int32(), false));
        fields.push_back(arrow::field(pmsf::kParquetSeedV1CoordIdColumn, arrow::int32(), false));
        fields.push_back(arrow::field(pmsf::kParquetVertexCoordIdsColumn, arrow::list(arrow::int32()), false));
        fields.push_back(arrow::field(pmsf::kParquetFaceVerticesAreCoordIdsColumn, arrow::boolean(), true));
        if (enableQuantization_) {
            fields.push_back(arrow::field(pmsf::kParquetSharedVertexQXColumn, arrow::list(qCoordType), true));
            fields.push_back(arrow::field(pmsf::kParquetSharedVertexQYColumn, arrow::list(qCoordType), true));
            fields.push_back(arrow::field(pmsf::kParquetSharedVertexQZColumn, arrow::list(qCoordType), true));
        } else {
            fields.push_back(arrow::field(pmsf::kParquetSharedVertexXColumn, arrow::list(arrow::float32()), true));
            fields.push_back(arrow::field(pmsf::kParquetSharedVertexYColumn, arrow::list(arrow::float32()), true));
            fields.push_back(arrow::field(pmsf::kParquetSharedVertexZColumn, arrow::list(arrow::float32()), true));
        }
        fields.push_back(arrow::field(pmsf::kParquetFaceVerticesColumn, arrow::list(arrow::int32()), false));
        fields.push_back(arrow::field(pmsf::kParquetFaceDegreesColumn,  arrow::list(arrow::int32()), false));
        return arrow::schema(fields);
    }
    if (enableQuantization_) {
        // 量化 seed 坐标
        fields.push_back(arrow::field(pmsf::kParquetSeedV0QXColumn,  qCoordType, false));
        fields.push_back(arrow::field(pmsf::kParquetSeedV0QYColumn,  qCoordType, false));
        fields.push_back(arrow::field(pmsf::kParquetSeedV0QZColumn,  qCoordType, false));
        fields.push_back(arrow::field(pmsf::kParquetSeedV1QXColumn,  qCoordType, false));
        fields.push_back(arrow::field(pmsf::kParquetSeedV1QYColumn,  qCoordType, false));
        fields.push_back(arrow::field(pmsf::kParquetSeedV1QZColumn,  qCoordType, false));
        // 量化顶点坐标（int16 列表）
        fields.push_back(arrow::field(pmsf::kParquetVertexQXColumn,   arrow::list(qCoordType), false));
        fields.push_back(arrow::field(pmsf::kParquetVertexQYColumn,   arrow::list(qCoordType), false));
        fields.push_back(arrow::field(pmsf::kParquetVertexQZColumn,   arrow::list(qCoordType), false));
    } else {
        // 浮点 seed 坐标（始终存，用于精确定位 seed 半边）
        fields.push_back(arrow::field(pmsf::kParquetSeedV0XColumn,   arrow::float32(),  false));
        fields.push_back(arrow::field(pmsf::kParquetSeedV0YColumn,   arrow::float32(),  false));
        fields.push_back(arrow::field(pmsf::kParquetSeedV0ZColumn,   arrow::float32(),  false));
        fields.push_back(arrow::field(pmsf::kParquetSeedV1XColumn,   arrow::float32(),  false));
        fields.push_back(arrow::field(pmsf::kParquetSeedV1YColumn,   arrow::float32(),  false));
        fields.push_back(arrow::field(pmsf::kParquetSeedV1ZColumn,   arrow::float32(),  false));
        // 浮点顶点坐标
        fields.push_back(arrow::field(pmsf::kParquetVertexXColumn,    arrow::list(arrow::float32()), false));
        fields.push_back(arrow::field(pmsf::kParquetVertexYColumn,    arrow::list(arrow::float32()), false));
        fields.push_back(arrow::field(pmsf::kParquetVertexZColumn,    arrow::list(arrow::float32()), false));
    }
    // 面片数据（支持多边形面片）
    fields.push_back(arrow::field(pmsf::kParquetFaceVerticesColumn, arrow::list(arrow::int32()),   false));
    fields.push_back(arrow::field(pmsf::kParquetFaceDegreesColumn,  arrow::list(arrow::int32()),   false));
    return arrow::schema(fields);
}

std::shared_ptr<arrow::Schema> ColumnarSerializeOperator::buildBoundarySchema() {
    return arrow::schema({
        arrow::field(pmsf::kParquetGroupAColumn,       arrow::int32(),    false),
        arrow::field(pmsf::kParquetGroupBColumn,       arrow::int32(),    false),
        arrow::field(pmsf::kParquetBoundaryStColumn,   arrow::list(arrow::int32()), false),
        arrow::field(pmsf::kParquetBoundaryEdColumn,   arrow::list(arrow::int32()), false),
        arrow::field(pmsf::kParquetBoundaryStopColumn, arrow::list(arrow::int32()), false),
        arrow::field("boundary_scheme", arrow::int32(), false),
    });
}

std::shared_ptr<parquet::WriterProperties> ColumnarSerializeOperator::buildWriterProperties() {
    auto builder = parquet::WriterProperties::Builder();

    // PPMC-scoped lean writer (env set by the PPMC algorithm): drop per-column
    // statistics and the page index.  The decoder never reads them, and for a
    // one-row BLOB payload table they only add footer bytes.
    const bool leanWriter = [] {
        const char* v = std::getenv("PMSF_PPMC_LEAN_PARQUET");
        return v && v[0] == '1';
    }();
    if (leanWriter) {
        builder.disable_statistics();
        builder.disable_write_page_index();
    }

    // ========== 全局压缩：ZSTD level 3 ==========
    // 在 rc_blob / int16 坐标 / 量化点流这些 **近随机字节** 的列上，ZSTD 9 相比 3
    // 只多 <2% 的文件体积，但写入时间会变慢 3-5 倍（ZSTD 9 搜索窗口开销巨大）。
    // 我们的 BPV 压缩力主要来自 collector 自己的熵编码，Parquet ZSTD 只是二次保底。
    // 实验数据（bunny RD v2）:
    //     level 9 → 199.8 KB / 4467 ms enc
    //     level 3 → 201.7 KB / 1120 ms enc   （+0.95% 体积，-75% 编码时间）
    // 解压时间基本不变（ZSTD 解码独立于压缩级别）。
    builder.compression(parquet::Compression::ZSTD);
    int zstdLevel = 3;
    if (const char* env = std::getenv("PMSF_PARQUET_ZSTD_LEVEL")) {
        const int v = std::atoi(env);
        if (v >= 1 && v <= 22) zstdLevel = v;
    }
    builder.compression_level(zstdLevel);

    // ========== 整数 ID/索引列：DELTA_BINARY_PACKED（利用递增/局部相关性）==========
    builder.encoding(pmsf::kParquetGroupIdColumn, parquet::Encoding::DELTA_BINARY_PACKED);
    builder.encoding(pmsf::kParquetRoundIdColumn, parquet::Encoding::DELTA_BINARY_PACKED);
    builder.encoding(pmsf::kParquetNumVerticesColumn, parquet::Encoding::DELTA_BINARY_PACKED);
    builder.encoding(pmsf::kParquetNumFacesColumn, parquet::Encoding::DELTA_BINARY_PACKED);
    builder.encoding(pmsf::kParquetNumFacetSymbolsColumn, parquet::Encoding::DELTA_BINARY_PACKED);
    builder.encoding(pmsf::kParquetNumEdgeSymbolsColumn, parquet::Encoding::DELTA_BINARY_PACKED);
    builder.encoding(pmsf::kParquetNumVertexSymbolsColumn, parquet::Encoding::DELTA_BINARY_PACKED);
    builder.encoding(pmsf::kParquetNumInsertedPointsColumn, parquet::Encoding::DELTA_BINARY_PACKED);
    builder.encoding(pmsf::kParquetNumVertexPointsColumn, parquet::Encoding::DELTA_BINARY_PACKED);
    builder.encoding(pmsf::kParquetTotalVerticesColumn, parquet::Encoding::DELTA_BINARY_PACKED);
    builder.encoding(pmsf::kParquetTotalFacesColumn, parquet::Encoding::DELTA_BINARY_PACKED);
    builder.encoding(pmsf::kParquetGroupAColumn, parquet::Encoding::DELTA_BINARY_PACKED);
    builder.encoding(pmsf::kParquetGroupBColumn, parquet::Encoding::DELTA_BINARY_PACKED);
    builder.encoding(pmsf::kParquetSeedV0CoordIdColumn, parquet::Encoding::DELTA_BINARY_PACKED);
    builder.encoding(pmsf::kParquetSeedV1CoordIdColumn, parquet::Encoding::DELTA_BINARY_PACKED);

    // ========== 浮点坐标列：BYTE_STREAM_SPLIT（分离字节流，ZSTD 压缩效果提升 30-50%）==========
    builder.encoding(pmsf::kParquetSeedV0XColumn, parquet::Encoding::BYTE_STREAM_SPLIT);
    builder.encoding(pmsf::kParquetSeedV0YColumn, parquet::Encoding::BYTE_STREAM_SPLIT);
    builder.encoding(pmsf::kParquetSeedV0ZColumn, parquet::Encoding::BYTE_STREAM_SPLIT);
    builder.encoding(pmsf::kParquetSeedV1XColumn, parquet::Encoding::BYTE_STREAM_SPLIT);
    builder.encoding(pmsf::kParquetSeedV1YColumn, parquet::Encoding::BYTE_STREAM_SPLIT);
    builder.encoding(pmsf::kParquetSeedV1ZColumn, parquet::Encoding::BYTE_STREAM_SPLIT);
    builder.encoding(pmsf::kParquetBBoxMinXColumn, parquet::Encoding::BYTE_STREAM_SPLIT);
    builder.encoding(pmsf::kParquetBBoxMinYColumn, parquet::Encoding::BYTE_STREAM_SPLIT);
    builder.encoding(pmsf::kParquetBBoxMinZColumn, parquet::Encoding::BYTE_STREAM_SPLIT);
    builder.encoding(pmsf::kParquetQuantStepColumn, parquet::Encoding::BYTE_STREAM_SPLIT);
    builder.encoding(pmsf::kParquetPointXColumn, parquet::Encoding::BYTE_STREAM_SPLIT);
    builder.encoding(pmsf::kParquetPointYColumn, parquet::Encoding::BYTE_STREAM_SPLIT);
    builder.encoding(pmsf::kParquetPointZColumn, parquet::Encoding::BYTE_STREAM_SPLIT);
    builder.encoding(pmsf::kParquetVertexPointXColumn, parquet::Encoding::BYTE_STREAM_SPLIT);
    builder.encoding(pmsf::kParquetVertexPointYColumn, parquet::Encoding::BYTE_STREAM_SPLIT);
    builder.encoding(pmsf::kParquetVertexPointZColumn, parquet::Encoding::BYTE_STREAM_SPLIT);

    // ========== Page 大小优化：128KB（让更多数据在同一 page 内编码）==========
    builder.data_pagesize(128 * 1024);
    builder.write_batch_size(4096);
    builder.enable_statistics();

    // rc_blob 列：内容已由 collector 的 range-coder 熵编码过，接近随机字节，
    // 字典编码 / DELTA / RLE 都不会再有收益，反而会略增 metadata。
    // 这里显式关闭该列的字典编码并让它走 PLAIN，外层 ZSTD 仍可压一层 padding/0。
    builder.disable_dictionary(pmsf::kParquetRCBlobColumn);
    builder.encoding(pmsf::kParquetRCBlobColumn, parquet::Encoding::PLAIN);
    builder.disable_dictionary(pmsf::kParquetCoordBlobColumn);
    builder.encoding(pmsf::kParquetCoordBlobColumn, parquet::Encoding::PLAIN);
    builder.disable_dictionary(pmsf::kParquetBaseBlobColumn);
    builder.encoding(pmsf::kParquetBaseBlobColumn, parquet::Encoding::PLAIN);

    // codec=4 列式段表：data 列已是熵编码字节，禁用字典，走 PLAIN + 外层 ZSTD。
    builder.disable_dictionary(pmsf::kParquetBitDataColumn);
    builder.encoding(pmsf::kParquetBitDataColumn, parquet::Encoding::PLAIN);
    builder.disable_dictionary(pmsf::kParquetResidualDataColumn);
    builder.encoding(pmsf::kParquetResidualDataColumn, parquet::Encoding::PLAIN);

    return builder.build();
}

std::shared_ptr<parquet::ArrowWriterProperties> ColumnarSerializeOperator::buildArrowWriterProperties() {
    const char* lean = std::getenv("PMSF_PPMC_LEAN_PARQUET");
    if (lean && lean[0] == '1') {
        // PPMC-scoped: no embedded Arrow schema metadata (decoder reads the
        // Parquet physical schema); saves ~435 B of footer per artifact.
        return parquet::ArrowWriterProperties::Builder().build();
    }
    return parquet::ArrowWriterProperties::Builder().store_schema()->build();
}

// ============================================================================
// Arrow Builder 辅助函数
// ============================================================================

namespace {

std::shared_ptr<arrow::Array> buildFloatList(const std::vector<float>& vals) {
    auto vb = std::make_shared<arrow::FloatBuilder>();
    arrow::ListBuilder lb(arrow::default_memory_pool(), vb);
    (void)lb.Append();
    for (float v : vals) (void)vb->Append(v);
    std::shared_ptr<arrow::Array> a;
    (void)lb.Finish(&a);
    return a;
}

std::shared_ptr<arrow::Array> buildInt32List(const std::vector<int32_t>& vals) {
    auto vb = std::make_shared<arrow::Int32Builder>();
    arrow::ListBuilder lb(arrow::default_memory_pool(), vb);
    (void)lb.Append();
    for (int32_t v : vals) (void)vb->Append(v);
    std::shared_ptr<arrow::Array> a;
    (void)lb.Finish(&a);
    return a;
}

std::shared_ptr<arrow::Array> buildBoolListFromDeque(const std::deque<char>& syms) {
    auto vb = std::make_shared<arrow::BooleanBuilder>();
    arrow::ListBuilder lb(arrow::default_memory_pool(), vb);
    (void)lb.Append();
    for (char s : syms) (void)vb->Append(s != 0);
    std::shared_ptr<arrow::Array> a;
    (void)lb.Finish(&a);
    return a;
}

std::shared_ptr<arrow::Array> buildFloatListFromPoints(std::deque<MCGAL::Point>& pts, int dim) {
    auto vb = std::make_shared<arrow::FloatBuilder>();
    arrow::ListBuilder lb(arrow::default_memory_pool(), vb);
    (void)lb.Append();
    for (auto& p : pts) (void)vb->Append((float)p[dim]);
    std::shared_ptr<arrow::Array> a;
    (void)lb.Finish(&a);
    return a;
}


std::shared_ptr<arrow::Array> makeNullArr(std::shared_ptr<arrow::DataType> type, int64_t length) {
    auto result = arrow::MakeArrayOfNull(type, length);
    return result.ok() ? result.ValueOrDie() : nullptr;
}

// 单值 builder 辅助
std::shared_ptr<arrow::Array> makeInt32(int32_t val) {
    arrow::Int32Builder b; (void)b.Append(val);
    std::shared_ptr<arrow::Array> a; (void)b.Finish(&a); return a;
}
std::shared_ptr<arrow::Array> makeFloat(float val) {
    arrow::FloatBuilder b; (void)b.Append(val);
    std::shared_ptr<arrow::Array> a; (void)b.Finish(&a); return a;
}
std::shared_ptr<arrow::Array> makeBool(bool val) {
    arrow::BooleanBuilder b; (void)b.Append(val);
    std::shared_ptr<arrow::Array> a; (void)b.Finish(&a); return a;
}

} // anonymous namespace

// ============================================================================
// 数据表构建
// ============================================================================

std::shared_ptr<arrow::Table> ColumnarSerializeOperator::buildMetadataTable() {
    auto schema = buildMetadataSchema();
    std::vector<std::shared_ptr<arrow::Array>> columns;

    columns.push_back(makeInt32(header_->getRound()));
    columns.push_back(makeInt32(header_->isEnableSegmentation() ? header_->getGroupNumber() : 1));
    columns.push_back(makeBool(header_->isEnableCompress()));
    columns.push_back(makeBool(header_->isEnableQuantization()));
    columns.push_back(makeBool(header_->isEnableSegmentation()));
    columns.push_back(makeBool(header_->isEnableAdjustment()));

    if (enableQuantization_) {
        columns.push_back(makeInt32(mesh()->i_nbQuantBits));
        columns.push_back(makeFloat(mesh()->bboxMin.x()));
        columns.push_back(makeFloat(mesh()->bboxMin.y()));
        columns.push_back(makeFloat(mesh()->bboxMin.z()));
        columns.push_back(makeFloat(mesh()->f_quantStep));
    } else {
        columns.push_back(makeNullArr(arrow::int32(), 1));
        columns.push_back(makeNullArr(arrow::float32(), 1));
        columns.push_back(makeNullArr(arrow::float32(), 1));
        columns.push_back(makeNullArr(arrow::float32(), 1));
        columns.push_back(makeNullArr(arrow::float32(), 1));
    }
    columns.push_back(makeInt32(totalVertices_));
    columns.push_back(makeInt32(totalFaces_));
    // V2 新增元数据
    columns.push_back(makeInt32(codecVersion_));
    columns.push_back(makeBool(enablePrediction_));
    columns.push_back(makeInt32(static_cast<int32_t>(blobLayout_)));
    columns.push_back(makeInt32(canonicalSourceLayout_));
    // 新增：enable_triangulation。header_ 默认 true，兼容 old-behavior。
    columns.push_back(makeBool(header_->isEnableTriangulation()));
    // 新增：3 个 PPMC 风格预测开关。header_ 默认 false，老 .pmsf 文件该列缺失时
    // ColumnarDeserializeOperator 默认填 false（同语义），保持向后兼容。
    columns.push_back(makeBool(header_->isEnableConnectivityPredictionFaces()));
    columns.push_back(makeBool(header_->isEnableConnectivityPredictionEdges()));
    columns.push_back(makeBool(header_->isEnableTriangleMeshBalance()));

    return arrow::Table::Make(schema, columns, 1);
}

std::shared_ptr<arrow::Table> ColumnarSerializeOperator::buildBaseMeshTable() {
    auto schema = buildBaseMeshSchema();
    int numRows = groupBaseMeshData_.size();
    const bool sharedCoords = sharedBaseMeshCoordinates_ && header_ &&
                              header_->isEnableSegmentation() &&
                              groupBaseMeshData_.size() > 1;

    // 为每列构建数组
    arrow::Int32Builder groupIdB, numVertB, numFaceB;
    if (sharedCoords) {
        arrow::Int32Builder seedV0CoordB, seedV1CoordB;
        arrow::BooleanBuilder faceCoordIdsB;
        arrow::FloatBuilder sv0xB, sv0yB, sv0zB, sv1xB, sv1yB, sv1zB;
        arrow::Int16Builder sv0qxB, sv0qyB, sv0qzB, sv1qxB, sv1qyB, sv1qzB;
        auto inlinePosVb = std::make_shared<arrow::Int32Builder>();
        arrow::ListBuilder inlinePosLb(arrow::default_memory_pool(), inlinePosVb);
        auto sharedPosVb = std::make_shared<arrow::Int32Builder>();
        arrow::ListBuilder sharedPosLb(arrow::default_memory_pool(), sharedPosVb);
        auto vcidVb = std::make_shared<arrow::Int32Builder>();
        arrow::ListBuilder vcidLb(arrow::default_memory_pool(), vcidVb);
        auto vxVb = std::make_shared<arrow::FloatBuilder>();
        auto vyVb = std::make_shared<arrow::FloatBuilder>();
        auto vzVb = std::make_shared<arrow::FloatBuilder>();
        arrow::ListBuilder vxLb(arrow::default_memory_pool(), vxVb);
        arrow::ListBuilder vyLb(arrow::default_memory_pool(), vyVb);
        arrow::ListBuilder vzLb(arrow::default_memory_pool(), vzVb);
        auto vqxVb = std::make_shared<arrow::Int16Builder>();
        auto vqyVb = std::make_shared<arrow::Int16Builder>();
        auto vqzVb = std::make_shared<arrow::Int16Builder>();
        arrow::ListBuilder vqxLb(arrow::default_memory_pool(), vqxVb);
        arrow::ListBuilder vqyLb(arrow::default_memory_pool(), vqyVb);
        arrow::ListBuilder vqzLb(arrow::default_memory_pool(), vqzVb);

        auto sharedQXVb = std::make_shared<arrow::Int16Builder>();
        auto sharedQYVb = std::make_shared<arrow::Int16Builder>();
        auto sharedQZVb = std::make_shared<arrow::Int16Builder>();
        arrow::ListBuilder sharedQXLb(arrow::default_memory_pool(), sharedQXVb);
        arrow::ListBuilder sharedQYLb(arrow::default_memory_pool(), sharedQYVb);
        arrow::ListBuilder sharedQZLb(arrow::default_memory_pool(), sharedQZVb);

        auto sharedXVb = std::make_shared<arrow::FloatBuilder>();
        auto sharedYVb = std::make_shared<arrow::FloatBuilder>();
        auto sharedZVb = std::make_shared<arrow::FloatBuilder>();
        arrow::ListBuilder sharedXLb(arrow::default_memory_pool(), sharedXVb);
        arrow::ListBuilder sharedYLb(arrow::default_memory_pool(), sharedYVb);
        arrow::ListBuilder sharedZLb(arrow::default_memory_pool(), sharedZVb);

        auto fvVb = std::make_shared<arrow::Int32Builder>();
        auto fdVb = std::make_shared<arrow::Int32Builder>();
        arrow::ListBuilder fvLb(arrow::default_memory_pool(), fvVb);
        arrow::ListBuilder fdLb(arrow::default_memory_pool(), fdVb);

        for (std::size_t row = 0; row < groupBaseMeshData_.size(); ++row) {
            const auto& data = groupBaseMeshData_[row];
            (void)groupIdB.Append(data.groupId);
            (void)numVertB.Append(data.numVertices);
            (void)numFaceB.Append(data.numFaces);
            if (enableQuantization_) {
                (void)sv0qxB.Append((int16_t)data.seedV0QX); (void)sv0qyB.Append((int16_t)data.seedV0QY); (void)sv0qzB.Append((int16_t)data.seedV0QZ);
                (void)sv1qxB.Append((int16_t)data.seedV1QX); (void)sv1qyB.Append((int16_t)data.seedV1QY); (void)sv1qzB.Append((int16_t)data.seedV1QZ);
            } else {
                (void)sv0xB.Append(data.seedV0X); (void)sv0yB.Append(data.seedV0Y); (void)sv0zB.Append(data.seedV0Z);
                (void)sv1xB.Append(data.seedV1X); (void)sv1yB.Append(data.seedV1Y); (void)sv1zB.Append(data.seedV1Z);
            }
            (void)seedV0CoordB.Append(data.seedV0CoordId);
            (void)seedV1CoordB.Append(data.seedV1CoordId);
            (void)faceCoordIdsB.Append(data.faceVerticesAreCoordIds);

            std::set<int32_t> sharedPositions(data.vertexSharedPositions.begin(),
                                              data.vertexSharedPositions.end());
            (void)inlinePosLb.Append();
            if (enableQuantization_) {
                (void)vqxLb.Append(); (void)vqyLb.Append(); (void)vqzLb.Append();
                for (size_t i = 0; i < data.vertexQX.size(); ++i) {
                    if (sharedPositions.count(static_cast<int32_t>(i))) continue;
                    (void)inlinePosVb->Append(static_cast<int32_t>(i));
                    (void)vqxVb->Append(static_cast<int16_t>(data.vertexQX[i]));
                    (void)vqyVb->Append(static_cast<int16_t>(data.vertexQY[i]));
                    (void)vqzVb->Append(static_cast<int16_t>(data.vertexQZ[i]));
                }
            } else {
                (void)vxLb.Append(); (void)vyLb.Append(); (void)vzLb.Append();
                for (size_t i = 0; i < data.vertexX.size(); ++i) {
                    if (sharedPositions.count(static_cast<int32_t>(i))) continue;
                    (void)inlinePosVb->Append(static_cast<int32_t>(i));
                    (void)vxVb->Append(data.vertexX[i]);
                    (void)vyVb->Append(data.vertexY[i]);
                    (void)vzVb->Append(data.vertexZ[i]);
                }
            }

            (void)sharedPosLb.Append();
            for (int32_t pos : data.vertexSharedPositions) (void)sharedPosVb->Append(pos);
            (void)vcidLb.Append();
            for (int32_t id : data.vertexCoordIds) (void)vcidVb->Append(id);

            if (enableQuantization_) {
                if (row == 0) {
                    (void)sharedQXLb.Append();
                    (void)sharedQYLb.Append();
                    (void)sharedQZLb.Append();
                    for (std::size_t i = 0; i < sharedVertexQX_.size(); ++i) {
                        (void)sharedQXVb->Append(static_cast<int16_t>(sharedVertexQX_[i]));
                        (void)sharedQYVb->Append(static_cast<int16_t>(sharedVertexQY_[i]));
                        (void)sharedQZVb->Append(static_cast<int16_t>(sharedVertexQZ_[i]));
                    }
                } else {
                    (void)sharedQXLb.AppendNull();
                    (void)sharedQYLb.AppendNull();
                    (void)sharedQZLb.AppendNull();
                }
            } else {
                if (row == 0) {
                    (void)sharedXLb.Append();
                    (void)sharedYLb.Append();
                    (void)sharedZLb.Append();
                    for (std::size_t i = 0; i < sharedVertexX_.size(); ++i) {
                        (void)sharedXVb->Append(sharedVertexX_[i]);
                        (void)sharedYVb->Append(sharedVertexY_[i]);
                        (void)sharedZVb->Append(sharedVertexZ_[i]);
                    }
                } else {
                    (void)sharedXLb.AppendNull();
                    (void)sharedYLb.AppendNull();
                    (void)sharedZLb.AppendNull();
                }
            }

            (void)fvLb.Append(); for (int32_t v : data.faceVertices) (void)fvVb->Append(v);
            (void)fdLb.Append(); for (int32_t d : data.faceDegrees) (void)fdVb->Append(d);
        }

        std::shared_ptr<arrow::Array> groupIdA, numVertA, numFaceA;
        std::shared_ptr<arrow::Array> seedV0CoordA, seedV1CoordA, faceCoordIdsA, inlinePosA, sharedPosA, vcidA;
        std::shared_ptr<arrow::Array> seedA0, seedA1, seedA2, seedA3, seedA4, seedA5;
        std::shared_ptr<arrow::Array> vertA0, vertA1, vertA2;
        std::shared_ptr<arrow::Array> sharedA0, sharedA1, sharedA2;
        std::shared_ptr<arrow::Array> fvA, fdA;
        (void)groupIdB.Finish(&groupIdA);
        (void)numVertB.Finish(&numVertA);
        (void)numFaceB.Finish(&numFaceA);
        (void)seedV0CoordB.Finish(&seedV0CoordA);
        (void)seedV1CoordB.Finish(&seedV1CoordA);
        (void)faceCoordIdsB.Finish(&faceCoordIdsA);
        (void)inlinePosLb.Finish(&inlinePosA);
        (void)sharedPosLb.Finish(&sharedPosA);
        (void)vcidLb.Finish(&vcidA);
        if (enableQuantization_) {
            (void)sv0qxB.Finish(&seedA0); (void)sv0qyB.Finish(&seedA1); (void)sv0qzB.Finish(&seedA2);
            (void)sv1qxB.Finish(&seedA3); (void)sv1qyB.Finish(&seedA4); (void)sv1qzB.Finish(&seedA5);
            (void)vqxLb.Finish(&vertA0); (void)vqyLb.Finish(&vertA1); (void)vqzLb.Finish(&vertA2);
            (void)sharedQXLb.Finish(&sharedA0);
            (void)sharedQYLb.Finish(&sharedA1);
            (void)sharedQZLb.Finish(&sharedA2);
        } else {
            (void)sv0xB.Finish(&seedA0); (void)sv0yB.Finish(&seedA1); (void)sv0zB.Finish(&seedA2);
            (void)sv1xB.Finish(&seedA3); (void)sv1yB.Finish(&seedA4); (void)sv1zB.Finish(&seedA5);
            (void)vxLb.Finish(&vertA0); (void)vyLb.Finish(&vertA1); (void)vzLb.Finish(&vertA2);
            (void)sharedXLb.Finish(&sharedA0);
            (void)sharedYLb.Finish(&sharedA1);
            (void)sharedZLb.Finish(&sharedA2);
        }
        (void)fvLb.Finish(&fvA);
        (void)fdLb.Finish(&fdA);

        return arrow::Table::Make(schema,
                                  {groupIdA, numVertA, numFaceA,
                                   seedV0CoordA, seedV1CoordA,
                                   vcidA,
                                   faceCoordIdsA,
                                   sharedA0, sharedA1, sharedA2,
                                   fvA, fdA},
                                  numRows);
    }
    arrow::FloatBuilder sv0xB, sv0yB, sv0zB, sv1xB, sv1yB, sv1zB;

    // 量化 seed 坐标用 int16
    arrow::Int16Builder sv0qxB, sv0qyB, sv0qzB, sv1qxB, sv1qyB, sv1qzB;

    // 顶点坐标列：量化模式用 int16，非量化用 float
    auto vxVb = std::make_shared<arrow::FloatBuilder>();
    auto vyVb = std::make_shared<arrow::FloatBuilder>();
    auto vzVb = std::make_shared<arrow::FloatBuilder>();
    arrow::ListBuilder vxLb(arrow::default_memory_pool(), vxVb);
    arrow::ListBuilder vyLb(arrow::default_memory_pool(), vyVb);
    arrow::ListBuilder vzLb(arrow::default_memory_pool(), vzVb);

    auto vqxVb = std::make_shared<arrow::Int16Builder>();
    auto vqyVb = std::make_shared<arrow::Int16Builder>();
    auto vqzVb = std::make_shared<arrow::Int16Builder>();
    arrow::ListBuilder vqxLb(arrow::default_memory_pool(), vqxVb);
    arrow::ListBuilder vqyLb(arrow::default_memory_pool(), vqyVb);
    arrow::ListBuilder vqzLb(arrow::default_memory_pool(), vqzVb);

    auto fvVb = std::make_shared<arrow::Int32Builder>();
    auto fdVb = std::make_shared<arrow::Int32Builder>();
    arrow::ListBuilder fvLb(arrow::default_memory_pool(), fvVb);
    arrow::ListBuilder fdLb(arrow::default_memory_pool(), fdVb);

    for (auto& data : groupBaseMeshData_) {
        (void)groupIdB.Append(data.groupId);
        (void)numVertB.Append(data.numVertices);
        (void)numFaceB.Append(data.numFaces);

        if (enableQuantization_) {
            // 量化模式：仅 int16 seed/顶点；不再写浮点（解码端用 quantized 反量化得到 snap）
            (void)sv0qxB.Append((int16_t)data.seedV0QX); (void)sv0qyB.Append((int16_t)data.seedV0QY); (void)sv0qzB.Append((int16_t)data.seedV0QZ);
            (void)sv1qxB.Append((int16_t)data.seedV1QX); (void)sv1qyB.Append((int16_t)data.seedV1QY); (void)sv1qzB.Append((int16_t)data.seedV1QZ);

            if (!data.vertexQX.empty()) {
                (void)vqxLb.Append(); for (int32_t v : data.vertexQX) (void)vqxVb->Append((int16_t)v);
                (void)vqyLb.Append(); for (int32_t v : data.vertexQY) (void)vqyVb->Append((int16_t)v);
                (void)vqzLb.Append(); for (int32_t v : data.vertexQZ) (void)vqzVb->Append((int16_t)v);
            } else {
                (void)vqxLb.Append(); (void)vqyLb.Append(); (void)vqzLb.Append();
            }
        } else {
            // 非量化模式：仅写浮点
            (void)sv0xB.Append(data.seedV0X); (void)sv0yB.Append(data.seedV0Y); (void)sv0zB.Append(data.seedV0Z);
            (void)sv1xB.Append(data.seedV1X); (void)sv1yB.Append(data.seedV1Y); (void)sv1zB.Append(data.seedV1Z);

            if (!data.vertexX.empty()) {
                (void)vxLb.Append(); for (float v : data.vertexX) (void)vxVb->Append(v);
                (void)vyLb.Append(); for (float v : data.vertexY) (void)vyVb->Append(v);
                (void)vzLb.Append(); for (float v : data.vertexZ) (void)vzVb->Append(v);
            } else {
                (void)vxLb.Append(); (void)vyLb.Append(); (void)vzLb.Append();
            }
        }

        // face_vertices / face_degrees
        (void)fvLb.Append(); for (int32_t v : data.faceVertices) (void)fvVb->Append(v);
        (void)fdLb.Append(); for (int32_t d : data.faceDegrees) (void)fdVb->Append(d);
    }

    // Finish all builders
    std::shared_ptr<arrow::Array> groupIdA, numVertA, numFaceA;
    std::shared_ptr<arrow::Array> fvA, fdA;

    (void)groupIdB.Finish(&groupIdA);
    (void)numVertB.Finish(&numVertA); (void)numFaceB.Finish(&numFaceA);
    (void)fvLb.Finish(&fvA); (void)fdLb.Finish(&fdA);

    // 按 Schema 顺序组装列：group_id, num_vertices, num_faces,
    //   [量化分支:  6 个 seed_vN_q? + 3 个 vertex_q? list]
    //   [非量化分支: 6 个 seed_vN_? float + 3 个 vertex_? float list]
    //   face_vertices, face_degrees
    std::vector<std::shared_ptr<arrow::Array>> columns = {
        groupIdA, numVertA, numFaceA,
    };
    if (enableQuantization_) {
        std::shared_ptr<arrow::Array> sv0qxA, sv0qyA, sv0qzA, sv1qxA, sv1qyA, sv1qzA;
        (void)sv0qxB.Finish(&sv0qxA); (void)sv0qyB.Finish(&sv0qyA); (void)sv0qzB.Finish(&sv0qzA);
        (void)sv1qxB.Finish(&sv1qxA); (void)sv1qyB.Finish(&sv1qyA); (void)sv1qzB.Finish(&sv1qzA);
        columns.insert(columns.end(), {sv0qxA, sv0qyA, sv0qzA, sv1qxA, sv1qyA, sv1qzA});
        std::shared_ptr<arrow::Array> vqxA, vqyA, vqzA;
        (void)vqxLb.Finish(&vqxA); (void)vqyLb.Finish(&vqyA); (void)vqzLb.Finish(&vqzA);
        columns.insert(columns.end(), {vqxA, vqyA, vqzA});
    } else {
        std::shared_ptr<arrow::Array> sv0xA, sv0yA, sv0zA, sv1xA, sv1yA, sv1zA;
        (void)sv0xB.Finish(&sv0xA); (void)sv0yB.Finish(&sv0yA); (void)sv0zB.Finish(&sv0zA);
        (void)sv1xB.Finish(&sv1xA); (void)sv1yB.Finish(&sv1yA); (void)sv1zB.Finish(&sv1zA);
        columns.insert(columns.end(), {sv0xA, sv0yA, sv0zA, sv1xA, sv1yA, sv1zA});
        std::shared_ptr<arrow::Array> vxA, vyA, vzA;
        (void)vxLb.Finish(&vxA); (void)vyLb.Finish(&vyA); (void)vzLb.Finish(&vzA);
        columns.insert(columns.end(), {vxA, vyA, vzA});
    }
    columns.insert(columns.end(), {fvA, fdA});

    return arrow::Table::Make(schema, columns, numRows);
}

std::shared_ptr<arrow::Table> ColumnarSerializeOperator::buildSymbolsTable() {
    pmsf::CollectorSymbolsTableContext ctx;
    ctx.collector = collector_.get();
    ctx.enableQuantization = enableQuantization_;
    ctx.enablePrediction = enablePrediction_;
    ctx.sharedBaseMeshCoordinates = sharedBaseMeshCoordinates_;
    ctx.sharedBaseMeshFaceCoordIds = sharedBaseMeshFaceCoordIds_;
    return pmsf::buildCollectorSymbolsTable(ctx);
}

// ============================================================================
// V2 BPV 优化路径：symbols 表退化为 {group_id, rc_blob} 两列，
// rc_blob 内容直接来自非 Parquet 版本 collector 的 exportToBuffer*()，
// 即与 BasicSerializeOperator 完全一致的熵编码比特流。
// ============================================================================

namespace {
bool wrapSymbolBlobZlib(const char* input,
                        int inputSize,
                        std::vector<std::uint8_t>& wrapped);
bool wrapSymbolBlobZstd(const char* input,
                        int inputSize,
                        std::vector<std::uint8_t>& wrapped);
}

std::shared_ptr<arrow::Schema> ColumnarSerializeOperator::buildSymbolsSchemaV2() {
    return arrow::schema({
        arrow::field(pmsf::kParquetGroupIdColumn, arrow::int32(),  false),
        arrow::field(pmsf::kParquetRCBlobColumn,  arrow::binary(), false),
    });
}

std::shared_ptr<arrow::Table> ColumnarSerializeOperator::buildSymbolsTableV2() {
    auto schema = buildSymbolsSchemaV2();

    // 当前版本只支持"每个压缩实例一行 blob"的语义，即把 collector 内部所有 round
    // 的符号一次性熵编码成单个 blob（与 BasicSerializeOperator 一致）。
    // 分区 BFS 下如果 collector 天然按 group 拆分，此处也只写一行 —— 解码端按
    // 单行读取、内部再按 round 消费，行为与非 Parquet 版本逐字节等价。
    //
    // 这样做的直接好处：
    //   * 解码端只需读 1 行 BINARY；
    //   * Parquet 的行元数据开销为常数；
    //   * 完全复用 OptimizedSymbolReadOperator::importFromRangeCodedBuffer 等
    //     已验证过的解码路径，数据完整性与原版保持 100% 一致。

    if (!collector_) {
        std::cerr << "[Columnar] buildSymbolsTableV2: collector 为空" << std::endl;
        return nullptr;
    }

    // 用一块足够大的缓冲承载 collector 产出的熵编码字节流，路径与 BasicSerializeOperator::serialize 完全一致。
    // 只会把 collector 实际写入的 usedBytes 交给 Arrow，避免对整块 10MB scratch 做无意义清零。
    std::unique_ptr<char[]> scratch(new char[BUFFER_SIZE]);
    int usedBytes = 0;

    // ---- collector 类型识别 ----
    // 根据 collector 类型选择编码路径与 blob layout：
    //   * OptimizedSymbolCollectOperator + prediction → exportToBufferWithRangeCoder（位面残差+RC）
    //   * PPMCSymbolCollectOperator                    → exportToBuffer  （多段 RC 串接）
    //   * BasicSymbolCollectOperator                   → exportToBuffer  ([int symSize][sym][pts]，可选 zlib）
    //   * OptimizedSymbolCollectOperator + !prediction → exportToBuffer  （4 段位流）
    //
    // 所有解码路径都依赖编码端 BFS 时的 `mesh->floatPosToInt(vertex->point())` 与
    // 解码端**字节级一致**，因此 B1（ColumnarDeserializeOperator 的 base mesh 量化 snap
    // 修复）是 PPMC/BASIC/OPTIMIZED_PLAIN 三个路径能正确工作的前提。
    const auto paperLayoutChoice =
        pmsf::paperCollectorLayoutChoice(collector_.get(), enablePrediction_);
    const auto paperExportResult = pmsf::paperExportCollectorPayloadBuffer(
        collector_.get(),
        scratch.get(),
        enableQuantization_,
        enablePrediction_,
        enableCompress_);
    if (!paperExportResult.handled) {
        std::cout << "[Columnar] 未识别的 collector 类型，回退到 codec_version=1" << std::endl;
        return nullptr;
    }
    usedBytes = paperExportResult.usedBytes;

    if (paperLayoutChoice.valid) {
        blobLayout_ = static_cast<BlobLayout>(paperLayoutChoice.blobLayout);
    }

    if (usedBytes <= 0) {
        std::cerr << "[Columnar] collector->exportToBuffer 返回 " << usedBytes
                  << "，回退到 codec_version=1" << std::endl;
        return nullptr;
    }

    if (usedBytes < 0) {
        std::cerr << "[Columnar] collector exportToBuffer 返回负值: " << usedBytes << std::endl;
        return nullptr;
    }

    
    std::vector<std::uint8_t> wrappedSymbolBlob;
    std::vector<std::uint8_t> wrappedSymbolBlobZlib;
    std::vector<std::uint8_t> wrappedSymbolBlobZstd;
    const bool symbolZlibOk =
        wrapSymbolBlobZlib(scratch.get(), usedBytes, wrappedSymbolBlobZlib);
    const bool symbolZstdOk =
        wrapSymbolBlobZstd(scratch.get(), usedBytes, wrappedSymbolBlobZstd);
    std::string symbolWrapperName = "raw";
    if (symbolZstdOk &&
        (!symbolZlibOk || wrappedSymbolBlobZstd.size() < wrappedSymbolBlobZlib.size())) {
        wrappedSymbolBlob.swap(wrappedSymbolBlobZstd);
        symbolWrapperName = "SBC1:zstd";
    } else if (symbolZlibOk) {
        wrappedSymbolBlob.swap(wrappedSymbolBlobZlib);
        symbolWrapperName = "SBC1:zlib";
    }
    const bool symbolBlobWrapped = !wrappedSymbolBlob.empty();
    const char* blobPtr = symbolBlobWrapped
                              ? reinterpret_cast<const char*>(wrappedSymbolBlob.data())
                              : scratch.get();
    const int blobBytes = symbolBlobWrapped
                              ? static_cast<int>(wrappedSymbolBlob.size())
                              : usedBytes;
    lastSymbolsBlob_.assign(reinterpret_cast<const std::uint8_t*>(blobPtr),
                            reinterpret_cast<const std::uint8_t*>(blobPtr) + blobBytes);

    arrow::Int32Builder gidB;
    arrow::BinaryBuilder blobB;
    (void)gidB.Append(0);  // 当前实现写单行
    (void)blobB.Append(blobPtr, static_cast<int32_t>(blobBytes));

    std::shared_ptr<arrow::Array> gidA, blobA;
    (void)gidB.Finish(&gidA);
    (void)blobB.Finish(&blobA);

    std::cout << "[Columnar] symbols blob: " << usedBytes << " bytes"
              << " (codec_version=" << codecVersion_
              << ", layout=" << static_cast<int>(blobLayout_)
              << ", prediction=" << enablePrediction_
              << ", compress=" << enableCompress_
              << ", stored_bytes=" << blobBytes
              << ", wrapper=" << symbolWrapperName
              << ")" << std::endl;

    return arrow::Table::Make(schema, {gidA, blobA}, 1);
}

// ============================================================================
// codec=4 列式段表：Connectivity (bit) + Geometry (residual) + LOD 偏移
// ============================================================================

std::shared_ptr<arrow::Schema> ColumnarSerializeOperator::buildBitSchemaV4() {
    return arrow::schema({
        arrow::field(pmsf::kParquetGroupIdColumn,      arrow::int32(),  false),
        arrow::field(pmsf::kParquetRoundIdColumn,      arrow::int32(),  false),
        arrow::field(pmsf::kParquetStreamIdColumn,     arrow::int32(),  false),
        arrow::field(pmsf::kParquetEncodingModeColumn, arrow::int8(),   false),
        arrow::field(pmsf::kParquetBitDataColumn,      arrow::binary(), false),
    });
}

std::shared_ptr<arrow::Schema> ColumnarSerializeOperator::buildResidualSchemaV4() {
    return arrow::schema({
        arrow::field(pmsf::kParquetGroupIdColumn,      arrow::int32(),  false),
        arrow::field(pmsf::kParquetRoundIdColumn,      arrow::int32(),  false),
        arrow::field(pmsf::kParquetEncodingModeColumn, arrow::int8(),   false),
        arrow::field(pmsf::kParquetResidualDataColumn, arrow::binary(), false),
    });
}

std::shared_ptr<arrow::Schema> ColumnarSerializeOperator::buildLodSchemaV4() {
    return arrow::schema({
        arrow::field(pmsf::kParquetGroupIdColumn,        arrow::int32(),            false),
        arrow::field(pmsf::kParquetRoundIdColumn,        arrow::int32(),            false),
        arrow::field(pmsf::kParquetBitStartColumn,        arrow::list(arrow::int32()), false),
        arrow::field(pmsf::kParquetBitLenColumn,          arrow::list(arrow::int32()), false),
        arrow::field(pmsf::kParquetResidualStartColumn,   arrow::int32(),            false),
        arrow::field(pmsf::kParquetResidualLenColumn,     arrow::int32(),            false),
        arrow::field("header_blob",                       arrow::binary(),           false),
    });
}

std::shared_ptr<arrow::Schema> ColumnarSerializeOperator::buildSideSchemaV4() {
    return arrow::schema({
        arrow::field(pmsf::kParquetGroupIdColumn,      arrow::int32(), false),
        arrow::field(pmsf::kParquetRoundIdColumn,      arrow::int32(), false),
        arrow::field("channel_name",                    arrow::utf8(),  false),
        arrow::field(pmsf::kParquetEncodingModeColumn, arrow::int8(),  false),
        arrow::field("side_data",                       arrow::binary(), false),
    });
}

ColumnarSerializeOperator::ColumnarSegments
ColumnarSerializeOperator::splitBlobToSegmentsV4(const char* blob,
                                                  int blobSize,
                                                  BlobLayout layout) {
    ColumnarSegments result;
    if (!blob || blobSize <= 0) return result;

    auto readInt32 = [&](size_t& off) -> int32_t {
        int32_t v = 0;
        std::memcpy(&v, blob + off, sizeof(int32_t));
        off += sizeof(int32_t);
        return v;
    };

    const bool isClassI =
        layout == BlobLayout::RANGE_CODED_PRED ||
        layout == BlobLayout::OPTIMIZED_PLAIN ||
        layout == BlobLayout::CODEC_006;

    if (isClassI) {
        // Class I header:
        // [rcFacetSize][rcEdgeSize]([rcVertexSize])[rcPointSize]
        // [totalFacetSymbols][totalEdgeSymbols]([totalVertexSymbols])[totalPointCoords]
        // [rcFacetBuf][rcEdgeBuf]([rcVertexBuf])[rcPointBuf]
        const bool hasVertex = header_ && header_->isEnableAdjustment();
        size_t off = 0;
        int32_t rcFacetSize = readInt32(off);
        int32_t rcEdgeSize = readInt32(off);
        int32_t rcVertexSize = 0;
        if (hasVertex) rcVertexSize = readInt32(off);
        int32_t rcPointSize = readInt32(off);
        // Header bytes include sizes + counts, up to the first payload buffer.
        const size_t headerEnd = off + sizeof(int32_t) * 2 +
                                 (hasVertex ? sizeof(int32_t) : 0) +
                                 sizeof(int32_t);
        result.headerBytes.assign(reinterpret_cast<const std::uint8_t*>(blob),
                                  reinterpret_cast<const std::uint8_t*>(blob + headerEnd));
        off = headerEnd;

        auto appendSeg = [&](int streamId, int32_t size) {
            if (size <= 0 || off + size > static_cast<size_t>(blobSize)) return;
            ColumnarSegment seg;
            seg.streamId = streamId;
            seg.encodingMode = 0;
            seg.bytes.assign(reinterpret_cast<const std::uint8_t*>(blob + off),
                             reinterpret_cast<const std::uint8_t*>(blob + off + size));
            result.bitSegments.push_back(std::move(seg));
            off += size;
        };

        appendSeg(0, rcFacetSize);   // facet
        appendSeg(1, rcEdgeSize);    // edge
        if (hasVertex) appendSeg(2, rcVertexSize);  // vertex

        if (rcPointSize > 0 && off + rcPointSize <= static_cast<size_t>(blobSize)) {
            result.residualBytes.assign(
                reinterpret_cast<const std::uint8_t*>(blob + off),
                reinterpret_cast<const std::uint8_t*>(blob + off + rcPointSize));
            result.residualEncodingMode = 0;
        }
        return result;
    }

    if (layout == BlobLayout::CODEC_003) {
        // CODEC_003: the entire blob is a single concatenated range-coded stream
        // produced by PPMCSymbolCollectOperator::exportToBuffer. It has no header
        // or segment sizes; the decoder reads it from the start. codec=4 stores it
        // as one residual segment with no bit segments, preserving byte layout.
        result.residualBytes.assign(
            reinterpret_cast<const std::uint8_t*>(blob),
            reinterpret_cast<const std::uint8_t*>(blob + blobSize));
        result.residualEncodingMode = 0;
        return result;
    }

    if (layout == BlobLayout::BASIC) {
        // Class III Basic:
        // [int32 compressSymbolSize][symbolBuf][pointBuf]
        size_t off = 0;
        int32_t compressSymbolSize = readInt32(off);
        result.headerBytes.assign(
            reinterpret_cast<const std::uint8_t*>(blob),
            reinterpret_cast<const std::uint8_t*>(blob + off));
        if (compressSymbolSize > 0 && off + compressSymbolSize <= static_cast<size_t>(blobSize)) {
            ColumnarSegment seg;
            seg.streamId = 0;
            seg.encodingMode = 0;
            seg.bytes.assign(
                reinterpret_cast<const std::uint8_t*>(blob + off),
                reinterpret_cast<const std::uint8_t*>(blob + off + compressSymbolSize));
            result.bitSegments.push_back(std::move(seg));
            off += compressSymbolSize;
        }
        int32_t pointSize = static_cast<int32_t>(blobSize) - static_cast<int32_t>(off);
        if (pointSize > 0) {
            result.residualBytes.assign(
                reinterpret_cast<const std::uint8_t*>(blob + off),
                reinterpret_cast<const std::uint8_t*>(blob + blobSize));
            result.residualEncodingMode = 0;
        }
        return result;
    }

    // Class II / Class S / other layouts: the blob is produced by a custom
    // collector (Valence/RD/PCATM/PCLTTM/MMSE/VQMMC/...) and consumed by the
    // matching algorithm-specific reader. We cannot split it into generic
    // facet/edge/point segments without a decoder pass, so codec=4 stores the
    // whole blob as one opaque residual segment. This removes the codec=3
    // fallback while preserving byte-identical round-trip; future phases can
    // add true columnar splits per algorithm.
    result.residualBytes.assign(
        reinterpret_cast<const std::uint8_t*>(blob),
        reinterpret_cast<const std::uint8_t*>(blob + blobSize));
    result.residualEncodingMode = 0;
    return result;
}

ColumnarSerializeOperator::ClassIDecodedStreams
ColumnarSerializeOperator::decodeClassIBlobToRawStreams(const char* blob,
                                                         int blobSize,
                                                         BlobLayout layout) {
    ClassIDecodedStreams out;
    if (!blob || blobSize <= 0) return out;

    const bool isClassI =
        layout == BlobLayout::RANGE_CODED_PRED ||
        layout == BlobLayout::OPTIMIZED_PLAIN ||
        layout == BlobLayout::CODEC_006;
    if (!isClassI) return out;
    if (!collector_ || !header_) return out;

    auto mesh = collector_->getMesh();
    if (!mesh) return out;

    // Use the serializer's runtime flags rather than header_ because some
    // pipelines (e.g. DefaultPPMCLiteColumnar) enable prediction without
    // carrying the flag into PMSFHeader. The blob itself was encoded with
    // these runtime flags, so the reader must mirror them.
    PMSFHeader decodeHeader = *header_;
    decodeHeader.setEnablePrediction(enablePrediction_);
    decodeHeader.setEnableQuantization(enableQuantization_);

    OptimizedSymbolReadOperator reader;
    reader.setMesh(mesh);
    reader.setCodecVersion(3);
    reader.init({0, 0, 0}, decodeHeader);
    if (enableQuantization_) {
        reader.enableQuantization(mesh->bboxMin, mesh->f_quantStep);
    }

    size_t offset = 0;
    const unsigned nbQuantBits =
        enableQuantization_ ? mesh->i_nbQuantBits : 0;
    // importFromRangeCodedBuffer takes a non-const char* but does not modify
    // the blob semantics (only reads through the rangecoder).
    reader.importFromRangeCodedBuffer(const_cast<char*>(blob), offset, nbQuantBits);

    // Edge symbols are left in the edgeRC_ state; pre-decode them all.
    // numEdgeSymbols is stored in the header portion of the blob.
    size_t headerOff = 0;
    auto readInt32 = [&](size_t& off) -> int32_t {
        int32_t v = 0;
        std::memcpy(&v, blob + off, sizeof(int32_t));
        off += sizeof(int32_t);
        return v;
    };
    (void)readInt32(headerOff);  // rcFacetSize
    (void)readInt32(headerOff);  // rcEdgeSize
    const bool hasVertex = header_->isEnableAdjustment();
    if (hasVertex) (void)readInt32(headerOff);  // rcVertexSize
    (void)readInt32(headerOff);  // rcPointSize
    (void)readInt32(headerOff);  // totalFacetSymbols
    const int numEdgeSymbols = readInt32(headerOff);
    reader.decodeAllEdgeSymbols(numEdgeSymbols);

    out.facetSymbols.assign(reader.getRcDecodedFacetSymbols().begin(),
                            reader.getRcDecodedFacetSymbols().end());
    out.edgeSymbols.assign(reader.getRcDecodedEdgeSymbols().begin(),
                           reader.getRcDecodedEdgeSymbols().end());
    out.hasVertex = hasVertex;
    if (hasVertex) {
        out.vertexSymbols.assign(reader.getRcDecodedVertexSymbols().begin(),
                                 reader.getRcDecodedVertexSymbols().end());
        out.vertexPoints.assign(reader.getRcDecodedVertexPoints().begin(),
                                reader.getRcDecodedVertexPoints().end());
    }
    out.useResidual = reader.isRcUsingResidual();
    if (out.useResidual) {
        out.residuals.assign(reader.getRcDecodedResiduals().begin(),
                             reader.getRcDecodedResiduals().end());
    } else {
        out.points.assign(reader.getRcDecodedPoints().begin(),
                          reader.getRcDecodedPoints().end());
    }
    return out;
}

namespace {

// Helper: write an Arrow table to a memory buffer using the given properties.
// Returns empty vector on failure.
std::vector<std::uint8_t> writeTableToMemoryBuffer(
    std::shared_ptr<arrow::Table> table,
    std::shared_ptr<parquet::WriterProperties> props) {
    if (!table || table->num_rows() <= 0) return {};
    auto schema = table->schema();
    auto arrowProps = parquet::ArrowWriterProperties::Builder().build();

    auto uniqueBuffer = arrow::AllocateResizableBuffer(0).ValueOrDie();
    auto buffer = std::shared_ptr<arrow::ResizableBuffer>(std::move(uniqueBuffer));
    auto outstream = std::make_shared<arrow::io::BufferOutputStream>(buffer);
    auto writerResult = parquet::arrow::FileWriter::Open(
        *schema, arrow::default_memory_pool(), outstream, props, arrowProps);
    if (!writerResult.ok()) return {};
    auto writer = std::move(writerResult).ValueOrDie();
    auto st = writer->WriteTable(*table, table->num_rows());
    if (!st.ok()) return {};
    st = writer->Close();
    if (!st.ok()) return {};
    st = outstream->Close();
    if (!st.ok()) return {};
    return std::vector<std::uint8_t>(buffer->data(),
                                      buffer->data() + buffer->size());
}

}  // namespace

std::vector<std::uint8_t> ColumnarSerializeOperator::serializeNativeBoolColumn(
    const std::vector<char>& symbols) {
    if (symbols.empty()) return {};
    auto valueType = arrow::boolean();
    auto listType = arrow::list(valueType);
    auto schema = arrow::schema({arrow::field("values", listType, false)});

    auto vb = std::make_shared<arrow::BooleanBuilder>();
    arrow::ListBuilder lb(arrow::default_memory_pool(), vb);
    (void)lb.Append();
    for (char v : symbols) {
        (void)vb->Append(v != 0);
    }
    std::shared_ptr<arrow::Array> listArray;
    (void)lb.Finish(&listArray);
    auto table = arrow::Table::Make(schema, {listArray}, 1);

    auto props = parquet::WriterProperties::Builder()
                     .disable_dictionary("values")
                     ->encoding("values", parquet::Encoding::RLE)
                     ->compression(parquet::Compression::ZSTD)
                     ->build();
    return writeTableToMemoryBuffer(table, props);
}

std::vector<std::uint8_t> ColumnarSerializeOperator::serializeNativeFloatColumn(
    const std::vector<MCGAL::Point>& points) {
    if (points.empty()) return {};
    auto valueType = arrow::float32();
    auto listType = arrow::list(valueType);
    auto schema = arrow::schema({arrow::field("values", listType, false)});

    auto vb = std::make_shared<arrow::FloatBuilder>();
    arrow::ListBuilder lb(arrow::default_memory_pool(), vb);
    (void)lb.Append();
    for (const auto& p : points) {
        (void)vb->Append(static_cast<float>(p.x()));
        (void)vb->Append(static_cast<float>(p.y()));
        (void)vb->Append(static_cast<float>(p.z()));
    }
    std::shared_ptr<arrow::Array> listArray;
    (void)lb.Finish(&listArray);
    auto table = arrow::Table::Make(schema, {listArray}, 1);

    auto props = parquet::WriterProperties::Builder()
                     .disable_dictionary("values")
                     ->encoding("values", parquet::Encoding::BYTE_STREAM_SPLIT)
                     ->compression(parquet::Compression::ZSTD)
                     ->build();
    return writeTableToMemoryBuffer(table, props);
}

std::vector<std::uint8_t> ColumnarSerializeOperator::serializeNativeQuantColumn(
    const std::vector<MCGAL::PointInt>& residuals) {
    if (residuals.empty()) return {};
    auto valueType = arrow::int32();
    auto listType = arrow::list(valueType);
    auto schema = arrow::schema({arrow::field("values", listType, false)});

    auto vb = std::make_shared<arrow::Int32Builder>();
    arrow::ListBuilder lb(arrow::default_memory_pool(), vb);
    (void)lb.Append();
    for (const auto& p : residuals) {
        (void)vb->Append(static_cast<int32_t>(p[0]));
        (void)vb->Append(static_cast<int32_t>(p[1]));
        (void)vb->Append(static_cast<int32_t>(p[2]));
    }
    std::shared_ptr<arrow::Array> listArray;
    (void)lb.Finish(&listArray);
    auto table = arrow::Table::Make(schema, {listArray}, 1);

    auto props = parquet::WriterProperties::Builder()
                     .disable_dictionary("values")
                     ->encoding("values", parquet::Encoding::DELTA_BINARY_PACKED)
                     ->compression(parquet::Compression::ZSTD)
                     ->build();
    return writeTableToMemoryBuffer(table, props);
}

int ColumnarSerializeOperator::measureNativeBoolStreamSize(
    const std::vector<char>& symbols) {
    auto bytes = serializeNativeBoolColumn(symbols);
    return static_cast<int>(bytes.empty() ? symbols.size() : bytes.size());
}

int ColumnarSerializeOperator::measureNativeFloatStreamSize(
    const std::vector<MCGAL::Point>& points) {
    auto bytes = serializeNativeFloatColumn(points);
    return static_cast<int>(bytes.empty() ? points.size() * sizeof(float) * 3
                                           : bytes.size());
}

int ColumnarSerializeOperator::measureNativeQuantStreamSize(
    const std::vector<MCGAL::PointInt>& residuals) {
    auto bytes = serializeNativeQuantColumn(residuals);
    return static_cast<int>(bytes.empty() ? residuals.size() * sizeof(int32_t) * 3
                                           : bytes.size());
}

ColumnarSerializeOperator::ClassIEncodingDecision
ColumnarSerializeOperator::chooseClassIEncodingModes(
    const ColumnarSegments& segs,
    const ClassIDecodedStreams& decoded) {
    ClassIEncodingDecision decision;
    // Map bit segments [streamId order] to decoded symbol vectors.
    // Class I bit segments are stored in order: facet (0), edge (1), vertex (2 if hasVertex).
    std::vector<const std::vector<char>*> bitSymbolPtrs;
    bitSymbolPtrs.push_back(&decoded.facetSymbols);
    bitSymbolPtrs.push_back(&decoded.edgeSymbols);
    if (decoded.hasVertex) {
        bitSymbolPtrs.push_back(&decoded.vertexSymbols);
    }

    for (size_t i = 0; i < segs.bitSegments.size(); ++i) {
        const int entropySize = static_cast<int>(segs.bitSegments[i].bytes.size());
        int nativeSize = entropySize;  // fallback if no decoded symbols
        if (i < bitSymbolPtrs.size() && !bitSymbolPtrs[i]->empty()) {
            nativeSize = measureNativeBoolStreamSize(*bitSymbolPtrs[i]);
        }
        decision.bitEntropySizes.push_back(entropySize);
        decision.bitNativeSizes.push_back(nativeSize);
        decision.bitModes.push_back(nativeSize < entropySize ? 1 : 0);
    }

    decision.residualEntropySize = static_cast<int>(segs.residualBytes.size());
    if (decoded.useResidual && !decoded.residuals.empty()) {
        decision.residualNativeSize = measureNativeQuantStreamSize(decoded.residuals);
    } else if (!decoded.points.empty()) {
        decision.residualNativeSize = measureNativeFloatStreamSize(decoded.points);
    } else {
        decision.residualNativeSize = decision.residualEntropySize;
    }
    decision.residualMode =
        decision.residualNativeSize < decision.residualEntropySize ? 1 : 0;

    // Phase S2：方案注入了流编码器时，bit/residual 段的字节由注入 coder 产生，
    // 不是内嵌 RC 字节 —— mode=1 会把这些字节替换成原生列，解码端将无法按
    // 方案还原（解码端按方案名再实例化注入 coder）。此时全部退回 mode=0。
    // 默认路径无方案注入，本分支不触发，行为不变。
    if (auto scheme = ::pmsf::activeStorageScheme();
        scheme && !scheme->coderFactory().empty()) {
        for (auto& m : decision.bitModes) m = 0;
        decision.residualMode = 0;
        return decision;
    }

    // ---- mode 必须**全有或全无**（D2.1）----
    //
    // 解码侧的 Class I 读法是**顺序**的：一个 range-coder 状态依次吐
    // facet → edge → vertex（见 decodeClassIBlobToRawStreams：单个
    // importFromRangeCodedBuffer + 顺序 decodeAllEdgeSymbols）。因此
    // **不能按流单独取字节**。
    //
    // 而 mode=1 的流存的是原生列、不是熵字节，无法参与顺序读。
    // 解码器现在的做法是「只要有 mode=1 的流，就把全部流重新编码成一个完整
    // blob」—— 但重编码需要**全部流的符号**，而 mode=0 的流只有熵字节、
    // 符号取不出来（顺序流里后一条无法脱离前一条单独解）。
    //
    // 结果：混合 mode 下重编码输入缺流，静默产出残缺 blob，解码器拿不到
    // 「本轮该分裂哪个面」的符号，网格永远停在 base mesh（实测某行解出 14 v
    // 而所有门禁全绿；具体行见 DEFECT_TRACKING D2.1）。
    //
    // 修法：混合时**全部退回 mode=0**。此时解码器根本不进重编码分支，
    // 走原来的顺序拼接路径，行为与引入 mode 之前一致。代价是放弃了
    // 「某条流用原生列更小」的收益 —— 但那点收益建立在解不出来的基础上。
    {
        bool anyMode1 = false, anyMode0 = false;
        for (int8_t m : decision.bitModes) (m == 1 ? anyMode1 : anyMode0) = true;
        if (anyMode1 && anyMode0) {
            for (auto& m : decision.bitModes) m = 0;
            decision.residualMode = 0;
        }
    }

    // Debug override: force all streams to mode=1 to exercise the re-encoder.
    if (const char* env = std::getenv("PMSF_CODEC4_FORCE_MODE1");
        env && env[0] == '1') {
        for (auto& m : decision.bitModes) m = 1;
        decision.residualMode = 1;
    }

    return decision;
}

int ColumnarSerializeOperator::reencodeClassIFromRawStreams(
    const ClassIDecodedStreams& decoded,
    BlobLayout layout,
    char* outBuffer,
    int maxSize) {
    if (!outBuffer || maxSize <= 0) return -1;
    const bool isClassI =
        layout == BlobLayout::RANGE_CODED_PRED ||
        layout == BlobLayout::OPTIMIZED_PLAIN ||
        layout == BlobLayout::CODEC_006;
    if (!isClassI) return -1;
    if (!collector_ || !header_) return -1;
    auto mesh = collector_->getMesh();
    if (!mesh) return -1;

    OptimizedSymbolCollectOperator tmpCollector;
    tmpCollector.init(mesh, {}, *header_);
    tmpCollector.setEnablePrediction(enablePrediction_);

    // Fold decoded flat vectors into single-round queues. The encoder processes
    // queues in reverse order, but with a single queue the in-queue order is
    // preserved and matches the decoder's output order.
    std::vector<std::deque<char>> facetQs(1);
    facetQs[0].assign(decoded.facetSymbols.begin(), decoded.facetSymbols.end());
    tmpCollector.setFacetSymbolQueuesForReencode(std::move(facetQs));

    std::vector<std::deque<char>> edgeQs(1);
    edgeQs[0].assign(decoded.edgeSymbols.begin(), decoded.edgeSymbols.end());
    tmpCollector.setEdgeSymbolQueuesForReencode(std::move(edgeQs));

    if (decoded.hasVertex) {
        std::vector<std::deque<char>> vertexQs(1);
        vertexQs[0].assign(decoded.vertexSymbols.begin(),
                           decoded.vertexSymbols.end());
        tmpCollector.setVertexSymbolQueuesForReencode(std::move(vertexQs));

        std::vector<std::deque<MCGAL::Point>> vertexPointQs(1);
        vertexPointQs[0].assign(decoded.vertexPoints.begin(),
                                decoded.vertexPoints.end());
        tmpCollector.setVertexPointQueuesForReencode(std::move(vertexPointQs));
    }

    const bool useResidualBitPlane = enablePrediction_ && enableQuantization_;
    if (useResidualBitPlane) {
        std::vector<std::deque<MCGAL::PointInt>> ipointQs(1);
        ipointQs[0].assign(decoded.residuals.begin(), decoded.residuals.end());
        tmpCollector.setIpointQueuesForReencode(std::move(ipointQs));
        // exportToBufferWithRangeCoder derives totalPointCoords from pointQueues
        // and vertexPointQueues, so provide matching dummy entries.
        std::vector<std::deque<MCGAL::Point>> pointQs(1);
        pointQs[0].resize(decoded.residuals.size());
        std::vector<std::deque<MCGAL::Point>> vertexPointQs(1);
        tmpCollector.setPointQueuesForReencode(std::move(pointQs));
        tmpCollector.setVertexPointQueuesForReencode(std::move(vertexPointQs));
    } else {
        std::vector<std::deque<MCGAL::Point>> pointQs(1);
        pointQs[0].assign(decoded.points.begin(), decoded.points.end());
        std::vector<std::deque<MCGAL::Point>> vertexPointQs(1);
        tmpCollector.setPointQueuesForReencode(std::move(pointQs));
        tmpCollector.setVertexPointQueuesForReencode(std::move(vertexPointQs));
    }

    return tmpCollector.exportToBufferWithRangeCoder(outBuffer, enableQuantization_);
}

std::shared_ptr<arrow::Table> ColumnarSerializeOperator::buildBitTableV4(
    const ColumnarSegments& segs) {
    auto schema = buildBitSchemaV4();
    arrow::Int32Builder gidB, lodB, sidB;
    arrow::Int8Builder modeB;
    arrow::BinaryBuilder dataB;

    for (const auto& seg : segs.bitSegments) {
        (void)gidB.Append(0);
        (void)lodB.Append(0);
        (void)sidB.Append(seg.streamId);
        (void)modeB.Append(static_cast<int8_t>(seg.encodingMode));
        (void)dataB.Append(reinterpret_cast<const char*>(seg.bytes.data()),
                           static_cast<int32_t>(seg.bytes.size()));
    }

    std::shared_ptr<arrow::Array> gidA, lodA, sidA, modeA, dataA;
    (void)gidB.Finish(&gidA);
    (void)lodB.Finish(&lodA);
    (void)sidB.Finish(&sidA);
    (void)modeB.Finish(&modeA);
    (void)dataB.Finish(&dataA);
    return arrow::Table::Make(schema, {gidA, lodA, sidA, modeA, dataA},
                              static_cast<int64_t>(segs.bitSegments.size()));
}

std::shared_ptr<arrow::Table> ColumnarSerializeOperator::buildResidualTableV4(
    const ColumnarSegments& segs) {
    auto schema = buildResidualSchemaV4();
    arrow::Int32Builder gidB, lodB;
    arrow::Int8Builder modeB;
    arrow::BinaryBuilder dataB;

    (void)gidB.Append(0);
    (void)lodB.Append(0);
    (void)modeB.Append(static_cast<int8_t>(segs.residualEncodingMode));
    (void)dataB.Append(reinterpret_cast<const char*>(segs.residualBytes.data()),
                       static_cast<int32_t>(segs.residualBytes.size()));

    std::shared_ptr<arrow::Array> gidA, lodA, modeA, dataA;
    (void)gidB.Finish(&gidA);
    (void)lodB.Finish(&lodA);
    (void)modeB.Finish(&modeA);
    (void)dataB.Finish(&dataA);
    return arrow::Table::Make(schema, {gidA, lodA, modeA, dataA}, 1);
}

std::shared_ptr<arrow::Table> ColumnarSerializeOperator::buildLodTableV4(
    const ColumnarSegments& segs,
    int bitTotalBytes,
    int residualTotalBytes) {
    auto schema = buildLodSchemaV4();
    arrow::Int32Builder gidB, lodB, resStartB, resLenB;
    arrow::BinaryBuilder headerB;
    auto bitStartVb = std::make_shared<arrow::Int32Builder>();
    auto bitLenVb = std::make_shared<arrow::Int32Builder>();
    arrow::ListBuilder bitStartLb(arrow::default_memory_pool(), bitStartVb);
    arrow::ListBuilder bitLenLb(arrow::default_memory_pool(), bitLenVb);

    (void)gidB.Append(0);
    (void)lodB.Append(0);
    int cur = 0;
    (void)bitStartLb.Append();
    (void)bitLenLb.Append();
    for (const auto& seg : segs.bitSegments) {
        (void)bitStartVb->Append(cur);
        (void)bitLenVb->Append(static_cast<int32_t>(seg.bytes.size()));
        cur += static_cast<int32_t>(seg.bytes.size());
    }
    (void)resStartB.Append(bitTotalBytes);
    (void)resLenB.Append(residualTotalBytes);
    (void)headerB.Append(reinterpret_cast<const char*>(segs.headerBytes.data()),
                         static_cast<int32_t>(segs.headerBytes.size()));

    std::shared_ptr<arrow::Array> gidA, lodA, bitStartA, bitLenA, resStartA, resLenA, headerA;
    (void)gidB.Finish(&gidA);
    (void)lodB.Finish(&lodA);
    (void)bitStartLb.Finish(&bitStartA);
    (void)bitLenLb.Finish(&bitLenA);
    (void)resStartB.Finish(&resStartA);
    (void)resLenB.Finish(&resLenA);
    (void)headerB.Finish(&headerA);
    return arrow::Table::Make(schema,
                              {gidA, lodA, bitStartA, bitLenA, resStartA, resLenA, headerA}, 1);
}

std::shared_ptr<arrow::Table> ColumnarSerializeOperator::buildBoundaryTable() {
    if (!boundaryGraph_) return nullptr;

    auto schema = buildBoundarySchema();
    auto& g = boundaryGraph_->graph();
    if (g.empty()) return nullptr;

    arrow::Int32Builder gaB, gbB;
    auto stVb = std::make_shared<arrow::Int32Builder>();
    arrow::ListBuilder stLb(arrow::default_memory_pool(), stVb);
    auto edVb = std::make_shared<arrow::Int32Builder>();
    arrow::ListBuilder edLb(arrow::default_memory_pool(), edVb);
    auto stopVb = std::make_shared<arrow::Int32Builder>();
    arrow::ListBuilder stopLb(arrow::default_memory_pool(), stopVb);
    arrow::Int32Builder schemeB;

    int totalRows = 0;
    for (auto& [groupA, neighbors] : g) {
        for (auto& [groupB, nodes] : neighbors) {
            (void)gaB.Append(groupA);
            (void)gbB.Append(groupB);

            (void)stLb.Append();
            (void)edLb.Append();
            (void)stopLb.Append();
            (void)schemeB.Append(static_cast<int>(boundaryGraph_->boundaryScheme()));
            for (auto& node : nodes) {
                (void)stVb->Append(node.st);
                if (boundaryGraph_->boundaryScheme() == BoundaryGraphScheme::ThreePoint) {
                    (void)edVb->Append(node.ed);
                    (void)stopVb->Append(node.stop);
                }
            }
            totalRows++;
        }
    }

    std::shared_ptr<arrow::Array> gaA, gbA, stA, edA, stopA, schemeA;
    (void)gaB.Finish(&gaA); (void)gbB.Finish(&gbA);
    (void)stLb.Finish(&stA); (void)edLb.Finish(&edA); (void)stopLb.Finish(&stopA);
    (void)schemeB.Finish(&schemeA);

    return arrow::Table::Make(schema, {gaA, gbA, stA, edA, stopA, schemeA}, totalRows);
}

// ============================================================================
// 写入单个 Parquet 文件
// ============================================================================

void ColumnarSerializeOperator::writeParquetFile(
    const std::string& path,
    std::shared_ptr<arrow::Schema> schema,
    std::shared_ptr<arrow::Table> table,
    int rowGroupSize) {

    if (!schema || !table) {
        std::cerr << "[Columnar] schema/table 为空，跳过写入: " << path << std::endl;
        return;
    }

    // PPMC-scoped lean residual channel (env set by impl/PPMC): instead of a
    // one-row Parquet file (footer/statistics/page-index ≈ 800 B for a blob
    // that is already range-coded and incompressible), write the payload as a
    // raw .bin sidecar.  The PPMC decode path falls back to reading it when the
    // _residual.parquet is absent.  Only PPMC sets the env ⇒ zero impact on
    // other algorithms' artifacts.
    const char* rawResidual = std::getenv("PMSF_PPMC_RAW_RESIDUAL");
    if (rawResidual && rawResidual[0] == '1') {
        static const std::string kResidualSuffix = "_residual.parquet";
        if (path.size() > kResidualSuffix.size() &&
            path.compare(path.size() - kResidualSuffix.size(), kResidualSuffix.size(),
                         kResidualSuffix) == 0) {
            auto dataCol = table->GetColumnByName(pmsf::kParquetResidualDataColumn);
            if (dataCol && table->num_rows() == 1) {
                auto dataArr = std::static_pointer_cast<arrow::BinaryArray>(dataCol->chunk(0));
                if (dataArr && dataArr->length() > 0) {
                    int32_t payloadLen = 0;
                    const std::uint8_t* payloadPtr = dataArr->GetValue(0, &payloadLen);
                    const std::string binPath =
                        path.substr(0, path.size() - kResidualSuffix.size()) + "_residual.bin";
                    if (payloadPtr && payloadLen > 0) {
                        if (FILE* fp = std::fopen(binPath.c_str(), "wb")) {
                            std::fwrite(payloadPtr, 1, static_cast<std::size_t>(payloadLen), fp);
                            std::fclose(fp);
                            std::cout << "[Columnar] raw residual blob: " << binPath
                                      << " bytes=" << payloadLen << std::endl;
                            return;
                        }
                    }
                }
            }
        }
    }

    auto writerProps = buildWriterProperties();
    auto arrowProps = buildArrowWriterProperties();

    auto result = arrow::io::FileOutputStream::Open(path);
    if (!result.ok()) {
        std::cerr << "[Columnar] 无法打开输出文件: " << path << std::endl;
        return;
    }
    auto outfile = result.ValueOrDie();

    auto writerResult = parquet::arrow::FileWriter::Open(
        *schema, arrow::default_memory_pool(), outfile, writerProps, arrowProps);
    if (!writerResult.ok()) {
        std::cerr << "[Columnar] 创建 Writer 失败: "
                  << writerResult.status().ToString() << std::endl;
        return;
    }
    auto writer = std::move(writerResult).ValueOrDie();

    int64_t rgSize = (rowGroupSize > 0) ? rowGroupSize : table->num_rows();
    if (rgSize <= 0) rgSize = 1;
    auto st = writer->WriteTable(*table, rgSize);
    if (!st.ok()) {
        std::cerr << "[Columnar] 写入表失败: " << st.ToString() << std::endl;
        return;
    }

    st = writer->Close();
    (void)outfile->Close();

    std::cout << "[Columnar] 写入: " << path << " 行数=" << table->num_rows() << std::endl;
}

// ============================================================================
// 主序列化方法
// ============================================================================

void ColumnarSerializeOperator::serialize(std::string path) {
    // 去掉扩展名作为基础名（仅去除文件名部分的扩展）
    std::string baseName = pmsf::paperStorageStripExtension(path);

    auto writeBaseMeshParquet = [&]() {
        auto schema = buildBaseMeshSchema();
        auto table = buildBaseMeshTable();
        // 每行一个 Row Group（每个 group 独立）
        writeParquetFile(pmsf::paperBaseMeshTablePath(baseName), schema, table, 1);
    };

    auto writeMetadataParquet = [&]() {
        auto schema = buildMetadataSchema();
        auto table = buildMetadataTable();
        writeParquetFile(pmsf::paperMetadataTablePath(baseName), schema, table);
    };

    bool requestedCompact = v2CompactMode_;
    if (!requestedCompact) {
        const char* env = std::getenv("PMSF_V2_COMPACT");
        if (env && env[0] == '1') requestedCompact = true;
    }
    bool requestedPartitionedCompact = partitionedCompactBaseMesh_;
    if (!requestedPartitionedCompact) {
        const char* env = std::getenv("PMSF_V2_PARTITIONED_BASE");
        if (env && env[0] == '1') requestedPartitionedCompact = true;
    }
    const bool skipCompatSymbolsTable =
        pmsf::parquetSkipCompatSymbolsTableRequested();
    bool skipCompatBaseTables =
        pmsf::parquetSkipCompatBaseTablesRequested();
    if (skipCompatBaseTables &&
        (requestedPartitionedCompact ||
         sharedBaseMeshCoordinates_ ||
         sharedBaseMeshFaceCoordIds_ ||
         partitionedCompactBaseMesh_)) {
        std::cerr << "[Columnar] unified base payload skip requested, but "
                  << "partitioned/shared base mesh mode is active; keep compat "
                  << "base artifacts for decode safety" << std::endl;
        skipCompatBaseTables = false;
    }
    std::vector<std::uint8_t> unifiedBasePayload;
    bool unifiedBasePayloadPresent = false;

    // If compact mode is requested up-front, metadata/base_mesh parquet files are
    // only fallback artifacts. Avoid writing and deleting them on the hot path.
    bool wroteBaseMeshParquet = false;
    bool wrotePartitionedCompactBase = false;
    if (!skipCompatBaseTables && !requestedCompact && !requestedPartitionedCompact) {
        writeBaseMeshParquet();
        wroteBaseMeshParquet = true;
    }

    // 环境变量覆盖 codec 版本（用于 Phase A/B 回归验证）。
    // 合法值：3 或 4；非法值忽略。记录是否显式设置，用于与算法覆盖互斥。
    bool codecEnvExplicitlySet = false;
    if (const char* codecEnv = std::getenv("PMSF_PARQUET_CODEC")) {
        int v = std::atoi(codecEnv);
        if (v == CODEC_COLUMNAR_SEG || v == 3) {
            codecVersion_ = v;
            codecEnvExplicitlySet = true;
        }
    }

    // collector 类型识别：部分算法（如 DefaultPPMCLite）通过 codecVersionOverride
    // 声明默认使用 codec=4。该覆盖仅在用户未显式通过环境变量指定 codec 时生效，
    // 保证回归测试可以用 PMSF_PARQUET_CODEC=3 强制走旧路径。
    const auto paperLayoutChoice = collector_
        ? pmsf::paperCollectorLayoutChoice(collector_.get(), enablePrediction_)
        : pmsf::PMSFCollectorLayoutChoice{};
    if (paperLayoutChoice.valid) {
        blobLayout_ = static_cast<BlobLayout>(paperLayoutChoice.blobLayout);
        if (!codecEnvExplicitlySet && paperLayoutChoice.codecVersionOverride > 0) {
            codecVersion_ = paperLayoutChoice.codecVersionOverride;
        }
    }

    // 2. 写入 symbols 表（先尝试 v2/v4，失败则回退 v1；codec_version_ 可能因此变化）
    bool wroteV2 = false;
    if (codecVersion_ == CODEC_COLUMNAR_SEG) {
        // ========== codec=4 列式段表路径 ==========
        if (collector_) {
            // Optional canonical facet/edge/vertex/point + side-channel path.
            const bool useCanonical =
                !skipCompatSymbolsTable &&
                std::getenv("PMSF_CODEC4_CANONICAL") != nullptr &&
                std::getenv("PMSF_CODEC4_CANONICAL")[0] == '1' &&
                pmsf::paperCollectorCanProjectToSymbolStream(collector_.get());
            if (useCanonical) {
                const auto& canonicalSpecs =
                    pmsf::SymbolStreamRegistry::instance().specsFor(
                        static_cast<std::int32_t>(blobLayout_));
                pmsf::CanonicalSymbolCollector canonicalCollector(
                    collector_.get(), canonicalSpecs);
                canonicalCollector.setUseResidual(enablePrediction_);
                auto payload = canonicalCollector.buildPayload();
                pmsf::CanonicalArrowTableBuilder arrowBuilder(payload);
                auto bitTable = arrowBuilder.buildBitTable();
                auto residualTable = arrowBuilder.buildResidualTable();
                auto lodTable = arrowBuilder.buildLodTable();
                auto sideTable = arrowBuilder.buildSideTable();
                if (!skipCompatSymbolsTable) {
                    if (bitTable && bitTable->num_rows() > 0) {
                        writeParquetFile(pmsf::paperBitTablePath(baseName),
                                         buildBitSchemaV4(), bitTable);
                    }
                    if (residualTable && residualTable->num_rows() > 0) {
                        writeParquetFile(pmsf::paperResidualTablePath(baseName),
                                         buildResidualSchemaV4(), residualTable);
                    }
                    if (lodTable && lodTable->num_rows() > 0) {
                        writeParquetFile(pmsf::paperLodTablePath(baseName),
                                         buildLodSchemaV4(), lodTable);
                    }
                    if (sideTable && sideTable->num_rows() > 0) {
                        writeParquetFile(pmsf::paperSideTablePath(baseName),
                                         buildSideSchemaV4(), sideTable);
                    }
                }
                canonicalSourceLayout_ =
                    static_cast<std::int32_t>(blobLayout_);
                blobLayout_ = static_cast<BlobLayout>(pmsf::kPaperBlobLayoutCanonical);
                std::size_t canonBitBytes = 0, canonSideBytes = 0,
                            canonResBytes = 0;
                for (const auto& s : payload.bitStreams) canonBitBytes += s.bytes.size();
                for (const auto& s : payload.sideChannels)
                    canonSideBytes += s.bytes.size();
                for (const auto& g : payload.geometry) {
                    canonResBytes += g.pointBytes.size() + g.vertexPointBytes.size();
                }
                std::cout << "[Columnar] codec=4 canonical path: rounds="
                          << payload.numRounds
                          << " bit_streams=" << payload.bitStreams.size()
                          << " side_channels=" << payload.sideChannels.size()
                          << " bit_bytes=" << canonBitBytes
                          << " side_bytes=" << canonSideBytes
                          << " residual_bytes=" << canonResBytes
                          << std::endl;
                wroteV2 = true;
            } else {
            std::unique_ptr<char[]> scratch(new char[BUFFER_SIZE]);
            const auto paperExportResult = pmsf::paperExportCollectorPayloadBuffer(
                collector_.get(),
                scratch.get(),
                enableQuantization_,
                enablePrediction_,
                enableCompress_);
            if (paperLayoutChoice.valid && paperExportResult.handled &&
                paperExportResult.usedBytes > 0) {
                auto segs = splitBlobToSegmentsV4(
                    scratch.get(), paperExportResult.usedBytes, blobLayout_);
                if (!segs.bitSegments.empty() || !segs.residualBytes.empty()) {
                    // Phase A3: decode and choose per-stream encoding mode.
                    // Only Class I layouts (Optimized/DefaultPPMC_LITE) expose the
                    // facet/edge/vertex/point split needed for native-column sizing.
                    const bool isClassILayout =
                        blobLayout_ == BlobLayout::RANGE_CODED_PRED ||
                        blobLayout_ == BlobLayout::OPTIMIZED_PLAIN ||
                        blobLayout_ == BlobLayout::CODEC_006;
                    ClassIEncodingDecision decision;
                    ClassIDecodedStreams decoded;
                    if (isClassILayout) {
                        decoded = decodeClassIBlobToRawStreams(
                            scratch.get(), paperExportResult.usedBytes, blobLayout_);
                        decision = chooseClassIEncodingModes(segs, decoded);
                    }

                    // Apply decisions to segment metadata and, for mode=1,
                    // replace entropy bytes with serialized native Parquet column bytes.
                    std::vector<const std::vector<char>*> bitSymbolPtrs;
                    bitSymbolPtrs.push_back(&decoded.facetSymbols);
                    bitSymbolPtrs.push_back(&decoded.edgeSymbols);
                    if (decoded.hasVertex) {
                        bitSymbolPtrs.push_back(&decoded.vertexSymbols);
                    }
                    for (size_t i = 0; i < segs.bitSegments.size(); ++i) {
                        segs.bitSegments[i].encodingMode = decision.bitModes[i];
                        if (decision.bitModes[i] == 1) {
                            const std::vector<char>* syms =
                                (i < bitSymbolPtrs.size()) ? bitSymbolPtrs[i] : nullptr;
                            if (syms && !syms->empty()) {
                                segs.bitSegments[i].bytes = serializeNativeBoolColumn(*syms);
                            }
                        }
                    }
                    segs.residualEncodingMode = decision.residualMode;
                    if (decision.residualMode == 1) {
                        if (decoded.useResidual && !decoded.residuals.empty()) {
                            segs.residualBytes = serializeNativeQuantColumn(decoded.residuals);
                        } else if (!decoded.points.empty()) {
                            segs.residualBytes = serializeNativeFloatColumn(decoded.points);
                        }

                        // Temporary self-check: re-encode the native column and
                        // decode it back to verify the round-trip preserves symbols/points.
                    }

                    auto bitTable = buildBitTableV4(segs);
                    auto residualTable = buildResidualTableV4(segs);
                    int bitTotal = 0;
                    for (const auto& s : segs.bitSegments) bitTotal += static_cast<int>(s.bytes.size());
                    auto lodTable = buildLodTableV4(
                        segs, bitTotal, static_cast<int>(segs.residualBytes.size()));
                    if (!skipCompatSymbolsTable) {
                        if (bitTable->num_rows() > 0) {
                            writeParquetFile(pmsf::paperBitTablePath(baseName),
                                             buildBitSchemaV4(), bitTable);
                        }
                        writeParquetFile(pmsf::paperResidualTablePath(baseName),
                                         buildResidualSchemaV4(), residualTable);
                        // For residual-only blobs (Class II/S fallback) there is no
                        // header and no bit stream, so a separate lod table adds only
                        // file-level overhead. Skip it and let the decoder reconstruct
                        // from the single residual row.
                        if (!segs.bitSegments.empty() || !segs.headerBytes.empty()) {
                            writeParquetFile(pmsf::paperLodTablePath(baseName),
                                             buildLodSchemaV4(), lodTable);
                        }
                    }
                    std::cout << "[Columnar] codec=4 columnar segments: bit_streams="
                              << segs.bitSegments.size()
                              << " bit_bytes=" << bitTotal
                              << " residual_bytes=" << segs.residualBytes.size()
                              << " layout=" << static_cast<int>(blobLayout_)
                              << std::endl;
                    if (const char* env = std::getenv("PMSF_BPV_DIAG");
                        env && env[0] == '1') {
                        std::cout << "[Columnar] codec=4 segment sizes:";
                        for (size_t i = 0; i < segs.bitSegments.size(); ++i) {
                            std::cout << " bit[" << segs.bitSegments[i].streamId
                                      << "]=" << segs.bitSegments[i].bytes.size()
                                      << " mode=" << static_cast<int>(segs.bitSegments[i].encodingMode);
                        }
                        std::cout << " residual=" << segs.residualBytes.size()
                                  << " mode=" << static_cast<int>(segs.residualEncodingMode)
                                  << std::endl;
                        std::cout << "[Columnar] codec=4 native candidate sizes:";
                        for (size_t i = 0; i < decision.bitModes.size(); ++i) {
                            std::cout << " bit[" << i << "]="
                                      << decision.bitEntropySizes[i]
                                      << "->" << decision.bitNativeSizes[i];
                        }
                        std::cout << " residual=" << decision.residualEntropySize
                                  << "->" << decision.residualNativeSize
                                  << std::endl;
                    }
                    wroteV2 = true;
                } else {
                    std::cerr << "[Columnar] codec=4 不支持当前 layout="
                              << static_cast<int>(blobLayout_)
                              << "，回退到 codec_version=3" << std::endl;
                    codecVersion_ = 3;
                }
            } else {
                std::cerr << "[Columnar] codec=4 collector 导出失败，回退到 codec_version=3"
                          << std::endl;
                codecVersion_ = 3;
            }
        }
    }
}
    if (!wroteV2 && codecVersion_ >= 2) {
        // ========== V2 BPV 优化路径：rc_blob 单列 ==========
        auto schema = buildSymbolsSchemaV2();
        auto table = buildSymbolsTableV2();
        if (table && table->num_rows() > 0) {
            if (!skipCompatSymbolsTable) {
                writeParquetFile(pmsf::paperSymbolsTablePath(baseName), schema, table);
            } else {
                std::cout << "[Columnar] skip compat _symbols.parquet; "
                          << "using unified "
                          << pmsf::parquetSymbolsRCPayloadSpec(getBlobLayout()).payloadName
                          << std::endl;
            }
            wroteV2 = true;
        } else {
            // collector 不兼容 v2 优化路径 → 自动回退到 codec_version=1
            std::cerr << "[Columnar] v2 路径不可用，回退到 codec_version=1" << std::endl;
            codecVersion_ = 1;
        }
    }
    if (!wroteV2) {
        // ========== 旧 bool-list 多列路径（向后兼容/回退） ==========
        auto schema = pmsf::buildCollectorSymbolsSchema(enableQuantization_);
        auto table = buildSymbolsTable();
        if (table && table->num_rows() > 0) {
            // 按 group 分 Row Group：每个 group 的所有 round 在一个 RG 中
            int groupNumber = header_->isEnableSegmentation() ? header_->getGroupNumber() : 1;
            if (groupNumber <= 0) groupNumber = 1;
            int roundsPerGroup = header_->getRound();
            if (roundsPerGroup <= 0) {
                roundsPerGroup = static_cast<int>((table->num_rows() + groupNumber - 1) / groupNumber);
            }
            if (roundsPerGroup <= 0) roundsPerGroup = 1;
            writeParquetFile(pmsf::paperSymbolsTablePath(baseName), schema, table, roundsPerGroup);
        }
    }

    // 3. 写入 metadata 表（最后写：codec_version_/blobLayout_ 已经反映 symbols 表的实际选择）
    if (skipCompatBaseTables) {
        unifiedBasePayloadPresent = buildV2CompactBinPayload(unifiedBasePayload);
        if (!unifiedBasePayloadPresent) {
            std::cerr << "[Columnar] unified base payload build failed; keep "
                      << "compat metadata/base artifacts" << std::endl;
            skipCompatBaseTables = false;
            if (!requestedCompact && !requestedPartitionedCompact && !wroteBaseMeshParquet) {
                writeBaseMeshParquet();
                wroteBaseMeshParquet = true;
            }
        }
    }

    if (requestedPartitionedCompact && !skipCompatBaseTables) {
        wrotePartitionedCompactBase = writePartitionedCompactBaseMesh(baseName);
        if (wrotePartitionedCompactBase) {
            pmsf::removePaperBaseMeshAndCompactArtifactsForBase(baseName);
        } else if (!requestedCompact && !wroteBaseMeshParquet) {
            writeBaseMeshParquet();
            wroteBaseMeshParquet = true;
        }
    }

    const bool metadataEmbeddedInPartitionedBase =
        wrotePartitionedCompactBase &&
        [] {
            const char* v = std::getenv("PMSF_EMBED_METADATA_IN_PARTITIONED_BASE");
            return v && *v && std::strcmp(v, "0") != 0;
        }();
    const bool symbolsEmbeddedInPartitionedBase =
        wrotePartitionedCompactBase &&
        [] {
            const char* v = std::getenv("PMSF_EMBED_SYMBOLS_IN_PARTITIONED_BASE");
            return v && *v && std::strcmp(v, "0") != 0;
        }();
    bool wantCompact = !wrotePartitionedCompactBase &&
                       (requestedCompact ||
                        getBlobLayout() == pmsf::kPaperBlobLayoutCodec025);
    if (wantCompact && sharedBaseMeshCoordinates_ && sharedBaseMeshFaceCoordIds_) {
        std::cerr << "[Columnar] compact bin 暂不支持 shared face coord ids，"
                  << "回退到 parquet base mesh" << std::endl;
        wantCompact = false;
        if (!wroteBaseMeshParquet) {
            writeBaseMeshParquet();
            wroteBaseMeshParquet = true;
        }
    }
    if (skipCompatBaseTables) {
        pmsf::removePaperCompactBaseArtifactsForBase(baseName);
        std::cout << "[Columnar] skip compat metadata/base artifacts; "
                  << "using unified "
                  << pmsf::parquetCompactBasePayloadSpec().payloadName
                  << std::endl;
    } else if (!wantCompact) {
        if (metadataEmbeddedInPartitionedBase) {
            pmsf::removePaperMetadataArtifactForBase(baseName);
            std::cout << "[Columnar] metadata embedded in partitioned base; "
                      << "skip standalone metadata parquet" << std::endl;
        } else {
            writeMetadataParquet();
        }
        if (!wrotePartitionedCompactBase) {
            pmsf::removePaperCompactBinArtifactForBase(baseName);
        }
    }
    if (symbolsEmbeddedInPartitionedBase) {
        pmsf::removePMSFSymbolsArtifactForBase(baseName);
        std::cout << "[Columnar] symbols embedded in partitioned base; "
                  << "remove standalone symbols parquet" << std::endl;
    }

    // 4. 写入 boundary 表（可选）
    if (boundaryGraph_) {
        auto schema = buildBoundarySchema();
        auto table = buildBoundaryTable();
        if (table && table->num_rows() > 0) {
            writeParquetFile(pmsf::paperBoundaryTablePath(baseName), schema, table);
        }
    }

    // 5. Tier 2 (2026-05-09)：紧凑模式 — 把 metadata + base_mesh 两张小 parquet 表
    //    打包成 zlib-deflated `<base>_compact.bin`，剔除 Parquet footer/schema
    //    在小表上的固定开销（bunny 上 ~14 KB → ~9 KB，省 −1.2 BPV）。通过
    //    setV2CompactMode(true) 或环境变量 PMSF_V2_COMPACT=1 启用。
    if (wantCompact && !skipCompatBaseTables) {
        const std::string compactPath = pmsf::paperCompactBinPath(baseName);
        if (writeV2CompactBin(compactPath)) {
            // 删除两张小 parquet 表（被 compact.bin 等价取代）
            const std::string metaPath = pmsf::paperMetadataTablePath(baseName);
            const std::string baseMeshPath = pmsf::paperBaseMeshTablePath(baseName);
            if (!requestedCompact) {
                pmsf::removePaperCompactBaseParquetArtifactsForBase(baseName);
                std::cout << "[Columnar] Tier 2 紧凑模式：删除 "
                          << metaPath << " + " << baseMeshPath << std::endl;
            } else {
                std::cout << "[Columnar] Tier 2 紧凑模式：跳过临时 parquet metadata/base_mesh"
                          << std::endl;
            }
        } else {
            std::cerr << "[Columnar] Tier 2 紧凑模式：writeV2CompactBin 失败，"
                      << "保留两张 parquet 表" << std::endl;
            if (requestedCompact) {
                writeBaseMeshParquet();
                writeMetadataParquet();
            }
        }
    }

        if (pmsf::unifiedStorageEnabled()) {
            std::vector<pmsf::PaperNamedBlob> namedBlobs;
            bool hasCollectorPayloadRows = false;
            if (collector_) {
                if (const auto* payloads = collector_->paperSymbolPayloadTable()) {
                    hasCollectorPayloadRows = !payloads->empty();
                    pmsf::appendPMSFPayloadRows(namedBlobs, *payloads);
                }
            }
            if (unifiedBasePayloadPresent) {
                pmsf::PMSFSymbolPayloadTable basePayloads;
                pmsf::appendParquetCompactBasePayload(
                    basePayloads, std::move(unifiedBasePayload));
                pmsf::appendPMSFPayloadRows(namedBlobs, basePayloads);
            }

            const bool mirrorSymbolsBlob =
                !lastSymbolsBlob_.empty() &&
                (skipCompatSymbolsTable || !hasCollectorPayloadRows);
            if (mirrorSymbolsBlob) {
                pmsf::PMSFSymbolPayloadTable symbolPayloads;
                pmsf::appendParquetSymbolsRCPayload(
                    symbolPayloads, lastSymbolsBlob_, getBlobLayout());
                pmsf::appendPMSFPayloadRows(namedBlobs, symbolPayloads);
            }

            auto addV2StorageSideValues = [&](pmsf::PMSFSymbolTable& table) {
                if (table.empty()) {
                    table.emplace_back();
                }
                const bool hasSymbolMapping =
                    pmsf::pmsfSymbolTableHasSymbolMapping(table);
                const bool hasAnyPayloadRows = !namedBlobs.empty();
                pmsf::appendParquetStorageAudit(
                    table.front().delta,
                    "columnar",
                    skipCompatSymbolsTable ? 1 : 0,
                    skipCompatBaseTables ? 1 : 0,
                    unifiedBasePayloadPresent ? 1 : 0,
                    mirrorSymbolsBlob ? 1 : 0,
                    hasCollectorPayloadRows ? 1 : 0,
                    hasAnyPayloadRows && !hasSymbolMapping ? 1 : 0,
                    hasSymbolMapping ? 1 : 0);
            };

            pmsf::PMSFSymbolTable sidecarTable;
            if (buildPMSFSymbolTableWithSerializerSideChannels(sidecarTable)) {
                addV2StorageSideValues(sidecarTable);
                pmsf::writeUnifiedPMSFArtifactsForTableAndNamedBlobs(
                    baseName, sidecarTable, std::move(namedBlobs), false);
            } else if (collector_) {
                if (const auto* table = collector_->pmsfSymbolTable()) {
                    sidecarTable = *table;
                    addV2StorageSideValues(sidecarTable);
                    pmsf::writeUnifiedPMSFArtifactsForTableAndNamedBlobs(
                        baseName, sidecarTable, std::move(namedBlobs), false);
                } else if (!namedBlobs.empty()) {
                    addV2StorageSideValues(sidecarTable);
                    pmsf::writeUnifiedPMSFArtifactsForTableAndNamedBlobs(
                        baseName, sidecarTable, std::move(namedBlobs), false);
                } else {
                    addV2StorageSideValues(sidecarTable);
                    pmsf::writeUnifiedPMSFArtifactsForTable(
                        baseName, sidecarTable, false);
                }
            } else {
                addV2StorageSideValues(sidecarTable);
                if (!namedBlobs.empty()) {
                    pmsf::writeUnifiedPMSFArtifactsForTableAndNamedBlobs(
                        baseName, sidecarTable, std::move(namedBlobs), false);
                } else {
                    pmsf::writeUnifiedPMSFArtifactsForTable(
                        baseName, sidecarTable, false);
                }
            }
        } else {
            writeUnifiedPMSFArtifactsForSerializer(baseName, false);
        }

    std::cout << "\n[ColumnarSerialize] 多表方案写入完成:"
              << "\n  基础名: " << baseName
              << "\n  总顶点: " << totalVertices_
              << "\n  总面片: " << totalFaces_
              << "\n  轮次: " << header_->getRound()
              << "\n  分区: " << (header_->isEnableSegmentation() ? header_->getGroupNumber() : 1)
              << "\n  紧凑: " << (wantCompact ? "ON" : "off")
              << std::endl;
}

// ============================================================================
// Tier 2 (2026-05-09)：V2 紧凑模式 — 把 metadata + base_mesh 两张小 parquet 表
// 打包成 zlib-deflated `<base>_compact.bin`。
//
// 字节布局（little-endian，与 V3 _v3_compact.bin 几乎一致，magic 改为 "V2CP"）：
//
//   [u32 rawLen]                         未压缩 payload 字节数
//   [u32 compLen]                        zlib 压缩后字节数
//   [byte × compLen]                     zlib deflate 整段 payload
//
// payload 内部布局：
//
//   "V2CP" magic (4 bytes)
//   u16 version=1/2
//   u16 flags  (bit 0=hasQuant, bit 1=hasSeg, bit 2=u16 compact,
//               bit 3=all triangle faces, bit 4=varint base, bit 5=edgebreaker base,
//               bit 6=split-residual base, bit 7=has canonicalSourceLayout)
//   --- metadata 段 ---
//   i32 round
//   i32 groupNumber (= 0 if !hasSeg)
//   u8  flagBits (bit 0=enableCompress, 1=enableQuant, 2=enableSeg, 3=enableAdj, 4=enablePred,
//                 5=enableTriangulation, 6=connPredFaces, 7=connPredEdges)
//   u8  flagBits2 (bit 0=enableTriangleMeshBalance；PPMC 用)
//   i32 codecVersion
//   i32 blobLayout
//   if (flags & 0x80) i32 canonicalSourceLayout   ← 仅当 canonicalSourceLayout_ != 0
//   i32 totalVertices
//   i32 totalFaces
//   if hasQuant:
//     i32 quantBits
//     f32 bboxMinX, bboxMinY, bboxMinZ, quantStep
//   --- base mesh 段 ---
//   i32 nGroups
//   for g in groups:
//     i32 groupId, numVertices, numFaces
//     if hasQuant:
//       i32 × 6  : seedV0Q{X,Y,Z} + seedV1Q{X,Y,Z}
//       i32 × numVertices × 3 : non-seed vertex quantized coords
//     else:
//       f32 × 6  : seedV0/V1 float
//       f32 × numVertices × 3 : non-seed vertex float coords
//     i32 nDegrees + i32 × nDegrees
//     i32 nFaceVerts + i32 × nFaceVerts
//
// 与 V3 compact 完全独立（V3 用 "V3CP" magic 与 `_v3_compact.bin`），
// 互不影响。
// ============================================================================
namespace {

inline void packU16LE(std::vector<std::uint8_t>& v, std::uint16_t x) {
    v.push_back(static_cast<std::uint8_t>(x & 0xFF));
    v.push_back(static_cast<std::uint8_t>((x >> 8) & 0xFF));
}
inline void packI32LE(std::vector<std::uint8_t>& v, std::int32_t x) {
    auto u = static_cast<std::uint32_t>(x);
    for (int i = 0; i < 4; ++i) v.push_back(static_cast<std::uint8_t>((u >> (8 * i)) & 0xFF));
}
inline void packF32LE(std::vector<std::uint8_t>& v, float f) {
    std::uint32_t u;
    std::memcpy(&u, &f, 4);
    for (int i = 0; i < 4; ++i) v.push_back(static_cast<std::uint8_t>((u >> (8 * i)) & 0xFF));
}
inline void packU8(std::vector<std::uint8_t>& v, std::uint8_t x) { v.push_back(x); }
inline void packU32LE(std::vector<std::uint8_t>& v, std::uint32_t x) {
    for (int i = 0; i < 4; ++i) v.push_back(static_cast<std::uint8_t>((x >> (8 * i)) & 0xFF));
}
inline std::uint32_t zigzagI32(std::int32_t x) {
    return (static_cast<std::uint32_t>(x) << 1) ^
           static_cast<std::uint32_t>(x >> 31);
}
inline void packVarU32(std::vector<std::uint8_t>& v, std::uint32_t x) {
    while (x >= 0x80u) {
        v.push_back(static_cast<std::uint8_t>((x & 0x7fu) | 0x80u));
        x >>= 7;
    }
    v.push_back(static_cast<std::uint8_t>(x));
}
inline void packVarI32(std::vector<std::uint8_t>& v, std::int32_t x) {
    packVarU32(v, zigzagI32(x));
}

bool envOn(const char* name) {
    const char* v = std::getenv(name);
    return v && *v && std::strcmp(v, "0") != 0;
}

bool wrapSymbolBlobZlib(const char* input,
                        int inputSize,
                        std::vector<std::uint8_t>& wrapped) {
    wrapped.clear();
    if (!input || inputSize <= 64 || !envOn("PMSF_SYMBOL_BLOB_ZLIB")) {
        return false;
    }
    uLongf compressedCap = compressBound(static_cast<uLong>(inputSize));
    std::vector<std::uint8_t> compressed(compressedCap);
    uLongf compressedLen = compressedCap;
    const int zret = compress2(compressed.data(), &compressedLen,
                               reinterpret_cast<const Bytef*>(input),
                               static_cast<uLong>(inputSize),
                               Z_BEST_COMPRESSION);
    if (zret != Z_OK || compressedLen + 13u >= static_cast<uLongf>(inputSize)) {
        return false;
    }
    compressed.resize(static_cast<std::size_t>(compressedLen));
    wrapped.reserve(13u + compressed.size());
    wrapped.push_back('S');
    wrapped.push_back('B');
    wrapped.push_back('C');
    wrapped.push_back('1');
    wrapped.push_back(1);  // codec: zlib/deflate
    packU32LE(wrapped, static_cast<std::uint32_t>(inputSize));
    packU32LE(wrapped, static_cast<std::uint32_t>(compressed.size()));
    wrapped.insert(wrapped.end(), compressed.begin(), compressed.end());
    return true;
}

bool wrapSymbolBlobZstd(const char* input,
                        int inputSize,
                        std::vector<std::uint8_t>& wrapped) {
#ifdef PMSF_HAS_ZSTD
    wrapped.clear();
    if (!input || inputSize <= 64 || !envOn("PMSF_SYMBOL_BLOB_ZLIB")) {
        return false;
    }
    int level = 22;
    if (const char* env = std::getenv("PMSF_SYMBOL_BLOB_ZSTD_LEVEL")) {
        const int parsed = std::atoi(env);
        if (parsed > 0) level = parsed;
    }
    const std::size_t cap = ZSTD_compressBound(static_cast<std::size_t>(inputSize));
    std::vector<std::uint8_t> compressed(cap);
    const std::size_t compressedLen =
        ZSTD_compress(compressed.data(), compressed.size(),
                      input, static_cast<std::size_t>(inputSize), level);
    if (ZSTD_isError(compressedLen) || compressedLen == 0 ||
        compressedLen + 13u >= static_cast<std::size_t>(inputSize)) {
        return false;
    }
    compressed.resize(compressedLen);
    wrapped.reserve(13u + compressed.size());
    wrapped.push_back('S');
    wrapped.push_back('B');
    wrapped.push_back('C');
    wrapped.push_back('1');
    wrapped.push_back(2);  // codec: zstd
    packU32LE(wrapped, static_cast<std::uint32_t>(inputSize));
    packU32LE(wrapped, static_cast<std::uint32_t>(compressed.size()));
    wrapped.insert(wrapped.end(), compressed.begin(), compressed.end());
    return true;
#else
    (void)input;
    (void)inputSize;
    wrapped.clear();
    return false;
#endif
}

bool wrapBinaryBlobZlib(const std::vector<std::uint8_t>& input,
                        const char magic[4],
                        std::vector<std::uint8_t>& wrapped) {
    wrapped.clear();
    if (input.size() <= 64) return false;
    uLongf compressedCap = compressBound(static_cast<uLong>(input.size()));
    std::vector<std::uint8_t> compressed(compressedCap);
    uLongf compressedLen = compressedCap;
    const int zret = compress2(compressed.data(), &compressedLen,
                               input.data(), static_cast<uLong>(input.size()),
                               Z_BEST_COMPRESSION);
    if (zret != Z_OK || compressedLen + 12u >= static_cast<uLongf>(input.size())) {
        return false;
    }
    compressed.resize(static_cast<std::size_t>(compressedLen));
    wrapped.reserve(12u + compressed.size());
    wrapped.push_back(magic[0]);
    wrapped.push_back(magic[1]);
    wrapped.push_back(magic[2]);
    wrapped.push_back(magic[3]);
    packU32LE(wrapped, static_cast<std::uint32_t>(input.size()));
    packU32LE(wrapped, static_cast<std::uint32_t>(compressed.size()));
    wrapped.insert(wrapped.end(), compressed.begin(), compressed.end());
    return true;
}

bool zlibCompressBest(const std::uint8_t* data,
                      std::size_t size,
                      std::vector<std::uint8_t>& compressed) {
    if (!data || size == 0) return false;
    uLongf compressedCap = compressBound(static_cast<uLong>(size));
    compressed.resize(compressedCap);
    uLongf compressedLen = compressedCap;
    const int zret = compress2(compressed.data(), &compressedLen,
                               data, static_cast<uLong>(size),
                               Z_BEST_COMPRESSION);
    if (zret != Z_OK || compressedLen == 0) return false;
    compressed.resize(static_cast<std::size_t>(compressedLen));
    return true;
}

bool skipPackedVarint(const std::vector<std::uint8_t>& input, std::size_t& off) {
    for (int i = 0; i < 5 && off < input.size(); ++i) {
        const std::uint8_t b = input[off++];
        if ((b & 0x80u) == 0) return true;
    }
    return false;
}

bool readPackedVarU32ForSplit(const std::vector<std::uint8_t>& input,
                              std::size_t& off,
                              std::uint32_t& value) {
    value = 0;
    int shift = 0;
    for (int i = 0; i < 5 && off < input.size(); ++i) {
        const std::uint8_t b = input[off++];
        value |= static_cast<std::uint32_t>(b & 0x7fu) << shift;
        if ((b & 0x80u) == 0) return true;
        shift += 7;
    }
    return false;
}

bool findPartitionedBaseCoordSplit(const std::vector<std::uint8_t>& input,
                                   std::size_t& splitOffset) {
    if (input.size() < 8 || input[0] != 'P' || input[1] != 'B' ||
        input[2] != 'S') {
        return false;
    }
    std::size_t off = 4;
    std::uint32_t coordCount = 0;
    std::uint32_t groupCount = 0;
    if (!readPackedVarU32ForSplit(input, off, coordCount) ||
        !readPackedVarU32ForSplit(input, off, groupCount) ||
        coordCount == 0) {
        return false;
    }
    (void)groupCount;
    for (std::uint32_t axis = 0; axis < 3; ++axis) {
        for (std::uint32_t i = 0; i < coordCount; ++i) {
            if (!skipPackedVarint(input, off)) return false;
        }
    }
    if (off >= input.size()) return false;
    splitOffset = off;
    return true;
}

bool wrapPartitionedBaseBlobZlibSplit(const std::vector<std::uint8_t>& input,
                                      std::vector<std::uint8_t>& wrapped) {
    wrapped.clear();
    std::size_t splitOffset = 0;
    if (!findPartitionedBaseCoordSplit(input, splitOffset) ||
        splitOffset == 0 || splitOffset >= input.size()) {
        return false;
    }

    std::vector<std::uint8_t> coordCompressed;
    std::vector<std::uint8_t> groupCompressed;
    if (!zlibCompressBest(input.data(), splitOffset, coordCompressed) ||
        !zlibCompressBest(input.data() + splitOffset,
                          input.size() - splitOffset,
                          groupCompressed)) {
        return false;
    }

    const std::size_t wrappedSize = 20u + coordCompressed.size() + groupCompressed.size();
    if (wrappedSize >= input.size()) return false;
    wrapped.reserve(wrappedSize);
    wrapped.push_back('P'); wrapped.push_back('B'); wrapped.push_back('Z'); wrapped.push_back('2');
    packU32LE(wrapped, static_cast<std::uint32_t>(input.size()));
    packU32LE(wrapped, static_cast<std::uint32_t>(splitOffset));
    packU32LE(wrapped, static_cast<std::uint32_t>(coordCompressed.size()));
    packU32LE(wrapped, static_cast<std::uint32_t>(groupCompressed.size()));
    wrapped.insert(wrapped.end(), coordCompressed.begin(), coordCompressed.end());
    wrapped.insert(wrapped.end(), groupCompressed.begin(), groupCompressed.end());
    return true;
}

bool wrapBinaryBlobZstd(const std::vector<std::uint8_t>& input,
                        std::vector<std::uint8_t>& wrapped) {
#ifdef PMSF_HAS_ZSTD
    wrapped.clear();
    if (input.size() <= 64) return false;
    int level = 22;
    if (const char* env = std::getenv("PMSF_PARTITIONED_BASE_ZSTD_LEVEL")) {
        const int parsed = std::atoi(env);
        if (parsed > 0) level = parsed;
    }
    const std::size_t cap = ZSTD_compressBound(input.size());
    std::vector<std::uint8_t> compressed(cap);
    const std::size_t compressedLen =
        ZSTD_compress(compressed.data(), compressed.size(),
                      input.data(), input.size(), level);
    if (ZSTD_isError(compressedLen) || compressedLen == 0 ||
        compressedLen + 12u >= input.size()) {
        return false;
    }
    compressed.resize(compressedLen);
    wrapped.reserve(12u + compressed.size());
    wrapped.push_back('P'); wrapped.push_back('B'); wrapped.push_back('Z'); wrapped.push_back('3');
    packU32LE(wrapped, static_cast<std::uint32_t>(input.size()));
    packU32LE(wrapped, static_cast<std::uint32_t>(compressed.size()));
    wrapped.insert(wrapped.end(), compressed.begin(), compressed.end());
    return true;
#else
    (void)input;
    wrapped.clear();
    return false;
#endif
}

std::vector<std::uint8_t> encodeColumnarMetadataBlob(const PMSFHeader& header,
                                                      bool enableQuantization,
                                                      bool enablePrediction,
                                                      int codecVersion,
                                                      int blobLayout,
                                                      int canonicalSourceLayout,
                                                      int totalVertices,
                                                      int totalFaces,
                                                      const std::shared_ptr<MCGAL::Mesh>& mesh) {
    std::vector<std::uint8_t> blob;
    blob.reserve(96);
    blob.push_back('P'); blob.push_back('M'); blob.push_back('D'); blob.push_back('1');
    packI32LE(blob, header.getRound());
    packI32LE(blob, header.isEnableSegmentation() ? header.getGroupNumber() : 1);
    std::uint8_t flags = 0;
    if (header.isEnableCompress()) flags |= 1u << 0;
    if (header.isEnableQuantization()) flags |= 1u << 1;
    if (header.isEnableSegmentation()) flags |= 1u << 2;
    if (header.isEnableAdjustment()) flags |= 1u << 3;
    if (enablePrediction) flags |= 1u << 4;
    if (header.isEnableTriangulation()) flags |= 1u << 5;
    if (header.isEnableConnectivityPredictionFaces()) flags |= 1u << 6;
    if (header.isEnableConnectivityPredictionEdges()) flags |= 1u << 7;
    packU8(blob, flags);
    packU8(blob, header.isEnableTriangleMeshBalance() ? 1u : 0u);
    packI32LE(blob, codecVersion);
    packI32LE(blob, blobLayout);
    packI32LE(blob, canonicalSourceLayout);
    packI32LE(blob, totalVertices);
    packI32LE(blob, totalFaces);
    if (enableQuantization && mesh) {
        packI32LE(blob, static_cast<std::int32_t>(mesh->i_nbQuantBits));
        packF32LE(blob, static_cast<float>(mesh->bboxMin.x()));
        packF32LE(blob, static_cast<float>(mesh->bboxMin.y()));
        packF32LE(blob, static_cast<float>(mesh->bboxMin.z()));
        packF32LE(blob, mesh->f_quantStep);
    } else {
        packI32LE(blob, 0);
        packF32LE(blob, 0.0f);
        packF32LE(blob, 0.0f);
        packF32LE(blob, 0.0f);
        packF32LE(blob, 0.0f);
    }
    return blob;
}

std::string shellQuote(const std::filesystem::path& p) {
    std::string s = p.string();
    std::string out = "'";
    for (char c : s) {
        if (c == '\'') out += "'\\''";
        else out.push_back(c);
    }
    out += "'";
    return out;
}

std::filesystem::path edgeBreakerTool(const char* exeName) {
    if (const char* dir = std::getenv("PMSF_EDGEBREAKER_DIR")) {
        std::filesystem::path base(dir);
        std::filesystem::path p = base / exeName;
        if (std::filesystem::exists(p)) return p;
        p = base / (std::string(exeName) + ".exe");
        if (std::filesystem::exists(p)) return p;
    }
    std::filesystem::path cwd = std::filesystem::current_path();
    for (const auto& root : {cwd, cwd.parent_path()}) {
        std::filesystem::path p = root / "deps" / "EdgeBreaker" / "src" / exeName;
        if (std::filesystem::exists(p)) return p;
        p = root / "deps" / "EdgeBreaker" / "src" / (std::string(exeName) + ".exe");
        if (std::filesystem::exists(p)) return p;
    }
    return {};
}

bool readWholeFile(const std::filesystem::path& path, std::vector<std::uint8_t>& out) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) return false;
    ifs.seekg(0, std::ios::end);
    const std::streamoff n = ifs.tellg();
    if (n <= 0) return false;
    ifs.seekg(0, std::ios::beg);
    out.resize(static_cast<size_t>(n));
    ifs.read(reinterpret_cast<char*>(out.data()), n);
    return static_cast<bool>(ifs);
}

template <typename GroupData>
bool encodeBaseMeshWithEdgeBreaker(
    const GroupData& g,
    const MCGAL::Point& bboxMin,
    float quantStep,
    std::vector<std::uint8_t>& beb) {
    const std::filesystem::path ebc = edgeBreakerTool("ebc");
    if (ebc.empty()) return false;

    const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
    std::filesystem::path tmp =
        std::filesystem::temp_directory_path() /
        ("pmsf_ebc_" + std::to_string(static_cast<long long>(stamp)));
    std::error_code ec;
    std::filesystem::create_directories(tmp, ec);
    if (ec) return false;

    const std::filesystem::path offPath = tmp / "base.off";
    {
        std::ofstream ofs(offPath);
        if (!ofs) {
            std::filesystem::remove_all(tmp, ec);
            return false;
        }
        const int vertexCount = g.numVertices + 2;
        ofs << "OFF\n" << vertexCount << " " << g.numFaces << " 0\n";
        ofs << std::setprecision(9);
        auto writeQ = [&](int qx, int qy, int qz) {
            ofs << bboxMin.x() + (static_cast<float>(qx) + 0.5f) * quantStep << ' '
                << bboxMin.y() + (static_cast<float>(qy) + 0.5f) * quantStep << ' '
                << bboxMin.z() + (static_cast<float>(qz) + 0.5f) * quantStep << '\n';
        };
        writeQ(g.seedV0QX, g.seedV0QY, g.seedV0QZ);
        writeQ(g.seedV1QX, g.seedV1QY, g.seedV1QZ);
        for (int i = 0; i < g.numVertices; ++i) {
            writeQ(g.vertexQX[i], g.vertexQY[i], g.vertexQZ[i]);
        }
        size_t off = 0;
        for (int f = 0; f < g.numFaces; ++f) {
            ofs << "3 " << g.faceVertices[off] << ' '
                << g.faceVertices[off + 1] << ' '
                << g.faceVertices[off + 2] << '\n';
            off += 3;
        }
    }

    int timeoutSec = 10;
    if (const char* timeoutEnv = std::getenv("PMSF_EDGEBREAKER_TIMEOUT_SEC")) {
        timeoutSec = std::atoi(timeoutEnv);
    }
    const std::string timeoutPrefix =
        timeoutSec > 0 ? ("timeout " + std::to_string(timeoutSec) + "s ") : "";
    const std::string cmd =
        "cd " + shellQuote(tmp) + " && " + timeoutPrefix + shellQuote(ebc) +
        " base.off B >/dev/null 2>&1";
    const int rc = std::system(cmd.c_str());
    const bool ok = (rc == 0) && readWholeFile(tmp / "output.beb", beb);
    std::filesystem::remove_all(tmp, ec);
    return ok;
}

int bitWidthForSize(std::size_t n) {
    if (n <= 1) return 0;
    int bits = 0;
    std::size_t v = n - 1;
    while (v > 0) {
        ++bits;
        v >>= 1;
    }
    return bits;
}

void packFixedBits(std::vector<std::uint8_t>& out,
                   const std::vector<int32_t>& values,
                   int bits) {
    if (bits <= 0) return;
    std::uint64_t acc = 0;
    int accBits = 0;
    const std::uint64_t mask = bits == 64 ? ~0ull : ((1ull << bits) - 1ull);
    for (int32_t value : values) {
        acc |= (static_cast<std::uint64_t>(value) & mask) << accBits;
        accBits += bits;
        while (accBits >= 8) {
            out.push_back(static_cast<std::uint8_t>(acc & 0xffu));
            acc >>= 8;
            accBits -= 8;
        }
    }
    if (accBits > 0) {
        out.push_back(static_cast<std::uint8_t>(acc & 0xffu));
    }
}

std::size_t varU32ByteCount(std::uint32_t x) {
    std::size_t n = 1;
    while (x >= 0x80u) {
        x >>= 7;
        ++n;
    }
    return n;
}

std::size_t deltaCoordIdByteCount(const std::vector<int32_t>& coordIds) {
    std::size_t bytes = 0;
    int32_t prevCoordId = 0;
    for (int32_t coordId : coordIds) {
        if (coordId < prevCoordId) {
            return bytes;
        }
        bytes += varU32ByteCount(static_cast<std::uint32_t>(coordId - prevCoordId));
        prevCoordId = coordId;
    }
    return bytes;
}

template <typename GroupData>
std::vector<int32_t> collectPartitionLocalCoordIds(const GroupData& g) {
    std::vector<int32_t> localCoordIds;
    localCoordIds.reserve(g.faceVertices.size() + 2);
    if (g.seedV0CoordId >= 0) localCoordIds.push_back(g.seedV0CoordId);
    if (g.seedV1CoordId >= 0) localCoordIds.push_back(g.seedV1CoordId);
    for (int32_t coordId : g.faceVertices) {
        if (coordId >= 0) localCoordIds.push_back(coordId);
    }
    std::sort(localCoordIds.begin(), localCoordIds.end());
    localCoordIds.erase(std::unique(localCoordIds.begin(), localCoordIds.end()),
                        localCoordIds.end());
    return localCoordIds;
}

template <typename GroupData>
void logBoundaryCoordDictionaryEstimate(const std::vector<GroupData>& groups) {
    std::unordered_map<int32_t, int> coordGroupCount;
    coordGroupCount.reserve(groups.size() * 256);

    std::vector<std::vector<int32_t>> groupCoordIds;
    groupCoordIds.reserve(groups.size());

    std::uint64_t localCoordRefs = 0;
    std::uint64_t currentDictBytes = 0;
    std::uint64_t interiorOnlyDictBytes = 0;
    std::uint64_t faceCorners = 0;
    std::size_t maxLocalCoords = 0;

    for (const auto& g : groups) {
        auto ids = collectPartitionLocalCoordIds(g);
        localCoordRefs += ids.size();
        currentDictBytes += deltaCoordIdByteCount(ids);
        faceCorners += g.faceVertices.size();
        maxLocalCoords = std::max(maxLocalCoords, ids.size());
        for (int32_t coordId : ids) {
            coordGroupCount[coordId] += 1;
        }
        groupCoordIds.push_back(std::move(ids));
    }

    std::vector<int32_t> boundaryOnceIds;
    boundaryOnceIds.reserve(coordGroupCount.size());
    std::uint64_t repeatedRefs = 0;
    for (const auto& [coordId, count] : coordGroupCount) {
        if (count > 1) {
            boundaryOnceIds.push_back(coordId);
            repeatedRefs += static_cast<std::uint64_t>(count - 1);
        }
    }
    std::sort(boundaryOnceIds.begin(), boundaryOnceIds.end());

    for (const auto& ids : groupCoordIds) {
        std::vector<int32_t> interiorIds;
        interiorIds.reserve(ids.size());
        for (int32_t coordId : ids) {
            auto it = coordGroupCount.find(coordId);
            if (it != coordGroupCount.end() && it->second == 1) {
                interiorIds.push_back(coordId);
            }
        }
        interiorOnlyDictBytes += deltaCoordIdByteCount(interiorIds);
    }

    const std::uint64_t boundaryOnceBytes = deltaCoordIdByteCount(boundaryOnceIds);
    const std::uint64_t idealSharedDictBytes =
        interiorOnlyDictBytes + boundaryOnceBytes;
    const std::uint64_t estimatedSavingBytes =
        currentDictBytes > idealSharedDictBytes
            ? currentDictBytes - idealSharedDictBytes
            : 0;

    std::cout << "[PMSF_BOUNDARY_DICT_ESTIMATE]"
              << " groups=" << groups.size()
              << " unique_coords=" << coordGroupCount.size()
              << " local_coord_refs=" << localCoordRefs
              << " multi_group_coords=" << boundaryOnceIds.size()
              << " repeated_refs=" << repeatedRefs
              << " max_local_coords=" << maxLocalCoords
              << " face_corners=" << faceCorners
              << " current_dict_bytes=" << currentDictBytes
              << " ideal_shared_dict_bytes=" << idealSharedDictBytes
              << " estimated_saving_bytes=" << estimatedSavingBytes
              << std::endl;
}

template <typename GroupData>
bool encodeBoundaryOwnerManifestBlob(const std::vector<int32_t>& qx,
                                     const std::vector<GroupData>& groups,
                                     std::vector<std::uint8_t>& blob) {
    blob.clear();
    if (qx.empty()) return false;

    struct CoordUse {
        std::set<int32_t> groups;
        std::uint32_t refs = 0;
    };
    std::unordered_map<int32_t, CoordUse> coordUses;
    coordUses.reserve(groups.size() * 256);
    std::vector<std::pair<int32_t, std::vector<int32_t>>> groupLocalCoordIds;
    groupLocalCoordIds.reserve(groups.size());

    for (const auto& g : groups) {
        if (!g.faceVerticesAreCoordIds) return false;
        auto ids = collectPartitionLocalCoordIds(g);
        for (int32_t coordId : ids) {
            if (coordId < 0 || static_cast<std::size_t>(coordId) >= qx.size()) return false;
            CoordUse& use = coordUses[coordId];
            use.groups.insert(g.groupId);
            ++use.refs;
        }
        groupLocalCoordIds.emplace_back(g.groupId, std::move(ids));
    }

    std::vector<int32_t> boundaryCoordIds;
    boundaryCoordIds.reserve(coordUses.size());
    for (const auto& [coordId, use] : coordUses) {
        if (use.groups.size() > 1) boundaryCoordIds.push_back(coordId);
    }
    std::sort(boundaryCoordIds.begin(), boundaryCoordIds.end());

    std::unordered_map<int32_t, int32_t> boundaryIndexByCoordId;
    boundaryIndexByCoordId.reserve(boundaryCoordIds.size() * 2 + 1);
    for (std::size_t i = 0; i < boundaryCoordIds.size(); ++i) {
        boundaryIndexByCoordId.emplace(boundaryCoordIds[i], static_cast<int32_t>(i));
    }

    auto edgeKey = [](int32_t a, int32_t b) -> std::uint64_t {
        if (a > b) std::swap(a, b);
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(a)) << 32) |
               static_cast<std::uint32_t>(b);
    };

    std::vector<std::vector<int32_t>> adjacency(boundaryCoordIds.size());
    std::unordered_set<std::uint64_t> seenEdges;
    seenEdges.reserve(boundaryCoordIds.size() * 2 + 1);
    std::uint64_t boundaryEdges = 0;
    for (const auto& g : groups) {
        std::size_t off = 0;
        for (int f = 0; f < g.numFaces; ++f) {
            if (static_cast<std::size_t>(f) >= g.faceDegrees.size()) return false;
            const int32_t degree = std::abs(g.faceDegrees[static_cast<std::size_t>(f)]);
            if (degree <= 0 || off + static_cast<std::size_t>(degree) > g.faceVertices.size()) {
                return false;
            }
            for (int32_t i = 0; i < degree; ++i) {
                const int32_t aCoord = g.faceVertices[off + static_cast<std::size_t>(i)];
                const int32_t bCoord =
                    g.faceVertices[off + static_cast<std::size_t>((i + 1) % degree)];
                auto ai = boundaryIndexByCoordId.find(aCoord);
                auto bi = boundaryIndexByCoordId.find(bCoord);
                if (ai == boundaryIndexByCoordId.end() ||
                    bi == boundaryIndexByCoordId.end() ||
                    ai->second == bi->second) {
                    continue;
                }
                const std::uint64_t key = edgeKey(ai->second, bi->second);
                if (!seenEdges.insert(key).second) continue;
                adjacency[static_cast<std::size_t>(ai->second)].push_back(bi->second);
                adjacency[static_cast<std::size_t>(bi->second)].push_back(ai->second);
                ++boundaryEdges;
            }
            off += static_cast<std::size_t>(degree);
        }
        if (off != g.faceVertices.size()) return false;
    }

    std::vector<int32_t> componentIdByBoundaryIndex(boundaryCoordIds.size(), -1);
    struct ComponentInfo {
        int32_t ownerGroup = 0;
        std::set<int32_t> incidentGroups;
        std::uint32_t coordCount = 0;
    };
    std::vector<ComponentInfo> components;
    std::vector<int32_t> stack;
    std::size_t maxComponentCoords = 0;
    for (std::size_t i = 0; i < boundaryCoordIds.size(); ++i) {
        if (componentIdByBoundaryIndex[i] >= 0) continue;
        const int32_t componentId = static_cast<int32_t>(components.size());
        components.push_back(ComponentInfo{});
        ComponentInfo& component = components.back();
        component.ownerGroup = std::numeric_limits<int32_t>::max();
        stack.clear();
        stack.push_back(static_cast<int32_t>(i));
        componentIdByBoundaryIndex[i] = componentId;
        while (!stack.empty()) {
            const int32_t idx = stack.back();
            stack.pop_back();
            ++component.coordCount;
            const int32_t coordId = boundaryCoordIds[static_cast<std::size_t>(idx)];
            const auto useIt = coordUses.find(coordId);
            if (useIt != coordUses.end()) {
                for (int32_t gid : useIt->second.groups) {
                    component.incidentGroups.insert(gid);
                    component.ownerGroup = std::min(component.ownerGroup, gid);
                }
            }
            for (int32_t next : adjacency[static_cast<std::size_t>(idx)]) {
                if (next < 0 ||
                    static_cast<std::size_t>(next) >= componentIdByBoundaryIndex.size() ||
                    componentIdByBoundaryIndex[static_cast<std::size_t>(next)] >= 0) {
                    continue;
                }
                componentIdByBoundaryIndex[static_cast<std::size_t>(next)] = componentId;
                stack.push_back(next);
            }
        }
        if (component.ownerGroup == std::numeric_limits<int32_t>::max()) {
            component.ownerGroup = 0;
        }
        maxComponentCoords = std::max<std::size_t>(maxComponentCoords, component.coordCount);
    }

    blob.reserve(32 + boundaryCoordIds.size() * 3 + components.size() * 8);
    blob.push_back('P'); blob.push_back('B'); blob.push_back('O'); blob.push_back('1');
    packVarU32(blob, static_cast<std::uint32_t>(qx.size()));
    packVarU32(blob, static_cast<std::uint32_t>(groups.size()));
    packVarU32(blob, static_cast<std::uint32_t>(boundaryCoordIds.size()));
    packVarU32(blob, static_cast<std::uint32_t>(components.size()));

    int32_t prevCoord = 0;
    for (int32_t coordId : boundaryCoordIds) {
        if (coordId < prevCoord) return false;
        packVarU32(blob, static_cast<std::uint32_t>(coordId - prevCoord));
        prevCoord = coordId;
    }
    for (int32_t componentId : componentIdByBoundaryIndex) {
        if (componentId < 0) return false;
        packVarU32(blob, static_cast<std::uint32_t>(componentId));
    }

    std::uint64_t componentIncidentRefs = 0;
    for (const auto& component : components) {
        packVarU32(blob, static_cast<std::uint32_t>(component.ownerGroup));
        packVarU32(blob, component.coordCount);
        packVarU32(blob, static_cast<std::uint32_t>(component.incidentGroups.size()));
        int32_t prevGroup = 0;
        for (int32_t gid : component.incidentGroups) {
            if (gid < prevGroup) return false;
            packVarU32(blob, static_cast<std::uint32_t>(gid - prevGroup));
            prevGroup = gid;
            ++componentIncidentRefs;
        }
    }

    packVarU32(blob, static_cast<std::uint32_t>(groupLocalCoordIds.size()));
    std::uint64_t groupBoundaryRefs = 0;
    for (const auto& [groupId, ids] : groupLocalCoordIds) {
        std::vector<int32_t> boundaryRefs;
        boundaryRefs.reserve(ids.size());
        for (int32_t coordId : ids) {
            auto it = boundaryIndexByCoordId.find(coordId);
            if (it != boundaryIndexByCoordId.end()) boundaryRefs.push_back(it->second);
        }
        std::sort(boundaryRefs.begin(), boundaryRefs.end());
        packVarU32(blob, static_cast<std::uint32_t>(groupId));
        packVarU32(blob, static_cast<std::uint32_t>(boundaryRefs.size()));
        int32_t prevBoundaryIndex = 0;
        for (int32_t boundaryIndex : boundaryRefs) {
            if (boundaryIndex < prevBoundaryIndex) return false;
            packVarU32(blob, static_cast<std::uint32_t>(boundaryIndex - prevBoundaryIndex));
            prevBoundaryIndex = boundaryIndex;
            ++groupBoundaryRefs;
        }
    }

    std::cout << "[PMSF_BOS1_OWNER_MANIFEST]"
              << " encoded=1"
              << " owner_blob_bytes=" << blob.size()
              << " coord_count=" << qx.size()
              << " groups=" << groups.size()
              << " boundary_coords=" << boundaryCoordIds.size()
              << " components=" << components.size()
              << " boundary_edges=" << boundaryEdges
              << " component_incident_refs=" << componentIncidentRefs
              << " group_refs=" << groupBoundaryRefs
              << " max_component_coords=" << maxComponentCoords
              << std::endl;
    return true;
}

std::uint64_t part1By2(std::uint32_t x) {
    std::uint64_t v = x & 0x1fffffu;
    v = (v | (v << 32)) & 0x1f00000000ffffull;
    v = (v | (v << 16)) & 0x1f0000ff0000ffull;
    v = (v | (v << 8))  & 0x100f00f00f00f00full;
    v = (v | (v << 4))  & 0x10c30c30c30c30c3ull;
    v = (v | (v << 2))  & 0x1249249249249249ull;
    return v;
}

std::uint64_t morton3D(std::int32_t x, std::int32_t y, std::int32_t z) {
    return part1By2(static_cast<std::uint32_t>(std::max(0, x))) |
           (part1By2(static_cast<std::uint32_t>(std::max(0, y))) << 1) |
           (part1By2(static_cast<std::uint32_t>(std::max(0, z))) << 2);
}

template <typename GroupData>
void reorderCompactBaseCoordinates(std::vector<int32_t>& qx,
                                   std::vector<int32_t>& qy,
                                   std::vector<int32_t>& qz,
                                   std::vector<GroupData>& groups) {
    const std::size_t n = qx.size();
    if (n == 0 || qy.size() != n || qz.size() != n) return;
    const char* env = std::getenv("PMSF_PARTITIONED_BASE_REORDER");
    const std::string mode = env && *env ? std::string(env) : std::string("morton");
    if (mode == "0" || mode == "off" || mode == "none") return;

    std::vector<int32_t> order(n);
    std::iota(order.begin(), order.end(), 0);
    if (mode == "lex") {
        std::sort(order.begin(), order.end(), [&](int32_t a, int32_t b) {
            if (qx[a] != qx[b]) return qx[a] < qx[b];
            if (qy[a] != qy[b]) return qy[a] < qy[b];
            if (qz[a] != qz[b]) return qz[a] < qz[b];
            return a < b;
        });
    } else {
        std::sort(order.begin(), order.end(), [&](int32_t a, int32_t b) {
            const std::uint64_t ma = morton3D(qx[a], qy[a], qz[a]);
            const std::uint64_t mb = morton3D(qx[b], qy[b], qz[b]);
            if (ma != mb) return ma < mb;
            if (qx[a] != qx[b]) return qx[a] < qx[b];
            if (qy[a] != qy[b]) return qy[a] < qy[b];
            if (qz[a] != qz[b]) return qz[a] < qz[b];
            return a < b;
        });
    }

    bool identity = true;
    for (std::size_t i = 0; i < n; ++i) {
        if (order[i] != static_cast<int32_t>(i)) {
            identity = false;
            break;
        }
    }
    if (identity) return;

    std::vector<int32_t> oldToNew(n, -1);
    for (std::size_t newId = 0; newId < n; ++newId) {
        oldToNew[static_cast<std::size_t>(order[newId])] = static_cast<int32_t>(newId);
    }

    auto remapId = [&](int32_t& id) {
        if (id >= 0 && static_cast<std::size_t>(id) < oldToNew.size()) {
            id = oldToNew[static_cast<std::size_t>(id)];
        }
    };
    for (auto& g : groups) {
        remapId(g.seedV0CoordId);
        remapId(g.seedV1CoordId);
        for (int32_t& id : g.vertexCoordIds) remapId(id);
        for (int32_t& id : g.faceVertices) remapId(id);
    }

    std::vector<int32_t> nx(n), ny(n), nz(n);
    for (std::size_t newId = 0; newId < n; ++newId) {
        const std::size_t oldId = static_cast<std::size_t>(order[newId]);
        nx[newId] = qx[oldId];
        ny[newId] = qy[oldId];
        nz[newId] = qz[oldId];
    }
    qx.swap(nx);
    qy.swap(ny);
    qz.swap(nz);

    std::cout << "[Columnar] partitioned compact base coord reorder: mode="
              << mode << " vertices=" << n << std::endl;
}

void snapCompactBaseCoordinates(std::vector<int32_t>& qx,
                                std::vector<int32_t>& qy,
                                std::vector<int32_t>& qz,
                                int quantBits) {
    const char* env = std::getenv("PMSF_PARTITIONED_BASE_COORD_SNAP_BITS");
    if (!env || !*env) return;
    const int snapBits = std::max(0, std::atoi(env));
    if (snapBits <= 0 || snapBits >= quantBits || snapBits >= 16) return;
    const int step = 1 << snapBits;
    const int half = step >> 1;
    const int maxQ = (quantBits > 0 && quantBits < 31) ? ((1 << quantBits) - 1) : INT_MAX;
    auto snap = [&](int v) {
        int q = ((v + half) / step) * step;
        if (q < 0) q = 0;
        if (q > maxQ) q = maxQ;
        return q;
    };
    for (int32_t& v : qx) v = snap(v);
    for (int32_t& v : qy) v = snap(v);
    for (int32_t& v : qz) v = snap(v);
    std::cout << "[Columnar] partitioned compact base coord snap: low_bits="
              << snapBits << " vertices=" << qx.size() << std::endl;
}

bool encodeQuantCoordBlock(const std::vector<int32_t>& qx,
                           const std::vector<int32_t>& qy,
                           const std::vector<int32_t>& qz,
                           std::size_t first,
                           std::size_t count,
                           std::vector<std::uint8_t>& blob) {
    if (first + count > qx.size() || first + count > qy.size() || first + count > qz.size()) {
        return false;
    }
    blob.clear();
    blob.reserve(16 + count * 6);
    blob.push_back('Q'); blob.push_back('C'); blob.push_back('B'); blob.push_back('1');
    packVarU32(blob, static_cast<std::uint32_t>(count));
    int prev = 0;
    for (std::size_t i = 0; i < count; ++i) {
        const int v = qx[first + i];
        packVarI32(blob, v - prev);
        prev = v;
    }
    prev = 0;
    for (std::size_t i = 0; i < count; ++i) {
        const int v = qy[first + i];
        packVarI32(blob, v - prev);
        prev = v;
    }
    prev = 0;
    for (std::size_t i = 0; i < count; ++i) {
        const int v = qz[first + i];
        packVarI32(blob, v - prev);
        prev = v;
    }
    return true;
}

template <typename GroupData>
bool encodePartitionBaseBlob(const GroupData& g, std::vector<std::uint8_t>& blob) {
    if (!g.faceVerticesAreCoordIds || g.seedV0CoordId < 0 || g.seedV1CoordId < 0) {
        return false;
    }

    bool allTri = !g.faceDegrees.empty() &&
                  g.faceDegrees.size() == static_cast<std::size_t>(g.numFaces);
    for (int32_t degree : g.faceDegrees) {
        if (degree != 3) {
            allTri = false;
            break;
        }
    }

    std::vector<int32_t> encodedFaceVertices = g.faceVertices;
    if (allTri && encodedFaceVertices.size() == static_cast<std::size_t>(g.numFaces) * 3u) {
        std::vector<std::array<int32_t, 3>> tris;
        tris.reserve(static_cast<std::size_t>(g.numFaces));
        for (int f = 0; f < g.numFaces; ++f) {
            tris.push_back({encodedFaceVertices[static_cast<std::size_t>(3 * f + 0)],
                            encodedFaceVertices[static_cast<std::size_t>(3 * f + 1)],
                            encodedFaceVertices[static_cast<std::size_t>(3 * f + 2)]});
        }
        std::sort(tris.begin(), tris.end(), [](const auto& a, const auto& b) {
            const int amin = std::min({a[0], a[1], a[2]});
            const int bmin = std::min({b[0], b[1], b[2]});
            if (amin != bmin) return amin < bmin;
            return a < b;
        });
        encodedFaceVertices.clear();
        encodedFaceVertices.reserve(tris.size() * 3u);
        for (const auto& tri : tris) {
            encodedFaceVertices.push_back(tri[0]);
            encodedFaceVertices.push_back(tri[1]);
            encodedFaceVertices.push_back(tri[2]);
        }
    }

    std::vector<int32_t> sortedCoordIds;
    sortedCoordIds.reserve(encodedFaceVertices.size() + 2);
    sortedCoordIds.push_back(g.seedV0CoordId);
    sortedCoordIds.push_back(g.seedV1CoordId);
    for (int32_t coordId : encodedFaceVertices) {
        if (coordId < 0) return false;
        sortedCoordIds.push_back(coordId);
    }
    std::sort(sortedCoordIds.begin(), sortedCoordIds.end());
    sortedCoordIds.erase(std::unique(sortedCoordIds.begin(), sortedCoordIds.end()),
                         sortedCoordIds.end());
    if (sortedCoordIds.empty()) return false;

    std::vector<int32_t> firstUseCoordIds;
    firstUseCoordIds.reserve(sortedCoordIds.size());
    std::unordered_map<int32_t, bool> seen;
    seen.reserve(sortedCoordIds.size() * 2);
    auto addFirstUse = [&](int32_t coordId) {
        if (coordId < 0 || seen[coordId]) return;
        seen[coordId] = true;
        firstUseCoordIds.push_back(coordId);
    };
    addFirstUse(g.seedV0CoordId);
    addFirstUse(g.seedV1CoordId);
    for (int32_t coordId : encodedFaceVertices) addFirstUse(coordId);

    auto buildCandidate = [&](const std::vector<int32_t>& localCoordIds,
                              bool signedCoordDeltas,
                              std::vector<std::uint8_t>& out) -> bool {
        if (localCoordIds.empty()) return false;
        std::unordered_map<int32_t, int32_t> localByCoord;
        localByCoord.reserve(localCoordIds.size() * 2);
        for (std::size_t i = 0; i < localCoordIds.size(); ++i) {
            localByCoord[localCoordIds[i]] = static_cast<int32_t>(i);
        }
        auto localForCoord = [&](int32_t coordId) -> int32_t {
            auto it = localByCoord.find(coordId);
            return it == localByCoord.end() ? -1 : it->second;
        };

        std::vector<int32_t> faceLocal;
        faceLocal.reserve(encodedFaceVertices.size());
        for (int32_t coordId : encodedFaceVertices) {
            const int32_t local = localForCoord(coordId);
            if (local < 0) return false;
            faceLocal.push_back(local);
        }

        out.clear();
        out.reserve(32 + localCoordIds.size() * 3 + faceLocal.size() * 2);
        out.push_back('P'); out.push_back('B'); out.push_back('M'); out.push_back('1');
        std::uint8_t flags = 0;
        if (allTri) flags |= 0x1;
        if (signedCoordDeltas) flags |= 0x4;
        const size_t flagsPos = out.size();
        packU8(out, flags);
        packVarU32(out, static_cast<std::uint32_t>(localCoordIds.size()));
        packVarU32(out, static_cast<std::uint32_t>(g.numFaces));
        packVarU32(out, static_cast<std::uint32_t>(localForCoord(g.seedV0CoordId)));
        packVarU32(out, static_cast<std::uint32_t>(localForCoord(g.seedV1CoordId)));

        int32_t prevCoordId = 0;
        for (int32_t coordId : localCoordIds) {
            if (signedCoordDeltas) {
                packVarI32(out, coordId - prevCoordId);
            } else {
                if (coordId < prevCoordId) return false;
                packVarU32(out, static_cast<std::uint32_t>(coordId - prevCoordId));
            }
            prevCoordId = coordId;
        }

        if (!allTri) {
            packVarU32(out, static_cast<std::uint32_t>(g.faceDegrees.size()));
            for (int32_t degree : g.faceDegrees) {
                packVarI32(out, degree);
            }
        }

        std::vector<std::uint8_t> bitpackedIndices;
        packVarU32(bitpackedIndices, static_cast<std::uint32_t>(faceLocal.size()));
        const int bits = bitWidthForSize(localCoordIds.size());
        packU8(bitpackedIndices, static_cast<std::uint8_t>(bits));
        packFixedBits(bitpackedIndices, faceLocal, bits);

        std::vector<std::uint8_t> deltaIndices;
        packVarU32(deltaIndices, static_cast<std::uint32_t>(faceLocal.size()));
        if (allTri && faceLocal.size() == static_cast<std::size_t>(g.numFaces) * 3u) {
            for (int corner = 0; corner < 3; ++corner) {
                int prev = 0;
                for (int f = 0; f < g.numFaces; ++f) {
                    const int v = faceLocal[static_cast<std::size_t>(3 * f + corner)];
                    packVarI32(deltaIndices, v - prev);
                    prev = v;
                }
            }
        } else {
            int prev = 0;
            for (int v : faceLocal) {
                packVarI32(deltaIndices, v - prev);
                prev = v;
            }
        }

        if (deltaIndices.size() < bitpackedIndices.size()) {
            flags |= 0x2;  // face local ids are delta-varint encoded
            out.insert(out.end(), deltaIndices.begin(), deltaIndices.end());
        } else {
            out.insert(out.end(), bitpackedIndices.begin(), bitpackedIndices.end());
        }
        out[flagsPos] = flags;
        return true;
    };

    std::vector<std::uint8_t> sortedBlob;
    if (!buildCandidate(sortedCoordIds, false, sortedBlob)) return false;
    std::vector<std::uint8_t> firstUseBlob;
    const bool firstUseOk = buildCandidate(firstUseCoordIds, true, firstUseBlob);
    if (firstUseOk && firstUseBlob.size() < sortedBlob.size()) {
        blob.swap(firstUseBlob);
    } else {
        blob.swap(sortedBlob);
    }
    return true;
}

template <typename GroupData>
bool encodePartitionedBaseSuperBlob(const std::vector<int32_t>& qx,
                                    const std::vector<int32_t>& qy,
                                    const std::vector<int32_t>& qz,
                                    const std::vector<GroupData>& groups,
                                    std::vector<std::uint8_t>& blob) {
    if (qx.empty() || qx.size() != qy.size() || qx.size() != qz.size()) return false;
    blob.clear();
    blob.reserve(32 + qx.size() * 4 + groups.size() * 128);
    blob.push_back('P'); blob.push_back('B'); blob.push_back('S'); blob.push_back('2');
    packVarU32(blob, static_cast<std::uint32_t>(qx.size()));
    packVarU32(blob, static_cast<std::uint32_t>(groups.size()));
    const std::size_t coordStart = blob.size();
    int prev = 0;
    for (int32_t v : qx) {
        packVarI32(blob, v - prev);
        prev = v;
    }
    prev = 0;
    for (int32_t v : qy) {
        packVarI32(blob, v - prev);
        prev = v;
    }
    prev = 0;
    for (int32_t v : qz) {
        packVarI32(blob, v - prev);
        prev = v;
    }
    const std::size_t coordBytes = blob.size() - coordStart;
    const std::size_t groupStart = blob.size();
    std::vector<std::uint8_t> groupBlob;
    for (const auto& g : groups) {
        if (!encodePartitionBaseBlob(g, groupBlob)) return false;
        packVarU32(blob, static_cast<std::uint32_t>(g.groupId));
        packVarU32(blob, static_cast<std::uint32_t>(groupBlob.size()));
        blob.insert(blob.end(), groupBlob.begin(), groupBlob.end());
    }
    std::cout << "[Columnar] partitioned compact base superblob parts:"
              << " coord_bytes=" << coordBytes
              << " group_bytes=" << (blob.size() - groupStart)
              << std::endl;
    return true;
}

template <typename GroupData>
bool encodePartitionedBaseGlobalFaceSuperBlob(const std::vector<int32_t>& qx,
                                              const std::vector<int32_t>& qy,
                                              const std::vector<int32_t>& qz,
                                              const std::vector<GroupData>& groups,
                                              std::vector<std::uint8_t>& blob) {
    if (qx.empty() || qx.size() != qy.size() || qx.size() != qz.size()) return false;
    blob.clear();
    blob.reserve(32 + qx.size() * 4 + groups.size() * 64);
    blob.push_back('P'); blob.push_back('B'); blob.push_back('S'); blob.push_back('4');
    packVarU32(blob, static_cast<std::uint32_t>(qx.size()));
    packVarU32(blob, static_cast<std::uint32_t>(groups.size()));
    const std::size_t coordStart = blob.size();
    int prev = 0;
    for (int32_t v : qx) {
        packVarI32(blob, v - prev);
        prev = v;
    }
    prev = 0;
    for (int32_t v : qy) {
        packVarI32(blob, v - prev);
        prev = v;
    }
    prev = 0;
    for (int32_t v : qz) {
        packVarI32(blob, v - prev);
        prev = v;
    }
    const std::size_t coordBytes = blob.size() - coordStart;
    const std::size_t groupStart = blob.size();

    for (const auto& g : groups) {
        if (!g.faceVerticesAreCoordIds || g.seedV0CoordId < 0 || g.seedV1CoordId < 0) {
            return false;
        }
        if (g.faceDegrees.size() != static_cast<std::size_t>(g.numFaces)) {
            return false;
        }

        struct EncodedFace {
            int32_t storedDegree = 0;
            std::vector<int32_t> verts;
        };
        std::vector<EncodedFace> faces;
        faces.reserve(static_cast<std::size_t>(g.numFaces));
        std::size_t off = 0;
        for (int f = 0; f < g.numFaces; ++f) {
            const int32_t storedDegree = g.faceDegrees[static_cast<std::size_t>(f)];
            const int32_t degree = std::abs(storedDegree);
            if (degree <= 0 || off + static_cast<std::size_t>(degree) > g.faceVertices.size()) {
                return false;
            }
            EncodedFace face;
            face.storedDegree = storedDegree;
            face.verts.reserve(static_cast<std::size_t>(degree));
            for (int32_t i = 0; i < degree; ++i) {
                const int32_t coordId = g.faceVertices[off++];
                if (coordId < 0) return false;
                face.verts.push_back(coordId);
            }
            faces.push_back(std::move(face));
        }
        if (off != g.faceVertices.size()) return false;
        std::sort(faces.begin(), faces.end(), [](const EncodedFace& a, const EncodedFace& b) {
            const int amin = *std::min_element(a.verts.begin(), a.verts.end());
            const int bmin = *std::min_element(b.verts.begin(), b.verts.end());
            if (amin != bmin) return amin < bmin;
            if (a.verts != b.verts) return a.verts < b.verts;
            return a.storedDegree < b.storedDegree;
        });

        packVarU32(blob, static_cast<std::uint32_t>(g.groupId));
        packVarU32(blob, static_cast<std::uint32_t>(g.numFaces));
        packVarU32(blob, static_cast<std::uint32_t>(g.seedV0CoordId));
        packVarU32(blob, static_cast<std::uint32_t>(g.seedV1CoordId));
        packVarU32(blob, static_cast<std::uint32_t>(faces.size()));
        for (const auto& face : faces) {
            packVarI32(blob, face.storedDegree);
        }
        std::uint32_t cornerCount = 0;
        for (const auto& face : faces) {
            cornerCount += static_cast<std::uint32_t>(face.verts.size());
        }
        packVarU32(blob, cornerCount);
        int32_t prevCoord = 0;
        for (const auto& face : faces) {
            for (int32_t coordId : face.verts) {
                packVarI32(blob, coordId - prevCoord);
                prevCoord = coordId;
            }
        }
    }

    std::cout << "[Columnar] partitioned compact base global-face superblob parts:"
              << " coord_bytes=" << coordBytes
              << " group_bytes=" << (blob.size() - groupStart)
              << std::endl;
    return true;
}

template <typename GroupData>
bool encodePartitionedBaseGlobalFaceDegreeModeSuperBlob(const std::vector<int32_t>& qx,
                                                        const std::vector<int32_t>& qy,
                                                        const std::vector<int32_t>& qz,
                                                        const std::vector<GroupData>& groups,
                                                        std::vector<std::uint8_t>& blob) {
    if (qx.empty() || qx.size() != qy.size() || qx.size() != qz.size()) return false;
    blob.clear();
    blob.reserve(32 + qx.size() * 4 + groups.size() * 64);
    blob.push_back('P'); blob.push_back('B'); blob.push_back('S'); blob.push_back('6');
    packVarU32(blob, static_cast<std::uint32_t>(qx.size()));
    packVarU32(blob, static_cast<std::uint32_t>(groups.size()));
    const std::size_t coordStart = blob.size();
    int prev = 0;
    for (int32_t v : qx) {
        packVarI32(blob, v - prev);
        prev = v;
    }
    prev = 0;
    for (int32_t v : qy) {
        packVarI32(blob, v - prev);
        prev = v;
    }
    prev = 0;
    for (int32_t v : qz) {
        packVarI32(blob, v - prev);
        prev = v;
    }
    const std::size_t coordBytes = blob.size() - coordStart;
    const std::size_t groupStart = blob.size();
    std::uint64_t degreeMode0 = 0, degreeMode1 = 0, degreeMode2 = 0;

    for (const auto& g : groups) {
        if (!g.faceVerticesAreCoordIds || g.seedV0CoordId < 0 || g.seedV1CoordId < 0) {
            return false;
        }
        if (g.faceDegrees.size() != static_cast<std::size_t>(g.numFaces)) {
            return false;
        }

        struct EncodedFace {
            int32_t storedDegree = 0;
            std::vector<int32_t> verts;
        };
        std::vector<EncodedFace> faces;
        faces.reserve(static_cast<std::size_t>(g.numFaces));
        std::size_t off = 0;
        bool allAbsTri = true;
        bool anyNegativeTri = false;
        for (int f = 0; f < g.numFaces; ++f) {
            const int32_t storedDegree = g.faceDegrees[static_cast<std::size_t>(f)];
            const int32_t degree = std::abs(storedDegree);
            if (degree != 3) allAbsTri = false;
            if (storedDegree < 0) anyNegativeTri = true;
            if (degree <= 0 || off + static_cast<std::size_t>(degree) > g.faceVertices.size()) {
                return false;
            }
            EncodedFace face;
            face.storedDegree = storedDegree;
            face.verts.reserve(static_cast<std::size_t>(degree));
            for (int32_t i = 0; i < degree; ++i) {
                const int32_t coordId = g.faceVertices[off++];
                if (coordId < 0) return false;
                face.verts.push_back(coordId);
            }
            faces.push_back(std::move(face));
        }
        if (off != g.faceVertices.size()) return false;
        std::sort(faces.begin(), faces.end(), [](const EncodedFace& a, const EncodedFace& b) {
            const int amin = *std::min_element(a.verts.begin(), a.verts.end());
            const int bmin = *std::min_element(b.verts.begin(), b.verts.end());
            if (amin != bmin) return amin < bmin;
            if (a.verts != b.verts) return a.verts < b.verts;
            return a.storedDegree < b.storedDegree;
        });

        packVarU32(blob, static_cast<std::uint32_t>(g.groupId));
        packVarU32(blob, static_cast<std::uint32_t>(g.numFaces));
        packVarU32(blob, static_cast<std::uint32_t>(g.seedV0CoordId));
        packVarU32(blob, static_cast<std::uint32_t>(g.seedV1CoordId));

        std::uint8_t degreeMode = 2;
        if (allAbsTri) degreeMode = anyNegativeTri ? 1 : 0;
        packU8(blob, degreeMode);
        if (degreeMode == 0) {
            ++degreeMode0;
        } else if (degreeMode == 1) {
            ++degreeMode1;
            std::uint8_t acc = 0;
            int bits = 0;
            for (const auto& face : faces) {
                if (face.storedDegree < 0) acc |= static_cast<std::uint8_t>(1u << bits);
                ++bits;
                if (bits == 8) {
                    blob.push_back(acc);
                    acc = 0;
                    bits = 0;
                }
            }
            if (bits > 0) blob.push_back(acc);
        } else {
            ++degreeMode2;
            packVarU32(blob, static_cast<std::uint32_t>(faces.size()));
            for (const auto& face : faces) {
                packVarI32(blob, face.storedDegree);
            }
        }

        std::uint32_t cornerCount = 0;
        for (const auto& face : faces) {
            cornerCount += static_cast<std::uint32_t>(face.verts.size());
        }
        if (degreeMode == 2) {
            packVarU32(blob, cornerCount);
        }
        int32_t prevCoord = 0;
        for (const auto& face : faces) {
            for (int32_t coordId : face.verts) {
                packVarI32(blob, coordId - prevCoord);
                prevCoord = coordId;
            }
        }
    }

    std::cout << "[Columnar] partitioned compact base global-face degree-mode superblob parts:"
              << " coord_bytes=" << coordBytes
              << " group_bytes=" << (blob.size() - groupStart)
              << " mode0=" << degreeMode0
              << " mode1=" << degreeMode1
              << " mode2=" << degreeMode2
              << std::endl;
    return true;
}

template <typename GroupData>
bool encodePartitionedBaseRotatedFaceSuperBlob(const std::vector<int32_t>& qx,
                                               const std::vector<int32_t>& qy,
                                               const std::vector<int32_t>& qz,
                                               const std::vector<GroupData>& groups,
                                               std::vector<std::uint8_t>& blob) {
    if (qx.empty() || qx.size() != qy.size() || qx.size() != qz.size()) return false;
    blob.clear();
    blob.reserve(32 + qx.size() * 4 + groups.size() * 64);
    blob.push_back('P'); blob.push_back('B'); blob.push_back('S'); blob.push_back('7');
    packVarU32(blob, static_cast<std::uint32_t>(qx.size()));
    packVarU32(blob, static_cast<std::uint32_t>(groups.size()));
    const std::size_t coordStart = blob.size();
    int prev = 0;
    for (int32_t v : qx) {
        packVarI32(blob, v - prev);
        prev = v;
    }
    prev = 0;
    for (int32_t v : qy) {
        packVarI32(blob, v - prev);
        prev = v;
    }
    prev = 0;
    for (int32_t v : qz) {
        packVarI32(blob, v - prev);
        prev = v;
    }
    const std::size_t coordBytes = blob.size() - coordStart;
    const std::size_t groupStart = blob.size();

    struct EncodedFace {
        int32_t degree = 0;
        std::vector<int32_t> verts;
    };

    for (const auto& g : groups) {
        if (!g.faceVerticesAreCoordIds || g.seedV0CoordId < 0 || g.seedV1CoordId < 0) {
            return false;
        }
        if (g.faceDegrees.size() != static_cast<std::size_t>(g.numFaces)) {
            return false;
        }

        std::vector<EncodedFace> faces;
        faces.reserve(static_cast<std::size_t>(g.numFaces));
        std::size_t off = 0;
        std::uint64_t cornerCount = 0;
        for (int f = 0; f < g.numFaces; ++f) {
            const int32_t storedDegree = g.faceDegrees[static_cast<std::size_t>(f)];
            if (storedDegree < 0) return false;
            const int32_t degree = storedDegree;
            if (degree <= 0 || off + static_cast<std::size_t>(degree) > g.faceVertices.size()) {
                return false;
            }
            EncodedFace face;
            face.degree = degree;
            face.verts.reserve(static_cast<std::size_t>(degree));
            for (int32_t i = 0; i < degree; ++i) {
                const int32_t coordId = g.faceVertices[off++];
                if (coordId < 0) return false;
                face.verts.push_back(coordId);
            }

            const auto minIt = std::min_element(face.verts.begin(), face.verts.end());
            if (minIt == face.verts.end()) return false;
            const std::size_t rot = static_cast<std::size_t>(minIt - face.verts.begin());
            std::rotate(face.verts.begin(), face.verts.begin() + static_cast<std::ptrdiff_t>(rot),
                        face.verts.end());
            if (*std::min_element(face.verts.begin(), face.verts.end()) != face.verts[0]) {
                return false;
            }
            cornerCount += static_cast<std::uint64_t>(degree);
            faces.push_back(std::move(face));
        }
        if (off != g.faceVertices.size()) return false;

        std::sort(faces.begin(), faces.end(), [](const EncodedFace& a, const EncodedFace& b) {
            if (a.verts[0] != b.verts[0]) return a.verts[0] < b.verts[0];
            if (a.verts != b.verts) return a.verts < b.verts;
            return a.degree < b.degree;
        });

        packVarU32(blob, static_cast<std::uint32_t>(g.groupId));
        packVarU32(blob, static_cast<std::uint32_t>(g.numFaces));
        packVarU32(blob, static_cast<std::uint32_t>(g.seedV0CoordId));
        packVarU32(blob, static_cast<std::uint32_t>(g.seedV1CoordId));
        packVarU32(blob, static_cast<std::uint32_t>(faces.size()));
        for (const auto& face : faces) {
            packVarU32(blob, static_cast<std::uint32_t>(face.degree - 1));
        }
        packVarU32(blob, static_cast<std::uint32_t>(cornerCount));
        int32_t prevAnchor = 0;
        for (const auto& face : faces) {
            const int32_t anchor = face.verts[0];
            if (anchor < prevAnchor) return false;
            packVarU32(blob, static_cast<std::uint32_t>(anchor - prevAnchor));
            prevAnchor = anchor;
            for (std::size_t i = 1; i < face.verts.size(); ++i) {
                const int32_t offset = face.verts[i] - anchor;
                if (offset < 0) return false;
                packVarU32(blob, static_cast<std::uint32_t>(offset));
            }
        }
    }

    std::cout << "[Columnar] partitioned compact base rotated-face superblob parts:"
              << " coord_bytes=" << coordBytes
              << " group_bytes=" << (blob.size() - groupStart)
              << std::endl;
    return true;
}

template <typename GroupData>
bool encodePartitionedBaseSparseDegreeRotatedFaceSuperBlob(
    const std::vector<int32_t>& qx,
    const std::vector<int32_t>& qy,
    const std::vector<int32_t>& qz,
    const std::vector<GroupData>& groups,
    std::vector<std::uint8_t>& blob) {
    if (qx.empty() || qx.size() != qy.size() || qx.size() != qz.size()) return false;
    blob.clear();
    blob.reserve(32 + qx.size() * 4 + groups.size() * 64);
    blob.push_back('P'); blob.push_back('B'); blob.push_back('S'); blob.push_back('9');
    packVarU32(blob, static_cast<std::uint32_t>(qx.size()));
    packVarU32(blob, static_cast<std::uint32_t>(groups.size()));
    const std::size_t coordStart = blob.size();
    int prev = 0;
    for (int32_t v : qx) {
        packVarI32(blob, v - prev);
        prev = v;
    }
    prev = 0;
    for (int32_t v : qy) {
        packVarI32(blob, v - prev);
        prev = v;
    }
    prev = 0;
    for (int32_t v : qz) {
        packVarI32(blob, v - prev);
        prev = v;
    }
    const std::size_t coordBytes = blob.size() - coordStart;
    const std::size_t groupStart = blob.size();
    std::uint64_t degreeMode0 = 0, degreeMode1 = 0, degreeMode2 = 0;
    std::uint64_t sparseDegreeEntries = 0;

    struct EncodedFace {
        int32_t degree = 0;
        std::vector<int32_t> verts;
    };

    for (const auto& g : groups) {
        if (!g.faceVerticesAreCoordIds || g.seedV0CoordId < 0 || g.seedV1CoordId < 0) {
            return false;
        }
        if (g.faceDegrees.size() != static_cast<std::size_t>(g.numFaces)) {
            return false;
        }

        std::vector<EncodedFace> faces;
        faces.reserve(static_cast<std::size_t>(g.numFaces));
        std::size_t off = 0;
        std::uint64_t cornerCount = 0;
        for (int f = 0; f < g.numFaces; ++f) {
            const int32_t storedDegree = g.faceDegrees[static_cast<std::size_t>(f)];
            if (storedDegree < 0) return false;
            const int32_t degree = storedDegree;
            if (degree <= 0 || off + static_cast<std::size_t>(degree) > g.faceVertices.size()) {
                return false;
            }
            EncodedFace face;
            face.degree = degree;
            face.verts.reserve(static_cast<std::size_t>(degree));
            for (int32_t i = 0; i < degree; ++i) {
                const int32_t coordId = g.faceVertices[off++];
                if (coordId < 0) return false;
                face.verts.push_back(coordId);
            }
            const auto minIt = std::min_element(face.verts.begin(), face.verts.end());
            if (minIt == face.verts.end()) return false;
            const std::size_t rot = static_cast<std::size_t>(minIt - face.verts.begin());
            std::rotate(face.verts.begin(),
                        face.verts.begin() + static_cast<std::ptrdiff_t>(rot),
                        face.verts.end());
            cornerCount += static_cast<std::uint64_t>(degree);
            faces.push_back(std::move(face));
        }
        if (off != g.faceVertices.size()) return false;

        std::sort(faces.begin(), faces.end(), [](const EncodedFace& a, const EncodedFace& b) {
            if (a.verts[0] != b.verts[0]) return a.verts[0] < b.verts[0];
            if (a.verts != b.verts) return a.verts < b.verts;
            return a.degree < b.degree;
        });

        std::vector<int32_t> nonTriFaceIndices;
        nonTriFaceIndices.reserve(faces.size() / 8 + 1);
        for (std::size_t i = 0; i < faces.size(); ++i) {
            if (faces[i].degree != 3) {
                nonTriFaceIndices.push_back(static_cast<int32_t>(i));
            }
        }

        std::vector<std::uint8_t> sparseDegreeBlob;
        packVarU32(sparseDegreeBlob, static_cast<std::uint32_t>(nonTriFaceIndices.size()));
        int32_t prevFace = 0;
        for (int32_t faceIndex : nonTriFaceIndices) {
            if (faceIndex < prevFace) return false;
            packVarU32(sparseDegreeBlob, static_cast<std::uint32_t>(faceIndex - prevFace));
            packVarU32(sparseDegreeBlob,
                       static_cast<std::uint32_t>(faces[static_cast<std::size_t>(faceIndex)].degree - 1));
            prevFace = faceIndex;
        }

        std::vector<std::uint8_t> fullDegreeBlob;
        packVarU32(fullDegreeBlob, static_cast<std::uint32_t>(faces.size()));
        for (const auto& face : faces) {
            packVarU32(fullDegreeBlob, static_cast<std::uint32_t>(face.degree - 1));
        }

        std::uint8_t degreeMode = 2;
        if (nonTriFaceIndices.empty()) {
            degreeMode = 0;
            ++degreeMode0;
        } else if (sparseDegreeBlob.size() < fullDegreeBlob.size()) {
            degreeMode = 1;
            ++degreeMode1;
            sparseDegreeEntries += nonTriFaceIndices.size();
        } else {
            ++degreeMode2;
        }

        packVarU32(blob, static_cast<std::uint32_t>(g.groupId));
        packVarU32(blob, static_cast<std::uint32_t>(g.numFaces));
        packVarU32(blob, static_cast<std::uint32_t>(g.seedV0CoordId));
        packVarU32(blob, static_cast<std::uint32_t>(g.seedV1CoordId));
        packU8(blob, degreeMode);
        if (degreeMode == 1) {
            blob.insert(blob.end(), sparseDegreeBlob.begin(), sparseDegreeBlob.end());
        } else if (degreeMode == 2) {
            blob.insert(blob.end(), fullDegreeBlob.begin(), fullDegreeBlob.end());
        }
        packVarU32(blob, static_cast<std::uint32_t>(cornerCount));
        int32_t prevAnchor = 0;
        for (const auto& face : faces) {
            const int32_t anchor = face.verts[0];
            if (anchor < prevAnchor) return false;
            packVarU32(blob, static_cast<std::uint32_t>(anchor - prevAnchor));
            prevAnchor = anchor;
            for (std::size_t i = 1; i < face.verts.size(); ++i) {
                const int32_t offset = face.verts[i] - anchor;
                if (offset < 0) return false;
                packVarU32(blob, static_cast<std::uint32_t>(offset));
            }
        }
    }

    std::cout << "[Columnar] partitioned compact base sparse-degree rotated-face superblob parts:"
              << " coord_bytes=" << coordBytes
              << " group_bytes=" << (blob.size() - groupStart)
              << " mode0=" << degreeMode0
              << " mode1=" << degreeMode1
              << " mode2=" << degreeMode2
              << " sparse_degree_entries=" << sparseDegreeEntries
              << std::endl;
    return true;
}

template <typename GroupData>
bool encodePartitionedBaseAnchorDeltaFaceSuperBlob(
    const std::vector<int32_t>& qx,
    const std::vector<int32_t>& qy,
    const std::vector<int32_t>& qz,
    const std::vector<GroupData>& groups,
    std::vector<std::uint8_t>& blob) {
    if (qx.empty() || qx.size() != qy.size() || qx.size() != qz.size()) return false;
    blob.clear();
    blob.reserve(32 + qx.size() * 4 + groups.size() * 64);
    blob.push_back('P'); blob.push_back('B'); blob.push_back('S'); blob.push_back('A');
    packVarU32(blob, static_cast<std::uint32_t>(qx.size()));
    packVarU32(blob, static_cast<std::uint32_t>(groups.size()));
    const std::size_t coordStart = blob.size();
    int prev = 0;
    for (int32_t v : qx) {
        packVarI32(blob, v - prev);
        prev = v;
    }
    prev = 0;
    for (int32_t v : qy) {
        packVarI32(blob, v - prev);
        prev = v;
    }
    prev = 0;
    for (int32_t v : qz) {
        packVarI32(blob, v - prev);
        prev = v;
    }
    const std::size_t coordBytes = blob.size() - coordStart;
    const std::size_t groupStart = blob.size();
    std::uint64_t degreeMode0 = 0, degreeMode1 = 0, degreeMode2 = 0;
    std::uint64_t sparseDegreeEntries = 0;
    std::uint64_t reusedFaces = 0;

    struct EncodedFace {
        int32_t degree = 0;
        std::vector<int32_t> verts;
    };

    for (const auto& g : groups) {
        if (!g.faceVerticesAreCoordIds || g.seedV0CoordId < 0 || g.seedV1CoordId < 0) {
            return false;
        }
        if (g.faceDegrees.size() != static_cast<std::size_t>(g.numFaces)) {
            return false;
        }

        std::vector<EncodedFace> faces;
        faces.reserve(static_cast<std::size_t>(g.numFaces));
        std::size_t off = 0;
        std::uint64_t cornerCount = 0;
        for (int f = 0; f < g.numFaces; ++f) {
            const int32_t storedDegree = g.faceDegrees[static_cast<std::size_t>(f)];
            if (storedDegree < 0) return false;
            const int32_t degree = storedDegree;
            if (degree <= 0 || off + static_cast<std::size_t>(degree) > g.faceVertices.size()) {
                return false;
            }
            EncodedFace face;
            face.degree = degree;
            face.verts.reserve(static_cast<std::size_t>(degree));
            for (int32_t i = 0; i < degree; ++i) {
                const int32_t coordId = g.faceVertices[off++];
                if (coordId < 0) return false;
                face.verts.push_back(coordId);
            }
            const auto minIt = std::min_element(face.verts.begin(), face.verts.end());
            if (minIt == face.verts.end()) return false;
            const std::size_t rot = static_cast<std::size_t>(minIt - face.verts.begin());
            std::rotate(face.verts.begin(),
                        face.verts.begin() + static_cast<std::ptrdiff_t>(rot),
                        face.verts.end());
            cornerCount += static_cast<std::uint64_t>(degree);
            faces.push_back(std::move(face));
        }
        if (off != g.faceVertices.size()) return false;

        std::sort(faces.begin(), faces.end(), [](const EncodedFace& a, const EncodedFace& b) {
            if (a.verts[0] != b.verts[0]) return a.verts[0] < b.verts[0];
            if (a.verts != b.verts) return a.verts < b.verts;
            return a.degree < b.degree;
        });

        std::vector<int32_t> nonTriFaceIndices;
        nonTriFaceIndices.reserve(faces.size() / 8 + 1);
        for (std::size_t i = 0; i < faces.size(); ++i) {
            if (faces[i].degree != 3) {
                nonTriFaceIndices.push_back(static_cast<int32_t>(i));
            }
        }

        std::vector<std::uint8_t> sparseDegreeBlob;
        packVarU32(sparseDegreeBlob, static_cast<std::uint32_t>(nonTriFaceIndices.size()));
        int32_t prevFace = 0;
        for (int32_t faceIndex : nonTriFaceIndices) {
            if (faceIndex < prevFace) return false;
            packVarU32(sparseDegreeBlob, static_cast<std::uint32_t>(faceIndex - prevFace));
            packVarU32(sparseDegreeBlob,
                       static_cast<std::uint32_t>(faces[static_cast<std::size_t>(faceIndex)].degree - 1));
            prevFace = faceIndex;
        }

        std::vector<std::uint8_t> fullDegreeBlob;
        packVarU32(fullDegreeBlob, static_cast<std::uint32_t>(faces.size()));
        for (const auto& face : faces) {
            packVarU32(fullDegreeBlob, static_cast<std::uint32_t>(face.degree - 1));
        }

        std::uint8_t degreeMode = 2;
        if (nonTriFaceIndices.empty()) {
            degreeMode = 0;
            ++degreeMode0;
        } else if (sparseDegreeBlob.size() < fullDegreeBlob.size()) {
            degreeMode = 1;
            ++degreeMode1;
            sparseDegreeEntries += nonTriFaceIndices.size();
        } else {
            ++degreeMode2;
        }

        packVarU32(blob, static_cast<std::uint32_t>(g.groupId));
        packVarU32(blob, static_cast<std::uint32_t>(g.numFaces));
        packVarU32(blob, static_cast<std::uint32_t>(g.seedV0CoordId));
        packVarU32(blob, static_cast<std::uint32_t>(g.seedV1CoordId));
        packU8(blob, degreeMode);
        if (degreeMode == 1) {
            blob.insert(blob.end(), sparseDegreeBlob.begin(), sparseDegreeBlob.end());
        } else if (degreeMode == 2) {
            blob.insert(blob.end(), fullDegreeBlob.begin(), fullDegreeBlob.end());
        }
        packVarU32(blob, static_cast<std::uint32_t>(cornerCount));
        int32_t prevAnchor = 0;
        std::vector<int32_t> prevFaceVerts;
        for (const auto& face : faces) {
            const int32_t anchor = face.verts[0];
            if (anchor < prevAnchor) return false;
            const int32_t anchorDelta = anchor - prevAnchor;
            packVarU32(blob, static_cast<std::uint32_t>(anchorDelta));
            const bool reusePrevFace = anchorDelta == 0 &&
                                       prevFaceVerts.size() == face.verts.size() &&
                                       static_cast<int32_t>(prevFaceVerts.size()) == face.degree;
            prevAnchor = anchor;
            for (std::size_t i = 1; i < face.verts.size(); ++i) {
                if (reusePrevFace) {
                    packVarI32(blob, face.verts[i] - prevFaceVerts[i]);
                } else {
                    const int32_t offset = face.verts[i] - anchor;
                    if (offset < 0) return false;
                    packVarU32(blob, static_cast<std::uint32_t>(offset));
                }
            }
            if (reusePrevFace) ++reusedFaces;
            prevFaceVerts = face.verts;
        }
    }

    std::cout << "[Columnar] partitioned compact base anchor-delta face superblob parts:"
              << " coord_bytes=" << coordBytes
              << " group_bytes=" << (blob.size() - groupStart)
              << " mode0=" << degreeMode0
              << " mode1=" << degreeMode1
              << " mode2=" << degreeMode2
              << " sparse_degree_entries=" << sparseDegreeEntries
              << " reused_faces=" << reusedFaces
              << std::endl;
    return true;
}

template <typename GroupData>
bool encodePartitionedBaseBoundaryRefSuperBlob(const std::vector<int32_t>& qx,
                                               const std::vector<int32_t>& qy,
                                               const std::vector<int32_t>& qz,
                                               const std::vector<GroupData>& groups,
                                               std::vector<std::uint8_t>& blob) {
    if (qx.empty() || qx.size() != qy.size() || qx.size() != qz.size()) return false;

    std::unordered_map<int32_t, int> coordGroupCount;
    coordGroupCount.reserve(groups.size() * 256);
    for (const auto& g : groups) {
        auto ids = collectPartitionLocalCoordIds(g);
        for (int32_t coordId : ids) {
            if (coordId < 0 || static_cast<std::size_t>(coordId) >= qx.size()) return false;
            coordGroupCount[coordId] += 1;
        }
    }

    std::vector<int32_t> boundaryCoordIds;
    boundaryCoordIds.reserve(coordGroupCount.size());
    for (const auto& [coordId, count] : coordGroupCount) {
        if (count > 1) boundaryCoordIds.push_back(coordId);
    }
    std::sort(boundaryCoordIds.begin(), boundaryCoordIds.end());

    std::unordered_map<int32_t, int32_t> boundaryIndexByCoord;
    boundaryIndexByCoord.reserve(boundaryCoordIds.size() * 2 + 1);
    for (std::size_t i = 0; i < boundaryCoordIds.size(); ++i) {
        boundaryIndexByCoord[boundaryCoordIds[i]] = static_cast<int32_t>(i);
    }

    blob.clear();
    blob.reserve(32 + qx.size() * 4 + groups.size() * 64);
    blob.push_back('P'); blob.push_back('B'); blob.push_back('S'); blob.push_back('8');
    packVarU32(blob, static_cast<std::uint32_t>(qx.size()));
    packVarU32(blob, static_cast<std::uint32_t>(groups.size()));
    const std::size_t coordStart = blob.size();
    int prev = 0;
    for (int32_t v : qx) {
        packVarI32(blob, v - prev);
        prev = v;
    }
    prev = 0;
    for (int32_t v : qy) {
        packVarI32(blob, v - prev);
        prev = v;
    }
    prev = 0;
    for (int32_t v : qz) {
        packVarI32(blob, v - prev);
        prev = v;
    }
    const std::size_t coordBytes = blob.size() - coordStart;

    const std::size_t boundaryStart = blob.size();
    packVarU32(blob, static_cast<std::uint32_t>(boundaryCoordIds.size()));
    prev = 0;
    for (int32_t coordId : boundaryCoordIds) {
        if (coordId < prev) return false;
        packVarU32(blob, static_cast<std::uint32_t>(coordId - prev));
        prev = coordId;
    }
    const std::size_t boundaryBytes = blob.size() - boundaryStart;

    struct EncodedFace {
        int32_t degree = 0;
        std::vector<int32_t> verts;
    };

    const std::size_t groupStart = blob.size();
    for (const auto& g : groups) {
        if (!g.faceVerticesAreCoordIds || g.seedV0CoordId < 0 || g.seedV1CoordId < 0) {
            return false;
        }
        if (g.faceDegrees.size() != static_cast<std::size_t>(g.numFaces)) {
            return false;
        }

        std::vector<EncodedFace> faces;
        faces.reserve(static_cast<std::size_t>(g.numFaces));
        std::size_t off = 0;
        bool allTri = true;
        bool canRotateFaces = true;
        for (int f = 0; f < g.numFaces; ++f) {
            const int32_t storedDegree = g.faceDegrees[static_cast<std::size_t>(f)];
            const int32_t degree = std::abs(storedDegree);
            if (storedDegree != 3) allTri = false;
            if (storedDegree < 0) canRotateFaces = false;
            if (degree <= 0 || off + static_cast<std::size_t>(degree) > g.faceVertices.size()) {
                return false;
            }
            EncodedFace face;
            face.degree = storedDegree;
            face.verts.reserve(static_cast<std::size_t>(degree));
            for (int32_t i = 0; i < degree; ++i) {
                const int32_t coordId = g.faceVertices[off++];
                if (coordId < 0 || static_cast<std::size_t>(coordId) >= qx.size()) return false;
                face.verts.push_back(coordId);
            }
            faces.push_back(std::move(face));
        }
        if (off != g.faceVertices.size()) return false;
        if (canRotateFaces) {
            for (auto& face : faces) {
                const auto minIt = std::min_element(face.verts.begin(), face.verts.end());
                if (minIt == face.verts.end()) return false;
                const std::size_t rot = static_cast<std::size_t>(minIt - face.verts.begin());
                std::rotate(face.verts.begin(),
                            face.verts.begin() + static_cast<std::ptrdiff_t>(rot),
                            face.verts.end());
            }
            std::sort(faces.begin(), faces.end(), [](const EncodedFace& a, const EncodedFace& b) {
                if (a.verts.empty() || b.verts.empty()) return a.verts.size() < b.verts.size();
                if (a.verts[0] != b.verts[0]) return a.verts[0] < b.verts[0];
                if (a.verts != b.verts) return a.verts < b.verts;
                return a.degree < b.degree;
            });
        }

        auto makeCandidate = [&](bool firstUseOrder, std::vector<std::uint8_t>& out) -> bool {
            std::vector<int32_t> interiorCoordIds;
            std::vector<int32_t> boundaryLocalIndices;
            interiorCoordIds.reserve(g.faceVertices.size() + 2);
            boundaryLocalIndices.reserve(g.faceVertices.size() / 4 + 2);

            auto addCoord = [&](int32_t coordId) {
                auto bit = boundaryIndexByCoord.find(coordId);
                if (bit != boundaryIndexByCoord.end()) {
                    boundaryLocalIndices.push_back(bit->second);
                } else {
                    interiorCoordIds.push_back(coordId);
                }
            };
            addCoord(g.seedV0CoordId);
            addCoord(g.seedV1CoordId);
            for (const auto& face : faces) {
                for (int32_t coordId : face.verts) addCoord(coordId);
            }

            if (firstUseOrder) {
                std::vector<int32_t> uniqueInterior;
                std::vector<int32_t> uniqueBoundary;
                uniqueInterior.reserve(interiorCoordIds.size());
                uniqueBoundary.reserve(boundaryLocalIndices.size());
                std::unordered_map<int32_t, bool> seenInterior;
                std::unordered_map<int32_t, bool> seenBoundary;
                seenInterior.reserve(interiorCoordIds.size() * 2 + 1);
                seenBoundary.reserve(boundaryLocalIndices.size() * 2 + 1);
                for (int32_t coordId : interiorCoordIds) {
                    if (!seenInterior[coordId]) {
                        seenInterior[coordId] = true;
                        uniqueInterior.push_back(coordId);
                    }
                }
                for (int32_t boundaryIndex : boundaryLocalIndices) {
                    if (!seenBoundary[boundaryIndex]) {
                        seenBoundary[boundaryIndex] = true;
                        uniqueBoundary.push_back(boundaryIndex);
                    }
                }
                interiorCoordIds.swap(uniqueInterior);
                boundaryLocalIndices.swap(uniqueBoundary);
            } else {
                std::sort(interiorCoordIds.begin(), interiorCoordIds.end());
                interiorCoordIds.erase(std::unique(interiorCoordIds.begin(),
                                                   interiorCoordIds.end()),
                                       interiorCoordIds.end());
                std::sort(boundaryLocalIndices.begin(), boundaryLocalIndices.end());
                boundaryLocalIndices.erase(std::unique(boundaryLocalIndices.begin(),
                                                       boundaryLocalIndices.end()),
                                           boundaryLocalIndices.end());
            }

            if (interiorCoordIds.empty() && boundaryLocalIndices.empty()) return false;

            std::unordered_map<int32_t, int32_t> interiorLocalByCoord;
            std::unordered_map<int32_t, int32_t> boundaryLocalByIndex;
            interiorLocalByCoord.reserve(interiorCoordIds.size() * 2 + 1);
            boundaryLocalByIndex.reserve(boundaryLocalIndices.size() * 2 + 1);
            for (std::size_t i = 0; i < interiorCoordIds.size(); ++i) {
                interiorLocalByCoord[interiorCoordIds[i]] = static_cast<int32_t>(i);
            }
            for (std::size_t i = 0; i < boundaryLocalIndices.size(); ++i) {
                boundaryLocalByIndex[boundaryLocalIndices[i]] = static_cast<int32_t>(i);
            }

            const int32_t interiorCount = static_cast<int32_t>(interiorCoordIds.size());
            const int32_t localNamespace =
                interiorCount + static_cast<int32_t>(boundaryLocalIndices.size());
            auto localRefForCoord = [&](int32_t coordId) -> int32_t {
                auto bit = boundaryIndexByCoord.find(coordId);
                if (bit != boundaryIndexByCoord.end()) {
                    auto lit = boundaryLocalByIndex.find(bit->second);
                    return lit == boundaryLocalByIndex.end() ? -1 : interiorCount + lit->second;
                }
                auto iit = interiorLocalByCoord.find(coordId);
                return iit == interiorLocalByCoord.end() ? -1 : iit->second;
            };

            const int32_t seed0Ref = localRefForCoord(g.seedV0CoordId);
            const int32_t seed1Ref = localRefForCoord(g.seedV1CoordId);
            if (seed0Ref < 0 || seed1Ref < 0) return false;

            std::vector<int32_t> faceRefs;
            faceRefs.reserve(g.faceVertices.size());
            for (const auto& face : faces) {
                for (int32_t coordId : face.verts) {
                    const int32_t ref = localRefForCoord(coordId);
                    if (ref < 0 || ref >= localNamespace) return false;
                    faceRefs.push_back(ref);
                }
            }

            std::vector<std::uint8_t> bitpackedRefs;
            packVarU32(bitpackedRefs, static_cast<std::uint32_t>(faceRefs.size()));
            const int bits = bitWidthForSize(static_cast<std::size_t>(localNamespace));
            packU8(bitpackedRefs, static_cast<std::uint8_t>(bits));
            packFixedBits(bitpackedRefs, faceRefs, bits);

            std::vector<std::uint8_t> deltaRefs;
            packVarU32(deltaRefs, static_cast<std::uint32_t>(faceRefs.size()));
            if (allTri && faceRefs.size() == static_cast<std::size_t>(g.numFaces) * 3u) {
                for (int corner = 0; corner < 3; ++corner) {
                    int32_t prevRef = 0;
                    for (int f = 0; f < g.numFaces; ++f) {
                        const int32_t ref = faceRefs[static_cast<std::size_t>(3 * f + corner)];
                        packVarI32(deltaRefs, ref - prevRef);
                        prevRef = ref;
                    }
                }
            } else {
                int32_t prevRef = 0;
                for (int32_t ref : faceRefs) {
                    packVarI32(deltaRefs, ref - prevRef);
                    prevRef = ref;
                }
            }

            out.clear();
            out.reserve(64 + interiorCoordIds.size() * 2 +
                        boundaryLocalIndices.size() * 2 + faceRefs.size());
            packVarU32(out, static_cast<std::uint32_t>(g.groupId));
            packVarU32(out, static_cast<std::uint32_t>(g.numFaces));
            std::uint8_t flags = 0;
            if (allTri) flags |= 0x1;
            if (deltaRefs.size() < bitpackedRefs.size()) flags |= 0x2;
            if (firstUseOrder) flags |= 0x4;  // signed interior coord deltas
            if (firstUseOrder) flags |= 0x8;  // signed boundary-index deltas
            packU8(out, flags);
            packVarU32(out, static_cast<std::uint32_t>(interiorCoordIds.size()));
            packVarU32(out, static_cast<std::uint32_t>(boundaryLocalIndices.size()));
            packVarU32(out, static_cast<std::uint32_t>(seed0Ref));
            packVarU32(out, static_cast<std::uint32_t>(seed1Ref));

            int32_t prevCoord = 0;
            for (int32_t coordId : interiorCoordIds) {
                if (firstUseOrder) {
                    packVarI32(out, coordId - prevCoord);
                } else {
                    if (coordId < prevCoord) return false;
                    packVarU32(out, static_cast<std::uint32_t>(coordId - prevCoord));
                }
                prevCoord = coordId;
            }
            int32_t prevBoundaryIndex = 0;
            for (int32_t boundaryIndex : boundaryLocalIndices) {
                if (firstUseOrder) {
                    packVarI32(out, boundaryIndex - prevBoundaryIndex);
                } else {
                    if (boundaryIndex < prevBoundaryIndex) return false;
                    packVarU32(out, static_cast<std::uint32_t>(boundaryIndex - prevBoundaryIndex));
                }
                prevBoundaryIndex = boundaryIndex;
            }

            if (!allTri) {
                packVarU32(out, static_cast<std::uint32_t>(g.faceDegrees.size()));
                for (int32_t degree : g.faceDegrees) packVarI32(out, degree);
            }

            if ((flags & 0x2) != 0) {
                out.insert(out.end(), deltaRefs.begin(), deltaRefs.end());
            } else {
                out.insert(out.end(), bitpackedRefs.begin(), bitpackedRefs.end());
            }
            return true;
        };

        std::vector<std::uint8_t> sortedGroupBlob;
        if (!makeCandidate(false, sortedGroupBlob)) return false;
        std::vector<std::uint8_t> firstUseGroupBlob;
        const bool firstUseOk = makeCandidate(true, firstUseGroupBlob);
        if (firstUseOk && firstUseGroupBlob.size() < sortedGroupBlob.size()) {
            blob.insert(blob.end(), firstUseGroupBlob.begin(), firstUseGroupBlob.end());
        } else {
            blob.insert(blob.end(), sortedGroupBlob.begin(), sortedGroupBlob.end());
        }
    }

    std::cout << "[Columnar] partitioned compact base boundary-ref superblob parts:"
              << " coord_bytes=" << coordBytes
              << " boundary_dict_bytes=" << boundaryBytes
              << " group_bytes=" << (blob.size() - groupStart)
              << " boundary_coords=" << boundaryCoordIds.size()
              << std::endl;
    return true;
}

template <typename GroupData>
void logPartitionedBaseEdgeBreakerEstimate(const std::vector<int32_t>& qx,
                                           const std::vector<int32_t>& qy,
                                           const std::vector<int32_t>& qz,
                                           const std::vector<GroupData>& groups,
                                           const MCGAL::Point& bboxMin,
                                           float quantStep,
                                           std::size_t selectedRawBytes) {
    if (!envOn("PMSF_PARTITIONED_BASE_EDGEBREAKER_ESTIMATE")) return;
    if (qx.empty() || qx.size() != qy.size() || qx.size() != qz.size()) return;

    struct LocalTriBase {
        int numVertices = 0;
        int numFaces = 0;
        int seedV0QX = 0, seedV0QY = 0, seedV0QZ = 0;
        int seedV1QX = 0, seedV1QY = 0, seedV1QZ = 0;
        std::vector<int> vertexQX, vertexQY, vertexQZ;
        std::vector<int> faceVertices;
    };

    std::size_t exactTriGroups = 0;
    std::size_t nonTriGroups = 0;
    std::size_t openFaceGroups = 0;
    std::size_t encodedGroups = 0;
    std::size_t failedGroups = 0;
    std::size_t fanTriangles = 0;
    std::size_t bebBytes = 0;

    for (const auto& g : groups) {
        if (!g.faceVerticesAreCoordIds || g.seedV0CoordId < 0 || g.seedV1CoordId < 0 ||
            static_cast<std::size_t>(g.seedV0CoordId) >= qx.size() ||
            static_cast<std::size_t>(g.seedV1CoordId) >= qx.size()) {
            ++failedGroups;
            continue;
        }

        bool allTri = true;
        bool hasOpenFace = false;
        LocalTriBase local;
        std::vector<int32_t> localCoordIds;
        std::unordered_map<int32_t, int32_t> localByCoord;
        auto addCoord = [&](int32_t coordId) -> int32_t {
            auto it = localByCoord.find(coordId);
            if (it != localByCoord.end()) return it->second;
            if (coordId < 0 || static_cast<std::size_t>(coordId) >= qx.size()) return -1;
            const int32_t localId = static_cast<int32_t>(localCoordIds.size());
            localByCoord.emplace(coordId, localId);
            localCoordIds.push_back(coordId);
            return localId;
        };

        const int32_t seed0 = addCoord(g.seedV0CoordId);
        const int32_t seed1 = addCoord(g.seedV1CoordId);
        if (seed0 != 0 || seed1 != 1) {
            ++failedGroups;
            continue;
        }

        std::size_t off = 0;
        for (std::size_t fi = 0; fi < g.faceDegrees.size(); ++fi) {
            const int32_t storedDegree = g.faceDegrees[fi];
            if (storedDegree < 0) hasOpenFace = true;
            const int32_t degree = std::abs(storedDegree);
            if (degree != 3) allTri = false;
            if (degree < 3 || off + static_cast<std::size_t>(degree) > g.faceVertices.size()) {
                hasOpenFace = true;
                break;
            }

            std::vector<int32_t> vids;
            vids.reserve(static_cast<std::size_t>(degree));
            bool validFace = true;
            for (int32_t i = 0; i < degree; ++i) {
                const int32_t localId = addCoord(g.faceVertices[off++]);
                if (localId < 0) {
                    validFace = false;
                    break;
                }
                vids.push_back(localId);
            }
            if (!validFace) {
                hasOpenFace = true;
                break;
            }
            for (int32_t i = 1; i + 1 < degree; ++i) {
                local.faceVertices.push_back(vids[0]);
                local.faceVertices.push_back(vids[static_cast<std::size_t>(i)]);
                local.faceVertices.push_back(vids[static_cast<std::size_t>(i + 1)]);
                ++fanTriangles;
            }
        }
        if (off != g.faceVertices.size()) {
            ++failedGroups;
            continue;
        }
        if (allTri && !hasOpenFace) ++exactTriGroups;
        if (!allTri) ++nonTriGroups;
        if (hasOpenFace) {
            ++openFaceGroups;
            ++failedGroups;
            continue;
        }
        if (local.faceVertices.empty() || localCoordIds.size() < 3) {
            ++failedGroups;
            continue;
        }

        local.seedV0QX = qx[static_cast<std::size_t>(g.seedV0CoordId)];
        local.seedV0QY = qy[static_cast<std::size_t>(g.seedV0CoordId)];
        local.seedV0QZ = qz[static_cast<std::size_t>(g.seedV0CoordId)];
        local.seedV1QX = qx[static_cast<std::size_t>(g.seedV1CoordId)];
        local.seedV1QY = qy[static_cast<std::size_t>(g.seedV1CoordId)];
        local.seedV1QZ = qz[static_cast<std::size_t>(g.seedV1CoordId)];
        local.numVertices = static_cast<int>(localCoordIds.size()) - 2;
        local.numFaces = static_cast<int>(local.faceVertices.size() / 3);
        local.vertexQX.reserve(static_cast<std::size_t>(local.numVertices));
        local.vertexQY.reserve(static_cast<std::size_t>(local.numVertices));
        local.vertexQZ.reserve(static_cast<std::size_t>(local.numVertices));
        for (std::size_t i = 2; i < localCoordIds.size(); ++i) {
            const int32_t coordId = localCoordIds[i];
            local.vertexQX.push_back(qx[static_cast<std::size_t>(coordId)]);
            local.vertexQY.push_back(qy[static_cast<std::size_t>(coordId)]);
            local.vertexQZ.push_back(qz[static_cast<std::size_t>(coordId)]);
        }

        std::vector<std::uint8_t> beb;
        if (encodeBaseMeshWithEdgeBreaker(local, bboxMin, quantStep, beb)) {
            ++encodedGroups;
            bebBytes += beb.size();
        } else {
            ++failedGroups;
        }
    }

    const double ratio = selectedRawBytes > 0
                             ? static_cast<double>(bebBytes) /
                                   static_cast<double>(selectedRawBytes)
                             : 0.0;
    std::cout << "[Columnar] partitioned compact base EdgeBreaker estimate:"
              << " groups=" << groups.size()
              << " exact_tri_groups=" << exactTriGroups
              << " non_tri_groups=" << nonTriGroups
              << " open_face_groups=" << openFaceGroups
              << " encoded_groups=" << encodedGroups
              << " failed_groups=" << failedGroups
              << " fan_triangles=" << fanTriangles
              << " beb_bytes=" << bebBytes
              << " selected_raw_bytes=" << selectedRawBytes
              << " ratio=" << std::fixed << std::setprecision(3) << ratio
              << " diagnostic_only=1"
              << std::defaultfloat
              << std::endl;
}

}  // namespace

bool ColumnarSerializeOperator::writePartitionedCompactBaseMesh(const std::string& baseName) {
    if (!header_ || !enableQuantization_) {
        std::cerr << "[Columnar] partitioned compact base: unsupported state" << std::endl;
        return false;
    }

    std::vector<GroupBaseMeshData> compactGroups = groupBaseMeshData_;
    std::vector<int32_t> coordQX = sharedVertexQX_;
    std::vector<int32_t> coordQY = sharedVertexQY_;
    std::vector<int32_t> coordQZ = sharedVertexQZ_;

    if (compactGroups.size() == 1 && coordQX.empty()) {
        auto& g = compactGroups[0];
        coordQX.clear();
        coordQY.clear();
        coordQZ.clear();
        coordQX.push_back(g.seedV0QX);
        coordQY.push_back(g.seedV0QY);
        coordQZ.push_back(g.seedV0QZ);
        coordQX.push_back(g.seedV1QX);
        coordQY.push_back(g.seedV1QY);
        coordQZ.push_back(g.seedV1QZ);
        for (std::size_t i = 0; i < g.vertexQX.size(); ++i) {
            coordQX.push_back(g.vertexQX[i]);
            coordQY.push_back(g.vertexQY[i]);
            coordQZ.push_back(g.vertexQZ[i]);
        }
        g.seedV0CoordId = 0;
        g.seedV1CoordId = 1;
        g.faceVerticesAreCoordIds = true;
        g.numVertices = static_cast<int>(coordQX.size());
    } else if (!header_->isEnableSegmentation() || !sharedBaseMeshCoordinates_ ||
               !sharedBaseMeshFaceCoordIds_) {
        std::cerr << "[Columnar] partitioned compact base: requires shared face coord ids"
                  << std::endl;
        return false;
    }

    if (coordQX.empty() || coordQX.size() != coordQY.size() ||
        coordQX.size() != coordQZ.size()) {
        std::cerr << "[Columnar] partitioned compact base: missing shared quantized vertices"
                  << std::endl;
        return false;
    }
    snapCompactBaseCoordinates(coordQX, coordQY, coordQZ,
                               mesh() ? static_cast<int>(mesh()->i_nbQuantBits) : 0);
    reorderCompactBaseCoordinates(coordQX, coordQY, coordQZ, compactGroups);
    logBoundaryCoordDictionaryEstimate(compactGroups);

    if (envOn("PMSF_PARTITIONED_BASE_SUPERBLOB")) {
        std::vector<std::uint8_t> superBlob;
        std::vector<std::uint8_t> localBlob;
        std::vector<std::uint8_t> globalFaceBlob;
        std::vector<std::uint8_t> degreeModeBlob;
        std::vector<std::uint8_t> rotatedFaceBlob;
        std::vector<std::uint8_t> sparseDegreeRotatedFaceBlob;
        std::vector<std::uint8_t> anchorDeltaFaceBlob;
        std::vector<std::uint8_t> boundaryRefBlob;
        const bool localOk = encodePartitionedBaseSuperBlob(coordQX, coordQY, coordQZ,
                                                            compactGroups, localBlob);
        const bool globalOk = encodePartitionedBaseGlobalFaceSuperBlob(
            coordQX, coordQY, coordQZ, compactGroups, globalFaceBlob);
        const bool degreeModeOk = encodePartitionedBaseGlobalFaceDegreeModeSuperBlob(
            coordQX, coordQY, coordQZ, compactGroups, degreeModeBlob);
        const bool rotatedFaceOk = encodePartitionedBaseRotatedFaceSuperBlob(
            coordQX, coordQY, coordQZ, compactGroups, rotatedFaceBlob);
        const bool sparseDegreeRotatedFaceOk =
            encodePartitionedBaseSparseDegreeRotatedFaceSuperBlob(
                coordQX, coordQY, coordQZ, compactGroups, sparseDegreeRotatedFaceBlob);
        const bool anchorDeltaFaceOk =
            encodePartitionedBaseAnchorDeltaFaceSuperBlob(
                coordQX, coordQY, coordQZ, compactGroups, anchorDeltaFaceBlob);
        const bool boundaryRefOk = encodePartitionedBaseBoundaryRefSuperBlob(
            coordQX, coordQY, coordQZ, compactGroups, boundaryRefBlob);
        const char* faceCodecEnv = std::getenv("PMSF_PARTITIONED_BASE_FACE_CODEC");
        const std::string faceCodec = faceCodecEnv && *faceCodecEnv
                                          ? std::string(faceCodecEnv)
                                          : std::string("auto");
        if ((faceCodec == "boundary" || faceCodec == "pbs8") && boundaryRefOk) {
            superBlob.swap(boundaryRefBlob);
        } else if ((faceCodec == "anchor-delta" || faceCodec == "pbsa") &&
                   anchorDeltaFaceOk) {
            superBlob.swap(anchorDeltaFaceBlob);
        } else if ((faceCodec == "sparse-degree" || faceCodec == "pbs9") &&
                   sparseDegreeRotatedFaceOk) {
            superBlob.swap(sparseDegreeRotatedFaceBlob);
        } else if ((faceCodec == "rotated" || faceCodec == "pbs7") && rotatedFaceOk) {
            superBlob.swap(rotatedFaceBlob);
        } else if ((faceCodec == "degree" || faceCodec == "pbs6") && degreeModeOk) {
            superBlob.swap(degreeModeBlob);
        } else if (faceCodec == "global" && globalOk) {
            superBlob.swap(globalFaceBlob);
        } else if (faceCodec == "local" && localOk) {
            superBlob.swap(localBlob);
        } else if (boundaryRefOk &&
                   (!sparseDegreeRotatedFaceOk ||
                    boundaryRefBlob.size() < sparseDegreeRotatedFaceBlob.size()) &&
                   (!rotatedFaceOk || boundaryRefBlob.size() < rotatedFaceBlob.size()) &&
                   (!degreeModeOk || boundaryRefBlob.size() < degreeModeBlob.size()) &&
                   (!globalOk || boundaryRefBlob.size() < globalFaceBlob.size()) &&
                   (!localOk || boundaryRefBlob.size() < localBlob.size())) {
            superBlob.swap(boundaryRefBlob);
        } else if (sparseDegreeRotatedFaceOk &&
                   (!rotatedFaceOk ||
                    sparseDegreeRotatedFaceBlob.size() < rotatedFaceBlob.size()) &&
                   (!degreeModeOk ||
                    sparseDegreeRotatedFaceBlob.size() < degreeModeBlob.size()) &&
                   (!globalOk ||
                    sparseDegreeRotatedFaceBlob.size() < globalFaceBlob.size()) &&
                   (!localOk || sparseDegreeRotatedFaceBlob.size() < localBlob.size())) {
            superBlob.swap(sparseDegreeRotatedFaceBlob);
        } else if (rotatedFaceOk &&
                   (!degreeModeOk || rotatedFaceBlob.size() < degreeModeBlob.size()) &&
                   (!globalOk || rotatedFaceBlob.size() < globalFaceBlob.size()) &&
                   (!localOk || rotatedFaceBlob.size() < localBlob.size())) {
            superBlob.swap(rotatedFaceBlob);
        } else if (degreeModeOk &&
                   (!globalOk || degreeModeBlob.size() < globalFaceBlob.size()) &&
                   (!localOk || degreeModeBlob.size() < localBlob.size())) {
            superBlob.swap(degreeModeBlob);
        } else if (globalOk && (!localOk || globalFaceBlob.size() < localBlob.size())) {
            superBlob.swap(globalFaceBlob);
        } else if (localOk) {
            superBlob.swap(localBlob);
        }
        if (superBlob.empty()) {
            std::cerr << "[Columnar] partitioned compact base superblob encode failed"
                      << std::endl;
            return false;
        }
        if (mesh()) {
            logPartitionedBaseEdgeBreakerEstimate(coordQX, coordQY, coordQZ,
                                                  compactGroups,
                                                  mesh()->bboxMin,
                                                  mesh()->f_quantStep,
                                                  superBlob.size());
        }
        const std::string rawCodecName =
            superBlob.size() >= 4
                ? std::string(reinterpret_cast<const char*>(superBlob.data()), 4)
                : std::string("unknown");
        std::vector<std::uint8_t> wrappedBaseBlob;
        std::string wrapperName = "raw";
        if (envOn("PMSF_PARTITIONED_BASE_BLOB_ZLIB")) {
            std::vector<std::uint8_t> wholeWrapped;
            std::vector<std::uint8_t> splitWrapped;
            std::vector<std::uint8_t> zstdWrapped;
            const bool wholeOk = wrapBinaryBlobZlib(superBlob, "PBZ1", wholeWrapped);
            const bool splitOk = wrapPartitionedBaseBlobZlibSplit(superBlob, splitWrapped);
            const bool zstdOk = wrapBinaryBlobZstd(superBlob, zstdWrapped);
            if (zstdOk &&
                (!wholeOk || zstdWrapped.size() < wholeWrapped.size()) &&
                (!splitOk || zstdWrapped.size() < splitWrapped.size())) {
                wrappedBaseBlob.swap(zstdWrapped);
                wrapperName = "PBZ3:zstd";
            } else if (splitOk && (!wholeOk || splitWrapped.size() < wholeWrapped.size())) {
                wrappedBaseBlob.swap(splitWrapped);
                wrapperName = "PBZ2:zlib-split";
            } else if (wholeOk) {
                wrappedBaseBlob.swap(wholeWrapped);
                wrapperName = "PBZ1:zlib";
            }
        }
        const bool baseBlobWrapped = !wrappedBaseBlob.empty();
        const std::vector<std::uint8_t>& storedBaseBlob =
            baseBlobWrapped ? wrappedBaseBlob : superBlob;
        const bool embedMetadata = envOn("PMSF_EMBED_METADATA_IN_PARTITIONED_BASE");
        const bool embedSymbols =
            envOn("PMSF_EMBED_SYMBOLS_IN_PARTITIONED_BASE") &&
            !lastSymbolsBlob_.empty();
        const bool embedBoundaryOwner =
            envOn("PMSF_EMBED_BOUNDARY_OWNER_IN_PARTITIONED_BASE");
        const bool logBoundaryOwner =
            embedBoundaryOwner || envOn("PMSF_BOS1_OWNER_MANIFEST");
        std::vector<std::uint8_t> boundaryOwnerBlob;
        const bool boundaryOwnerOk =
            logBoundaryOwner &&
            encodeBoundaryOwnerManifestBlob(coordQX, compactGroups, boundaryOwnerBlob);
        std::vector<std::shared_ptr<arrow::Field>> fields = {
            arrow::field(pmsf::kParquetBaseModeColumn, arrow::int32(), false),
            arrow::field(pmsf::kParquetBaseBlobColumn, arrow::binary(), false),
        };
        if (embedMetadata) {
            fields.push_back(arrow::field(pmsf::kParquetMetadataBlobColumn, arrow::binary(), false));
        }
        if (embedSymbols) {
            fields.push_back(arrow::field(pmsf::kParquetSymbolsBlobColumn, arrow::binary(), false));
        }
        if (embedBoundaryOwner && boundaryOwnerOk) {
            fields.push_back(arrow::field(pmsf::kParquetBoundaryOwnerBlobColumn, arrow::binary(), false));
        }
        auto schema = arrow::schema(fields);
        arrow::Int32Builder modeB;
        arrow::BinaryBuilder blobB;
        (void)modeB.Append(2);
        (void)blobB.Append(storedBaseBlob.data(),
                           static_cast<int32_t>(storedBaseBlob.size()));
        std::shared_ptr<arrow::Array> modeA, blobA;
        (void)modeB.Finish(&modeA);
        (void)blobB.Finish(&blobA);
        std::vector<std::shared_ptr<arrow::Array>> columns = {modeA, blobA};
        if (embedMetadata) {
            const auto metadataBlob = encodeColumnarMetadataBlob(
                *header_, enableQuantization_, enablePrediction_, codecVersion_,
                static_cast<int>(blobLayout_), canonicalSourceLayout_,
                totalVertices_, totalFaces_, mesh());
            arrow::BinaryBuilder metaB;
            (void)metaB.Append(metadataBlob.data(),
                               static_cast<int32_t>(metadataBlob.size()));
            std::shared_ptr<arrow::Array> metaA;
            (void)metaB.Finish(&metaA);
            columns.push_back(metaA);
        }
        if (embedSymbols) {
            arrow::BinaryBuilder symB;
            (void)symB.Append(lastSymbolsBlob_.data(),
                              static_cast<int32_t>(lastSymbolsBlob_.size()));
            std::shared_ptr<arrow::Array> symA;
            (void)symB.Finish(&symA);
            columns.push_back(symA);
        }
        if (embedBoundaryOwner && boundaryOwnerOk) {
            arrow::BinaryBuilder ownerB;
            (void)ownerB.Append(boundaryOwnerBlob.data(),
                                static_cast<int32_t>(boundaryOwnerBlob.size()));
            std::shared_ptr<arrow::Array> ownerA;
            (void)ownerB.Finish(&ownerA);
            columns.push_back(ownerA);
        }
        auto table = arrow::Table::Make(schema, columns, 1);
        writeParquetFile(pmsf::paperPartitionedBaseTablePath(baseName), schema, table, 1);
        std::cout << "[Columnar] partitioned compact base superblob: vertices="
                  << coordQX.size()
                  << " groups=" << compactGroups.size()
                  << " raw_blob_bytes=" << superBlob.size()
                  << " stored_blob_bytes=" << storedBaseBlob.size()
                  << " codec=" << rawCodecName
                  << " wrapper=" << wrapperName
                  << " metadata_embedded=" << (embedMetadata ? 1 : 0)
                  << " symbols_embedded=" << (embedSymbols ? 1 : 0)
                  << " boundary_owner_embedded="
                  << ((embedBoundaryOwner && boundaryOwnerOk) ? 1 : 0)
                  << std::endl;
        return true;
    }

    int blockSize = 4096;
    if (const char* env = std::getenv("PMSF_PARTITION_VERTEX_BLOCK_SIZE")) {
        const int v = std::atoi(env);
        if (v > 0) blockSize = v;
    }
    int rowGroupSize = 64;
    if (const char* env = std::getenv("PMSF_PARTITION_ROW_GROUP_SIZE")) {
        const int v = std::atoi(env);
        if (v > 0) rowGroupSize = v;
    }

    auto vertexSchema = arrow::schema({
        arrow::field(pmsf::kParquetBlockIdColumn,       arrow::int32(), false),
        arrow::field(pmsf::kParquetFirstCoordIdColumn, arrow::int32(), false),
        arrow::field(pmsf::kParquetCountColumn,          arrow::int32(), false),
        arrow::field(pmsf::kParquetCoordBlobColumn,     arrow::binary(), false),
    });

    arrow::Int32Builder blockIdB, firstCoordB, countB;
    arrow::BinaryBuilder coordBlobB;
    std::vector<std::uint8_t> blob;
    int blockId = 0;
    for (std::size_t first = 0; first < coordQX.size(); first += blockSize) {
        const std::size_t count =
            std::min<std::size_t>(static_cast<std::size_t>(blockSize),
                                  coordQX.size() - first);
        if (!encodeQuantCoordBlock(coordQX, coordQY, coordQZ,
                                   first, count, blob)) {
            return false;
        }
        (void)blockIdB.Append(blockId++);
        (void)firstCoordB.Append(static_cast<int32_t>(first));
        (void)countB.Append(static_cast<int32_t>(count));
        (void)coordBlobB.Append(blob.data(), static_cast<int32_t>(blob.size()));
    }

    std::shared_ptr<arrow::Array> blockIdA, firstCoordA, countA, coordBlobA;
    (void)blockIdB.Finish(&blockIdA);
    (void)firstCoordB.Finish(&firstCoordA);
    (void)countB.Finish(&countA);
    (void)coordBlobB.Finish(&coordBlobA);
    auto vertexTable = arrow::Table::Make(vertexSchema,
                                          {blockIdA, firstCoordA, countA, coordBlobA},
                                          blockId);

    auto partitionSchema = arrow::schema({
        arrow::field(pmsf::kParquetGroupIdColumn,          arrow::int32(), false),
        arrow::field(pmsf::kParquetNumBaseVerticesColumn, arrow::int32(), false),
        arrow::field(pmsf::kParquetNumBaseFacesColumn,    arrow::int32(), false),
        arrow::field(pmsf::kParquetSeedV0CoordIdColumn,  arrow::int32(), false),
        arrow::field(pmsf::kParquetSeedV1CoordIdColumn,  arrow::int32(), false),
        arrow::field(pmsf::kParquetBaseModeColumn,         arrow::int32(), false),
        arrow::field(pmsf::kParquetBaseBlobColumn,         arrow::binary(), false),
    });

    arrow::Int32Builder groupIdB, numVertB, numFaceB, seed0B, seed1B, modeB;
    arrow::BinaryBuilder baseBlobB;
    for (const auto& g : compactGroups) {
        if (!encodePartitionBaseBlob(g, blob)) {
            std::cerr << "[Columnar] partitioned compact base: encode failed for group "
                      << g.groupId << std::endl;
            return false;
        }
        (void)groupIdB.Append(g.groupId);
        (void)numVertB.Append(g.numVertices);
        (void)numFaceB.Append(g.numFaces);
        (void)seed0B.Append(g.seedV0CoordId);
        (void)seed1B.Append(g.seedV1CoordId);
        (void)modeB.Append(1);  // 1 = sorted local coord map + bitpacked local face ids
        (void)baseBlobB.Append(blob.data(), static_cast<int32_t>(blob.size()));
    }

    std::shared_ptr<arrow::Array> groupIdA, numVertA, numFaceA, seed0A, seed1A, modeA, baseBlobA;
    (void)groupIdB.Finish(&groupIdA);
    (void)numVertB.Finish(&numVertA);
    (void)numFaceB.Finish(&numFaceA);
    (void)seed0B.Finish(&seed0A);
    (void)seed1B.Finish(&seed1A);
    (void)modeB.Finish(&modeA);
    (void)baseBlobB.Finish(&baseBlobA);
    auto partitionTable = arrow::Table::Make(partitionSchema,
                                             {groupIdA, numVertA, numFaceA,
                                              seed0A, seed1A, modeA, baseBlobA},
                                             static_cast<int64_t>(compactGroups.size()));

    writeParquetFile(pmsf::paperVerticesTablePath(baseName), vertexSchema, vertexTable, rowGroupSize);
    writeParquetFile(pmsf::paperPartitionsTablePath(baseName), partitionSchema, partitionTable, rowGroupSize);
    std::cout << "[Columnar] partitioned compact base: vertices="
              << coordQX.size()
              << " blocks=" << blockId
              << " groups=" << compactGroups.size()
              << " block_size=" << blockSize
              << " row_group_size=" << rowGroupSize
              << std::endl;
    return true;
}

bool ColumnarSerializeOperator::buildV2CompactBinPayload(std::vector<std::uint8_t>& compactBin) {
    compactBin.clear();
    if (!header_) {
        std::cerr << "[Columnar] buildV2CompactBinPayload: header_ null" << std::endl;
        return false;
    }

    // ---- 1. 组装 payload ----
    std::vector<std::uint8_t> payload;
    payload.reserve(64 * 1024);

    const bool hasQuant = header_->isEnableQuantization();
    const bool hasSeg   = header_->isEnableSegmentation();

    bool compactU16 = hasQuant;
    bool triFaces = true;
    if (hasQuant) {
        for (const auto& g : groupBaseMeshData_) {
            auto inU16 = [](int v) { return v >= 0 && v <= 65535; };
            compactU16 = compactU16 &&
                          inU16(g.seedV0QX) && inU16(g.seedV0QY) && inU16(g.seedV0QZ) &&
                          inU16(g.seedV1QX) && inU16(g.seedV1QY) && inU16(g.seedV1QZ);
            for (int i = 0; i < static_cast<int>(g.vertexQX.size()); ++i) {
                compactU16 = compactU16 &&
                              inU16(g.vertexQX[i]) &&
                              inU16(g.vertexQY[i]) &&
                              inU16(g.vertexQZ[i]);
            }
            if (g.numVertices + 2 > 65535) compactU16 = false;
            for (int fv : g.faceVertices) {
                if (fv < 0 || fv > 65535) compactU16 = false;
            }
            for (int d : g.faceDegrees) {
                if (d != 3) triFaces = false;
            }
        }
    } else {
        compactU16 = false;
        triFaces = false;
    }

    const bool varintBase = hasQuant;
    bool edgeBreakerBase = false;
    std::vector<std::vector<std::uint8_t>> edgeBreakerBlobs(groupBaseMeshData_.size());
    int edgeBreakerMinVertices = 128;
    if (const char* minEnv = std::getenv("PMSF_PCATM_BASE_EDGEBREAKER_MIN_VERTICES")) {
        const int v = std::atoi(minEnv);
        if (v >= 0) edgeBreakerMinVertices = v;
    }
    const bool forceEdgeBreaker = envOn("PMSF_PCATM_BASE_EDGEBREAKER_FORCE");
    const int baseVertexCountForEb =
        groupBaseMeshData_.empty() ? 0 : (groupBaseMeshData_[0].numVertices + 2);
    if (envOn("PMSF_PCATM_BASE_EDGEBREAKER") &&
        hasQuant && !hasSeg && triFaces && groupBaseMeshData_.size() == 1 && mesh()) {
        if (forceEdgeBreaker || baseVertexCountForEb >= edgeBreakerMinVertices) {
            edgeBreakerBase = true;
            for (size_t gi = 0; gi < groupBaseMeshData_.size(); ++gi) {
                if (!encodeBaseMeshWithEdgeBreaker(groupBaseMeshData_[gi],
                                                   mesh()->bboxMin,
                                                   mesh()->f_quantStep,
                                                   edgeBreakerBlobs[gi])) {
                    edgeBreakerBase = false;
                    edgeBreakerBlobs.clear();
                    std::cerr << "[Columnar] EdgeBreaker base mesh encode failed; "
                              << "fallback to varint compact base mesh" << std::endl;
                    break;
                }
            }
        } else {
            std::cerr << "[Columnar] EdgeBreaker base mesh skipped: baseVertices="
                      << baseVertexCountForEb
                      << " < PMSF_PCATM_BASE_EDGEBREAKER_MIN_VERTICES="
                      << edgeBreakerMinVertices
                      << " (set PMSF_PCATM_BASE_EDGEBREAKER_FORCE=1 to force)"
                      << std::endl;
        }
    }

    // ---- native in-process EdgeBreaker base (PMSF_BASE_EDGEBREAKER_NATIVE) ----
    // Works for the single-group (optionally segmented) quantized triangular
    // base. The connectivity + geometry are rebuilt from the live mesh via
    // buildEbPlan so the EB traversal order matches the facet->partition
    // channel (SegmentedOptimizedSymbolCollectOperator uses the same plan).
    bool nativeEdgeBreakerBase = false;
    std::vector<std::uint8_t> nativeEbBlob;
    if (envOn("PMSF_BASE_EDGEBREAKER_NATIVE") && hasQuant && triFaces &&
        groupBaseMeshData_.size() == 1 && mesh()) {
        const bool preservesLifted =
            collector_ ? collector_->preservesLiftedBaseMesh() : false;
        const int baseQuantBits =
            static_cast<int>(mesh()->i_nbQuantBits) + (preservesLifted ? 1 : 0);
        pmsf_eb::EbMeshPlan plan =
            pmsf_eb::buildEbPlan(mesh().get(), baseQuantBits, /*wantCoords=*/true);
        if (plan.ok && plan.allTriangles) {
            nativeEbBlob = pmsf_eb::encodeEdgeBreaker(plan.verts, plan.faces);
            if (!nativeEbBlob.empty())
                nativeEdgeBreakerBase = true;
            else
                std::cerr << "[Columnar] native EdgeBreaker base encode failed; "
                          << "fallback to splitResidualBase" << std::endl;
        }
    }

    // magic "V2CP" + version + flags
    payload.push_back('V'); payload.push_back('2');
    payload.push_back('C'); payload.push_back('P');
    const bool splitResidualBase = varintBase && triFaces && !edgeBreakerBase && !nativeEdgeBreakerBase;
    packU16LE(payload, nativeEdgeBreakerBase ? 6 : (edgeBreakerBase ? 4 : (splitResidualBase ? 5 : (varintBase ? 3 : (compactU16 ? 2 : 1)))));
    std::uint16_t flags = 0;
    if (hasQuant) flags |= 0x1;
    if (hasSeg)   flags |= 0x2;
    if (compactU16) flags |= 0x4;
    if (triFaces)   flags |= 0x8;
    if (varintBase) flags |= 0x10;
    if (edgeBreakerBase) flags |= 0x20;
    if (splitResidualBase) flags |= 0x40;
    if (nativeEdgeBreakerBase) flags |= 0x200;
    if (canonicalSourceLayout_ != 0) flags |= 0x80;
    // Phase S0（R13）：仅在**指定了非默认存储方案**时置位并写自描述块。
    // 未指定 ⇒ 该位为 0、不写任何字节 ⇒ 产物与引入本机制前逐字节相同。
    const ::pmsf::StorageSchemePtr scheme = ::pmsf::activeStorageScheme();
    const bool hasScheme = (scheme != nullptr) && !scheme->info().name.empty();
    if (hasScheme) flags |= 0x100;
    packU16LE(payload, flags);

    // --- metadata 段 ---
    packI32LE(payload, header_->getRound());
    packI32LE(payload, hasSeg ? header_->getGroupNumber() : 0);
    std::uint8_t flagBits = 0;
    if (header_->isEnableCompress())     flagBits |= 0x01;
    if (header_->isEnableQuantization()) flagBits |= 0x02;
    if (header_->isEnableSegmentation()) flagBits |= 0x04;
    if (header_->isEnableAdjustment())   flagBits |= 0x08;
    // V2 metadata 表的 enable_prediction 列来自 ColumnarSerializeOperator::enablePrediction_
    // 成员（构造参数），不是 PMSFHeader（除 PPMC 外大多数算法不把 prediction 放到 header_）。
    if (enablePrediction_) flagBits |= 0x10;
    if (header_->isEnableTriangulation()) flagBits |= 0x20;
    if (header_->isEnableConnectivityPredictionFaces()) flagBits |= 0x40;
    if (header_->isEnableConnectivityPredictionEdges()) flagBits |= 0x80;
    packU8(payload, flagBits);
    // PPMC tri-balance 用第二字节（u8 flagBits2，扩展位）
    std::uint8_t flagBits2 = 0;
    if (header_->isEnableTriangleMeshBalance()) flagBits2 |= 0x01;
    // bit 1: PPMC local curvature prediction（扩展位；旧产物 = 0 → false）
    if (header_->isEnableCurvaturePrediction()) flagBits2 |= 0x02;
    if (std::getenv("PMSF_PPMC_VARINT_DEG") != nullptr) flagBits2 |= 0x04;
    packU8(payload, flagBits2);
    packI32LE(payload, codecVersion_);
    packI32LE(payload, static_cast<std::int32_t>(blobLayout_));
    if (canonicalSourceLayout_ != 0) {
        packI32LE(payload, canonicalSourceLayout_);
    }
    // 线格式字段名为 origTotalVertices/origTotalFaces（解码端靠它按原始规模
    // 注册内存池）。历史上这里误填基网数（totalVertices_），导致解码端
    // 只能回落到内置默认小池。管道在序列化前通过 setOriginalMeshCounts
    // 注入真实输入规模；未注入时保持旧行为（写基网数），字节宽度不变。
    packI32LE(payload, originalInputVertices_ > 0 ? originalInputVertices_
                                                  : totalVertices_);
    packI32LE(payload, originalInputFaces_ > 0 ? originalInputFaces_
                                               : totalFaces_);
    if (hasQuant && mesh()) {
        packI32LE(payload, mesh()->i_nbQuantBits);
        packF32LE(payload, mesh()->bboxMin.x());
        packF32LE(payload, mesh()->bboxMin.y());
        packF32LE(payload, mesh()->bboxMin.z());
        packF32LE(payload, mesh()->f_quantStep);
    }
    if (hasScheme) {
        ::pmsf::appendPaperCompactStorageScheme(payload, scheme->info().name,
                                        scheme->info().version);
    }

    // --- base mesh 段 ---
    //
    // [PHASE61 诊断，默认关闭] `PMSF_BASE_MESH_DIAG=1` 时逐段记账 base mesh 段
    // 的字节数（组头 / 种子 / 顶点坐标 / 面索引）。
    //
    // 为什么要它：论文运行点是 `PMSF_V2_COMPACT=1`（Tier 2），此时基网**不再单独
    // 落盘**（跳过 `base_mesh.parquet`），它整块被 zlib 装进 `_compact.bin` ⇒
    // 六列表里的 Base 列在**落盘面**上无法分离。本计数器按实际写入字节记账，
    // 只读数、不改变字节（下面的 `check` 字段自证四段之和 == payload 全长）。
    const bool baseDiag = envOn("PMSF_BASE_MESH_DIAG");
    packI32LE(payload, static_cast<std::int32_t>(groupBaseMeshData_.size()));
    // preamble = magic + 版本 + flags + metadata 段 + 组数；余下四段逐组累加。
    const std::size_t diagPreambleBytes = payload.size();
    std::size_t diagGroupHdrBytes = 0, diagSeedBytes = 0, diagCoordBytes = 0, diagFaceBytes = 0;
    for (const auto& g : groupBaseMeshData_) {
        const std::size_t m0 = payload.size();
        packI32LE(payload, g.groupId);
        packI32LE(payload, g.numVertices);
        packI32LE(payload, g.numFaces);
        const std::size_t m1 = payload.size();
        if (baseDiag)
            diagGroupHdrBytes += m1 - m0;
        std::size_t m2 = m1;  // 种子段末尾
        std::size_t m3 = m1;  // 坐标段末尾
        if (nativeEdgeBreakerBase) {
            packI32LE(payload, g.seedV0QX); packI32LE(payload, g.seedV0QY); packI32LE(payload, g.seedV0QZ);
            packI32LE(payload, g.seedV1QX); packI32LE(payload, g.seedV1QY); packI32LE(payload, g.seedV1QZ);
            packI32LE(payload, static_cast<std::int32_t>(nativeEbBlob.size()));
            payload.insert(payload.end(), nativeEbBlob.begin(), nativeEbBlob.end());
            if (baseDiag)
                diagSeedBytes += payload.size() - m1;
            continue;
        }
        if (edgeBreakerBase) {
            packI32LE(payload, g.seedV0QX); packI32LE(payload, g.seedV0QY); packI32LE(payload, g.seedV0QZ);
            packI32LE(payload, g.seedV1QX); packI32LE(payload, g.seedV1QY); packI32LE(payload, g.seedV1QZ);
            const auto& beb = edgeBreakerBlobs[static_cast<size_t>(&g - groupBaseMeshData_.data())];
            packI32LE(payload, static_cast<std::int32_t>(beb.size()));
            payload.insert(payload.end(), beb.begin(), beb.end());
            // EB 路径把「种子 + EB blob」一并记入种子段（efoss 走不到这里：
            // 该分支要求 PMSF_PCATM_BASE_EDGEBREAKER 且非分区）。
            if (baseDiag)
                diagSeedBytes += payload.size() - m1;
            continue;
        }
        if (hasQuant) {
            if (varintBase) {
                packVarU32(payload, static_cast<std::uint32_t>(g.seedV0QX));
                packVarU32(payload, static_cast<std::uint32_t>(g.seedV0QY));
                packVarU32(payload, static_cast<std::uint32_t>(g.seedV0QZ));
                packVarU32(payload, static_cast<std::uint32_t>(g.seedV1QX));
                packVarU32(payload, static_cast<std::uint32_t>(g.seedV1QY));
                packVarU32(payload, static_cast<std::uint32_t>(g.seedV1QZ));
            } else if (compactU16) {
                packU16LE(payload, static_cast<std::uint16_t>(g.seedV0QX));
                packU16LE(payload, static_cast<std::uint16_t>(g.seedV0QY));
                packU16LE(payload, static_cast<std::uint16_t>(g.seedV0QZ));
                packU16LE(payload, static_cast<std::uint16_t>(g.seedV1QX));
                packU16LE(payload, static_cast<std::uint16_t>(g.seedV1QY));
                packU16LE(payload, static_cast<std::uint16_t>(g.seedV1QZ));
            } else {
                packI32LE(payload, g.seedV0QX); packI32LE(payload, g.seedV0QY); packI32LE(payload, g.seedV0QZ);
                packI32LE(payload, g.seedV1QX); packI32LE(payload, g.seedV1QY); packI32LE(payload, g.seedV1QZ);
            }
            m2 = payload.size();
            if (splitResidualBase) {
                int prevQX = g.seedV1QX;
                for (int q : g.vertexQX) {
                    packVarI32(payload, q - prevQX);
                    prevQX = q;
                }
                int prevQY = g.seedV1QY;
                for (int q : g.vertexQY) {
                    packVarI32(payload, q - prevQY);
                    prevQY = q;
                }
                int prevQZ = g.seedV1QZ;
                for (int q : g.vertexQZ) {
                    packVarI32(payload, q - prevQZ);
                    prevQZ = q;
                }
            } else {
                int prevQX = g.seedV1QX;
                int prevQY = g.seedV1QY;
                int prevQZ = g.seedV1QZ;
                for (int i = 0; i < static_cast<int>(g.vertexQX.size()); ++i) {
                    if (varintBase) {
                        packVarI32(payload, g.vertexQX[i] - prevQX);
                        packVarI32(payload, g.vertexQY[i] - prevQY);
                        packVarI32(payload, g.vertexQZ[i] - prevQZ);
                        prevQX = g.vertexQX[i];
                        prevQY = g.vertexQY[i];
                        prevQZ = g.vertexQZ[i];
                    } else if (compactU16) {
                        packU16LE(payload, static_cast<std::uint16_t>(g.vertexQX[i]));
                        packU16LE(payload, static_cast<std::uint16_t>(g.vertexQY[i]));
                        packU16LE(payload, static_cast<std::uint16_t>(g.vertexQZ[i]));
                    } else {
                        packI32LE(payload, g.vertexQX[i]);
                        packI32LE(payload, g.vertexQY[i]);
                        packI32LE(payload, g.vertexQZ[i]);
                    }
                }
            }
        } else {
            packF32LE(payload, g.seedV0X); packF32LE(payload, g.seedV0Y); packF32LE(payload, g.seedV0Z);
            packF32LE(payload, g.seedV1X); packF32LE(payload, g.seedV1Y); packF32LE(payload, g.seedV1Z);
            m2 = payload.size();
            for (int i = 0; i < static_cast<int>(g.vertexX.size()); ++i) {
                packF32LE(payload, g.vertexX[i]);
                packF32LE(payload, g.vertexY[i]);
                packF32LE(payload, g.vertexZ[i]);
            }
        }
        m3 = payload.size();
        const bool varintFaceDegrees = std::getenv("PMSF_PPMC_VARINT_DEG") != nullptr;
        if (!triFaces) {
            if (varintFaceDegrees) {
                // PPMC 专用紧凑基网格：face degree 由 int32 改为 varint（小 deg 1 字节），
                // 由 flagBits2 bit2 标识；其它算法不设置该 env，字节不变。
                packVarU32(payload, static_cast<std::uint32_t>(g.faceDegrees.size()));
                for (int d : g.faceDegrees) packVarI32(payload, d);
            } else {
                packI32LE(payload, static_cast<std::int32_t>(g.faceDegrees.size()));
                for (int d : g.faceDegrees) packI32LE(payload, d);
            }
        }
        packI32LE(payload, static_cast<std::int32_t>(g.faceVertices.size()));
        if (splitResidualBase && g.faceVertices.size() == static_cast<std::size_t>(g.numFaces) * 3u) {
            for (int corner = 0; corner < 3; ++corner) {
                int prevFaceVertex = 0;
                for (int f = 0; f < g.numFaces; ++f) {
                    const int fv = g.faceVertices[static_cast<std::size_t>(3 * f + corner)];
                    packVarI32(payload, fv - prevFaceVertex);
                    prevFaceVertex = fv;
                }
            }
        } else {
            int prevFaceVertex = 0;
            for (int fv : g.faceVertices) {
                if (varintBase) {
                    packVarI32(payload, fv - prevFaceVertex);
                    prevFaceVertex = fv;
                } else if (compactU16) {
                    packU16LE(payload, static_cast<std::uint16_t>(fv));
                } else {
                    packI32LE(payload, fv);
                }
            }
        }
        if (baseDiag) {
            diagSeedBytes += m2 - m1;
            diagCoordBytes += m3 - m2;
            diagFaceBytes += payload.size() - m3;
        }
    }

    if (baseDiag) {
        const std::size_t diagSum = diagPreambleBytes + diagGroupHdrBytes + diagSeedBytes + diagCoordBytes + diagFaceBytes;
        std::cout << "[BASE_MESH_DIAG] groups=" << groupBaseMeshData_.size() << " payload=" << payload.size() << " preamble=" << diagPreambleBytes
                  << " group_hdr=" << diagGroupHdrBytes << " seeds=" << diagSeedBytes << " coords=" << diagCoordBytes << " faces=" << diagFaceBytes
                  << " sum=" << diagSum << " check=" << (diagSum == payload.size() ? 1 : 0) << std::endl;
    }

    // ---- 2. 容器字节压缩 ----
    //
    // [PHASE61 物品1① 容器级压缩后端，**默认关闭**]
    // 走 `pmsf::compressCompactBinPayload`：未设 `PMSF_COMPACT_CODEC` 时仍是
    // 原来的 zlib-9 分支（写出字节逐字节不变）；设成 brotli 时换 brotli-11，
    // 并靠 `rawLen` 最高位把「扩展后端」写进线格式，解码端自动识别。
    // 只换容器压缩器，payload 语义、量化位深、drop 一律不动。
    const std::uint32_t rawLen = static_cast<std::uint32_t>(payload.size());
    if (!::pmsf::compressCompactBinPayload(payload, compactBin)) {
        std::cerr << "[Columnar] writeV2CompactBin: 容器压缩失败" << std::endl;
        return false;
    }
    const std::uint32_t compLen = static_cast<std::uint32_t>(compactBin.size() - 8u);
    const std::uint8_t codec = ::pmsf::requestedCompactBinCodec();

    std::cout << "[Columnar] compact bin payload:"
              << " raw=" << rawLen << " B"
              << " " << (codec == ::pmsf::kCompactBinCodecBrotli ? "brotli" : "zlib") << "=" << compLen << " B"
              << " base=" << (edgeBreakerBase ? "edgebreaker" : "varint") << " (" << (100.0 * compLen / rawLen) << "%)" << std::endl;
    return true;
}

bool ColumnarSerializeOperator::writeV2CompactBin(const std::string& path) {
    std::vector<std::uint8_t> compactBin;
    if (!buildV2CompactBinPayload(compactBin)) {
        return false;
    }
    std::ofstream ofs(path, std::ios::binary);
    if (!ofs) {
        std::cerr << "[Columnar] writeV2CompactBin: 打开失败 " << path << std::endl;
        return false;
    }
    ofs.write(reinterpret_cast<const char*>(compactBin.data()),
              static_cast<std::streamsize>(compactBin.size()));
    ofs.close();
    std::cout << "[Columnar] compact bin: " << path
              << " bytes=" << compactBin.size() << " B" << std::endl;
    return true;
}
