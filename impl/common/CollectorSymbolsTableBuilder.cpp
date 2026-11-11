#include "CollectorSymbolsTableBuilder.h"
#include "common/PaperLayoutAliases.h"
#include "compress/SymbolCollectOperator.h"
#include "compress/BasicSymbolCollectOperator.h"
#include "compress/OptimizedSymbolCollectOperator.h"
#include "compress/SegmentationSymbolCollectOperator.h"
#include "PPMC/compress/PPMCSymbolCollectOperator.h"
#include "RD/decompress/RDTagCanonicalReassembler.h"
#include "PCLTTM/decompress/PCLTTMTagCanonicalReassembler.h"
#include "PCATM/decompress/PCATMColorCanonicalReassembler.h"
#include "decompress/PatchColorCanonicalReassembler.h"
#include "PCATM/decompress/PCATMPureCanonicalReassembler.h"
#include "storage/PMSFSymbolStorageAdapter.h"
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂
#include "Mesh.h"
#include "Point.h"

#include <arrow/api.h>
#include <arrow/builder.h>
#include <cstdint>
#include <limits>
#include <vector>

namespace pmsf {

std::shared_ptr<arrow::Schema> buildCollectorSymbolsSchema(bool enableQuantization) {
    // 量化模式下只存量化坐标（int16），非量化模式只存浮点坐标
    auto qCoordType = arrow::int16();
    std::vector<std::shared_ptr<arrow::Field>> fields = {
        arrow::field(pmsf::kParquetGroupIdColumn,            arrow::int32(),    false),
        arrow::field(pmsf::kParquetRoundIdColumn,            arrow::int32(),    false),
        // 符号统计
        arrow::field(pmsf::kParquetNumFacetSymbolsColumn,   arrow::int32(),    false),
        arrow::field(pmsf::kParquetNumEdgeSymbolsColumn,    arrow::int32(),    false),
        arrow::field(pmsf::kParquetNumVertexSymbolsColumn,  arrow::int32(),    true),
        arrow::field(pmsf::kParquetNumInsertedPointsColumn, arrow::int32(),    false),
        arrow::field(pmsf::kParquetNumVertexPointsColumn,   arrow::int32(),    true),
        // 符号数据
        arrow::field(pmsf::kParquetFacetSymbolsColumn,       arrow::list(arrow::boolean()), false),
        arrow::field(pmsf::kParquetEdgeSymbolsColumn,        arrow::list(arrow::boolean()), false),
        arrow::field(pmsf::kParquetVertexSymbolsColumn,      arrow::list(arrow::boolean()), true),
    };
    if (enableQuantization) {
        // 量化点坐标（int16 列表）
        fields.push_back(arrow::field(pmsf::kParquetPointQXColumn, arrow::list(qCoordType), true));
        fields.push_back(arrow::field(pmsf::kParquetPointQYColumn, arrow::list(qCoordType), true));
        fields.push_back(arrow::field(pmsf::kParquetPointQZColumn, arrow::list(qCoordType), true));
        // 顶点附加点流（QEM/Adjustment）
        fields.push_back(arrow::field(pmsf::kParquetVertexPointQXColumn, arrow::list(qCoordType), true));
        fields.push_back(arrow::field(pmsf::kParquetVertexPointQYColumn, arrow::list(qCoordType), true));
        fields.push_back(arrow::field(pmsf::kParquetVertexPointQZColumn, arrow::list(qCoordType), true));
    } else {
        // 浮点点坐标
        fields.push_back(arrow::field(pmsf::kParquetPointXColumn, arrow::list(arrow::float32()), true));
        fields.push_back(arrow::field(pmsf::kParquetPointYColumn, arrow::list(arrow::float32()), true));
        fields.push_back(arrow::field(pmsf::kParquetPointZColumn, arrow::list(arrow::float32()), true));
        // 顶点附加点流（QEM/Adjustment）
        fields.push_back(arrow::field(pmsf::kParquetVertexPointXColumn, arrow::list(arrow::float32()), true));
        fields.push_back(arrow::field(pmsf::kParquetVertexPointYColumn, arrow::list(arrow::float32()), true));
        fields.push_back(arrow::field(pmsf::kParquetVertexPointZColumn, arrow::list(arrow::float32()), true));
    }
    return arrow::schema(fields);
}

std::shared_ptr<arrow::Schema> buildCollectorSymbolsSchema() {
    return buildCollectorSymbolsSchema(false);
}

namespace {

inline int16_t clampToInt16(int32_t v) {
    if (v > std::numeric_limits<int16_t>::max()) return std::numeric_limits<int16_t>::max();
    if (v < std::numeric_limits<int16_t>::min()) return std::numeric_limits<int16_t>::min();
    return static_cast<int16_t>(v);
}

inline int32_t clampQuantizedCoord(int32_t v, int quantBits) {
    if (v < 0) return 0;
    if (quantBits > 0 && quantBits < 31) {
        const int32_t maxV = (1 << quantBits) - 1;
        if (v > maxV) return maxV;
    }
    return v;
}

} // anonymous namespace

std::shared_ptr<arrow::Table> buildCollectorSymbolsTable(const CollectorSymbolsTableContext& ctx) {
    auto schema = buildCollectorSymbolsSchema(ctx.enableQuantization);

    bool isSegmented = ctx.collector->header_.isEnableSegmentation();
    int groupNumber = isSegmented ? ctx.collector->header_.getGroupNumber() : 1;
    int numRounds = ctx.collector->header_.getRound();
    bool enableAdj = ctx.collector->header_.isEnableAdjustment();
    int quantBits = ctx.collector->getMesh() ? ctx.collector->getMesh()->i_nbQuantBits : 0;

    auto* segCollector = dynamic_cast<SegmentationSymbolCollectOperator*>(ctx.collector);
    auto* optCollector = dynamic_cast<OptimizedSymbolCollectOperator*>(ctx.collector);
    auto* basicCollector = dynamic_cast<BasicSymbolCollectOperator*>(ctx.collector);
    auto* ppmcCollector = dynamic_cast<PPMCSymbolCollectOperator*>(ctx.collector);

    // 构建所有行的 builders
    arrow::Int32Builder groupIdB, roundIdB, numFacetB, numEdgeB, numVertexB, numPointsB, numVertexPointsB;

    auto fsVb = std::make_shared<arrow::BooleanBuilder>();
    arrow::ListBuilder fsLb(arrow::default_memory_pool(), fsVb);
    auto esVb = std::make_shared<arrow::BooleanBuilder>();
    arrow::ListBuilder esLb(arrow::default_memory_pool(), esVb);
    auto vsVb = std::make_shared<arrow::BooleanBuilder>();
    arrow::ListBuilder vsLb(arrow::default_memory_pool(), vsVb);

    auto pxVb = std::make_shared<arrow::FloatBuilder>();
    arrow::ListBuilder pxLb(arrow::default_memory_pool(), pxVb);
    auto pyVb = std::make_shared<arrow::FloatBuilder>();
    arrow::ListBuilder pyLb(arrow::default_memory_pool(), pyVb);
    auto pzVb = std::make_shared<arrow::FloatBuilder>();
    arrow::ListBuilder pzLb(arrow::default_memory_pool(), pzVb);

    auto vpxVb = std::make_shared<arrow::FloatBuilder>();
    arrow::ListBuilder vpxLb(arrow::default_memory_pool(), vpxVb);
    auto vpyVb = std::make_shared<arrow::FloatBuilder>();
    arrow::ListBuilder vpyLb(arrow::default_memory_pool(), vpyVb);
    auto vpzVb = std::make_shared<arrow::FloatBuilder>();
    arrow::ListBuilder vpzLb(arrow::default_memory_pool(), vpzVb);

    auto pqxVb = std::make_shared<arrow::Int16Builder>();
    arrow::ListBuilder pqxLb(arrow::default_memory_pool(), pqxVb);
    auto pqyVb = std::make_shared<arrow::Int16Builder>();
    arrow::ListBuilder pqyLb(arrow::default_memory_pool(), pqyVb);
    auto pqzVb = std::make_shared<arrow::Int16Builder>();
    arrow::ListBuilder pqzLb(arrow::default_memory_pool(), pqzVb);

    auto vpqxVb = std::make_shared<arrow::Int16Builder>();
    arrow::ListBuilder vpqxLb(arrow::default_memory_pool(), vpqxVb);
    auto vpqyVb = std::make_shared<arrow::Int16Builder>();
    arrow::ListBuilder vpqyLb(arrow::default_memory_pool(), vpqyVb);
    auto vpqzVb = std::make_shared<arrow::Int16Builder>();
    arrow::ListBuilder vpqzLb(arrow::default_memory_pool(), vpqzVb);

    int totalRows = 0;

    if (isSegmented && segCollector) {
        // ========== 分区模式：从 SegmentationSymbolCollectOperator 提取 ==========
        auto& facetQueues = segCollector->getSegmentFacetSymbolQueues();
        auto& edgeQueues = segCollector->getSegmentEdgeSymbolQueues();
        auto& pointQueues = segCollector->getSegmentPointQueues();

        for (int gid = 0; gid < groupNumber && gid < (int)facetQueues.size(); gid++) {
            int rounds = facetQueues[gid].size();
            for (int r = 0; r < rounds; r++) {
                (void)groupIdB.Append(gid);
                (void)roundIdB.Append(r);

                auto& facetSym = facetQueues[gid][r];
                auto& edgeSym = (gid < (int)edgeQueues.size() && r < (int)edgeQueues[gid].size())
                    ? edgeQueues[gid][r] : facetQueues[gid][r]; // fallback

                (void)numFacetB.Append(facetSym.size());
                (void)numEdgeB.Append(edgeSym.size());
                (void)numVertexB.AppendNull();
                (void)numVertexPointsB.AppendNull();

                // facet_symbols
                (void)fsLb.Append();
                for (char s : facetSym) (void)fsVb->Append(s != 0);

                // edge_symbols
                (void)esLb.Append();
                if (gid < (int)edgeQueues.size() && r < (int)edgeQueues[gid].size()) {
                    for (char s : edgeQueues[gid][r]) (void)esVb->Append(s != 0);
                }

                // vertex_symbols (分区模式暂不支持)
                (void)vsLb.AppendNull();

                // 点坐标
                if (gid < (int)pointQueues.size() && r < (int)pointQueues[gid].size()) {
                    auto& pts = pointQueues[gid][r];
                    (void)numPointsB.Append(pts.size());

                    if (!ctx.enableQuantization) {
                        (void)pxLb.Append(); for (auto& p : pts) (void)pxVb->Append((float)p.x());
                        (void)pyLb.Append(); for (auto& p : pts) (void)pyVb->Append((float)p.y());
                        (void)pzLb.Append(); for (auto& p : pts) (void)pzVb->Append((float)p.z());
                    } else {
                        (void)pqxLb.Append();
                        for (auto& p : pts) {
                            MCGAL::PointInt pi = ctx.collector->getMesh()->floatPosToInt(p);
                            (void)pqxVb->Append(clampToInt16(clampQuantizedCoord(pi[0], quantBits)));
                        }
                        (void)pqyLb.Append();
                        for (auto& p : pts) {
                            MCGAL::PointInt pi = ctx.collector->getMesh()->floatPosToInt(p);
                            (void)pqyVb->Append(clampToInt16(clampQuantizedCoord(pi[1], quantBits)));
                        }
                        (void)pqzLb.Append();
                        for (auto& p : pts) {
                            MCGAL::PointInt pi = ctx.collector->getMesh()->floatPosToInt(p);
                            (void)pqzVb->Append(clampToInt16(clampQuantizedCoord(pi[2], quantBits)));
                        }
                    }
                } else {
                    (void)numPointsB.Append(0);
                    if (!ctx.enableQuantization) {
                        (void)pxLb.AppendNull(); (void)pyLb.AppendNull(); (void)pzLb.AppendNull();
                    } else {
                        (void)pqxLb.AppendNull(); (void)pqyLb.AppendNull(); (void)pqzLb.AppendNull();
                    }
                }

                // 分区模式暂不输出顶点附加点流
                if (!ctx.enableQuantization) {
                    (void)vpxLb.AppendNull(); (void)vpyLb.AppendNull(); (void)vpzLb.AppendNull();
                } else {
                    (void)vpqxLb.AppendNull(); (void)vpqyLb.AppendNull(); (void)vpqzLb.AppendNull();
                }

                totalRows++;
            }
        }
    } else if (optCollector) {
        // ========== 非分区模式：从 OptimizedSymbolCollectOperator 提取 ==========
        auto& facetQueues = optCollector->getFacetSymbolQueues();
        auto& edgeQueues = optCollector->getEdgeSymbolQueues();
        auto& vertexQueues = optCollector->getVertexSymbolQueues();
        auto& pointQueues = optCollector->getPointQueues();
        auto& vertexPointQueues = optCollector->getVertexPointQueues();

        int rounds = facetQueues.size();
        for (int r = rounds - 1; r >= 0; r--) {
            (void)groupIdB.Append(0);
            (void)roundIdB.Append(rounds - r - 1);

            (void)numFacetB.Append(facetQueues[r].size());
            (void)numEdgeB.Append(r < (int)edgeQueues.size() ? edgeQueues[r].size() : 0);

            // facet_symbols
            (void)fsLb.Append();
            for (char s : facetQueues[r]) (void)fsVb->Append(s != 0);

            // edge_symbols
            (void)esLb.Append();
            if (r < (int)edgeQueues.size()) {
                for (char s : edgeQueues[r]) (void)esVb->Append(s != 0);
            }

            // vertex_symbols
            if (enableAdj && r < (int)vertexQueues.size() && !vertexQueues[r].empty()) {
                (void)numVertexB.Append(vertexQueues[r].size());
                (void)vsLb.Append();
                for (char s : vertexQueues[r]) (void)vsVb->Append(s != 0);
            } else {
                (void)numVertexB.AppendNull();
                (void)vsLb.AppendNull();
            }

            // 顶点附加点流（用于 adjustment/QEM）
            if (enableAdj && r < (int)vertexPointQueues.size()) {
                auto& vpts = vertexPointQueues[r];
                (void)numVertexPointsB.Append(vpts.size());
                if (!ctx.enableQuantization) {
                    (void)vpxLb.Append(); for (auto& p : vpts) (void)vpxVb->Append((float)p.x());
                    (void)vpyLb.Append(); for (auto& p : vpts) (void)vpyVb->Append((float)p.y());
                    (void)vpzLb.Append(); for (auto& p : vpts) (void)vpzVb->Append((float)p.z());
                } else {
                    (void)vpqxLb.Append();
                    for (auto& p : vpts) {
                        MCGAL::PointInt pi = ctx.collector->getMesh()->floatPosToInt(p);
                        (void)vpqxVb->Append(clampToInt16(clampQuantizedCoord(pi[0], quantBits)));
                    }
                    (void)vpqyLb.Append();
                    for (auto& p : vpts) {
                        MCGAL::PointInt pi = ctx.collector->getMesh()->floatPosToInt(p);
                        (void)vpqyVb->Append(clampToInt16(clampQuantizedCoord(pi[1], quantBits)));
                    }
                    (void)vpqzLb.Append();
                    for (auto& p : vpts) {
                        MCGAL::PointInt pi = ctx.collector->getMesh()->floatPosToInt(p);
                        (void)vpqzVb->Append(clampToInt16(clampQuantizedCoord(pi[2], quantBits)));
                    }
                }
            } else {
                (void)numVertexPointsB.AppendNull();
                if (!ctx.enableQuantization) {
                    (void)vpxLb.AppendNull(); (void)vpyLb.AppendNull(); (void)vpzLb.AppendNull();
                } else {
                    (void)vpqxLb.AppendNull(); (void)vpqyLb.AppendNull(); (void)vpqzLb.AppendNull();
                }
            }

            // 点坐标
            if (r < (int)pointQueues.size()) {
                auto& pts = pointQueues[r];
                (void)numPointsB.Append(pts.size());

                if (!ctx.enableQuantization) {
                    (void)pxLb.Append(); for (auto& p : pts) (void)pxVb->Append((float)p.x());
                    (void)pyLb.Append(); for (auto& p : pts) (void)pyVb->Append((float)p.y());
                    (void)pzLb.Append(); for (auto& p : pts) (void)pzVb->Append((float)p.z());
                } else {
                        (void)pqxLb.Append();
                        for (auto& p : pts) {
                            MCGAL::PointInt pi = ctx.collector->getMesh()->floatPosToInt(p);
                            (void)pqxVb->Append(clampToInt16(clampQuantizedCoord(pi[0], quantBits)));
                        }
                        (void)pqyLb.Append();
                        for (auto& p : pts) {
                            MCGAL::PointInt pi = ctx.collector->getMesh()->floatPosToInt(p);
                            (void)pqyVb->Append(clampToInt16(clampQuantizedCoord(pi[1], quantBits)));
                        }
                        (void)pqzLb.Append();
                        for (auto& p : pts) {
                            MCGAL::PointInt pi = ctx.collector->getMesh()->floatPosToInt(p);
                            (void)pqzVb->Append(clampToInt16(clampQuantizedCoord(pi[2], quantBits)));
                        }

                }
            } else {
                (void)numPointsB.Append(0);
                if (!ctx.enableQuantization) {
                    (void)pxLb.AppendNull(); (void)pyLb.AppendNull(); (void)pzLb.AppendNull();
                } else {
                    (void)pqxLb.AppendNull(); (void)pqyLb.AppendNull(); (void)pqzLb.AppendNull();
                }
            }
            totalRows++;
        }
    } else if (basicCollector) {
        // ========== QEM/Basic 模式：从 BasicSymbolCollectOperator 提取 ==========
        auto& facetQueues = basicCollector->getFacetSymbolQueues();
        auto& edgeQueues = basicCollector->getEdgeSymbolQueues();
        auto& vertexQueues = basicCollector->getVertexSymbolQueues();
        auto& pointQueues = basicCollector->getPointQueues();
        auto& vertexPointQueues = basicCollector->getVertexPointQueues();

        int rounds = facetQueues.size();
        for (int r = rounds - 1; r >= 0; r--) {
            (void)groupIdB.Append(0);
            (void)roundIdB.Append(rounds - r - 1);

            (void)numFacetB.Append(facetQueues[r].size());
            (void)numEdgeB.Append(r < (int)edgeQueues.size() ? edgeQueues[r].size() : 0);

            // facet_symbols
            (void)fsLb.Append();
            for (char s : facetQueues[r]) (void)fsVb->Append(s != 0);

            // edge_symbols
            (void)esLb.Append();
            if (r < (int)edgeQueues.size()) {
                for (char s : edgeQueues[r]) (void)esVb->Append(s != 0);
            }

            // vertex_symbols
            if (enableAdj && r < (int)vertexQueues.size()) {
                (void)numVertexB.Append(vertexQueues[r].size());
                (void)vsLb.Append();
                for (char s : vertexQueues[r]) (void)vsVb->Append(s != 0);
            } else {
                (void)numVertexB.AppendNull();
                (void)vsLb.AppendNull();
            }

            // facet 点流
            if (r < (int)pointQueues.size()) {
                auto& pts = pointQueues[r];
                (void)numPointsB.Append(pts.size());
                if (!ctx.enableQuantization) {
                    (void)pxLb.Append(); for (auto& p : pts) (void)pxVb->Append((float)p.x());
                    (void)pyLb.Append(); for (auto& p : pts) (void)pyVb->Append((float)p.y());
                    (void)pzLb.Append(); for (auto& p : pts) (void)pzVb->Append((float)p.z());
                } else {
                    (void)pqxLb.Append();
                    for (auto& p : pts) {
                        MCGAL::PointInt pi = ctx.collector->getMesh()->floatPosToInt(p);
                        (void)pqxVb->Append(clampToInt16(clampQuantizedCoord(pi[0], quantBits)));
                    }
                    (void)pqyLb.Append();
                    for (auto& p : pts) {
                        MCGAL::PointInt pi = ctx.collector->getMesh()->floatPosToInt(p);
                        (void)pqyVb->Append(clampToInt16(clampQuantizedCoord(pi[1], quantBits)));
                    }
                    (void)pqzLb.Append();
                    for (auto& p : pts) {
                        MCGAL::PointInt pi = ctx.collector->getMesh()->floatPosToInt(p);
                        (void)pqzVb->Append(clampToInt16(clampQuantizedCoord(pi[2], quantBits)));
                    }
                }
            } else {
                (void)numPointsB.Append(0);
                if (!ctx.enableQuantization) {
                    (void)pxLb.AppendNull(); (void)pyLb.AppendNull(); (void)pzLb.AppendNull();
                } else {
                    (void)pqxLb.AppendNull(); (void)pqyLb.AppendNull(); (void)pqzLb.AppendNull();
                }
            }

            // 顶点附加点流（QEM 必需）
            if (enableAdj && r < (int)vertexPointQueues.size()) {
                auto& vpts = vertexPointQueues[r];
                (void)numVertexPointsB.Append(vpts.size());
                if (!ctx.enableQuantization) {
                    (void)vpxLb.Append(); for (auto& p : vpts) (void)vpxVb->Append((float)p.x());
                    (void)vpyLb.Append(); for (auto& p : vpts) (void)vpyVb->Append((float)p.y());
                    (void)vpzLb.Append(); for (auto& p : vpts) (void)vpzVb->Append((float)p.z());
                } else {
                    (void)vpqxLb.Append();
                    for (auto& p : vpts) {
                        MCGAL::PointInt pi = ctx.collector->getMesh()->floatPosToInt(p);
                        (void)vpqxVb->Append(clampToInt16(clampQuantizedCoord(pi[0], quantBits)));
                    }
                    (void)vpqyLb.Append();
                    for (auto& p : vpts) {
                        MCGAL::PointInt pi = ctx.collector->getMesh()->floatPosToInt(p);
                        (void)vpqyVb->Append(clampToInt16(clampQuantizedCoord(pi[1], quantBits)));
                    }
                    (void)vpqzLb.Append();
                    for (auto& p : vpts) {
                        MCGAL::PointInt pi = ctx.collector->getMesh()->floatPosToInt(p);
                        (void)vpqzVb->Append(clampToInt16(clampQuantizedCoord(pi[2], quantBits)));
                    }
                }
            } else {
                (void)numVertexPointsB.AppendNull();
                if (!ctx.enableQuantization) {
                    (void)vpxLb.AppendNull(); (void)vpyLb.AppendNull(); (void)vpzLb.AppendNull();
                } else {
                    (void)vpqxLb.AppendNull(); (void)vpqyLb.AppendNull(); (void)vpqzLb.AppendNull();
                }
            }

            totalRows++;
        }
    } else if (ppmcCollector) {
        // ========== PPMC 模式：从 PPMCSymbolCollectOperator 提取（不使用 range coder） ==========
        auto& faceSym = ppmcCollector->getConnectFaceSym();
        auto& edgeSym = ppmcCollector->getConnectEdgeSym();
        auto& geomSym = ppmcCollector->getGeometrySym();
        int decimations = ppmcCollector->getDecimationCount();

        // PPMC 的 symbol 顺序是从后往前导出（与 exportToBuffer 一致）
        for (int r = decimations - 1; r >= 0; r--) {
            (void)groupIdB.Append(0);
            (void)roundIdB.Append(decimations - r - 1);

            // facet_symbols: connectFaceSym[r][i].first (0=unsplit, 1=split)
            int numFacet = (r < (int)faceSym.size()) ? faceSym[r].size() : 0;
            (void)numFacetB.Append(numFacet);
            (void)fsLb.Append();
            if (r < (int)faceSym.size()) {
                for (auto& sym : faceSym[r]) {
                    (void)fsVb->Append(sym.first != 0);
                }
            }

            // edge_symbols: connectEdgeSym[r][i].first (0=original, 1=inserted)
            int numEdge = (r < (int)edgeSym.size()) ? edgeSym[r].size() : 0;
            (void)numEdgeB.Append(numEdge);
            (void)esLb.Append();
            if (r < (int)edgeSym.size()) {
                for (auto& sym : edgeSym[r]) {
                    (void)esVb->Append(sym.first != 0);
                }
            }

            // vertex_symbols: PPMC 不使用
            (void)numVertexB.AppendNull();
            (void)numVertexPointsB.AppendNull();
            (void)vsLb.AppendNull();

            // 点坐标: geometrySym[r] 中的 PointInt
            int numPoints = (r < (int)geomSym.size()) ? geomSym[r].size() : 0;
            (void)numPointsB.Append(numPoints);

            if (numPoints > 0) {
                // ⚠️ P0 对齐：PPMC 的 geometrySym[r] 现在存的是 **Frenet 旋转后的残差**
                //   （residual = Q(removedPos) - Q(barycenter(h))，再经 Frenet 投影），
                //   分量可正可负，量级一般 ±2^14 以内。**不能**走 clampQuantizedCoord
                //   把 [<0] 截断为 0，也**不能**用 (pi+0.5)*step + bboxMin 反量化
                //   公式（这把残差错误当作绝对量化坐标）。
                //   - 量化模式：直接以 int16 存储残差（保留正负号）。
                //   - 非量化模式：把残差三个分量当作浮点存储，解码端 importFromParquet
                //     的反量化公式需要识别 PPMC 模式才不会对残差再做一次错误的取整。
                //     当前 PPMC 仅用量化模式，非量化路径仅作降级保留。
                if (!ctx.enableQuantization) {
                    // 非量化模式：把残差当作浮点直存（语义=残差，不是绝对坐标）
                    (void)pxLb.Append();
                    (void)pyLb.Append();
                    (void)pzLb.Append();
                    for (auto& pi : geomSym[r]) {
                        (void)pxVb->Append(static_cast<float>(pi.x()));
                        (void)pyVb->Append(static_cast<float>(pi.y()));
                        (void)pzVb->Append(static_cast<float>(pi.z()));
                    }
                } else {
                    // 量化模式：直接存储 PointInt 残差为 int16（保留符号）
                    (void)pqxLb.Append();
                    (void)pqyLb.Append();
                    (void)pqzLb.Append();
                    for (auto& pi : geomSym[r]) {
                        (void)pqxVb->Append(clampToInt16(pi.x()));
                        (void)pqyVb->Append(clampToInt16(pi.y()));
                        (void)pqzVb->Append(clampToInt16(pi.z()));
                    }
                }
            } else {
                if (!ctx.enableQuantization) {
                    (void)pxLb.AppendNull(); (void)pyLb.AppendNull(); (void)pzLb.AppendNull();
                } else {
                    (void)pqxLb.AppendNull(); (void)pqyLb.AppendNull(); (void)pqzLb.AppendNull();
                }
            }

            // PPMC 不输出顶点附加点流
            if (!ctx.enableQuantization) {
                (void)vpxLb.AppendNull(); (void)vpyLb.AppendNull(); (void)vpzLb.AppendNull();
            } else {
                (void)vpqxLb.AppendNull(); (void)vpqyLb.AppendNull(); (void)vpqzLb.AppendNull();
            }

            totalRows++;
        }
    }

    // Finish all builders
    std::shared_ptr<arrow::Array> groupIdA, roundIdA, numFacetA, numEdgeA, numVertexA, numPointsA, numVertexPointsA;
    std::shared_ptr<arrow::Array> fsA, esA, vsA;

    (void)groupIdB.Finish(&groupIdA); (void)roundIdB.Finish(&roundIdA);
    (void)numFacetB.Finish(&numFacetA); (void)numEdgeB.Finish(&numEdgeA);
    (void)numVertexB.Finish(&numVertexA); (void)numPointsB.Finish(&numPointsA);
    (void)numVertexPointsB.Finish(&numVertexPointsA);
    (void)fsLb.Finish(&fsA); (void)esLb.Finish(&esA); (void)vsLb.Finish(&vsA);

    // 按 Schema 顺序组装列
    std::vector<std::shared_ptr<arrow::Array>> columns = {
        groupIdA, roundIdA,
        numFacetA, numEdgeA, numVertexA, numPointsA, numVertexPointsA,
        fsA, esA, vsA,
    };
    if (ctx.enableQuantization) {
        std::shared_ptr<arrow::Array> pqxA, pqyA, pqzA;
        std::shared_ptr<arrow::Array> vpqxA, vpqyA, vpqzA;
        (void)pqxLb.Finish(&pqxA); (void)pqyLb.Finish(&pqyA); (void)pqzLb.Finish(&pqzA);
        (void)vpqxLb.Finish(&vpqxA); (void)vpqyLb.Finish(&vpqyA); (void)vpqzLb.Finish(&vpqzA);
        columns.insert(columns.end(), {pqxA, pqyA, pqzA, vpqxA, vpqyA, vpqzA});
    } else {
        std::shared_ptr<arrow::Array> pxA, pyA, pzA;
        std::shared_ptr<arrow::Array> vpxA, vpyA, vpzA;
        (void)pxLb.Finish(&pxA); (void)pyLb.Finish(&pyA); (void)pzLb.Finish(&pzA);
        (void)vpxLb.Finish(&vpxA); (void)vpyLb.Finish(&vpyA); (void)vpzLb.Finish(&vpzA);
        columns.insert(columns.end(), {pxA, pyA, pzA, vpxA, vpyA, vpzA});
    }

    return arrow::Table::Make(schema, columns, totalRows);
}

} // namespace pmsf
