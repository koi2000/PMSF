/*
 * @Author: koi
 * @Date: 2025-09-01 20:09:27
 * @Description:
 */

#include "BasicSerializeOperator.h"
#include "entropy/RangeCoderEntropyCoder.h"

#include <stdexcept>
#include "BufferUtils.h"
#include "Configuration.h"
#include "EntropyCodec.h"
#include "Halfedge.h"
#include "MeshUtils.h"
#include "Point.h"
#include "common/DebugTool.h"
#include "storage/PMSFUnifiedArtifactWriter.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <fstream>
#include <string>
#include <utility>
#include <vector>

namespace {
constexpr int kEdgeFlipSymbolMagic = 0x45464C50;  // "EFLP"
constexpr int kEdgeFlipSymbolFormatV2 = 2;
constexpr int kEdgeFlipSymbolFormatV3 = 3;
constexpr int kEdgeFlipSymbolFormatV4 = 4;
constexpr int kEdgeFlipSymbolFormatV5 = 5;
constexpr int kEdgeFlipSymbolFormatV8 = 8;
constexpr int kBoundaryBitMagic = 0x31444E42;  // 'BND1' LE
constexpr int kEdgeFlipSymbolFormatV6 = 6;
constexpr int kEdgeFlipSymbolFormatV7 = 7;
constexpr int kEdgeFlipSymbolFormatV9 = 9;  // one bit per all-edge, canonical (q-sorted) order
}

void BasicSerializeOperator::init() {
    buffer_ = new char[BUFFER_SIZE];
}

void BasicSerializeOperator::serializeInt(int val) {
    writeInt(buffer_, dataOffset_, val);
}

void BasicSerializeOperator::serializeBaseMesh(std::shared_ptr<MCGAL::Mesh> mesh) {}

void BasicSerializeOperator::serializeBaseMeshWithSeed(std::shared_ptr<MCGAL::Mesh> mesh, std::vector<MCGAL::Halfedge*> seeds) {
    mesh->garbage_collection();
    unsigned i_nbVerticesBaseMesh = mesh->size_of_vertices();
    unsigned i_nbFacesBaseMesh = mesh->size_of_facets();

    if (enableQuantization_) {
        writeChar(buffer_, dataOffset_, mesh->i_nbQuantBits);
        writePoint(buffer_, dataOffset_, mesh->bboxMin);
        writeFloat(buffer_, dataOffset_, mesh->f_quantStep);
    }

    writeInt(buffer_, dataOffset_, i_nbVerticesBaseMesh);
    writeInt(buffer_, dataOffset_, i_nbFacesBaseMesh);
    if (std::getenv("PMSF_EDGEFLIP_BASE_FACETABLE")) {
        auto vkey = [&](MCGAL::Vertex* v) {
            if (!v) return std::array<long long, 3>{0, 0, 0};
            if (enableQuantization_) {
                MCGAL::PointInt q = mesh->floatPosToInt(v->point());
                return std::array<long long, 3>{q[0], q[1], q[2]};
            }
            constexpr double scale = 100000000.0;
            return std::array<long long, 3>{
                static_cast<long long>(std::llround(static_cast<double>(v->x()) * scale)),
                static_cast<long long>(std::llround(static_cast<double>(v->y()) * scale)),
                static_cast<long long>(std::llround(static_cast<double>(v->z()) * scale))};
        };
        MCGAL::Halfedge* seed = seeds.empty() ? nullptr : seeds[0];
        std::vector<MCGAL::Vertex*> order;
        order.reserve(i_nbVerticesBaseMesh);
        if (seed && seed->vertex() && seed->end_vertex()) {
            order.push_back(seed->vertex());
            order.push_back(seed->end_vertex());
        }
        std::vector<MCGAL::Vertex*> rest;
        for (MCGAL::Vertex* v : mesh->vertices()) {
            if (!v || v->isRemoved()) continue;
            if (!order.empty() && (v == order[0] || v == order[1])) continue;
            rest.push_back(v);
        }
        std::stable_sort(rest.begin(), rest.end(), [&](MCGAL::Vertex* a, MCGAL::Vertex* b) {
            return vkey(a) < vkey(b);
        });
        order.insert(order.end(), rest.begin(), rest.end());
        std::unordered_map<MCGAL::Vertex*, int> index;
        index.reserve(order.size() * 2 + 1);
        for (size_t i = 0; i < order.size(); ++i) {
            order[i]->setVid(static_cast<int>(i));
            index[order[i]] = static_cast<int>(i);
        }
        unsigned bitOffset = 0;
        const int maxQuantizedValue = enableQuantization_ ? ((1 << mesh->i_nbQuantBits) - 1) : 0;
        for (MCGAL::Vertex* v : order) {
            const MCGAL::Point point = v->point();
            if (enableQuantization_) {
                MCGAL::PointInt p = mesh->floatPosToInt(point);
                for (int c = 0; c < 3; ++c) {
                    p[c] = std::clamp(p[c], 0, maxQuantizedValue);
                    writeBitsOff(static_cast<uint32_t>(p[c]), mesh->i_nbQuantBits, buffer_, bitOffset, dataOffset_);
                }
            } else {
                writePoint(buffer_, dataOffset_, point);
            }
        }
        if (enableQuantization_) {
            bitOffset = 0;
            dataOffset_++;
            const int connBit = static_cast<int>(std::ceil(std::log2(static_cast<double>(i_nbVerticesBaseMesh) + 1.0)));
            writeChar(buffer_, dataOffset_, static_cast<char>(connBit));
            unsigned faceBitOffset = 0;
            for (MCGAL::Facet* fit : mesh->faces()) {
                if (!fit || fit->isRemoved()) continue;
                for (MCGAL::Halfedge* h : *fit) {
                    if (!h || !h->vertex()) continue;
                    auto it = index.find(h->vertex());
                    const int vid = it == index.end() ? 0 : it->second;
                    writeBitsOff(static_cast<uint32_t>(vid), connBit, buffer_, faceBitOffset, dataOffset_);
                }
            }
            faceBitOffset = 0;
            dataOffset_++;
        } else {
            for (MCGAL::Facet* fit : mesh->faces()) {
                if (!fit || fit->isRemoved()) continue;
                for (MCGAL::Halfedge* h : *fit) {
                    if (!h || !h->vertex()) continue;
                    auto it = index.find(h->vertex());
                    writeInt(buffer_, dataOffset_, it == index.end() ? 0 : it->second);
                }
            }
        }
    } else {
    int id = 0;
    unsigned i_bitOffset = 0;
    MCGAL::Halfedge* seed = seeds[0];
    int groupId = 0;
    seed->vertex()->setVid(id++);
    seed->end_vertex()->setVid(id++);
    if (enableQuantization_) {
        MCGAL::Point p = seed->vertex()->point();
        MCGAL::PointInt pi = mesh->floatPosToInt(p);
        const int maxQuantizedValue =
            (1 << mesh->i_nbQuantBits) - 1;
        for (unsigned i = 0; i < 3; ++i) {
            pi[i] = std::clamp(pi[i], 0, maxQuantizedValue);
            writeBitsOff((uint32_t)pi[i], mesh->i_nbQuantBits, buffer_, i_bitOffset, dataOffset_);
        }
        p = seed->end_vertex()->point();
        pi = mesh->floatPosToInt(p);
        for (unsigned i = 0; i < 3; ++i) {
            pi[i] = std::clamp(pi[i], 0, maxQuantizedValue);
            writeBitsOff((uint32_t)pi[i], mesh->i_nbQuantBits, buffer_, i_bitOffset, dataOffset_);
        }
    } else {
        writePoint(buffer_, dataOffset_, seed->vertex()->point());
        writePoint(buffer_, dataOffset_, seed->end_vertex()->point());
    }

    if (seamCutBase_ && enableQuantization_) {
        std::vector<MCGAL::Vertex*> others;
        others.reserve(mesh->vertices().size());
        for (MCGAL::Vertex* vit : mesh->vertices()) {
            if (!vit || vit->isRemoved()) continue;
            if (vit->poolId() == seed->vertex()->poolId() ||
                vit->poolId() == seed->end_vertex()->poolId()) continue;
            others.push_back(vit);
        }
        auto vertexKey = [&](MCGAL::Vertex* v) {
            const MCGAL::PointInt q = mesh->floatPosToInt(v->point());
            return std::array<int, 3>{q[0], q[1], q[2]};
        };
        std::stable_sort(others.begin(), others.end(), [&](MCGAL::Vertex* a, MCGAL::Vertex* b) {
            return vertexKey(a) < vertexKey(b);
        });
        std::vector<unsigned char> isDup(others.size(), 0);
        std::vector<int> dupOrds;
        for (size_t i = 1; i < others.size(); ++i) {
            if (vertexKey(others[i]) == vertexKey(others[i - 1])) {
                isDup[i] = 1;
                dupOrds.push_back(static_cast<int>(i));
            }
        }
        if (i_bitOffset != 0) { i_bitOffset = 0; dataOffset_++; }
        writeInt(buffer_, dataOffset_, static_cast<int>(dupOrds.size()));
        auto writeVarint = [&](int value) {
            std::uint32_t z = (static_cast<std::uint32_t>(value) << 1) ^
                              static_cast<std::uint32_t>(value >> 31);
            while (z >= 0x80u) {
                writeChar(buffer_, dataOffset_, static_cast<char>((z & 0x7Fu) | 0x80u));
                z >>= 7;
            }
            writeChar(buffer_, dataOffset_, static_cast<char>(z));
        };
        int prevOrd = -1;
        const size_t dedupStart = dataOffset_;
        for (int ord : dupOrds) {
            writeVarint(ord - prevOrd);
            prevOrd = ord;
        }
        if (std::getenv("PMSF_SEAM_DEBUG")) {
            std::cerr << "[seam-dedup] dups=" << dupOrds.size()
                      << " mapBytes=" << (dataOffset_ - dedupStart) << std::endl;
        }
        const int maxQuantizedValue = (1 << mesh->i_nbQuantBits) - 1;
        for (size_t i = 0; i < others.size(); ++i) {
            if (isDup[i]) continue;
            MCGAL::PointInt q = mesh->floatPosToInt(others[i]->point());
            for (int c = 0; c < 3; ++c) {
                q[c] = std::clamp(q[c], 0, maxQuantizedValue);
                writeBitsOff(static_cast<uint32_t>(q[c]), mesh->i_nbQuantBits,
                             buffer_, i_bitOffset, dataOffset_);
            }
        }
        i_bitOffset = 0;
        if (others.size() > dupOrds.size()) dataOffset_++;
        for (MCGAL::Vertex* vit : others) vit->setVid(id++);
    } else {
    for (MCGAL::Vertex* vit : mesh->vertices()) {
        if (vit->isRemoved()) {
            continue;
        }
        if (vit->poolId() == seed->vertex()->poolId() || vit->poolId() == seed->end_vertex()->poolId()) {
            continue;
        }
        MCGAL::Point point = vit->point();
        if (enableQuantization_) {
            MCGAL::PointInt p = mesh->floatPosToInt(point);
            const int maxQuantizedValue =
                (1 << mesh->i_nbQuantBits) - 1;
            for (unsigned i = 0; i < 3; ++i) {
                // The quantizer uses [0, 2^q - 1].  Writing 2^q into
                // q bits silently wraps to zero and corrupts the base mesh.
                p[i] = std::clamp(p[i], 0, maxQuantizedValue);
                writeBitsOff(p[i], mesh->i_nbQuantBits, buffer_, i_bitOffset, dataOffset_);
            }
        } else {
            writePoint(buffer_, dataOffset_, point);
        }
        vit->setVid(id++);
    }
    }

    char connBit = 0;
    if (enableQuantization_) {
        i_bitOffset = 0;
        dataOffset_++;
        connBit = ceil(log2(i_nbVerticesBaseMesh + 1));
        writeChar(buffer_, dataOffset_, connBit);
    }
    int idx = 0;
    FILE* baseDump = nullptr;
    if (baseDump) {
        std::fprintf(baseDump, "H %u %u %d\n", i_nbVerticesBaseMesh, i_nbFacesBaseMesh, connBit);
        for (MCGAL::Vertex* v : mesh->vertices()) {
            if (!v || v->isRemoved()) continue;
            auto q = mesh->floatPosToInt(v->point());
            std::fprintf(baseDump, "V %d %d %d %d\n", v->vid(), q.x(), q.y(), q.z());
        }
    }
    std::vector<uint32_t> fs;
    for (MCGAL::Facet* fit : mesh->faces()) {
        if (fit->isRemoved()) {
            continue;
        }
        int i_faceDegree = 0;

        MCGAL::Halfedge* st = fit->proxyHalfedge();
        MCGAL::Halfedge* ed = st;
        // do {
        //     i_faceDegree++;
        //     st = st->next();
        // } while (st != ed);
        // writeInt(buffer_, dataOffset_, i_faceDegree);
        // writeBits(i_faceDegree, 2, buffer_, i_bitOffset, dataOffset_);
        if (idx == 27) {
            int i = 0;
        }
        std::array<int, 3> dumpVids{-1, -1, -1};
        int dumpK = 0;
        do {
            // writeInt(buffer_, dataOffset_, st->vertex()->vid());
            if (enableQuantization_) {
                writeBitsOff((uint32_t)st->vertex()->vid(), connBit, buffer_, i_bitOffset, dataOffset_);
                fs.push_back(st->vertex()->vid());
            } else {
                writeInt(buffer_, dataOffset_, st->vertex()->vid());
            }
            if (dumpK < 3) dumpVids[dumpK++] = st->vertex()->vid();
            st = st->next();
        } while (st != ed);
        if (baseDump) {
            std::fprintf(baseDump, "F %d %d %d %d\n", idx, dumpVids[0], dumpVids[1], dumpVids[2]);
        }
        // DEBUG_DUMP_CONTAINER<uint32_t>(fs, "./fsc.txt");
        idx++;
    }
    if (baseDump) { std::fclose(baseDump); baseDump = nullptr; }
    if (enableQuantization_) {
        i_bitOffset = 0;
        dataOffset_++;
    }
    }

    if (!boundaryRoundBitCounts_.empty()) {
        int totalBits = 0;
        for (int c : boundaryRoundBitCounts_) totalBits += std::max(c, 0);
        // v4: compact mode-64 residual stream (all ops are len=7): per op we drop
        // the redundant length int and pack (kA,kB) into one small/distance varint.
        std::vector<int> compactCounts;
        std::vector<int> compactResiduals;
        bool compactV4 = boundaryApplyMode_ == 64;
        if (compactV4) {
            int resOff = 0;
            for (int c : boundaryRoundBitCounts_) {
                if (c < 0 || (c % 8) != 0) { compactV4 = false; break; }
                const int ops = c / 8;
                compactCounts.push_back(ops * 7);
                for (int o = 0; o < ops && compactV4; ++o) {
                    const int b = resOff + o * 8;
                    if (b + 7 >= static_cast<int>(boundaryRoundResiduals_.size()) ||
                        boundaryRoundResiduals_[static_cast<std::size_t>(b)] != 7) {
                        compactV4 = false;
                        break;
                    }
                    const int kA = boundaryRoundResiduals_[static_cast<std::size_t>(b + 4)] / 64;
                    const int kB = boundaryRoundResiduals_[static_cast<std::size_t>(b + 4)] % 64;
                    int pack = 0;
                    if (kA >= 2 && kA <= 9 && kB >= 2 && kB <= 9) {
                        pack = ((kA - 2) << 3) | (kB - 2);
                    } else {
                        pack = 64 + kA * 64 + kB;
                    }
                    compactResiduals.push_back(boundaryRoundResiduals_[static_cast<std::size_t>(b + 1)]);
                    compactResiduals.push_back(boundaryRoundResiduals_[static_cast<std::size_t>(b + 2)]);
                    compactResiduals.push_back(boundaryRoundResiduals_[static_cast<std::size_t>(b + 3)]);
                    compactResiduals.push_back(pack);
                    compactResiduals.push_back(boundaryRoundResiduals_[static_cast<std::size_t>(b + 5)]);
                    compactResiduals.push_back(boundaryRoundResiduals_[static_cast<std::size_t>(b + 6)]);
                    compactResiduals.push_back(boundaryRoundResiduals_[static_cast<std::size_t>(b + 7)]);
                }
                if (!compactV4) break;
                resOff += c;
            }
            if (!compactV4) { compactCounts.clear(); compactResiduals.clear(); }
        }
        writeInt(buffer_, dataOffset_, kBoundaryBitMagic);
        writeInt(buffer_, dataOffset_, compactV4 ? 4 : 3);
        writeInt(buffer_, dataOffset_, static_cast<int>(boundaryRoundBitCounts_.size()));
        writeInt(buffer_, dataOffset_, boundaryApplyMode_);
        writeInt(buffer_, dataOffset_, totalBits);
        if (compactV4) {
            for (int c : compactCounts) writeInt(buffer_, dataOffset_, c);
        } else {
            for (int c : boundaryRoundBitCounts_) writeInt(buffer_, dataOffset_, c);
        }
        std::vector<std::uint32_t> symbols;
        symbols.reserve(static_cast<size_t>(totalBits));
        const int n = std::min(totalBits, static_cast<int>(boundaryRoundBits_.size()));
        for (int i = 0; i < n; ++i) symbols.push_back(boundaryRoundBits_[static_cast<size_t>(i)] ? 1u : 0u);
        std::vector<std::uint8_t> coded;
        pmsf::RangeCoderEntropyCoder coder(1);
        coder.encode(symbols, coded);
        writeInt(buffer_, dataOffset_, static_cast<int>(coded.size()));
        for (std::uint8_t b : coded) writeChar(buffer_, dataOffset_, static_cast<char>(b));
        const std::vector<int>& residualsToWrite =
            compactV4 ? compactResiduals : boundaryRoundResiduals_;
        const int residualInts = static_cast<int>(residualsToWrite.size());
        writeInt(buffer_, dataOffset_, residualInts);
        const bool zigzagMode =
            boundaryApplyMode_ == 8 || boundaryApplyMode_ == 23 ||
            boundaryApplyMode_ == 30 || boundaryApplyMode_ == 31 ||
            boundaryApplyMode_ == 40 || boundaryApplyMode_ == 41 ||
            boundaryApplyMode_ == 42 || boundaryApplyMode_ == 50 ||
            boundaryApplyMode_ == 13 || boundaryApplyMode_ == 61 ||
            boundaryApplyMode_ == 62 || boundaryApplyMode_ == 63 ||
            boundaryApplyMode_ == 64 || boundaryApplyMode_ == 65 || boundaryApplyMode_ == 66 ||
            boundaryApplyMode_ == 70;
        if (zigzagMode) {
            for (int r : residualsToWrite) {
                std::uint32_t z = (static_cast<std::uint32_t>(r) << 1) ^
                                  static_cast<std::uint32_t>(r >> 31);
                while (z >= 0x80u) {
                    writeChar(buffer_, dataOffset_, static_cast<char>((z & 0x7Fu) | 0x80u));
                    z >>= 7;
                }
                writeChar(buffer_, dataOffset_, static_cast<char>(z));
            }
        } else {
            for (int r : residualsToWrite) writeInt(buffer_, dataOffset_, r);
        }
    }

    if (enableEdgeFlipPostProcess_) {
        writeInt(buffer_, dataOffset_, kEdgeFlipSymbolMagic);
        if (!edgeFlipRoundBitCounts_.empty()) {
            int totalBits = 0;
            for (int c : edgeFlipRoundBitCounts_) {
                totalBits += std::max(c, 0);
            }
            const int packedBytes = (totalBits + 7) / 8;
            const bool hasCoordSymbols = !edgeFlipCoordSymbols_.empty();
            const bool canonicalBits =
                !hasCoordSymbols && std::getenv("PMSF_EDGEFLIP_BITS_CANONICAL") != nullptr;
            writeInt(buffer_, dataOffset_, hasCoordSymbols
                                                  ? kEdgeFlipSymbolFormatV7
                                                  : (canonicalBits ? kEdgeFlipSymbolFormatV9
                                                                   : kEdgeFlipSymbolFormatV8));
            writeInt(buffer_, dataOffset_, static_cast<int>(edgeFlipRoundBitCounts_.size()));
            writeInt(buffer_, dataOffset_, totalBits);
            if (hasCoordSymbols) {
                writeInt(buffer_, dataOffset_, static_cast<int>(edgeFlipCoordSymbols_.size()));
            }
            for (int c : edgeFlipRoundBitCounts_) {
                writeInt(buffer_, dataOffset_, c);
            }
            if (hasCoordSymbols) {
                for (int c : edgeFlipRoundCounts_) {
                    writeInt(buffer_, dataOffset_, c);
                }
            }
            if (!hasCoordSymbols) {
                std::vector<std::uint32_t> flipSymbols;
                flipSymbols.reserve(static_cast<size_t>(totalBits));
                const int nBits = std::min(totalBits, static_cast<int>(edgeFlipRoundBits_.size()));
                for (int i = 0; i < nBits; ++i) {
                    flipSymbols.push_back(edgeFlipRoundBits_[static_cast<size_t>(i)] ? 1u : 0u);
                }
                std::vector<std::uint8_t> coded;
                pmsf::RangeCoderEntropyCoder coder(1);
                coder.encode(flipSymbols, coded);
                writeInt(buffer_, dataOffset_, static_cast<int>(coded.size()));
                for (std::uint8_t b : coded) writeChar(buffer_, dataOffset_, static_cast<char>(b));
                return;
            }
            std::vector<unsigned char> packed(static_cast<size_t>(packedBytes), 0);
            const int n = std::min(totalBits, static_cast<int>(edgeFlipRoundBits_.size()));
            for (int i = 0; i < n; ++i) {
                if (edgeFlipRoundBits_[static_cast<size_t>(i)] == 0) continue;
                packed[static_cast<size_t>(i >> 3)] |=
                    static_cast<unsigned char>(1u << (i & 7));
            }
            for (int i = 0; i < packedBytes; ++i) {
                writeChar(buffer_, dataOffset_, static_cast<char>(packed[static_cast<size_t>(i)]));
            }
            if (hasCoordSymbols) {
                for (const auto& sym : edgeFlipCoordSymbols_) {
                    writeInt(buffer_, dataOffset_, sym.a[0]);
                    writeInt(buffer_, dataOffset_, sym.a[1]);
                    writeInt(buffer_, dataOffset_, sym.a[2]);
                    writeInt(buffer_, dataOffset_, sym.b[0]);
                    writeInt(buffer_, dataOffset_, sym.b[1]);
                    writeInt(buffer_, dataOffset_, sym.b[2]);
                    writeInt(buffer_, dataOffset_, sym.oa[0]);
                    writeInt(buffer_, dataOffset_, sym.oa[1]);
                    writeInt(buffer_, dataOffset_, sym.oa[2]);
                    writeInt(buffer_, dataOffset_, sym.ob[0]);
                    writeInt(buffer_, dataOffset_, sym.ob[1]);
                    writeInt(buffer_, dataOffset_, sym.ob[2]);
                    writeInt(buffer_, dataOffset_, sym.f0Proxy);
                    writeInt(buffer_, dataOffset_, sym.f1Proxy);
                    writeInt(buffer_, dataOffset_, sym.vaProxy);
                    writeInt(buffer_, dataOffset_, sym.vbProxy);
                    writeInt(buffer_, dataOffset_, sym.vcProxy);
                    writeInt(buffer_, dataOffset_, sym.vdProxy);
                }
            }
            return;
        }
        const bool hasCoordSymbols = !edgeFlipCoordSymbols_.empty();
        writeInt(buffer_, dataOffset_, hasCoordSymbols ? kEdgeFlipSymbolFormatV4 : kEdgeFlipSymbolFormatV2);
        writeInt(buffer_, dataOffset_, static_cast<int>(edgeFlipRoundCounts_.size()));
        writeInt(buffer_, dataOffset_, hasCoordSymbols
                                       ? static_cast<int>(edgeFlipCoordSymbols_.size())
                                       : static_cast<int>(edgeFlipSymbols_.size()));
        for (int c : edgeFlipRoundCounts_) {
            writeInt(buffer_, dataOffset_, c);
        }
        if (hasCoordSymbols) {
            for (const auto& sym : edgeFlipCoordSymbols_) {
                writeInt(buffer_, dataOffset_, sym.a[0]);
                writeInt(buffer_, dataOffset_, sym.a[1]);
                writeInt(buffer_, dataOffset_, sym.a[2]);
                writeInt(buffer_, dataOffset_, sym.b[0]);
                writeInt(buffer_, dataOffset_, sym.b[1]);
                writeInt(buffer_, dataOffset_, sym.b[2]);
                writeInt(buffer_, dataOffset_, sym.oa[0]);
                writeInt(buffer_, dataOffset_, sym.oa[1]);
                writeInt(buffer_, dataOffset_, sym.oa[2]);
                writeInt(buffer_, dataOffset_, sym.ob[0]);
                writeInt(buffer_, dataOffset_, sym.ob[1]);
                writeInt(buffer_, dataOffset_, sym.ob[2]);
            }
        } else {
            for (const auto& sym : edgeFlipSymbols_) {
                writeInt(buffer_, dataOffset_, sym.first);
                writeInt(buffer_, dataOffset_, sym.second);
            }
        }
    }
}

void BasicSerializeOperator::serialize(std::string path) {
    int size = 0;
    if (enablePrediction_) {
        // RD 预测路径：残差 + range coder（位面 / 自适应模型，见 OptimizedSymbolCollectOperator::exportToBufferWithRangeCoder）
        //
        // 契约：开了 prediction 就必须有一个实现了 range-coder 导出的 collector。
        // 否则基类默认实现返回 0，产物里只剩 base mesh —— 不报错、不告警，
        // 解码端从缓冲区之外读数据，几轮之后才在别处崩。这种「静默产空」比崩溃
        // 难查得多，所以这里直接拒绝。
        if (!collector_->supportsRangeCoderExport()) {
            throw std::runtime_error(
                "[BasicSerializeOperator] 配置开了 prediction，但当前 collector 没有实现"
                " range-coder 导出（exportToBufferWithRangeCoder 会返回 0 字节，产物将只含"
                " base mesh）。要么换一个支持该导出的 collector，要么关掉 prediction。");
        }
        collector_->setEnablePrediction(true);
        size = collector_->exportToBufferWithRangeCoder(buffer_ + dataOffset_, enableQuantization_);
    } else {
        size = collector_->exportToBuffer(buffer_ + dataOffset_, enableQuantization_, enableCompress_);
    }
    char* outBlock = buffer_;
    int outBlockSize = dataOffset_ + size;

    writeLengthPrefixedLegacySingleFile(path, outBlock,
                                        static_cast<std::size_t>(outBlockSize),
                                        0);
}
