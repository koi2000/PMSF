// SPDX-License-Identifier: Apache-2.0
//
// PCLTTMTagSymbolReadOperator.cpp — see header for design contract.
//
// sv=3 Phase 2：砍 edgeSym 流。decoder 按 polyFace 拓扑（proxyHalfedge + valence）
// 重建 N-3 条内部对角线 halfedge 列表，不再消费 edgeSym bit。默认仍是
// v0-fan；PMSF_PCLTTM_USE_TAG_TABLE_TRIANGULATION=1 时跨 added internal
// edges 收集非 fan patch 对角线。
//
// 关键依赖：base_mesh 通过 reset_proxy_first 重建后，face.proxyHalfedge.vertex()
//   = encoder 端 v0 语义（与 encoder serialize faceVids[0] 对齐）。tag-table
//   模式还要求 encoder 侧用同一环境变量写出 added internal edges。

#include "PCLTTM/decompress/PCLTTMTagSymbolReadOperator.h"
#include "entropy/StreamEntropyCoder.h"
#include "common/PaperLayoutAliases.h"

#include "BFSVersionManager.h"
#include "compress/ConnectivityBlobContainer.h"
#include "RD/compress/RDSymbolCollectOperator.h"  // RDValenceCode
#include "storage/PaperBlobLayoutRegistry.h"
#include "common/PaperBlobAlgorithmSpecs.h"  // 算法专有 blob 规格工厂
#include "ContextPool.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Vertex.h"
#include "core.h"
#include "predict/RDPredictor.h"  // kVPropRDR1, rdCurvaturePrediction
#include "PCLTTM/PCLTTMFrenetLayer.h"
#include "predict/FrenetResidualCodec.h"

#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"
#include "Point.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <queue>
#include <span>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

// =============================================================================
// =============================================================================
namespace {

struct FrenetFrame {
    double nx = 0.0, ny = 0.0, nz = 1.0;
    double tx = 1.0, ty = 0.0, tz = 0.0;
    double bx = 0.0, by = 1.0, bz = 0.0;
};

// =============================================================================
// PR-FRENET-FIX（task-13）：decoder 侧 frame 的**量化对称**构造。
// 与 encoder 端（PCLTTMTagSymbolCollectOperator.cpp）保持同一份实现：
// frenet env 开启时位置统一走 int 网格（与 Mesh::floatPosToInt/intPosToFloat
// 逐位同源），env 关闭时退回原浮点路径（且 frame 只在 frenet 分支被调用
// ⇒ 默认 RD/PCLTTM 产物逐字节不变）。
// =============================================================================
MCGAL::Point frenetGridPoint(MCGAL::Vertex* v,
                             const MCGAL::Point* bboxMin,
                             float quantStep) {
    if (v == nullptr) return MCGAL::Point(0.0f, 0.0f, 0.0f);
    const MCGAL::Point p = v->point();
    if (bboxMin == nullptr || quantStep <= 0.0f) return p;
    const int qx = static_cast<int>((p.x() - bboxMin->x()) / quantStep);
    const int qy = static_cast<int>((p.y() - bboxMin->y()) / quantStep);
    const int qz = static_cast<int>((p.z() - bboxMin->z()) / quantStep);
    return MCGAL::Point(bboxMin->x() + (qx + 0.5) * quantStep,
                        bboxMin->y() + (qy + 0.5) * quantStep,
                        bboxMin->z() + (qz + 0.5) * quantStep);
}

MCGAL::Vector3 frenetFrameNormal(MCGAL::Facet* f,
                                 const MCGAL::Point* bboxMin,
                                 float quantStep) {
    // 只在 frenet 分支被调用（encoder env / decoder sv=7|8）；参数缺失时退回原路径。
    if (bboxMin == nullptr || quantStep <= 0.0f) {
        return f->computeNormal();
    }
    MCGAL::Halfedge* h = f->proxyHalfedge();
    if (h == nullptr || h->next() == nullptr) {
        return MCGAL::Vector3(0, 0, 0);
    }
    MCGAL::Halfedge* hn = h->next();
    MCGAL::Halfedge* hnn = hn->next();
    MCGAL::Vertex* v3 = hnn ? hnn->vertex() : hn->end_vertex();
    const MCGAL::Point p1 = frenetGridPoint(h->vertex(), bboxMin, quantStep);
    const MCGAL::Point p2 = frenetGridPoint(hn->vertex(), bboxMin, quantStep);
    const MCGAL::Point p3 = frenetGridPoint(v3, bboxMin, quantStep);
    const MCGAL::Vector3 v1(p2.x() - p1.x(), p2.y() - p1.y(), p2.z() - p1.z());
    const MCGAL::Vector3 v2(p3.x() - p1.x(), p3.y() - p1.y(), p3.z() - p1.z());
    return v1.cross(v2).normalize();
}

FrenetFrame residualFrenetFrame(MCGAL::Facet* f,
                                const MCGAL::Point* bboxMin,
                                float quantStep) {
    FrenetFrame frame;
    if (!f) return frame;

    MCGAL::Vector3 n = frenetFrameNormal(f, bboxMin, quantStep);
    double nlen = std::sqrt((double)n.x() * n.x() +
                            (double)n.y() * n.y() +
                            (double)n.z() * n.z());
    if (nlen > 1e-12) {
        frame.nx = n.x() / nlen;
        frame.ny = n.y() / nlen;
        frame.nz = n.z() / nlen;
    }

    MCGAL::Halfedge* hp = f->proxyHalfedge();
    double gx = 1.0, gy = 0.0, gz = 0.0;
    if (hp && hp->opposite() && hp->vertex() && hp->opposite()->vertex()) {
        MCGAL::Point p = frenetGridPoint(hp->vertex(), bboxMin, quantStep);
        MCGAL::Point q =
            frenetGridPoint(hp->opposite()->vertex(), bboxMin, quantStep);
        gx = (double)p.x() - q.x();
        gy = (double)p.y() - q.y();
        gz = (double)p.z() - q.z();
    }

    const double dotGN = gx * frame.nx + gy * frame.ny + gz * frame.nz;
    frame.tx = gx - dotGN * frame.nx;
    frame.ty = gy - dotGN * frame.ny;
    frame.tz = gz - dotGN * frame.nz;
    double tlen = std::sqrt(frame.tx * frame.tx +
                            frame.ty * frame.ty +
                            frame.tz * frame.tz);
    if (tlen <= 1e-12) {
        const bool normalMostlyX =
            std::fabs(frame.nx) >= std::fabs(frame.ny) &&
            std::fabs(frame.nx) >= std::fabs(frame.nz);
        gx = normalMostlyX ? 0.0 : 1.0;
        gy = normalMostlyX ? 1.0 : 0.0;
        gz = 0.0;
        const double dotFallback = gx * frame.nx + gy * frame.ny + gz * frame.nz;
        frame.tx = gx - dotFallback * frame.nx;
        frame.ty = gy - dotFallback * frame.ny;
        frame.tz = gz - dotFallback * frame.nz;
        tlen = std::sqrt(frame.tx * frame.tx +
                         frame.ty * frame.ty +
                         frame.tz * frame.tz);
    }
    if (tlen > 1e-12) {
        frame.tx /= tlen;
        frame.ty /= tlen;
        frame.tz /= tlen;
    }

    frame.bx = frame.ny * frame.tz - frame.nz * frame.ty;
    frame.by = frame.nz * frame.tx - frame.nx * frame.tz;
    frame.bz = frame.nx * frame.ty - frame.ny * frame.tx;
    return frame;
}

bool usePclttmTagTableTriangulation() {
    const char* v = std::getenv("PMSF_PCLTTM_USE_TAG_TABLE_TRIANGULATION");
    if (v && *v && std::strcmp(v, "0") != 0) {
        throw std::runtime_error(
            "[PCLTTMTag decoder] tag-table triangulation is not serialized in "
            "BlobLayout=16/sv=3 (D25); refusing to decode with a mode that "
            "would silently produce wrong diagonals.");
    }
    return false;
}

std::vector<MCGAL::Facet*> collectPatchFacesAcrossAddedEdges(
    MCGAL::Facet* start,
    int valence) {

    std::vector<MCGAL::Facet*> faces;
    if (!start || valence < 3) return faces;
    const int targetFaces = valence - 2;
    std::queue<MCGAL::Facet*> q;
    std::unordered_set<int> seen;
    q.push(start);
    seen.insert(start->poolId());

    while (!q.empty() && static_cast<int>(faces.size()) < targetFaces) {
        MCGAL::Facet* f = q.front();
        q.pop();
        if (!f || f->isRemoved()) continue;
        faces.push_back(f);
        for (MCGAL::Halfedge* h : *f) {
            if (!h || !h->isAdded() || !h->opposite()) continue;
            MCGAL::Facet* nf = h->opposite()->face();
            if (!nf || nf->isRemoved()) continue;
            if (seen.insert(nf->poolId()).second) {
                q.push(nf);
            }
        }
    }
    return faces;
}

int appendPatchInternalDiagonals(const std::vector<MCGAL::Facet*>& faces,
                                 std::vector<MCGAL::Halfedge*>& out) {
    std::unordered_set<int> faceIds;
    for (MCGAL::Facet* f : faces) {
        if (f) faceIds.insert(f->poolId());
    }
    std::unordered_set<int> edgeIds;
    int pushed = 0;
    for (MCGAL::Facet* f : faces) {
        if (!f) continue;
        for (MCGAL::Halfedge* h : *f) {
            if (!h || !h->isAdded() || !h->opposite()) continue;
            MCGAL::Facet* of = h->opposite()->face();
            if (!of || !faceIds.count(of->poolId())) continue;
            const int a = h->poolId();
            const int b = h->opposite()->poolId();
            const int key = (a < b) ? a : b;
            if (!edgeIds.insert(key).second) continue;
            out.push_back(h);
            ++pushed;
        }
    }
    return pushed;
}


// ============================================================================
// PR-FRENET-LIFT（task-13）：可逆整数 lifting 版局部系变换。
//
// 为什么需要：float 旋转 + round 是**不可逆**的（decoder 反旋转后残差有 ±0.5~1
// 量化单位的损失，经 50 层预测链累积到 ~39 quant unit ⇒ sv=7 decode coord_max
// 1.75e-3 / topology false；sv=8 exact correction 修得好但 correction 流
// 4,225 B > frenet 省下的 2,171 B，净亏）。这里把 R（rows=n,t,b）分解成
// Euler ZYX = Rz(α)Ry(β)Rx(γ)，每个 2D 旋转用 3 个 lifting shear
// （u += round(a·v) 这种整数可逆步）实现 ⇒ 整数域**严格可逆**（decoder 逐位逆回），
// 不需要 correction 流。R 由量化位置构造（frenetFrameNormal）⇒ 双端 bit-exact。
// ============================================================================

int liftRound(double v) { return static_cast<int>(std::llround(v)); }

void lift2dInverse(int& u, int& v, double c, double s) {
    if (c < 0.0) {
        u = -u;
        v = -v;
        c = -c;
        s = -s;
    }
    if (std::fabs(s) < 1e-15) return;
    const double a = (c - 1.0) / s;
    u -= liftRound(a * static_cast<double>(v));
    v -= liftRound(s * static_cast<double>(u));
    u -= liftRound(a * static_cast<double>(v));
}

/// R = Rz(alpha)·Ry(beta)·Rx(gamma)（ZYX Euler）。R 的行 = (n, t, b)。
void frenetEulerAngles(const FrenetFrame& fr, double& alpha, double& beta,
                       double& gamma) {
    double sBeta = -fr.bx;  // r20 = -sin(beta)
    if (sBeta > 1.0) sBeta = 1.0;
    if (sBeta < -1.0) sBeta = -1.0;
    beta = std::asin(sBeta);
    if (std::fabs(std::cos(beta)) > 1e-8) {
        gamma = std::atan2(fr.by, fr.bz);  // r21, r22
        alpha = std::atan2(fr.tx, fr.nx);  // r10, r00
    } else {
        // gimbal lock（beta≈±90°）：只有 alpha∓gamma 可辨，取 gamma=0。
        gamma = 0.0;
        alpha = std::atan2(-fr.ny, fr.ty);
    }
}

/// inverse: r = R^-1·p（liftResidualToFrenet 的整数逆）。
MCGAL::PointInt liftResidualFromFrenet(const MCGAL::PointInt& p,
                                       const FrenetFrame& fr) {
    double alpha = 0.0, beta = 0.0, gamma = 0.0;
    frenetEulerAngles(fr, alpha, beta, gamma);
    int x = p.x(), y = p.y(), z = p.z();
    lift2dInverse(x, y, std::cos(alpha), std::sin(alpha));   // Rz^-1
    lift2dInverse(x, z, std::cos(beta), -std::sin(beta));    // Ry^-1
    lift2dInverse(y, z, std::cos(gamma), std::sin(gamma));   // Rx^-1
    return MCGAL::PointInt(x, y, z);
}

MCGAL::PointInt unprojectResidualFromFrenet(const MCGAL::PointInt& r,
                                            MCGAL::Facet* f,
                                            const MCGAL::Point* bboxMin,
                                            float quantStep) {
    const FrenetFrame frame = residualFrenetFrame(f, bboxMin, quantStep);
    return liftResidualFromFrenet(r, frame);
}

bool rdtagFrenetGateFrameEnabled() {
    const char* e = std::getenv("PMSF_RDTAG_FRENET_GATE_FRAME");
    return e && e[0] == '1';
}




bool rdtagKeepR1AcrossRounds() {
    const char* e = std::getenv("PMSF_RDTAG_KEEP_R1");
    return e && e[0] == '1';
}

bool rdtagFrenetRoundEnabled(bool streamUsesFrenet,
                             int decoderRoundIdx,
                             int decodedRoundCount) {
    if (!streamUsesFrenet) return false;
    int encoderRoundIdx = decoderRoundIdx;
    if (decodedRoundCount > 0) {
        encoderRoundIdx = decodedRoundCount - 1 - decoderRoundIdx;
    }
    int start = 0;
    int end = 1 << 30;
    if (const char* s = std::getenv("PMSF_RDTAG_FRENET_START")) {
        start = std::atoi(s);
    }
    if (const char* s = std::getenv("PMSF_RDTAG_FRENET_END")) {
        end = std::atoi(s);
    }
    return encoderRoundIdx >= start && encoderRoundIdx <= end;
}

inline void markFacetSplittableForDecodedPoly(MCGAL::Facet* f) {
    if (!f || f->isSplittable()) return;
    if (f->isUnsplittable()) {
        f->resetSplittableFlag();
    }
    f->setSplittable();
}

}  // namespace

void PCLTTMTagSymbolReadOperator::setPMSFSymbolContext(const pmsf::PMSFSymbolTable* symbols,
                                                       const pmsf::PMSFSymbolPayloadTable* payloads) {
    SymbolReadOperator::setPMSFSymbolContext(symbols,payloads);
    if (!symbols || symbols->empty()) return;
    using namespace pmsf::pclttm_frenet;
    expectFrenetRelations_=true;
    if (!symbols->front().delta.side.count(kGeometryStream)) return;
    frenetDenominators_.clear();
    faceBitsPerRound_.clear();
    rcDecodedIsPoly_.clear(); rcDecodedValCodes_.clear(); rcDecodedOtherVals_.clear();
    rcDecodedVertexTags_.clear(); rcDecodedResiduals_.clear();
    currentRoundIdx_=0;
    for (std::size_t r=0; r<symbols->size(); ++r) {
        const auto& row=(*symbols)[r];const auto& delta=row.delta;
        if(row.objectId!=0||row.partitionId!=0||row.lodId!=static_cast<std::int32_t>(r)||
           !delta.triangulationEdgeSymbols.empty()||!delta.flipEdgeSymbols.empty()||
           !delta.removedVertexCoords.empty()||!delta.removedVertexResiduals.empty()||
           !delta.shiftedVertexCoords.empty()||!delta.shiftedVertexResiduals.empty())
            throw std::runtime_error("invalid PCLTTM Frenet typed role/order mapping");
        auto stream=[&](const char* name)->std::span<const std::uint8_t> {
            const auto it=delta.side.find(name);
            if (it==delta.side.end())return {};
            if (it->second.size()!=1||!std::holds_alternative<std::vector<std::uint8_t>>(it->second.front()))
                throw std::runtime_error("PCLTTM Frenet typed stream is not one binary cell");
            return std::get<std::vector<std::uint8_t>>(it->second.front());
        };
        const auto layer=decodeLayer(stream(kGeometryStream),delta.vertexSymbols.size());
        const auto escapes=stream(kValenceEscape);
        std::size_t polyCount=0,escapeCount=0;
        for(auto value:delta.facetSymbols) {
            if(value<0||value>3)throw std::runtime_error("invalid PCLTTM typed facet symbol");
            rcDecodedIsPoly_.push_back(value!=0);
            if(value) {++polyCount;rcDecodedVertexTags_.push_back(static_cast<std::uint8_t>(value-1));}
        }
        for(auto value:delta.vertexSymbols) {
            if(value<0||value>static_cast<int>(RDValenceCode::VAL_OTHER))
                throw std::runtime_error("invalid PCLTTM typed valence symbol");
            rcDecodedValCodes_.push_back(static_cast<std::uint8_t>(value));
            escapeCount+=value==static_cast<int>(RDValenceCode::VAL_OTHER);
        }
        if(polyCount!=layer.coefficients.size()||escapeCount!=escapes.size())
            throw std::runtime_error("PCLTTM Frenet typed role/count mismatch");
        rcDecodedOtherVals_.insert(rcDecodedOtherVals_.end(),escapes.begin(),escapes.end());
        for(const auto& q:layer.coefficients)rcDecodedResiduals_.emplace_back(q[0],q[1],q[2]);
        faceBitsPerRound_.push_back(static_cast<int>(delta.facetSymbols.size()));
        frenetDenominators_.push_back(layer.denominator);
    }
}

void PCLTTMTagSymbolReadOperator::importFromRangeCodedBuffer(
    char* buffer,
    size_t& dataOffset,
    unsigned nbQuantBits) {
    constexpr size_t kArbitraryLargeView = static_cast<size_t>(1) << 32;
    std::span<const uint8_t> view(reinterpret_cast<const uint8_t*>(buffer + dataOffset),
                                   kArbitraryLargeView);

    pmsf::ConnectivityBlobReader rd(view);
    const auto& hdr = rd.header();
    const bool frenet = hdr.scheme_version == pmsf::kPCLTTMTagFrenetVersion;
    frenetDenominators_.clear();
    expectFrenetRelations_=false;
    const auto spec = pmsf::pclttmTagConnectivityBlobSpec(
        static_cast<std::int32_t>(hdr.num_rounds),
        frenet ? pmsf::kPCLTTMTagFrenetVersion : pmsf::kPaperBlobSchemeVersionPCLTTMTag);
    pmsf::requirePaperConnectivityBlobHeaderMatchesSpec(
        hdr, spec, "PCLTTMTag");

    const int numRounds = static_cast<int>(hdr.num_rounds);

    rcUsingResidualLocal_ = header_.isEnablePrediction() && header_.isEnableQuantization();
    if (!rcUsingResidualLocal_) {
        throw std::runtime_error(
            "[PCLTTMTag decoder] expects enablePrediction && enableQuantization");
    }
    const int BP_BITS = static_cast<int>(nbQuantBits) + 1;

    faceBitsPerRound_.assign(numRounds, 0);

    constexpr int lg_totf = 10;
    constexpr int rescale = 1 << 9;

    for (int r = 0; r < numRounds; ++r) {
        std::size_t patchCount=0;
        // ---- stream 0: valence ----
        {
            auto sl = rd.getStream(r, pmsf::kPaperBlobStreamPCLTTMTagValence);
            if (sl.size() < 8) throw std::runtime_error("[PCLTTMTag decoder] stream 0 too small");
            int32_t numPoly = 0, numOther = 0;
            std::memcpy(&numPoly,  sl.data(),     4);
            std::memcpy(&numOther, sl.data() + 4, 4);
            if (frenet && (numPoly<0 || numOther<0 || numOther>numPoly))
                throw std::runtime_error("invalid PCLTTM Frenet valence counts");
            patchCount=static_cast<std::size_t>(numPoly);

            // Phase S2：与编码侧同一采集缝。注入分支按展平约定逆变换：
            // 前 numPoly 个主符号，每个 VAL_OTHER 后紧跟一个 escape 符号。
            // 默认分支一字不动。
            if (auto injected = ::pmsf::injectedStreamCoder("valence")) {
                std::vector<std::uint32_t> syms;
                injected->reset();
                injected->decode(sl.data() + 8, sl.size() - 8,
                                 static_cast<std::size_t>(numPoly) +
                                     static_cast<std::size_t>(numOther),
                                 syms);
                std::size_t k = 0;
                for (int32_t i = 0; i < numPoly; ++i) {
                    const std::uint32_t sym = syms[k++];
                    rcDecodedValCodes_.push_back(static_cast<uint8_t>(sym));
                    if (sym == static_cast<std::uint32_t>(RDValenceCode::VAL_OTHER)) {
                        rcDecodedOtherVals_.push_back(static_cast<uint8_t>(syms[k++]));
                    }
                }
            } else {

            std::vector<char> rcStaging(sl.begin() + 8, sl.end());
            size_t rcOffset = 0;
            rangecoder rc;
            rc.p_data = rcStaging.data();
            rc.p_dataOffset = &rcOffset;
            if (start_decoding(&rc) != 0) {
                throw std::runtime_error("[PCLTTMTag decoder] valence RC start_decoding failed");
            }

            constexpr int alphabet_size = 8;
            qsmodel ms[4];
            for (int i = 0; i < 4; ++i) {
                initqsmodel(&ms[i], alphabet_size, lg_totf, rescale, nullptr, 0);
            }
            qsmodel esc;
            initqsmodel(&esc, 256, 10, 1 << 9, nullptr, 0);

            auto deriveCtx = [](int prev) {
                if (prev < 0) return 0;
                if (prev == (int)RDValenceCode::VAL_3 || prev == (int)RDValenceCode::VAL_4) return 1;
                if (prev == (int)RDValenceCode::VAL_5 || prev == (int)RDValenceCode::VAL_6) return 2;
                return 3;
            };

            int prev = -1, otherSeen = 0;
            for (int k = 0; k < numPoly; ++k) {
                int ctx = deriveCtx(prev);
                int syfreq, ltfreq;
                int ltval = decode_culshift(&rc, lg_totf);
                int sym = qsgetsym(&ms[ctx], ltval);
                qsgetfreq(&ms[ctx], sym, &syfreq, &ltfreq);
                decode_update(&rc, syfreq, ltfreq, 1 << lg_totf);
                qsupdate(&ms[ctx], sym);
                rcDecodedValCodes_.push_back((uint8_t)sym);

                if (sym == (int)RDValenceCode::VAL_OTHER) {
                    int eltval = decode_culshift(&rc, 10);
                    int esym = qsgetsym(&esc, eltval);
                    int esyfreq, eltfreq;
                    qsgetfreq(&esc, esym, &esyfreq, &eltfreq);
                    decode_update(&rc, esyfreq, eltfreq, 1 << 10);
                    qsupdate(&esc, esym);
                    rcDecodedOtherVals_.push_back((uint8_t)esym);
                    otherSeen++;
                }
                prev = sym;
            }
            done_decoding(&rc);
            for (auto& m : ms) deleteqsmodel(&m);
            deleteqsmodel(&esc);
            (void)otherSeen; (void)numOther;
            }
        }

        // ---- stream 1: fusedFaceSym ----
        {
            auto sl = rd.getStream(r, pmsf::kPaperBlobStreamPCLTTMTagFusedFaceSym);
            if (sl.size() < 4) throw std::runtime_error("[PCLTTMTag decoder sv=3] fusedFaceSym too small");
            int32_t numFusedSyms = 0;
            std::memcpy(&numFusedSyms, sl.data(), 4);
            faceBitsPerRound_[r] = numFusedSyms;

            // Phase S2：注入分支（与编码侧对称）；默认分支一字不动。
            if (auto injected = ::pmsf::injectedStreamCoder("fusedFaceSym")) {
                std::vector<std::uint32_t> syms;
                injected->reset();
                injected->decode(sl.data() + 4, sl.size() - 4,
                                 static_cast<std::size_t>(numFusedSyms), syms);
                for (std::uint32_t sym : syms) {
                    if (sym == 0) {
                        rcDecodedIsPoly_.push_back(0);
                    } else {
                        rcDecodedIsPoly_.push_back(1);
                        rcDecodedVertexTags_.push_back(static_cast<uint8_t>(sym - 1));
                    }
                }
            } else {
            std::vector<char> rcStaging(sl.begin() + 4, sl.end());
            size_t rcOffset = 0;
            rangecoder rc;
            rc.p_data = rcStaging.data();
            rc.p_dataOffset = &rcOffset;
            if (start_decoding(&rc) != 0) {
                throw std::runtime_error("[PCLTTMTag decoder sv=3] fusedFaceSym start_decoding failed");
            }
            qsmodel m;
            initqsmodel(&m, 4, lg_totf, rescale, nullptr, 0);
            for (int k = 0; k < numFusedSyms; ++k) {
                int syfreq, ltfreq;
                int ltval = decode_culshift(&rc, lg_totf);
                int sym = qsgetsym(&m, ltval);
                qsgetfreq(&m, sym, &syfreq, &ltfreq);
                decode_update(&rc, syfreq, ltfreq, 1 << lg_totf);
                qsupdate(&m, sym);
                if (sym == 0) {
                    rcDecodedIsPoly_.push_back(0);
                } else {
                    rcDecodedIsPoly_.push_back(1);
                    rcDecodedVertexTags_.push_back(static_cast<uint8_t>(sym - 1));
                }
            }
            done_decoding(&rc);
            deleteqsmodel(&m);
            }
        }

        if (frenet) {
            const auto layer=pmsf::pclttm_frenet::decodeLayer(
                rd.getStream(r,pmsf::kPaperBlobStreamPCLTTMTagResidual),patchCount);
            frenetDenominators_.push_back(layer.denominator);
            for(const auto& q:layer.coefficients)rcDecodedResiduals_.emplace_back(q[0],q[1],q[2]);
            continue;
        }
        // ---- stream 2: residual (bit-plane) ----
        {
            auto sl = rd.getStream(r, pmsf::kPaperBlobStreamPCLTTMTagResidual);
            if (sl.size() < 4) throw std::runtime_error("[PCLTTMTag decoder sv=3] residual too small");
            int32_t numCoords = 0;
            std::memcpy(&numCoords, sl.data(), 4);

            // Phase S2：注入分支，按 zigzag + 3N 展平约定逆变换；
            // 默认分支一字不动。
            if (auto injected = ::pmsf::injectedStreamCoder("residual")) {
                std::vector<std::uint32_t> syms;
                injected->reset();
                injected->decode(sl.data() + 4, sl.size() - 4,
                                 static_cast<std::size_t>(numCoords), syms);
                auto unzigzag = [](std::uint32_t zz) -> std::int32_t {
                    return static_cast<std::int32_t>(zz >> 1) ^
                           -static_cast<std::int32_t>(zz & 1u);
                };
                for (std::size_t k = 0; k + 2 < syms.size(); k += 3) {
                    rcDecodedResiduals_.push_back(MCGAL::PointInt(
                        unzigzag(syms[k]), unzigzag(syms[k + 1]),
                        unzigzag(syms[k + 2])));
                }
            } else {
            std::vector<char> rcStaging(sl.begin() + 4, sl.end());
            size_t rcOffset = 0;
            rangecoder rc;
            rc.p_data = rcStaging.data();
            rc.p_dataOffset = &rcOffset;
            if (start_decoding(&rc) != 0) {
                throw std::runtime_error("[PCLTTMTag decoder sv=3] residual start_decoding failed");
            }
            std::vector<qsmodel> bpFirst1(BP_BITS), bpAfter1(BP_BITS);
            for (int b = 0; b < BP_BITS; ++b) {
                initqsmodel(&bpFirst1[b], 2, 10, 1 << 9, nullptr, 0);
                initqsmodel(&bpAfter1[b], 2, 10, 1 << 9, nullptr, 0);
            }
            auto decResidualBP = [&]() -> int32_t {
                uint32_t z = 0;
                bool seenOne = false;
                for (int b = BP_BITS - 1; b >= 0; --b) {
                    qsmodel* mp = seenOne ? &bpAfter1[b] : &bpFirst1[b];
                    int syfreq, ltfreq;
                    int ltval = decode_culshift(&rc, 10);
                    int sym = qsgetsym(mp, ltval);
                    qsgetfreq(mp, sym, &syfreq, &ltfreq);
                    decode_update(&rc, syfreq, ltfreq, 1 << 10);
                    qsupdate(mp, sym);
                    z = (z << 1) | (uint32_t)sym;
                    if (sym) seenOne = true;
                }
                return (int32_t)((z >> 1) ^ -(int32_t)(z & 1));
            };
            int coordsConsumed = 0;
            while (coordsConsumed < numCoords) {
                int32_t rx = decResidualBP();
                int32_t ry = decResidualBP();
                int32_t rz = decResidualBP();
                rcDecodedResiduals_.push_back(MCGAL::PointInt(rx, ry, rz));
                coordsConsumed += 3;
            }
            done_decoding(&rc);
            for (int b = 0; b < BP_BITS; ++b) {
                deleteqsmodel(&bpFirst1[b]);
                deleteqsmodel(&bpAfter1[b]);
            }
            }
        }
    }

    const size_t blobSize = hdr.headerByteSize() +
                            (hdr.stream_offsets.empty() ? 0 : hdr.stream_offsets.back());
    dataOffset += blobSize;

    const char* env = std::getenv("PMSF_BPV_DIAG");
    if (env && env[0] == '1') {
        std::cerr << "[PCLTTMTag decoder sv=3] rounds=" << numRounds
                  << " totalIsPoly=" << rcDecodedIsPoly_.size()
                  << " totalValCodes=" << rcDecodedValCodes_.size()
                  << " totalVertexTags=" << rcDecodedVertexTags_.size()
                  << " totalResiduals=" << rcDecodedResiduals_.size()
                  << " blobSize=" << blobSize << std::endl;
    }
}

// ============================================================================
// collectFromDecoded — sv=3 不消费 edgeSym 流，按 polyFace 拓扑识别对角线
// ============================================================================
void PCLTTMTagSymbolReadOperator::collectFromDecoded(
    MCGAL::Halfedge* seed,
    std::vector<MCGAL::Vertex*>& vertices,
    std::vector<MCGAL::Halfedge*>& halfedge,
    std::vector<MCGAL::Facet*>& facets) {

    (void)vertices;

    const int roundIdx = currentRoundIdx_;
    const bool frenet = !frenetDenominators_.empty();
    if (expectFrenetRelations_&&!frenet) throw std::runtime_error("PCLTTM Frenet typed artifact lacks the arithmetic geometry stream");

    // ----------------------------------------------------------------------------
    // Phase 3 路径 A — 每轮入口清零所有 vertex 的 r1（与编码端 mirror）。
    //   清零范围：本 mesh pool 的全部 allocated vertex（vindexs[meshId] 个），
    //   包括 isRemoved 的（无害；后续 BFS 不访问 removed vertex）。
    //   编码端在 PCLTTMTagSymbolCollectOperator::collect 入口对 active vertex 清零，
    //   active vertex set 与本端等价（同 LOD 时刻）。
    //   两端清零后 round 内独立累积 r1 → pred_c2 严格 bit-exact 对称。
    // ----------------------------------------------------------------------------
    if (!frenet && !rdtagKeepR1AcrossRounds()) {
        const int meshId = DEFAULT_MESH_ID;
        const int nAlloc = MCGAL::contextPool.getVindex(meshId);
        for (int i = 0; i < nAlloc; ++i) {
            MCGAL::Vertex* v = MCGAL::contextPool.getVertexByIndexInSubPool(meshId, i);
            MCGAL::PointInt& r1Slot =
                MCGAL::contextPool.property<MCGAL::PointInt>(
                    meshId, MCGAL::kVPropRDR1, v);
            r1Slot = MCGAL::PointInt(0, 0, 0);
        }
    }

    std::queue<int> localGateQueue;
    if (seed != nullptr) {
        localGateQueue.push(seed->poolId());
    }
    int current_version = MCGAL::BFSVersionManager::current_version++;

    int bfsFaceIdx = 0;
    int polyFacesThisRound = 0;
    int totalDiagonalsPushed = 0;

    int expectedFaceBits = (roundIdx < (int)faceBitsPerRound_.size())
                               ? faceBitsPerRound_[roundIdx]
                               : -1;


    while (!localGateQueue.empty()) {
        int hid = localGateQueue.front();
        localGateQueue.pop();
        MCGAL::Halfedge* h =
            MCGAL::contextPool.getHalfedgeByIndexInSubPool(DEFAULT_MESH_ID, hid);
        if (h == nullptr || h->isVisited(current_version) || h->isRemoved()) continue;
        h->setVisited(current_version);

        MCGAL::Facet* f = h->face();
        bool faceFirstVisit = (f != nullptr) && !f->isVisited(current_version);
        if (faceFirstVisit) {
            f->setVisited(current_version);

            // Phase 2 Step 3: face 第一次访问时 normalize proxyHalfedge。
            // encoder collect 在 BFS 之前对 mesh 所有 face normalize 一次；
            // decoder 这里 lazy normalize（只对 BFS 实际走到的 face）—— 等价。
            // 用量化整数坐标字典序（bboxMin_/f_quantStep_ 来自基类
            // SymbolReadOperator::enableQuantization；ColumnarDeserializeOperator
            // 在 deserializeOneRound 入口已设好）。
            f->normalize_proxy_to_min_position_vertex(bboxMin_, f_quantStep_);

            if (rcDecodedIsPoly_.empty()) {
                std::cerr << "[PCLTTMTag sv=3 decoder] isPoly underflow round=" << roundIdx
                          << " face=" << bfsFaceIdx << std::endl;
                break;
            }
            uint8_t isPolyBit = rcDecodedIsPoly_.front();
            rcDecodedIsPoly_.pop_front();


            if (isPolyBit) {
                if (rcDecodedValCodes_.empty()) {
                    std::cerr << "[PCLTTMTag sv=3 decoder] valence underflow round=" << roundIdx << std::endl;
                    break;
                }
                uint8_t code = rcDecodedValCodes_.front();
                rcDecodedValCodes_.pop_front();
                int valence;
                if (code == (uint8_t)RDValenceCode::VAL_OTHER) {
                    if (rcDecodedOtherVals_.empty()) {
                        std::cerr << "[PCLTTMTag sv=3 decoder] V_OTHER underflow" << std::endl;
                        break;
                    }
                    valence = (int)rcDecodedOtherVals_.front();
                    rcDecodedOtherVals_.pop_front();
                } else {
                    valence = 3 + (int)code;
                }
                if (rcDecodedResiduals_.empty()) {
                    std::cerr << "[PCLTTMTag sv=3 decoder] residual underflow round=" << roundIdx << std::endl;
                    break;
                }
                decodedFacetQueue_.push_back(f);
                MCGAL::PointInt encoded = rcDecodedResiduals_.front();
                rcDecodedResiduals_.pop_front();

                // Phase 3 路径 A：从 encoded 还原 r1（一阶 raw residual）。
                //   pred_c2 = mean(neighbors' kVPropRDR1)；
                //   r1 = encoded + pred_c2；
                //   写邻居 r1 ← 必须在后续 BFS face 处理之前完成（与 encoder 时序对称）。
                // 注意：此处必须先 `setSplittable() + setPolyValence(valence) + 让 face 的 polygon vertex
                //       结构与 encoder 端 BFS 第 i 个 polyFace 的 polygon vertex 100% 对齐`。
                //       对齐由 Phase 2 已修通的 base_mesh `proxyHalfedge` 量化整数字典序 normalize
                //       + reset_proxy_first 保证。
                MCGAL::PointInt pred_c2 = frenet ? MCGAL::PointInt() : MCGAL::rdCurvaturePrediction(f, DEFAULT_MESH_ID);
                if (!frenet && rdtagFrenetRoundEnabled(rcUseFrenetResidual_,
                                            roundIdx,
                                            rcDecodedRoundCount_)) {
                    if (rdtagFrenetGateFrameEnabled()) {
                        const pmsf::FrenetFrame frame = pmsf::safeHalfedgeFrame(h);
                        MCGAL::Vector3i inv = pmsf::rotateResidualFromFrenet(
                            MCGAL::Vector3i(encoded.x(), encoded.y(), encoded.z()),
                            frame);
                        encoded = MCGAL::PointInt(inv[0], inv[1], inv[2]);
                    } else {
                        encoded = unprojectResidualFromFrenet(
                            encoded, f, &bboxMin_, f_quantStep_);
                    }
                    if (rcUseFrenetExactResidual_) {
                        if (rcDecodedFrenetCorrections_.empty()) {
                            std::cerr << "[PCLTTMTag sv=3 decoder] frenet correction underflow round="
                                      << roundIdx << std::endl;
                            break;
                        }
                        MCGAL::PointInt correction = rcDecodedFrenetCorrections_.front();
                        rcDecodedFrenetCorrections_.pop_front();
                        encoded = MCGAL::PointInt(encoded.x() + correction.x(),
                                                  encoded.y() + correction.y(),
                                                  encoded.z() + correction.z());
                    }
                }
                MCGAL::PointInt residual(encoded.x() + pred_c2.x(),
                                         encoded.y() + pred_c2.y(),
                                         encoded.z() + pred_c2.z());

                markFacetSplittableForDecodedPoly(f);
                f->setPolyValence(valence);
                f->setRemovedVertexPosInt(residual);
                facets.push_back(f);
                polyFacesThisRound++;

                // 写邻居 r1（与 encoder 端写邻居 r1 严格同序、同 vertex set）。
                if (!frenet) {
                    auto verts = MCGAL::rdCollectPolygonVerts(f);
                    for (MCGAL::Vertex* vn : verts) {
                        if (vn == nullptr) continue;
                        MCGAL::PointInt& r1Slot =
                            MCGAL::contextPool.property<MCGAL::PointInt>(
                                DEFAULT_MESH_ID, MCGAL::kVPropRDR1, vn);
                        r1Slot = residual;
                    }
                }

                // ---- Phase 2 Step 3: normalize polyFace.proxyHalfedge ----
                f->normalize_proxy_to_min_position_vertex(bboxMin_, f_quantStep_);

                uint8_t tagSym = 0;  // 0 = Untagged, 1 = Plus, 2 = Minus
                if (!rcDecodedVertexTags_.empty()) {
                    tagSym = rcDecodedVertexTags_.front();
                    rcDecodedVertexTags_.pop_front();
                }

                // ---- TRACE (decoder polyFace) ----

                // ---- Phase 2 核心：定位 N-3 条对角线 halfedge ----
                if (valence >= 4) {
                    if (usePclttmTagTableTriangulation()) {
                        const auto patchFaces =
                            collectPatchFacesAcrossAddedEdges(f, valence);
                        const int before = static_cast<int>(halfedge.size());
                        const int pushed = appendPatchInternalDiagonals(patchFaces, halfedge);
                        totalDiagonalsPushed += pushed;
                    } else {
                        MCGAL::Halfedge* proxyH = f->proxyHalfedge();
                        if (proxyH && proxyH->next() && proxyH->next()->next()) {
                            MCGAL::Halfedge* hd = proxyH->next()->next();  // 第 1 条对角线 v2→v0
                            halfedge.push_back(hd);
                            totalDiagonalsPushed++;


                            MCGAL::Halfedge* hd_cur = hd;
                            for (int i = 1; i <= valence - 4; ++i) {
                                if (!hd_cur->opposite()) break;
                                MCGAL::Halfedge* hd_op = hd_cur->opposite();   // v_0 → v_{i+1}
                                MCGAL::Halfedge* h0_fan = hd_op->next();
                                if (!h0_fan || !h0_fan->next()) break;
                                MCGAL::Halfedge* hd_next = h0_fan->next();     // v_{i+2} → v_0
                                halfedge.push_back(hd_next);
                                totalDiagonalsPushed++;


                                hd_cur = hd_next;
                            }
                        }
                    }
                }
            } else {
                // NotPoly：只 trace face entry vertex
            }
            bfsFaceIdx++;
        }

        // BFS neighbor enqueue (严格对称 RDGateOnly v3)
        MCGAL::Halfedge* hIt = h;
        do {
            MCGAL::Halfedge* hOpp = hIt->opposite();
            if (hOpp != nullptr && !hOpp->isVisited(current_version)) {
                localGateQueue.push(hOpp->poolId());
            }
            if (hIt != nullptr && !hIt->isVisited(current_version)) {
                localGateQueue.push(hIt->poolId());
            }
            hIt = hIt->next();
        } while (hIt != h && hIt != nullptr);
    }

    int tailNotPolyConsumed = 0;
    int tailPolySkipped = 0;
    while (expectedFaceBits >= 0 && bfsFaceIdx < expectedFaceBits &&
           !rcDecodedIsPoly_.empty()) {
        uint8_t isPolyBit = rcDecodedIsPoly_.front();
        rcDecodedIsPoly_.pop_front();
        ++bfsFaceIdx;
        if (!isPolyBit) {
            ++tailNotPolyConsumed;
            continue;
        }
        ++tailPolySkipped;
        if (!rcDecodedValCodes_.empty()) {
            uint8_t code = rcDecodedValCodes_.front();
            rcDecodedValCodes_.pop_front();
            if (code == (uint8_t)RDValenceCode::VAL_OTHER &&
                !rcDecodedOtherVals_.empty()) {
                rcDecodedOtherVals_.pop_front();
            }
        }
        if (!rcDecodedResiduals_.empty()) {
            rcDecodedResiduals_.pop_front();
        }
        if (!rcDecodedVertexTags_.empty()) {
            rcDecodedVertexTags_.pop_front();
        }
    }

    if ((tailNotPolyConsumed > 0 || tailPolySkipped > 0) && std::getenv("PMSF_PROFILE")) {
        std::cerr << "[PCLTTMTag sv=3 decoder] tail_align round=" << roundIdx
                  << " notPoly=" << tailNotPolyConsumed
                  << " polySkipped=" << tailPolySkipped << std::endl;
    }

    if (expectedFaceBits >= 0 && bfsFaceIdx != expectedFaceBits) {
        std::cerr << "[PCLTTMTag sv=3 decoder] WARN round=" << roundIdx
                  << " bfsFaceIdx=" << bfsFaceIdx
                  << " expectedFaceBits=" << expectedFaceBits << std::endl;
    }

    currentRoundIdx_++;
}
