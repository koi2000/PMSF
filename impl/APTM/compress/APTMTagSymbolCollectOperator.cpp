// SPDX-License-Identifier: Apache-2.0
//
// APTMTagSymbolCollectOperator.cpp — APTM P1 geometry coder prototype.
//
// 父类（PCLTTMTagSymbolCollectOperator）已经实现了论文 base algorithm（Alliez &
// Desbrun 2001）的 A&D/tag 连通性与 Cartesian residual。本文件的全部工作是
// **几何编码域**（论文 §4.1/§4.2），三档由 `PMSF_APTM_TAG_GEOM` 选择：
//
//   cartesian  父类原样（P1 阶段一基线，用于量化「连通性换血」带来的收益）
//   frenet     父类内置的**可逆整数 lifting** Frenet 局部系（RD PR-FRENET-LIFT
//              同一份实现，env `PMSF_RDTAG_FRENET_RESIDUAL=1` 触发）
//   polar      Frenet 分量再量化成 (R, θq, φq)：θ/φ 各 3-bit（论文 §4.2）。
//              **有损**：论文自己的 3-bit tangential 量化同样有损（§4.2 原文
//              "a different quantization for different components"），因此质量
//              口径必须用论文自己的 Metro RMS/bbox 指标，而不是无损 coord 相等。
//
// 0) 为什么能复用父类的 Frenet 实现而不改 impl/PCLTTM
//    - 父类 collect() 在 env 下把 residual queue 从 Cartesian (r1-pred_c2) 变成
//      可逆整数 lifting 后的 Frenet 分量 (c_n,c_t,c_b)；同时它用**同一个** lift
//      的逆把 encoder 的 r1 状态更新成 decoder 将来会写回的值（源码注释
//      "r1ForState = approxEncoded + pred_c2"）⇒ 内轮编解码对称。
//    - 父类 export() 对 BlobLayout=16 有一条保护性 throw（"stores Cartesian
//      residuals only"）。本类在 export 里临时摘掉那两个 env（先保存后恢复），
//      让父类把 lift 后的分量按 bit-plane 写出，再把容器 scheme_version 从 3
//      改成 APTM 自己的 64/65 ⇒ 子类 reader 可识别；父类/其它算法零改动。
//    - decoder 侧父类 collectFromDecoded 已有 Frenet 分支（`rcUseFrenetResidual_`
//      `rdtagFrenetRoundEnabled(...)` → `unprojectResidualFromFrenet(...)`，即
//      lift 的整数逆）。所以 reader 只需要：patch sv 回 3 → 调父类 import →
//      置 rcUseFrenetResidual_=true（polar 时再把 (R,θq,φq) 展开成 Frenet 分量）。
//
// 1) polar 模式的「内轮对称」处理
//    polar 量化是有损的，若只把量化值塞回队列，encoder 写进邻居的 r1 还是
//    **量化前**的值，decoder 写的是量化后的值 ⇒ pred_c2 链从第 2 个 polyFace
//    起分叉、误差逐面累积。本文件在 collect 末尾**重放本轮预测链**：
//      - 清零 r1 槽（与父类 collect 入口同一循环）；
//      - 按 facet 队列顺序（= BFS 顺序 = decoder collectFromDecoded 顺序）
//        对每个 polyFace：pred_c2 = rdCurvaturePrediction(f)；
//        把该面 residual 量化成 (R,θq,φq)；按 decoder 的展开公式算回 Frenet
//        分量 (c_n',c_t',c_b')；再用 **lift 的整数逆**（与 decoder 逐位同源）
//        得到量化后的 Cartesian residual v'；写回邻居 r1 = v' + pred_c2。
//      这样 decoder 在 collectFromDecoded 里走父类原有逻辑时，r1 状态与
//      encoder 重放逐位一致（前提：frame 一致；跨轮的 frame 依赖 decoder 已重建
//      位置 ⇒ 跨轮仍可能有残差，见 P1 报告 §质量口径）。
//
// 2) 帧（frame）构造必须用**量化整点位置**
//    父类 PR-FRENET-FIX 已把 frame 的全部位置源改成
//    `frenetGridPoint()`（floatPosToInt 截断 + intPosToFloat 半格）⇒ encoder 用
//    原始浮点、decoder 用量化重建位置时仍逐位同源。本文件重放用的 frame 必须
//    与父类**同一份数学**，因此下面复制了同一段实现（父类在 impl/PCLTTM 的
//    匿名 namespace 里，不可链接；P2 若做跨轮模拟需把它提到公共头）。

#include "compress/APTMTagSymbolCollectOperator.h"

#include "common/PaperBlobAlgorithmSpecs.h"
#include "compress/ConnectivityBlobContainer.h"
#include "predict/RDPredictor.h"  // kVPropRDR1, rdCurvaturePrediction
#include "storage/PaperBlobLayoutRegistry.h"

#include "ContextPool.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Point.h"
#include "Vertex.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

namespace pmsf {
namespace {

// ============================================================================
// 0) 与父类 PR-FRENET-FIX / PR-FRENET-LIFT 逐位同源的 frame + lift 数学。
//    父类实现位于 impl/PCLTTM/compress/PCLTTMTagSymbolCollectOperator.cpp 的
//    匿名 namespace（链接不可见），故此处复制。两边若日后分叉，P2 应提取公共头。
// ============================================================================

constexpr std::size_t kScratchBytes = 32u << 20;  // parent export 实测 <100KB

/// polar 角度级数（`PMSF_APTM_TAG_POLAR_LEVELS`，默认 8 = 论文 §4.2 的 3-bit）。
/// 用于 P1 的「精度-可解码性」扫描：3-bit 有损 ⇒ decoder 端量化坐标与 encoder
/// 不一致 ⇒ tag sv=3 的「量化坐标字典序 normalize」拓扑推断会选到不同 v0。
/// 扫描把 θ/φ 精度提高到近似无损，验证「有损 vs decode 有效」的边界。
int polarAngleLevels() {
    const char* v = std::getenv("PMSF_APTM_TAG_POLAR_LEVELS");
    if (!v || !*v) return aptm_tag::kPolarAngleLevels;
    const int n = std::atoi(v);
    if (n < 2 || n > (1 << 20)) return aptm_tag::kPolarAngleLevels;
    return n;
}

struct FrenetFrame {
    double nx = 0.0, ny = 0.0, nz = 1.0;
    double tx = 1.0, ty = 0.0, tz = 0.0;
    double bx = 0.0, by = 1.0, bz = 0.0;
};

/// 量化网格位置：与 Mesh::floatPosToInt（截断）再 intPosToFloat（+0.5 半格）同源。
MCGAL::Point frenetGridPoint(MCGAL::Vertex* v, const MCGAL::Point* bboxMin,
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

/// 与 Facet::computeNormal 同源（proxyHalfedge 前 3 顶点叉积 + normalize），
/// 仅位置源改走 frenetGridPoint。
MCGAL::Vector3 frenetFrameNormal(MCGAL::Facet* f, const MCGAL::Point* bboxMin,
                                 float quantStep) {
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

FrenetFrame residualFrenetFrame(MCGAL::Facet* f, const MCGAL::Point* bboxMin,
                                float quantStep) {
    FrenetFrame frame;
    if (!f) return frame;

    MCGAL::Vector3 n = frenetFrameNormal(f, bboxMin, quantStep);
    double nlen = std::sqrt((double)n.x() * n.x() + (double)n.y() * n.y() +
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
    double tlen = std::sqrt(frame.tx * frame.tx + frame.ty * frame.ty +
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
        tlen = std::sqrt(frame.tx * frame.tx + frame.ty * frame.ty +
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

int liftRound(double v) { return static_cast<int>(std::llround(v)); }

/// 2D rotation [[c,-s],[s,c]] 的 3-lift 逆（父类 lift2dInverse 的同一份实现）。
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

void frenetEulerAngles(const FrenetFrame& fr, double& alpha, double& beta,
                       double& gamma) {
    double sBeta = -fr.bx;
    if (sBeta > 1.0) sBeta = 1.0;
    if (sBeta < -1.0) sBeta = -1.0;
    beta = std::asin(sBeta);
    if (std::fabs(std::cos(beta)) > 1e-8) {
        gamma = std::atan2(fr.by, fr.bz);
        alpha = std::atan2(fr.tx, fr.nx);
    } else {
        gamma = 0.0;
        alpha = std::atan2(-fr.ny, fr.ty);
    }
}

/// inverse: r = R^-1·p（父类 liftResidualFromFrenet 的同一份实现）。
MCGAL::PointInt liftResidualFromFrenet(const MCGAL::PointInt& p,
                                       const FrenetFrame& fr) {
    double alpha = 0.0, beta = 0.0, gamma = 0.0;
    frenetEulerAngles(fr, alpha, beta, gamma);
    int x = p.x(), y = p.y(), z = p.z();
    lift2dInverse(x, y, std::cos(alpha), std::sin(alpha));
    lift2dInverse(x, z, std::cos(beta), -std::sin(beta));
    lift2dInverse(y, z, std::cos(gamma), std::sin(gamma));
    return MCGAL::PointInt(x, y, z);
}

// ============================================================================
// 1) polar 量化器（论文 §4.2）
//    θ ∈ [0,180°]，φ ∈ [0,360°)，各 3-bit（8 级）。半径 R 保留全精度（整数量化单位）。
//    encoder/decoder 的展开公式必须逐位一致 —— 见 expandPolarToFrenet。
// ============================================================================

/// (c_n,c_t,c_b) → (R, θq, φq)。返回值三元组塞回 PointInt。
MCGAL::PointInt quantizePolarFrenet(const MCGAL::PointInt& c, int levels) {
    const double cn = c.x();
    const double ct = c.y();
    const double cb = c.z();
    const double r = std::sqrt(cn * cn + ct * ct + cb * cb);
    if (r < 1e-9) {
        return MCGAL::PointInt(0, 0, 0);
    }
    const int radius = static_cast<int>(std::llround(r));
    double cosTheta = cn / r;
    if (cosTheta > 1.0) cosTheta = 1.0;
    if (cosTheta < -1.0) cosTheta = -1.0;
    const double theta = std::acos(cosTheta);  // [0, π]
    double phi = std::atan2(cb, ct);           // (-π, π]
    if (phi < 0.0) phi += 2.0 * M_PI;          // [0, 2π)
    int thetaQ = static_cast<int>(
        std::llround(theta / M_PI * static_cast<double>(levels - 1)));
    int phiQ = static_cast<int>(
        std::llround(phi / (2.0 * M_PI) * static_cast<double>(levels)));
    thetaQ = std::clamp(thetaQ, 0, levels - 1);
    if (phiQ >= levels) phiQ = 0;  // 360° ≡ 0°
    return MCGAL::PointInt(radius, thetaQ, phiQ);
}

/// (R, θq, φq) → Frenet 分量 (c_n', c_t', c_b')。decoder 侧逐位同一公式。
MCGAL::PointInt expandPolarToFrenet(const MCGAL::PointInt& p, int levels) {
    const double radius = static_cast<double>(p.x());
    if (radius <= 0.0) return MCGAL::PointInt(0, 0, 0);
    const double theta =
        (static_cast<double>(p.y()) / static_cast<double>(levels - 1)) * M_PI;
    const double phi =
        (static_cast<double>(p.z()) / static_cast<double>(levels)) * 2.0 * M_PI;
    const double cn = radius * std::cos(theta);
    const double rxy = radius * std::sin(theta);
    const double ct = rxy * std::cos(phi);
    const double cb = rxy * std::sin(phi);
    return MCGAL::PointInt(static_cast<int>(std::llround(cn)),
                           static_cast<int>(std::llround(ct)),
                           static_cast<int>(std::llround(cb)));
}

std::string getEnvOrEmpty(const char* name) {
    const char* v = std::getenv(name);
    return v ? std::string(v) : std::string();
}

void restoreEnv(const char* name, const std::string& value) {
    if (value.empty()) {
        unsetenv(name);
    } else {
        setenv(name, value.c_str(), 1);
    }
}

/// 临时摘掉 RD 的 Frenet env：父类 export 对 BlobLayout=16 会拒绝写 Frenet 流。
struct ScopedRdFrenetEnvOff {
    std::string residual = getEnvOrEmpty("PMSF_RDTAG_FRENET_RESIDUAL");
    std::string exact = getEnvOrEmpty("PMSF_RDTAG_FRENET_EXACT");
    ScopedRdFrenetEnvOff() {
        unsetenv("PMSF_RDTAG_FRENET_RESIDUAL");
        unsetenv("PMSF_RDTAG_FRENET_EXACT");
    }
    ~ScopedRdFrenetEnvOff() {
        restoreEnv("PMSF_RDTAG_FRENET_RESIDUAL", residual);
        restoreEnv("PMSF_RDTAG_FRENET_EXACT", exact);
    }
};

/// 清零本轮 r1 槽（与父类 collect 入口的循环逐项相同）。
void clearRdR1Slots(MCGAL::Mesh* mesh) {
    const int meshId = mesh->meshId();
    const int nAlloc = MCGAL::contextPool.getVindex(meshId);
    for (int i = 0; i < nAlloc; ++i) {
        MCGAL::Vertex* v =
            MCGAL::contextPool.getVertexByIndexInSubPool(meshId, i);
        MCGAL::contextPool.property<MCGAL::PointInt>(
            meshId, MCGAL::kVPropRDR1, v) = MCGAL::PointInt(0, 0, 0);
    }
}

/// 把 r1 值写回 polyFace 的 polygon 1-环（与 decoder `rdCollectPolygonVerts`
/// 同一组顶点；encoder 端用 proxyHalfedge 走一周）。
void writeR1Ring(MCGAL::Facet* f, const MCGAL::PointInt& value, int meshId) {
    if (f == nullptr) return;
    MCGAL::Halfedge* start = f->proxyHalfedge();
    if (start == nullptr) return;
    MCGAL::Halfedge* hit = start;
    do {
        MCGAL::Vertex* vn = hit ? hit->vertex() : nullptr;
        if (vn != nullptr) {
            MCGAL::contextPool.property<MCGAL::PointInt>(
                meshId, MCGAL::kVPropRDR1, vn) = value;
        }
        hit = hit ? hit->next() : nullptr;
    } while (hit && hit != start);
}

}  // namespace

// ============================================================================
// P2-pre 诊断：valence 分布 + v0 side-stream 代价
// ============================================================================
void APTMTagSymbolCollectOperator::emitValenceCostDiag() {
    if (gateValenceQueues_.empty()) return;
    long long nFaces = 0;
    long long cnt[5] = {0, 0, 0, 0, 0};  // V3,V4,V5,V6,V_OTHER
    double v0Bits = 0.0;
    for (std::size_t r = 0; r < gateValenceQueues_.size(); ++r) {
        std::size_t oi = 0;
        for (uint8_t code : gateValenceQueues_[r]) {
            int val = 0;
            if (code == 4) {  // RDValenceCode::VAL_OTHER
                if (oi < gateOtherValQueues_[r].size()) {
                    val = static_cast<int>(gateOtherValQueues_[r][oi++]);
                } else {
                    val = 3;
                }
            } else {
                val = 3 + static_cast<int>(code);
            }
            if (val >= 3 && val <= 6) {
                ++cnt[val - 3];
            } else {
                ++cnt[4];
            }
            ++nFaces;
            v0Bits += std::log2(static_cast<double>(std::max(val, 3)));
        }
    }
    const double bytes = v0Bits / 8.0;
    const long long nV3 = cnt[0];
    std::fprintf(stderr,
                 "[APTM-Tag valence] polyFaces=%lld V3=%lld(%.1f%%) V4=%lld "
                 "V5=%lld V6=%lld Vother=%lld v0IndexBits=%.0f "
                 "v0StreamBytes=%.0f v0StreamBpv=%.3f\n",
                 nFaces, nV3,
                 nFaces ? 100.0 * static_cast<double>(nV3) /
                              static_cast<double>(nFaces)
                        : 0.0,
                 cnt[1], cnt[2], cnt[3], cnt[4], v0Bits, bytes,
                 bytes * 8.0 / 19851.0);
    std::fflush(stderr);
}

// ============================================================================
// collect —— 父类 collect + （polar 模式）几何域变换 + 内轮预测链重放
// ============================================================================
void APTMTagSymbolCollectOperator::collect(
    MCGAL::Halfedge* seed, const std::vector<MCGAL::Halfedge*>& gates) {
    const aptm_tag::GeometryMode mode = aptm_tag::resolveGeometryMode();
    if (mode != aptm_tag::GeometryMode::Cartesian) {
        // 触发父类内置的可逆整数 lifting Frenet 变换（PCLTTM Tag collector）。
        // 强制覆盖：本路径的几何模式由 PMSF_APTM_TAG_GEOM 唯一决定，避免
        // 用户残留的 PMSF_RDTAG_FRENET_* 把协议弄成半开状态。
        setenv("PMSF_RDTAG_FRENET_RESIDUAL", "1", 1);
        unsetenv("PMSF_RDTAG_FRENET_EXACT");
        unsetenv("PMSF_RDTAG_FRENET_GATE_FRAME");
        unsetenv("PMSF_RDTAG_FRENET_START");
        unsetenv("PMSF_RDTAG_FRENET_END");
    }
    PCLTTMTagSymbolCollectOperator::collect(seed, gates);

    if (mode == aptm_tag::GeometryMode::Polar) {
        quantizePolarQueues();
    }
}

// ============================================================================
// quantizePolarQueues —— (c_n,c_t,c_b) → (R,θq,φq) + 重放本轮 r1 预测链
// ============================================================================
void APTMTagSymbolCollectOperator::quantizePolarQueues() {
    if (gateResidualQueues_.empty() || gateFacetQueues_.empty()) return;
    auto& resQ = gateResidualQueues_.back();
    auto& facetQ = gateFacetQueues_.back();
    if (resQ.size() != facetQ.size()) {
        std::fprintf(stderr,
                     "[APTM-Tag polar] residual/facet queue size mismatch "
                     "(%zu vs %zu) — skip replay (encoder state may drift)\n",
                     resQ.size(), facetQ.size());
        const int levels = polarAngleLevels();
        for (auto& r : resQ) r = quantizePolarFrenet(r, levels);
        return;
    }
    if (mesh_ == nullptr) return;
    const int meshId = mesh_->meshId();
    const int levels = polarAngleLevels();

    // ------------------------------------------------------------------
    // Pass 1：还原上一版父类预测链写下的 **原始 r1**。
    //
    // 父类 collect 在 Frenet 分支里把 queue 存成 c = r1 - pred_c2(parent)，
    // 并把 `r1ForState = unproject(lift(c)) + pred_c2 = r1`（lift 严格可逆）
    // 写进邻居槽。因此清零 r1 后按 facet 队列顺序重放同一公式，即可逐位复原
    // 父类写下的 r1 状态与每个 polyFace 的 pred_c2(parent)。
    // ------------------------------------------------------------------
    std::vector<MCGAL::PointInt> r1Original(resQ.size());
    clearRdR1Slots(mesh_.get());
    for (std::size_t k = 0; k < facetQ.size(); ++k) {
        MCGAL::Facet* f = facetQ[k];
        if (f == nullptr) continue;
        const MCGAL::PointInt predParent =
            MCGAL::rdCurvaturePrediction(f, meshId);
        const MCGAL::PointInt& c = resQ[k];
        r1Original[k] = MCGAL::PointInt(c.x() + predParent.x(),
                                        c.y() + predParent.y(),
                                        c.z() + predParent.z());
        writeR1Ring(f, r1Original[k], meshId);
    }

    // ------------------------------------------------------------------
    // Pass 2：用**量化后重建**的残差重放 decoder 预测链：
    //   pred_dec = 邻居槽上的量化后 r1（与 decoder collectFromDecoded 同序）；
    //   c_new   = r1(原始) - pred_dec；  polar = quantize(c_new)；
    //   u'      = lift^-1(expand(polar))（= decoder 会重建出的 Cartesian 残差）；
    //   r1_state= u' + pred_dec（写回邻居 = decoder 将来的状态）。
    // 这样 decoder 端 `residual = u' + pred_dec` 与 encoder 状态逐位一致，
    // 位置误差只剩「量化误差 + 跨轮 barycenter/frame 差」（P2 的模拟解码要修后者）。
    // ------------------------------------------------------------------
    clearRdR1Slots(mesh_.get());
    for (std::size_t k = 0; k < facetQ.size(); ++k) {
        MCGAL::Facet* f = facetQ[k];
        if (f == nullptr) continue;
        const MCGAL::PointInt predDec =
            MCGAL::rdCurvaturePrediction(f, meshId);
        const MCGAL::PointInt& r1 = r1Original[k];
        const MCGAL::PointInt cNew(r1.x() - predDec.x(),
                                   r1.y() - predDec.y(),
                                   r1.z() - predDec.z());
        const MCGAL::PointInt polar = quantizePolarFrenet(cNew, levels);
        resQ[k] = polar;
        const MCGAL::PointInt expanded = expandPolarToFrenet(polar, levels);
        const FrenetFrame frame =
            residualFrenetFrame(f, &mesh_->bboxMin, mesh_->f_quantStep);
        const MCGAL::PointInt cart =
            liftResidualFromFrenet(expanded, frame);
        writeR1Ring(f,
                    MCGAL::PointInt(cart.x() + predDec.x(),
                                    cart.y() + predDec.y(),
                                    cart.z() + predDec.z()),
                    meshId);
    }
}

// ============================================================================
// export —— 父类 3 段流 + 容器 sv 改写成 APTM 自己的 64/65
// ============================================================================
int APTMTagSymbolCollectOperator::exportToBufferWithRangeCoder(
    char* buffer, bool enableQuantization) {
    emitValenceCostDiag();
    const aptm_tag::GeometryMode mode = aptm_tag::resolveGeometryMode();
    if (mode == aptm_tag::GeometryMode::Cartesian) {
        lastSchemeVersion_ = 3;
        return PCLTTMTagSymbolCollectOperator::exportToBufferWithRangeCoder(
            buffer, enableQuantization);
    }

    if (buffer == nullptr) return 0;

    // 父类 export 会拒绝 BlobLayout=16 的 Frenet 流 —— 临时摘掉 env 再调用。
    std::vector<char> scratch(kScratchBytes);
    int bytes = 0;
    {
        ScopedRdFrenetEnvOff envOff;
        bytes = PCLTTMTagSymbolCollectOperator::exportToBufferWithRangeCoder(
            scratch.data(), enableQuantization);
    }
    if (bytes <= 0) {
        throw std::runtime_error(
            "[APTM-Tag] parent export returned no bytes for geometry mode");
    }

    // 容器 header 布局（见 ConnectivityBlobContainer.h）：
    //   [u32 magic][u16 blob_layout][u16 scheme_version][u32 rounds][u32 streams]
    // patch scheme_version（offset 6..8，LE u16）为 APTM 的值，layout 仍是 16。
    const std::uint16_t sv =
        (mode == aptm_tag::GeometryMode::Polar)
            ? aptm_tag::kSchemeVersionPolar
            : aptm_tag::kSchemeVersionFrenet;
    std::memcpy(scratch.data() + 6, &sv, sizeof(sv));
    std::memcpy(buffer, scratch.data(), static_cast<std::size_t>(bytes));

    // ---- 逐分量统计（证据脚本读；与父类 [PCLTTMTag bpv] 同口径） ----
    {
        const std::span<const std::uint8_t> view(
            reinterpret_cast<const std::uint8_t*>(scratch.data()),
            static_cast<std::size_t>(bytes));
        pmsf::ConnectivityBlobReader rd(view);
        const int numRounds = rd.numRounds();
        int valBytes = 0, fusedBytes = 0, resBytes = 0;
        for (int r = 0; r < numRounds; ++r) {
            valBytes += static_cast<int>(
                rd.getStream(r, pmsf::kPaperBlobStreamPCLTTMTagValence).size());
            fusedBytes += static_cast<int>(
                rd.getStream(r, pmsf::kPaperBlobStreamPCLTTMTagFusedFaceSym)
                    .size());
            resBytes += static_cast<int>(
                rd.getStream(r, pmsf::kPaperBlobStreamPCLTTMTagResidual).size());
        }
        lastValenceBytes_ = valBytes;
        lastFusedBytes_ = fusedBytes;
        lastResidualBytes_ = resBytes;
        lastContainerBytes_ = bytes - valBytes - fusedBytes - resBytes;
        lastSchemeVersion_ = sv;
        const long long nVerts =
            (mesh_ != nullptr) ? static_cast<long long>(mesh_->nb_vertices) : 0;
        std::fprintf(stderr,
                     "[APTM-Tag bpv] geom=%s sv=%d rounds=%d valence=%d "
                     "fusedFaceSym=%d residual=%d containerOverhead=%d "
                     "totalBlob=%d verts=%lld\n",
                     (mode == aptm_tag::GeometryMode::Polar) ? "polar"
                                                             : "frenet",
                     static_cast<int>(sv), numRounds, valBytes, fusedBytes,
                     resBytes, lastContainerBytes_, bytes, nVerts);
        if (mode == aptm_tag::GeometryMode::Polar) {
            std::fprintf(stderr, "[APTM-Tag polar] angleLevels=%d\n",
                         polarAngleLevels());
        }
    }
    return bytes;
}

}  // namespace pmsf
