// SPDX-License-Identifier: Apache-2.0
//
// APTMConquestGeometryHook.h — APTM P3-P2（Arch-2 spike 的 S1/S3）几何 hook。
//
// 接缝：conquest 引擎的 **H4 `ResidualQuantizer`**
// （`src/scheme/EngineHooks.h`）。引擎在 `recordGeometry()` 里算
//   delta = toCell(src) - toCell(pred)，然后调 `transform(delta, out, PatchView)`；
// 解码端 `decodeLayer()` 用**同一个 PatchView**（同一组邻居、同一环序 ringNeighbors）
// 调 `inverseTransform(stored, d, pv)`，再 `cell = toCell(pred)+d`。
// ⇒ 在这里插 Frenet+polar θ/φ 是**唯一需要改的两处对称函数**，不需要动引擎/流格式。
//
// 三种模式（`PMSF_APTM_CONQUEST_GEOM`）：
//   native（默认，不注入 hook）：原始 AD 恒等变换 —— P3-P1 的 G 16.5 bpv。
//   lossy（S1 实验）：把 delta 分量量化到 2^k 网格（`PMSF_APTM_CONQUEST_LOSSY_BITS=k`）。
//     用于证明「conquest 拓扑重放与坐标无关」：有损几何不会毁 counts/topology，
//     只会让 surface MSE 变大。
//   polar（S3 / 论文 §4.2）：
//     1) 由 PatchView 的环邻居位置构造局部 Frenet 系
//        （n = 环的 Newell 法向；t1 = 最长环边投影到切平面；t2 = n×t1）；
//     2) 把 cell-delta 投到 (n,t1,t2) 得 (c_n,c_t,c_b)；
//     3) polar 分解 R=|c|、θ=acos(c_n/R) ∈[0,π]、φ=atan2(c_b,c_t) ∈[0,2π)，
//        **θ/φ 各量化到 levels 级（默认 8 = 3 bit，论文 §4.2）**，R 保全精度；
//     4) 存 (R,θq,φq)；解码端 inverseTransform 展开回 (c_n',c_t',c_b') 再投回
//        Cartesian cell-delta。
//
// 对称性说明（重要，写进报告）：
//   - frame 的**构造公式/输入口径**双端一致（同一 PatchView 生成函数、同一环序）；
//   - 但 frame 的**数值**依赖邻居位置：encoder = 原网格（吸附后）位置，
//     decoder = 重建位置。polar 有损 ⇒ 解码位置 ≠ encoder 位置 ⇒ frame 漂移
//     ⇒ 后续预测误差累积（surface MSE 上升）。这**不会**破坏拓扑（conquest 的
//     decodeLayer 只消费符号流），是 Arch-2（编码端 decode 模拟）要修的几何问题。
//
// 兼容性：默认 native ⇒ 不注入 hook ⇒ 引擎走原路径，P3-P1 产物逐字节不变。

#ifndef PMSF_APTM_HOOKS_APTM_CONQUEST_GEOMETRY_HOOK_H
#define PMSF_APTM_HOOKS_APTM_CONQUEST_GEOMETRY_HOOK_H

#include "Conquest/hooks/AnchorVetoPredicate.h"
#include "scheme/EngineHooks.h"

#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <memory>
#include <string>
#include <unordered_set>

namespace pmsf {
namespace aptm_conquest {

enum class GeometryMode {
    Native = 0,  ///< 恒等（P3-P1 原路径；不注入 hook）
    Lossy = 1,   ///< S1：delta 量化到 2^k 网格（有损，证明拓扑鲁棒）
    Polar = 2,   ///< 论文 §4.2：Frenet + θ/φ 三级量化（原型）
};

inline GeometryMode resolveGeometryMode() {
    const char* v = std::getenv("PMSF_APTM_CONQUEST_GEOM");
    if (!v || !*v || std::string(v) == "native") return GeometryMode::Native;
    if (std::string(v) == "lossy") return GeometryMode::Lossy;
    if (std::string(v) == "polar") return GeometryMode::Polar;
    return GeometryMode::Native;
}

inline int resolvePolarLevels() {
    const char* v = std::getenv("PMSF_APTM_CONQUEST_POLAR_LEVELS");
    if (!v || !*v) return 8;  // 论文 §4.2：θ/φ 各 3 bit
    const int n = std::atoi(v);
    if (n < 2 || n > (1 << 20)) return 8;
    return n;
}

inline int resolveLossyBits() {
    const char* v = std::getenv("PMSF_APTM_CONQUEST_LOSSY_BITS");
    if (!v || !*v) return 1;
    const int n = std::atoi(v);
    if (n < 0 || n > 12) return 1;
    return n;
}

inline int resolveArch2Iters() {
    const char* v = std::getenv("PMSF_APTM_CONQUEST_ARCH2_ITERS");
    if (!v || !*v) return 0;
    const int n = std::atoi(v);
    if (n < 0 || n > 8) return 0;
    return n;
}

/// 由一圈**有序**邻居位置构造局部 Frenet 系（双端同一公式）。
/// 环序来源：`pmsf::ringNeighbors`（先 cw 后 ccw），encoder/decoder 同一条函数。
struct ConqFrame {
    double n[3] = {0.0, 0.0, 1.0};
    double t1[3] = {1.0, 0.0, 0.0};
    double t2[3] = {0.0, 1.0, 0.0};
    bool valid = false;
};

inline ConqFrame frameFromPatch(const PatchView& p) {
    ConqFrame f;
    const int N = p.neighborCount;
    if (N < 3 || p.neighborPos == nullptr) return f;
    const double* P = p.neighborPos;
    double c[3] = {0.0, 0.0, 0.0};
    for (int i = 0; i < N; ++i) {
        c[0] += P[3 * i + 0];
        c[1] += P[3 * i + 1];
        c[2] += P[3 * i + 2];
    }
    c[0] /= N;
    c[1] /= N;
    c[2] /= N;

    // Newell 法向：Σ cross(p_i - c, p_{i+1} - c)。
    double nn[3] = {0.0, 0.0, 0.0};
    for (int i = 0; i < N; ++i) {
        const int j = (i + 1) % N;
        const double ax = P[3 * i + 0] - c[0];
        const double ay = P[3 * i + 1] - c[1];
        const double az = P[3 * i + 2] - c[2];
        const double bx = P[3 * j + 0] - c[0];
        const double by = P[3 * j + 1] - c[1];
        const double bz = P[3 * j + 2] - c[2];
        nn[0] += ay * bz - az * by;
        nn[1] += az * bx - ax * bz;
        nn[2] += ax * by - ay * bx;
    }
    const double nl = std::sqrt(nn[0] * nn[0] + nn[1] * nn[1] + nn[2] * nn[2]);
    if (nl < 1e-15) return f;
    f.n[0] = nn[0] / nl;
    f.n[1] = nn[1] / nl;
    f.n[2] = nn[2] / nl;

    // t1：取**最长环边**投影到切平面（确定性：长度最大者优先，平局取先出现）。
    double best[3] = {0.0, 0.0, 0.0};
    double bestLen2 = 0.0;
    for (int i = 0; i < N; ++i) {
        const int j = (i + 1) % N;
        double e[3] = {P[3 * j + 0] - P[3 * i + 0],
                       P[3 * j + 1] - P[3 * i + 1],
                       P[3 * j + 2] - P[3 * i + 2]};
        const double dotn = e[0] * f.n[0] + e[1] * f.n[1] + e[2] * f.n[2];
        e[0] -= dotn * f.n[0];
        e[1] -= dotn * f.n[1];
        e[2] -= dotn * f.n[2];
        const double l2 = e[0] * e[0] + e[1] * e[1] + e[2] * e[2];
        if (l2 > bestLen2) {
            bestLen2 = l2;
            best[0] = e[0];
            best[1] = e[1];
            best[2] = e[2];
        }
    }
    if (bestLen2 < 1e-24) {
        // 退化：用与 n 最不平行的坐标轴（确定性）。
        double axis[3] = {0.0, 0.0, 0.0};
        const double ax = std::fabs(f.n[0]), ay = std::fabs(f.n[1]),
                     az = std::fabs(f.n[2]);
        if (ax <= ay && ax <= az) axis[0] = 1.0;
        else if (ay <= az) axis[1] = 1.0;
        else axis[2] = 1.0;
        const double dn = axis[0] * f.n[0] + axis[1] * f.n[1] + axis[2] * f.n[2];
        best[0] = axis[0] - dn * f.n[0];
        best[1] = axis[1] - dn * f.n[1];
        best[2] = axis[2] - dn * f.n[2];
        bestLen2 = best[0] * best[0] + best[1] * best[1] + best[2] * best[2];
        if (bestLen2 < 1e-24) return f;
    }
    const double bl = std::sqrt(bestLen2);
    f.t1[0] = best[0] / bl;
    f.t1[1] = best[1] / bl;
    f.t1[2] = best[2] / bl;
    f.t2[0] = f.n[1] * f.t1[2] - f.n[2] * f.t1[1];
    f.t2[1] = f.n[2] * f.t1[0] - f.n[0] * f.t1[2];
    f.t2[2] = f.n[0] * f.t1[1] - f.n[1] * f.t1[0];
    f.valid = true;
    return f;
}

/// H4 `ResidualQuantizer` 的 APTM 几何实现（encoder transform / decoder inverse）。
class APTMConquestResidualQuantizer final : public ResidualQuantizer {
  public:
    explicit APTMConquestResidualQuantizer(GeometryMode mode)
        : mode_(mode),
          levels_(resolvePolarLevels()),
          lossyBits_(resolveLossyBits()) {}

    GeometryMode mode() const { return mode_; }
    int levels() const { return levels_; }

    void transform(const int in[3], int out[3], const PatchView& p) const override {
        ++count_;
        if (mode_ == GeometryMode::Native) {
            out[0] = in[0];
            out[1] = in[1];
            out[2] = in[2];
            return;
        }
        if (mode_ == GeometryMode::Lossy) {
            for (int k = 0; k < 3; ++k) out[k] = quantShift(in[k]);
            return;
        }
        const ConqFrame f = frameFromPatch(p);
        if (!f.valid) {  // 退化：恒等（双端同一判据；见头文件"对称性说明"）
            out[0] = in[0];
            out[1] = in[1];
            out[2] = in[2];
            ++frameFallback_;
            return;
        }
        const double cn = in[0] * f.n[0] + in[1] * f.n[1] + in[2] * f.n[2];
        const double ct = in[0] * f.t1[0] + in[1] * f.t1[1] + in[2] * f.t1[2];
        const double cb = in[0] * f.t2[0] + in[1] * f.t2[1] + in[2] * f.t2[2];
        const double r = std::sqrt(cn * cn + ct * ct + cb * cb);
        if (r < 1e-9) {
            out[0] = out[1] = out[2] = 0;
            ++zeroResidual_;
            return;
        }
        const int R = static_cast<int>(std::llround(r));
        double cosT = cn / r;
        if (cosT > 1.0) cosT = 1.0;
        if (cosT < -1.0) cosT = -1.0;
        const double theta = std::acos(cosT);  // [0, π]
        double phi = std::atan2(cb, ct);       // (-π, π]
        if (phi < 0.0) phi += 2.0 * M_PI;      // [0, 2π)
        int tq = static_cast<int>(
            std::llround(theta / M_PI * static_cast<double>(levels_ - 1)));
        int pq = static_cast<int>(std::llround(
            phi / (2.0 * M_PI) * static_cast<double>(levels_)));
        if (tq < 0) tq = 0;
        if (tq > levels_ - 1) tq = levels_ - 1;
        if (pq >= levels_) pq = 0;
        out[0] = R;
        out[1] = tq;
        out[2] = pq;
        ++polarCount_;
        if (R > rMax_) rMax_ = R;
        // 直方图只有 8 桶（论文 3-bit 口径的默认统计）；精度扫描（levels>8）时
        // 高 bin 只计数不入桶，避免越界（纯诊断，不影响码流）。
        if (tq >= 0 && tq < 8) thetaHist_[tq]++;
        if (pq >= 0 && pq < 8) phiHist_[pq]++;
    }

    void inverseTransform(const int in[3], int out[3], const PatchView& p) const override {
        if (mode_ == GeometryMode::Native || mode_ == GeometryMode::Lossy) {
            // lossy：存储值就是量化后的 delta（不需要逆变换）。
            out[0] = in[0];
            out[1] = in[1];
            out[2] = in[2];
            return;
        }
        const ConqFrame f = frameFromPatch(p);
        if (!f.valid) {
            out[0] = in[0];
            out[1] = in[1];
            out[2] = in[2];
            return;
        }
        const double R = static_cast<double>(in[0]);
        const double theta =
            (static_cast<double>(in[1]) / static_cast<double>(levels_ - 1)) *
            M_PI;
        const double phi =
            (static_cast<double>(in[2]) / static_cast<double>(levels_)) *
            2.0 * M_PI;
        const double cn = R * std::cos(theta);
        const double rxy = R * std::sin(theta);
        const double ct = rxy * std::cos(phi);
        const double cb = rxy * std::sin(phi);
        out[0] = static_cast<int>(std::llround(cn * f.n[0] + ct * f.t1[0] +
                                               cb * f.t2[0]));
        out[1] = static_cast<int>(std::llround(cn * f.n[1] + ct * f.t1[1] +
                                               cb * f.t2[1]));
        out[2] = static_cast<int>(std::llround(cn * f.n[2] + ct * f.t1[2] +
                                               cb * f.t2[2]));
    }

    // ---- 诊断 ----
    long long count() const { return count_; }
    long long polarCount() const { return polarCount_; }
    long long zeroResidual() const { return zeroResidual_; }
    long long frameFallback() const { return frameFallback_; }
    int rMax() const { return rMax_; }
    const long long* thetaHist() const { return thetaHist_; }
    const long long* phiHist() const { return phiHist_; }
    long long histSize() const { return 8; }  // 按 8 级固定 8 桶统计

  private:
    int quantShift(int v) const {
        if (lossyBits_ <= 0) return v;
        const int shift = lossyBits_;
        if (v >= 0) return ((v + (1 << (shift - 1))) >> shift) << shift;
        return -((((-v) + (1 << (shift - 1))) >> shift) << shift);
    }

    GeometryMode mode_;
    int levels_;
    int lossyBits_;
    mutable long long count_ = 0;
    mutable long long polarCount_ = 0;
    mutable long long zeroResidual_ = 0;
    mutable long long frameFallback_ = 0;
    mutable int rMax_ = 0;
    mutable long long thetaHist_[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    mutable long long phiHist_[8] = {0, 0, 0, 0, 0, 0, 0, 0};
};

/// H3 包装：**录制**每次删点的中心 poolId（编码端 pass1 用；不改变预测值）。
/// 用于 Arch-2：把 layers 的删除顺序与 target cell 对齐。
class APTMRecordingPredictor final : public GeometryPredictor {
  public:
    std::function<void(std::uint32_t centerId)> onRemoved;
    void predict(const PatchView& p, double out[3]) const override {
        GeometryPredictor::predict(p, out);  // 默认 = 邻居算术平均（与解码端一致）
        if (onRemoved) onRemoved(p.centerId);
    }
};

/// H3 包装：**录制**每次插点的 PatchView（环位置）与 pred（解码端模拟用）。
/// 用于 Arch-2：在解码模拟里拿到每个被插顶点的环/预测，从而重算 delta。
struct APTMInsertRecord {
    std::vector<double> ringPos;            ///< 3 * count（解码端当时的环位置）
    std::vector<std::uint32_t> ringIds;     ///< count（解码端池号；Arch-2 用它查位置表）
    int count = 0;
    double pred[3] = {0.0, 0.0, 0.0};
};

class APTMCapturingPredictor final : public GeometryPredictor {
  public:
    std::vector<APTMInsertRecord>* out = nullptr;
    void predict(const PatchView& p, double outPred[3]) const override {
        GeometryPredictor::predict(p, outPred);
        if (out == nullptr) return;
        APTMInsertRecord rec;
        rec.count = p.neighborCount;
        rec.ringPos.assign(p.neighborPos, p.neighborPos + 3 * p.neighborCount);
        if (p.neighborIds != nullptr) {
            rec.ringIds.assign(p.neighborIds, p.neighborIds + p.neighborCount);
        }
        rec.pred[0] = outPred[0];
        rec.pred[1] = outPred[1];
        rec.pred[2] = outPred[2];
        out->push_back(std::move(rec));
    }
};

/// 构造 engine hooks：默认 + AnchorVeto（可选）+ APTM 几何 hook（可选）。
/// `outQuant` 回传几何 hook 实例（供诊断/统计；可为 nullptr）。
inline EngineHooksPtr makeAPTMConquestHooks(
    const std::unordered_set<std::uint32_t>* anchors,
    GeometryMode mode,
    std::shared_ptr<APTMConquestResidualQuantizer>* outQuant = nullptr,
    std::shared_ptr<GeometryPredictor> extraPredictor = nullptr) {
    auto hooks = std::make_shared<EngineHooks>();
    if (anchors != nullptr && !anchors->empty()) {
        hooks->decimation = std::make_shared<AnchorVetoPredicate>(*anchors);
    }
    if (extraPredictor != nullptr) {
        hooks->geometryPredictor = std::move(extraPredictor);
    }
    if (mode != GeometryMode::Native) {
        auto quant = std::make_shared<APTMConquestResidualQuantizer>(mode);
        hooks->residualQuantizer = quant;
        if (outQuant != nullptr) *outQuant = quant;
    } else if (outQuant != nullptr) {
        outQuant->reset();
    }
    return hooks;
}

}  // namespace aptm_conquest
}  // namespace pmsf

#endif  // PMSF_APTM_HOOKS_APTM_CONQUEST_GEOMETRY_HOOK_H
