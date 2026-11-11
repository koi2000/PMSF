// SPDX-License-Identifier: Apache-2.0
//
// PCATMZShapeGeometry.h — Single source of truth for PCATM Z-shape geometry
// (path-B, BlobLayout=15, sv=1).
//
// 论文：Cohen-Or, Levin, Remez 2002 *Progressive Compression of Arbitrary
// Triangular Meshes* §4 + Figure 2 (1)(2)(3) 合法范式；(4) mirror Z 非法。
//
// 单一来源契约：
//   - PCATMZShapeEliminateOperator (PR-2) 使用本 helper 计算 retriangulation
//     时新增的 N-3 条对角线 + N-2 个 triangle 的 color；
//   - PCATMZShapeReconstructOperator (PR-3) 使用同一 helper 反演 Z-shape，
//     按 (h_a, h_b) split_facet × (N-3) 序列；
//   - tests/test_pcatm_zshape_geometry.cpp (PR-1) 覆盖 N=4..9 + 反例。
//
// 校验规范由 docs/paper_alignment/pcatm_zshape_groundtruth.md 给出 N=4..9
// 全部 ground truth。

#ifndef SRC_COMMON_PCATM_ZSHAPE_GEOMETRY_H_
#define SRC_COMMON_PCATM_ZSHAPE_GEOMETRY_H_

#include <cstdint>
#include <vector>

namespace pmsf {
namespace pcatm {

struct Diagonal {
    int va;  // index into polygon ring (0..N-1)
    int vb;
    bool operator==(const Diagonal& o) const noexcept {
        return va == o.va && vb == o.vb;
    }
};

struct Triangle {
    int v0;
    int v1;
    int v2;        // index into polygon ring (0..N-1)
    int color;     // 0 or 1
    bool operator==(const Triangle& o) const noexcept {
        return v0 == o.v0 && v1 == o.v1 && v2 == o.v2 && color == o.color;
    }
};

enum class StartDir : std::uint8_t {
    Right = 0,   // canonical paper Z: continue from the opposite end
    Left  = 1,   // mirrored Z continuation
};

struct ZShape {
    std::vector<Diagonal> diagonals;  // size = N - 3 (N >= 4) or 0 (N == 3)
    std::vector<Triangle> triangles;  // size = N - 2
    bool valid = true;                // color seq alternating
};

// 纯函数 zigzag triangulation。
//
// 算法（与 docs/paper_alignment/pcatm_zshape_groundtruth.md N=4..9 ground truth 完全对齐）：
//
//   triangle 0: (v_0, v_1, v_2),       new diagonal: (v_0, v_2)
//
//   N == 3 → only triangle 0.
//   N == 4 → 加 triangle 1: (v_0, v_2, v_3)，无新增 diagonal（(v_0, v_3) 是 polygon edge）。
//
//   N >= 5：
//     triangle 1: (v_0, v_2, v_{N-1}),  new diagonal: (v_2, v_{N-1})
//     此时 v_0 完全脱离剩余链。剩余链 chain = [v_2, v_3, ..., v_{N-1}]，长度 N-2。
//     在 chain 上 alternating ear-clip：l=0, r=N-3。
//     起手 turnRight (即 step 2 切 right 端的 ear) —— 论文 Figure 2 默认。
//     while r - l >= 2:
//       turnRight: tri = (chain[l], chain[r-1], chain[r]),  if r-l > 2 then diag = (chain[l], chain[r-1]); r -= 1
//       turnLeft : tri = (chain[l], chain[l+1], chain[r]),  if r-l > 2 then diag = (chain[l+1], chain[r]); l += 1
//       turn 翻转。
//
// color 严格 alternating：tri[i].color = (startColor + i) % 2。
inline ZShape computeZShape(int N, int startColor, StartDir dir = StartDir::Right) {
    ZShape z;
    if (N < 3) return z;

    // Triangle 0：固定 (v_0, v_1, v_2)。
    z.triangles.push_back(Triangle{0, 1, 2, startColor & 1});
    if (N == 3) {
        z.valid = true;
        return z;
    }
    z.diagonals.push_back(Diagonal{0, 2});

    if (N == 4) {
        z.triangles.push_back(Triangle{0, 2, 3, (startColor + 1) & 1});
        z.valid = true;
        return z;
    }

    // N >= 5：triangle 1 = (v_0, v_2, v_{N-1})，新增 diagonal (v_2, v_{N-1})。
    z.triangles.push_back(Triangle{0, 2, N - 1, (startColor + 1) & 1});
    z.diagonals.push_back(Diagonal{2, N - 1});

    // 剩余链 chain = [v_2, v_3, ..., v_{N-1}]，长度 N-2。
    std::vector<int> chain;
    chain.reserve(static_cast<std::size_t>(N - 2));
    for (int i = 2; i < N; ++i) chain.push_back(i);

    int l = 0;
    int r = static_cast<int>(chain.size()) - 1;  // = N - 3
    // T1 already consumes the right end of the remaining chain.  The next
    // ear must therefore be taken from the opposite end; otherwise N >= 6
    // degenerates into a fan around chain[0] and is not the paper Z shape.
    bool takeRightEar = (dir == StartDir::Left);
    int triIdx = 2;

    while (r - l >= 2) {
        const int curColor = (startColor + triIdx) & 1;
        if (takeRightEar) {
            const int Cl  = chain[l];
            const int Cr  = chain[r];
            const int Crm = chain[r - 1];
            z.triangles.push_back(Triangle{Cl, Crm, Cr, curColor});
            if (r - l > 2) {
                z.diagonals.push_back(Diagonal{Cl, Crm});
            }
            r -= 1;
        } else {
            const int Cl  = chain[l];
            const int Cr  = chain[r];
            const int Clp = chain[l + 1];
            z.triangles.push_back(Triangle{Cl, Clp, Cr, curColor});
            if (r - l > 2) {
                z.diagonals.push_back(Diagonal{Clp, Cr});
            }
            l += 1;
        }
        takeRightEar = !takeRightEar;
        ++triIdx;
    }

    z.valid = true;
    return z;
}

// 校验 triangle 序列的 color 是否严格 alternating（patch 内 invariant，
// 论文 Figure 2 (1)(2)(3) 合法 vs (4) mirror Z 非法）。
inline bool isAlternating(const std::vector<Triangle>& tris) {
    for (std::size_t i = 1; i < tris.size(); ++i) {
        if (tris[i].color == tris[i - 1].color) return false;
    }
    return true;
}

}  // namespace pcatm
}  // namespace pmsf

#endif  // SRC_COMMON_PCATM_ZSHAPE_GEOMETRY_H_
