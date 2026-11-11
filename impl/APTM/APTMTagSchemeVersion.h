// SPDX-License-Identifier: Apache-2.0
//
// APTMTagSchemeVersion.h — APTM P1 产物的容器 scheme_version 定义（encoder/decoder 共用）。
//
// 产物仍是 CODEC_016 容器（blob_layout=16，3 段流：valence / fusedFaceSym /
// residual），只把 scheme_version 从 PCLTTM Tag 的 3 改写成 APTM 自己的值，
// residual 段的语义由 sv 决定：
//   3  = PCLTTM Tag Cartesian（APTM 不会写出；decoder 仍兼容，原样委托父类）
//   64 = APTM Tag，residual = 可逆整数 lifting 的 Frenet 分量 (c_n,c_t,c_b)（无损）
//   65 = APTM Tag，residual = (R, θq, φq) polar 量化三元组（θ/φ 3-bit，有损）
#ifndef PMSF_APTM_APTMTAG_SCHEME_VERSION_H
#define PMSF_APTM_APTMTAG_SCHEME_VERSION_H

#include <cstdint>

namespace pmsf {
namespace aptm_tag {

inline constexpr std::uint16_t kSchemeVersionPCLTTMTagCartesian = 3;
inline constexpr std::uint16_t kSchemeVersionFrenet = 64;
inline constexpr std::uint16_t kSchemeVersionPolar = 65;

/// polar 量化的角度级数：论文 §4.2「θ/φ 量化到 3 bit」⇒ 各 8 级。
inline constexpr int kPolarAngleLevels = 8;

}  // namespace aptm_tag
}  // namespace pmsf

#endif  // PMSF_APTM_APTMTAG_SCHEME_VERSION_H
