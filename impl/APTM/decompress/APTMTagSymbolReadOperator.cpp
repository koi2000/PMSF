// SPDX-License-Identifier: Apache-2.0
//
// APTMTagSymbolReadOperator.cpp — see header for design contract.

#include "decompress/APTMTagSymbolReadOperator.h"

#include "common/PaperBlobAlgorithmSpecs.h"
#include "compress/ConnectivityBlobContainer.h"
#include "storage/PaperBlobLayoutRegistry.h"

#include "Point.h"

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <span>
#include <stdexcept>
#include <vector>

namespace {

/// (R, θq, φq) → Frenet 分量 (c_n', c_t', c_b')。
/// 必须与 `APTMTagSymbolCollectOperator.cpp::expandPolarToFrenet` 逐位一致。
int polarAngleLevels() {
    const char* v = std::getenv("PMSF_APTM_TAG_POLAR_LEVELS");
    if (!v || !*v) return pmsf::aptm_tag::kPolarAngleLevels;
    const int n = std::atoi(v);
    if (n < 2 || n > (1 << 20)) return pmsf::aptm_tag::kPolarAngleLevels;
    return n;
}

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

}  // namespace

void APTMTagSymbolReadOperator::importFromRangeCodedBuffer(
    char* buffer, size_t& dataOffset, unsigned nbQuantBits) {
    if (buffer == nullptr) {
        throw std::runtime_error("[APTM-Tag reader] null rc blob buffer");
    }

    // 容器 header：[u32 magic][u16 blob_layout][u16 scheme_version][...]
    std::uint16_t sv = pmsf::aptm_tag::kSchemeVersionPCLTTMTagCartesian;
    std::memcpy(&sv, buffer + dataOffset + 6, sizeof(sv));
    lastSchemeVersion_ = static_cast<int>(sv);

    if (!pmsf::aptm_tag::isAPTMTagSchemeVersion(sv)) {
        // 非 APTM 的 APTM-Tag 产物（sv=3 = 纯 PCLTTM Tag Cartesian，例如 P1 阶段一
        // 基线）。原样委托父类；它自己会校验 layout16/sv3/3 段流。
        PCLTTMTagSymbolReadOperator::importFromRangeCodedBuffer(
            buffer, dataOffset, nbQuantBits);
        return;
    }

    // ---- APTM sv=64/65：先按父类的 CODEC_016/sv=3 约定解析，再展开语义 ----
    const std::span<const std::uint8_t> view(
        reinterpret_cast<const std::uint8_t*>(buffer + dataOffset),
        static_cast<std::size_t>(1) << 32);
    pmsf::ConnectivityBlobReader rd(view);
    const auto& hdr = rd.header();
    const std::size_t blobSize =
        hdr.headerByteSize() +
        (hdr.stream_offsets.empty() ? 0 : hdr.stream_offsets.back());

    std::vector<char> patched(buffer + dataOffset,
                              buffer + dataOffset + blobSize);
    const std::uint16_t sv3 = pmsf::aptm_tag::kSchemeVersionPCLTTMTagCartesian;
    std::memcpy(patched.data() + 6, &sv3, sizeof(sv3));

    std::size_t localOffset = 0;
    PCLTTMTagSymbolReadOperator::importFromRangeCodedBuffer(
        patched.data(), localOffset, nbQuantBits);
    dataOffset += blobSize;

    lastRawResidualCount_ = static_cast<int>(rcDecodedResiduals_.size());
    if (sv == pmsf::aptm_tag::kSchemeVersionPolar) {
        // residual 队列里是 (R, θq, φq)：按 encoder 同一公式展开成 Frenet 分量，
        // 让父类 collectFromDecoded 的 Frenet 反投影分支（lift 整数逆）处理。
        for (auto& r : rcDecodedResiduals_) {
            r = expandPolarToFrenet(r, polarAngleLevels());
        }
    }
    // sv=64：队列已经是 Frenet 分量，直接交给父类反投影。

    rcUseFrenetResidual_ = true;
    rcUseFrenetExactResidual_ = false;
    rcDecodedRoundCount_ = static_cast<int>(header_.getRound());

    const char* diag = std::getenv("PMSF_BPV_DIAG");
    if (diag && diag[0] == '1') {
        std::fprintf(stderr,
                     "[APTM-Tag reader] sv=%d rounds=%d residuals=%d "
                     "blobSize=%zu mode=%s\n",
                     static_cast<int>(sv), static_cast<int>(hdr.num_rounds),
                     lastRawResidualCount_, blobSize,
                     (sv == pmsf::aptm_tag::kSchemeVersionPolar) ? "polar"
                                                                 : "frenet");
        if (sv == pmsf::aptm_tag::kSchemeVersionPolar) {
            std::fprintf(stderr, "[APTM-Tag reader] angleLevels=%d\n",
                         polarAngleLevels());
        }
    }
}
