// SPDX-License-Identifier: Apache-2.0
//
// PCLTTMTagCanonicalCodec.h — shared CODEC_016 sv=3 encoders for the canonical path.
//
// These helpers mirror the exact entropy coding used by
// PCLTTMTagSymbolCollectOperator so that a canonical payload can be reassembled
// into a byte-identical CODEC_016 ConnectivityBlobContainer v3 blob.

#ifndef PMSF_COMPRESS_PCLTTM_TAG_CANONICAL_CODEC_H
#define PMSF_COMPRESS_PCLTTM_TAG_CANONICAL_CODEC_H

#include "common/RDTagCanonicalCodec.h"
#include "Point.h"
#include "rangecoder/qsmodel.h"
#include "rangecoder/rangecod.h"

#include <cstdint>
#include <cstring>
#include <deque>
#include <memory>
#include <vector>

namespace pmsf {
namespace pclttm_tag {

/// stream 1 (sv=3): 4-ary fused face symbol RC.
/// Byte layout: [int32 numSyms][bytes RC]
inline std::vector<std::uint8_t> encodeFusedFaceSym4RC(
    const std::deque<std::uint8_t>& syms) {
    const std::size_t bufSize =
        std::max<std::size_t>(syms.size() / 2 + 1024, 4096);
    std::unique_ptr<char[]> buf(new char[bufSize]);
    std::size_t off = 0;
    rangecoder rc;
    rc.p_data = buf.get();
    rc.p_dataOffset = &off;
    start_encoding(&rc, 0, 0);

    qsmodel m;
    initqsmodel(&m, 4, 10, 1 << 9, nullptr, 1);
    for (std::uint8_t s : syms) {
        int sym = static_cast<int>(s);
        if (sym < 0 || sym > 3) sym = 0;
        int syfreq = 0, ltfreq = 0;
        qsgetfreq(&m, sym, &syfreq, &ltfreq);
        encode_shift(&rc, syfreq, ltfreq, 10);
        qsupdate(&m, sym);
    }
    std::size_t rcSize = done_encoding(&rc);
    deleteqsmodel(&m);

    std::vector<std::uint8_t> out(4 + rcSize);
    std::int32_t n = static_cast<std::int32_t>(syms.size());
    std::memcpy(out.data(), &n, 4);
    std::memcpy(out.data() + 4, buf.get(), rcSize);
    return out;
}

// Reuse RD_TAG helpers for valence (5-ary 4-context) and residual bit-plane.
using rd_tag::encodeValence5WithContext;
using rd_tag::encodeResidualBitplane;

}  // namespace pclttm_tag
}  // namespace pmsf

#endif  // PMSF_COMPRESS_PCLTTM_TAG_CANONICAL_CODEC_H
