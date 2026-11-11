// EdgeBreakerCodec.h — native, in-process EdgeBreaker (CLERS) codec for a
// closed, genus-0, 2-manifold triangle mesh with INTEGER (quantized) vertex
// coordinates. Lossless connectivity + exact integer geometry.
//
// Reference: J. Rossignac, A. Safonova, A. Szymczak,
//   "3D Compression Made Simple: Edgebreaker on a Corner-Table" (SMI 2001).
// Connectivity decode uses the corner-table Wrap&Zip decompressor; geometry
// uses parallelogram prediction with exact integer residuals.
//
// Self-contained (STL only) so it is shared by the encoder, the decoder and
// the standalone unit test.
#ifndef PMSF_EDGE_BREAKER_CODEC_H
#define PMSF_EDGE_BREAKER_CODEC_H

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstddef>
#include <unordered_map>
#include <utility>
#include <vector>

namespace pmsf_eb {

using Vec3i = std::array<std::int32_t, 3>;
using Tri = std::array<int, 3>;

struct DecodeResult {
    std::vector<Vec3i> verts;  // vertices in EB discovery order
    std::vector<Tri> faces;    // faces in EB traversal order
    bool ok = false;
};

// ---- corner-table index helpers (3 corners per triangle) ----
inline int ebNext(int c) { return 3 * (c / 3) + (c + 1) % 3; }
inline int ebPrev(int c) { return 3 * (c / 3) + (c + 2) % 3; }

// ---- zigzag varint helpers ----
inline void ebPutVarU(std::vector<std::uint8_t>& out, std::uint32_t v) {
    while (v >= 0x80u) {
        out.push_back(static_cast<std::uint8_t>(v) | 0x80u);
        v >>= 7;
    }
    out.push_back(static_cast<std::uint8_t>(v));
}
inline std::uint32_t ebGetVarU(const std::uint8_t*& p, const std::uint8_t* end) {
    std::uint32_t v = 0;
    int shift = 0;
    while (p < end) {
        std::uint8_t b = *p++;
        v |= static_cast<std::uint32_t>(b & 0x7Fu) << shift;
        if (!(b & 0x80u)) break;
        shift += 7;
    }
    return v;
}
inline void ebPutVarZ(std::vector<std::uint8_t>& out, std::int32_t v) {
    ebPutVarU(out, (static_cast<std::uint32_t>(v) << 1) ^
                       static_cast<std::uint32_t>(v >> 31));
}
inline std::int32_t ebGetVarZ(const std::uint8_t*& p, const std::uint8_t* end) {
    std::uint32_t u = ebGetVarU(p, end);
    return static_cast<std::int32_t>((u >> 1) ^ (~(u & 1u) + 1u));
}

// ---- MSB-first bit stream for the CLERS symbols ----
// Symbol codes (Rossignac): C=0 ; S=100 ; R=101 ; L=110 ; E=111.
struct EbBitWriter {
    std::vector<std::uint8_t> bytes;
    std::uint8_t cur = 0;
    int nbits = 0;
    void putBit(int b) {
        cur = static_cast<std::uint8_t>((cur << 1) | (b & 1));
        if (++nbits == 8) { bytes.push_back(cur); cur = 0; nbits = 0; }
    }
    void flush() {
        if (nbits) { cur = static_cast<std::uint8_t>(cur << (8 - nbits));
                     bytes.push_back(cur); cur = 0; nbits = 0; }
    }
};
struct EbBitReader {
    const std::uint8_t* p = nullptr;
    std::size_t n = 0;
    std::size_t bitpos = 0;
    int getBit() {
        std::size_t byte = bitpos / 8;
        int off = 7 - static_cast<int>(bitpos % 8);
        ++bitpos;
        if (byte >= n) return 0;
        return (p[byte] >> off) & 1;
    }
};

enum EbSym { EB_C = 0, EB_L = 1, EB_R = 2, EB_S = 3, EB_E = 4 };

inline void ebWriteSym(EbBitWriter& w, EbSym s) {
    switch (s) {
        case EB_C: w.putBit(0); break;
        case EB_S: w.putBit(1); w.putBit(0); w.putBit(0); break;
        case EB_R: w.putBit(1); w.putBit(0); w.putBit(1); break;
        case EB_L: w.putBit(1); w.putBit(1); w.putBit(0); break;
        case EB_E: w.putBit(1); w.putBit(1); w.putBit(1); break;
    }
}
inline EbSym ebReadSym(EbBitReader& r) {
    if (r.getBit() == 0) return EB_C;
    int b1 = r.getBit();
    int b2 = r.getBit();
    if (b1 == 0 && b2 == 0) return EB_S;
    if (b1 == 0 && b2 == 1) return EB_R;
    if (b1 == 1 && b2 == 0) return EB_L;
    return EB_E;  // 1,1
}

// ============================================================================
// ENCODE
// ============================================================================
// Returns an empty vector outside the connected closed genus-zero domain.
inline std::vector<std::uint8_t> encodeEdgeBreaker(
    const std::vector<Vec3i>& verts, const std::vector<Tri>& faces,
    std::vector<int>* visitOrder = nullptr) {
    const int F = static_cast<int>(faces.size());
    const int Vn = static_cast<int>(verts.size());
    std::vector<std::uint8_t> blob;
    if (F < 1 || Vn < 3) return blob;

    const int C3 = 3 * F;
    std::vector<int> V(static_cast<std::size_t>(C3));
    for (int f = 0; f < F; ++f) {
        V[3 * f + 0] = faces[f][0];
        V[3 * f + 1] = faces[f][1];
        V[3 * f + 2] = faces[f][2];
        if (faces[f][0] < 0 || faces[f][0] >= Vn ||
            faces[f][1] < 0 || faces[f][1] >= Vn ||
            faces[f][2] < 0 || faces[f][2] >= Vn)
            return blob;
    }
    // Build opposite-corner table O via directed-edge map.
    std::vector<int> O(static_cast<std::size_t>(C3), -1);
    std::unordered_map<std::uint64_t, int> em;
    em.reserve(static_cast<std::size_t>(C3) * 2);
    auto key = [](int a, int b) {
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(a)) << 32) |
               static_cast<std::uint32_t>(b);
    };
    for (int c = 0; c < C3; ++c) {
        std::uint64_t k = key(V[ebNext(c)], V[ebPrev(c)]);
        if (!em.emplace(k, c).second) return blob;  // non-manifold / dup edge
    }
    for (int c = 0; c < C3; ++c) {
        auto it = em.find(key(V[ebPrev(c)], V[ebNext(c)]));
        if (it == em.end()) return blob;  // border edge -> not closed
        O[c] = it->second;
    }

    // This CLERS stream has no handle events. Closed edge pairing alone is
    // insufficient: accepting a torus emits a blob the Wrap&Zip decoder
    // cannot reconstruct. Let storage use its existing general base codec.
    if (std::int64_t(Vn) - std::int64_t(C3)/2 + F != 2) return blob;

    std::vector<char> M(static_cast<std::size_t>(Vn), 0);
    std::vector<char> U(static_cast<std::size_t>(F), 0);
    EbBitWriter clers;
    std::vector<std::uint8_t> deltas;

    const int c0 = 0;
    const Vec3i& g_p = verts[static_cast<std::size_t>(V[ebPrev(c0)])];
    const Vec3i& g_v = verts[static_cast<std::size_t>(V[c0])];
    const Vec3i& g_n = verts[static_cast<std::size_t>(V[ebNext(c0)])];
    for (int k = 0; k < 3; ++k) ebPutVarZ(deltas, g_p[k]);
    for (int k = 0; k < 3; ++k) ebPutVarZ(deltas, g_v[k] - g_p[k]);
    for (int k = 0; k < 3; ++k) ebPutVarZ(deltas, g_n[k] - g_v[k]);
    M[static_cast<std::size_t>(V[c0])] = 1;
    M[static_cast<std::size_t>(V[ebNext(c0)])] = 1;
    M[static_cast<std::size_t>(V[ebPrev(c0)])] = 1;
    U[static_cast<std::size_t>(c0 / 3)] = 1;
    if (visitOrder) { visitOrder->clear(); visitOrder->push_back(c0 / 3); }

    std::vector<int> stack;
    stack.push_back(O[c0]);
    while (!stack.empty()) {
        int c = stack.back();
        stack.pop_back();
        for (;;) {
            U[static_cast<std::size_t>(c / 3)] = 1;
            if (visitOrder) visitOrder->push_back(c / 3);
            const int vc = V[c];
            if (M[static_cast<std::size_t>(vc)] != 1) {
                const Vec3i& P = verts[static_cast<std::size_t>(V[ebPrev(c)])];
                const Vec3i& Nn = verts[static_cast<std::size_t>(V[ebNext(c)])];
                const Vec3i& Oo = verts[static_cast<std::size_t>(V[O[c]])];
                const Vec3i& Vv = verts[static_cast<std::size_t>(vc)];
                for (int k = 0; k < 3; ++k)
                    ebPutVarZ(deltas, Vv[k] - P[k] - Nn[k] + Oo[k]);
                ebWriteSym(clers, EB_C);
                M[static_cast<std::size_t>(vc)] = 1;
                c = O[ebNext(c)];  // c.r
            } else {
                const int cr = O[ebNext(c)];
                const int cl = O[ebPrev(c)];
                if (U[static_cast<std::size_t>(cr / 3)] == 1) {
                    if (U[static_cast<std::size_t>(cl / 3)] == 1) {
                        ebWriteSym(clers, EB_E);
                        break;
                    }
                    ebWriteSym(clers, EB_R);
                    c = cl;
                } else if (U[static_cast<std::size_t>(cl / 3)] == 1) {
                    ebWriteSym(clers, EB_L);
                    c = cr;
                } else {
                    ebWriteSym(clers, EB_S);
                    stack.push_back(cl);
                    c = cr;
                }
            }
        }
    }
    // Euler characteristic does not by itself imply one component (e.g. a
    // sphere plus a torus). The traversal must cover every face and vertex.
    if (std::find(U.begin(),U.end(),0)!=U.end() || std::find(M.begin(),M.end(),0)!=M.end()) {
        if (visitOrder) visitOrder->clear();
        return blob;
    }
    clers.flush();

    blob.push_back('E'); blob.push_back('B'); blob.push_back('1');
    blob.push_back(1);
    ebPutVarU(blob, static_cast<std::uint32_t>(F));
    ebPutVarU(blob, static_cast<std::uint32_t>(Vn));
    ebPutVarU(blob, static_cast<std::uint32_t>(clers.bytes.size()));
    blob.insert(blob.end(), clers.bytes.begin(), clers.bytes.end());
    blob.insert(blob.end(), deltas.begin(), deltas.end());
    return blob;
}

// ============================================================================
// DECODE
// ============================================================================
inline void ebZip(std::vector<int>& V, std::vector<int>& O, int c) {
    for (;;) {
        int b = ebNext(c);
        while (O[b] >= 0) b = ebNext(O[b]);
        if (O[b] != -1) return;
        O[c] = b; O[b] = c;
        int a = ebPrev(c);
        V[ebPrev(a)] = V[ebPrev(b)];
        while (O[a] >= 0 && b != a) {
            a = ebPrev(O[a]);
            V[ebPrev(a)] = V[ebPrev(b)];
        }
        c = ebPrev(c);
        while (O[c] >= 0 && c != b) c = ebPrev(O[c]);
        if (O[c] == -2) continue;  // Zip(c) tail recursion
        return;
    }
}

inline DecodeResult decodeEdgeBreaker(const std::vector<std::uint8_t>& blob) {
    DecodeResult res;
    const std::uint8_t* p = blob.data();
    const std::uint8_t* end = p + blob.size();
    if (blob.size() < 5 || p[0] != 'E' || p[1] != 'B' || p[2] != '1') return res;
    p += 4;
    const int F = static_cast<int>(ebGetVarU(p, end));
    const int Vn = static_cast<int>(ebGetVarU(p, end));
    const std::uint32_t clersLen = ebGetVarU(p, end);
    if (F < 1 || Vn < 3) return res;
    if (p + clersLen > end) return res;
    EbBitReader br;
    br.p = p; br.n = clersLen; br.bitpos = 0;
    const std::uint8_t* dp = p + clersLen;  // delta cursor

    const int C3 = 3 * F;
    std::vector<int> V(static_cast<std::size_t>(C3), 0);
    std::vector<int> O(static_cast<std::size_t>(C3), -3);
    V[0] = 0; V[1] = 1; V[2] = 2;
    O[0] = -1; O[1] = -3; O[2] = -1;
    int T = 0;
    int Ncount = 2;

    // C-events in CLERS order: (newVertexId, tipCorner).
    std::vector<std::pair<int, int>> cEvents;
    cEvents.reserve(static_cast<std::size_t>(Vn));

    std::vector<int> stack;
    stack.push_back(1);
    long long guard = 0;
    const long long guardMax = static_cast<long long>(F) * 8 + 64;
    while (!stack.empty()) {
        int c = stack.back();
        stack.pop_back();
        for (;;) {
            if (++guard > guardMax) { res.ok = false; return res; }
            ++T;
            if (3 * T + 2 >= C3) { res.ok = false; return res; }
            O[c] = 3 * T; O[3 * T] = c;
            V[3 * T + 1] = V[ebPrev(c)];
            V[3 * T + 2] = V[ebNext(c)];
            c = ebNext(O[c]);  // c = c.o.n = next(3T) = 3T+1
            EbSym s = ebReadSym(br);
            if (s == EB_C) {
                O[ebNext(c)] = -1;
                V[3 * T] = ++Ncount;
                cEvents.emplace_back(Ncount, 3 * T);
            } else if (s == EB_L) {
                O[ebNext(c)] = -2;
                ebZip(V, O, ebNext(c));
            } else if (s == EB_R) {
                O[c] = -2;
                c = ebNext(c);
            } else if (s == EB_S) {
                stack.push_back(ebNext(c));
                // continue on right branch with current c
            } else {  // EB_E
                O[c] = -2;
                O[ebNext(c)] = -2;
                ebZip(V, O, ebNext(c));
                break;
            }
        }
    }
    if (T != F - 1 || Ncount != Vn - 1) { res.ok = false; return res; }

    // Geometry: bootstrap first 3 vertices, then parallelogram per C event.
    std::vector<Vec3i> G(static_cast<std::size_t>(Vn), Vec3i{0, 0, 0});
    for (int k = 0; k < 3; ++k) G[0][k] = ebGetVarZ(dp, end);
    for (int k = 0; k < 3; ++k) G[1][k] = G[0][k] + ebGetVarZ(dp, end);
    for (int k = 0; k < 3; ++k) G[2][k] = G[1][k] + ebGetVarZ(dp, end);
    for (const auto& ev : cEvents) {
        const int vid = ev.first;
        const int tip = ev.second;
        const Vec3i& P = G[static_cast<std::size_t>(V[ebPrev(tip)])];
        const Vec3i& Nn = G[static_cast<std::size_t>(V[ebNext(tip)])];
        const Vec3i& Oo = G[static_cast<std::size_t>(V[O[tip]])];
        for (int k = 0; k < 3; ++k)
            G[static_cast<std::size_t>(vid)][k] =
                P[k] + Nn[k] - Oo[k] + ebGetVarZ(dp, end);
    }

    res.verts = std::move(G);
    res.faces.resize(static_cast<std::size_t>(F));
    for (int t = 0; t < F; ++t) {
        res.faces[static_cast<std::size_t>(t)] = {V[3 * t], V[3 * t + 1],
                                                  V[3 * t + 2]};
    }
    res.ok = true;
    return res;
}


// Visit order only (topology): visitOrder[k] = input face index visited k-th.
// Single source of truth: delegates to encodeEdgeBreaker (coords irrelevant to
// the CLERS traversal), so encoder and channel builder cannot diverge.
inline std::vector<int> computeEbVisitOrder(const std::vector<Tri>& faces,
                                            int numVerts) {
    std::vector<Vec3i> zero(static_cast<std::size_t>(numVerts < 3 ? 3 : numVerts),
                            Vec3i{0, 0, 0});
    std::vector<int> vo;
    (void)encodeEdgeBreaker(zero, faces, &vo);
    return vo;
}

}  // namespace pmsf_eb
#endif  // PMSF_EDGE_BREAKER_CODEC_H
