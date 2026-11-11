// MMSE task-64 read-only entropy audit: decode existing CQST RC streams and
// measure the 0th/1st-order entropy gap (bytes actually used vs symbol entropy).
#include "conquest/ConquestBlobFormat.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

using namespace pmsf::conquest_blob;

static double entropy0(const std::vector<int>& v) {
    if (v.empty()) return 0.0;
    std::map<int, std::size_t> c;
    for (int x : v) ++c[x];
    const double n = (double)v.size();
    double h = 0.0;
    for (auto& kv : c) { const double p = kv.second / n; h -= p * std::log2(p); }
    return h;
}
// order-1 conditional entropy H(X_i | X_{i-1}) within the sequence (per layer).
static double entropy1(const std::vector<int>& v) {
    if (v.size() < 2) return entropy0(v);
    std::map<int, std::map<int, std::size_t>> joint;
    std::map<int, std::size_t> prev;
    for (std::size_t i = 1; i < v.size(); ++i) { joint[v[i-1]][v[i]]++; prev[v[i-1]]++; }
    double bits = 0.0;
    for (auto& a : joint) {
        const double pn = (double)prev[a.first];
        for (auto& b : a.second) {
            const double p = b.second / pn;
            bits -= b.second * std::log2(p);
        }
    }
    return bits / (double)(v.size() - 1);
}

int main(int argc, char** argv) {
    if (argc < 2) { std::fprintf(stderr, "usage: %s <cqst_section_file> [dump_prefix]\n", argv[0]); return 2; }
    const std::string dumpPrefix = (argc > 2) ? argv[2] : "";
    std::FILE* f = std::fopen(argv[1], "rb");
    if (!f) { std::perror("open"); return 2; }
    std::fseek(f, 0, SEEK_END); long sz = std::ftell(f); std::fseek(f, 0, SEEK_SET);
    std::vector<char> data((std::size_t)sz);
    if (std::fread(data.data(), 1, data.size(), f) != data.size()) { std::fclose(f); return 2; }
    std::fclose(f);

    pmsf::conquest_blob::Stream s;
    if (!pmsf::conquest_blob::readStream(data.data(), data.size(), &s)) {
        std::fprintf(stderr, "readStream failed\n"); return 1;
    }
    std::printf("layers=%zu\n", s.layers.size());
    // Recompute the encoded byte sizes by re-encoding with the exact same coder,
    // then decode and measure entropy.
    double totDeltaBytes = 0, totDeltaH0 = 0, totDeltaH1 = 0;
    double totSymBytes = 0, totSymH0 = 0, totSymH1 = 0;
    double totUniform = 0;
    for (std::size_t li = 0; li < s.layers.size(); ++li) {
        const auto& L = s.layers[li];
        // symbols: bytes = codeSymbols(...).size()
        const auto symBytesVec = detail::codeSymbols(L.symbols);
        const double sb = (double)symBytesVec.size();
        const double sh0 = entropy0(L.symbols) * (double)L.symbols.size() / 8.0;
        const double sh1 = entropy1(L.symbols) * (double)L.symbols.size() / 8.0;
        totSymBytes += sb; totSymH0 += sh0; totSymH1 += sh1;
        if (!dumpPrefix.empty()) {
            std::string p = dumpPrefix + ".sym." + std::to_string(li);
            std::FILE* df = std::fopen(p.c_str(), "wb");
            for (int v : L.symbols) std::fputc((unsigned char)(v & 0xff), df);
            std::fclose(df);
        }
        double layerDeltaBytes = 0, layerDeltaH0 = 0, layerDeltaH1 = 0, layerUni = 0;
        for (int k = 0; k < 3; ++k) {
            std::int32_t lo = 0; std::uint32_t span = 0;
            const auto comp = detail::codeComponent(L.deltas, k, &lo, &span);
            std::vector<int> vals;
            vals.reserve(L.deltas.size());
            for (auto& d : L.deltas) vals.push_back(d[k]);
            const double b = (double)comp.size();
            const double h0 = entropy0(vals) * (double)vals.size() / 8.0;
            const double h1 = entropy1(vals) * (double)vals.size() / 8.0;
            layerDeltaBytes += b; layerDeltaH0 += h0; layerDeltaH1 += h1;
            if (!dumpPrefix.empty()) {
                std::string p = dumpPrefix + ".d." + std::to_string(li) + "." + std::to_string(k);
                std::FILE* df = std::fopen(p.c_str(), "wb");
                for (int v : vals) { unsigned int u = (unsigned int)(v - (int)lo); std::uint16_t v16 = (std::uint16_t)u; std::fwrite(&v16, 2, 1, df); }
                std::FILE* mf = std::fopen((p + ".meta").c_str(), "w");
                std::fprintf(mf, "%d %u %zu %zu\n", (int)lo, span, vals.size(), comp.size());
                std::fclose(mf);
                std::fclose(df);
            }
            layerUni += (double)vals.size() * std::log2((double)std::max<std::uint32_t>(span, 2)) / 8.0;
        }
        totDeltaBytes += layerDeltaBytes; totDeltaH0 += layerDeltaH0; totDeltaH1 += layerDeltaH1;
        totUniform += layerUni;
        std::printf("L%02zu gates=%5zu sym=%6.0fB (H0=%6.0f H1=%6.0f) delta=%7.0fB (H0=%7.0f H1=%7.0f uni=%7.0f)\n",
                    li, L.deltas.size(), sb, sh0, sh1, layerDeltaBytes, layerDeltaH0, layerDeltaH1, layerUni);
    }
    std::printf("\nTOTAL symbols: bytes=%.0f H0=%.0f H1=%.0f  gap0=%.0f gap1=%.0f\n",
                totSymBytes, totSymH0, totSymH1, totSymBytes - totSymH0, totSymBytes - totSymH1);
    std::printf("TOTAL deltas : bytes=%.0f H0=%.0f H1=%.0f uniform=%.0f gap0=%.0f gap1=%.0f\n",
                totDeltaBytes, totDeltaH0, totDeltaH1, totUniform,
                totDeltaBytes - totDeltaH0, totDeltaBytes - totDeltaH1);
    std::printf("TOTAL RC     : bytes=%.0f  headroom_static0=%.0f  headroom_ctx1=%.0f\n",
                totSymBytes + totDeltaBytes,
                (totSymBytes + totDeltaBytes) - (totSymH0 + totDeltaH0),
                (totSymBytes + totDeltaBytes) - (totSymH1 + totDeltaH1));
    return 0;
}
