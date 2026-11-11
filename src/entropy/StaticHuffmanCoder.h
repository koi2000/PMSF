/*
 * @file StaticHuffmanCoder.h
 * @brief Minimal static Huffman coder suitable for QEM-class baselines.
 *
 * Two-pass: build a frequency table → canonical lengths → serialize the
 * length table + packed bitstream. The companion decode() reverses the
 * process. Alphabet is bounded at 65536 symbols (uint16 codebook indices,
 * residual bins, etc.).
 *
 * Implementation notes:
 *   - Deterministic tie-breaking on equal frequencies, keyed by symbol id,
 *     so encoded bytes are reproducible across runs.
 *   - BitWriter/BitReader pack LSB-first so symbols stay human-inspectable
 *     with `xxd` for debugging.
 *   - Single-symbol alphabet: the symbol is emitted as a 1-bit code to
 *     sidestep the "empty prefix" corner case the naïve Huffman hits.
 */
#ifndef PMSF_ENTROPY_STATIC_HUFFMAN_CODER_H
#define PMSF_ENTROPY_STATIC_HUFFMAN_CODER_H

#include "EntropyCoder.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <map>
#include <queue>
#include <stdexcept>
#include <vector>

namespace pmsf {

class StaticHuffmanCoder final : public EntropyCoder {
public:
    void reset() override {}

    void encode(const std::vector<std::uint32_t>& symbols,
                std::vector<std::uint8_t>& out) override {
        // ------------ Frequencies (ordered for determinism) ------------
        std::map<std::uint32_t, std::uint64_t> freq;
        for (auto s : symbols) ++freq[s];

        // ------------ Build Huffman tree ------------
        struct Node { std::uint64_t f; std::uint32_t sym; int left = -1; int right = -1; };
        std::vector<Node> nodes;
        nodes.reserve(freq.size() * 2 + 4);

        // Priority queue: min-heap by (frequency, symbol id for internals use 0xFFFFFFFF + counter)
        struct HItem { std::uint64_t f; int id; std::uint64_t tag; };
        auto cmp = [](const HItem& a, const HItem& b) {
            if (a.f != b.f) return a.f > b.f;
            return a.tag > b.tag;
        };
        std::priority_queue<HItem, std::vector<HItem>, decltype(cmp)> heap(cmp);

        std::uint64_t tag_counter = 0;
        for (auto& [s, f] : freq) {
            nodes.push_back({f, s, -1, -1});
            heap.push({f, static_cast<int>(nodes.size()) - 1, tag_counter++});
        }

        if (heap.empty()) {
            writeU32(out, 0);
            writeU32(out, 0);
            return;
        }

        // Single-symbol alphabet: emit one entry with length 1, stream = n zero bits.
        if (heap.size() == 1) {
            const HItem only = heap.top();
            writeU32(out, 1u);
            writeU32(out, static_cast<std::uint32_t>(symbols.size()));
            writeU32(out, nodes[only.id].sym);
            writeU16(out, 1u);
            out.push_back(0u);  // one-bit code "0", LSB-aligned
            writeU32(out, static_cast<std::uint32_t>(symbols.size()));
            const std::size_t stream_bytes = (symbols.size() + 7) / 8;
            for (std::size_t i = 0; i < stream_bytes; ++i) out.push_back(0u);
            return;
        }

        while (heap.size() > 1) {
            HItem a = heap.top(); heap.pop();
            HItem b = heap.top(); heap.pop();
            nodes.push_back({a.f + b.f, 0u, a.id, b.id});
            heap.push({a.f + b.f, static_cast<int>(nodes.size()) - 1, tag_counter++});
        }

        std::map<std::uint32_t, std::vector<std::uint8_t>> codes;
        std::vector<std::uint8_t> prefix;
        buildCodes(nodes, heap.top().id, prefix, codes);

        // ------------ Serialize table ------------
        writeU32(out, static_cast<std::uint32_t>(codes.size()));
        writeU32(out, static_cast<std::uint32_t>(symbols.size()));
        for (auto& [sym, bits] : codes) {
            writeU32(out, sym);
            writeU16(out, static_cast<std::uint16_t>(bits.size()));
            packBits(out, bits);
        }

        // ------------ Serialize stream ------------
        std::vector<std::uint8_t> stream_bits;
        stream_bits.reserve(symbols.size() * 8);
        for (auto s : symbols) {
            const auto& bits = codes.at(s);
            stream_bits.insert(stream_bits.end(), bits.begin(), bits.end());
        }
        writeU32(out, static_cast<std::uint32_t>(stream_bits.size()));
        packBits(out, stream_bits);
    }

    std::size_t decode(const std::uint8_t* in,
                       std::size_t in_bytes,
                       std::size_t count,
                       std::vector<std::uint32_t>& out) override {
        std::size_t off = 0;
        const std::uint32_t nsyms = readU32(in, off);
        const std::uint32_t nsyms_total = readU32(in, off);
        (void)nsyms_total;

        if (nsyms == 0) { return off; }

        // Rebuild decode trie.
        struct TrieNode { int left = -1; int right = -1; std::uint32_t sym = 0; bool leaf = false; };
        std::vector<TrieNode> trie(1);
        for (std::uint32_t i = 0; i < nsyms; ++i) {
            const std::uint32_t sym = readU32(in, off);
            const std::uint16_t len = readU16(in, off);
            std::vector<std::uint8_t> bits = unpackBits(in, off, len);
            int node = 0;
            for (std::uint8_t bit : bits) {
                int child = bit ? trie[node].right : trie[node].left;
                if (child < 0) {
                    trie.push_back({});
                    child = static_cast<int>(trie.size()) - 1;
                    if (bit) trie[node].right = child;
                    else     trie[node].left = child;
                }
                node = child;
            }
            trie[node].leaf = true;
            trie[node].sym = sym;
        }

        // Single-symbol fast path.
        if (nsyms == 1) {
            std::uint32_t stream_bits = readU32(in, off);
            off += (stream_bits + 7) / 8;
            for (std::size_t i = 0; i < count; ++i) out.push_back(trie[0].sym != 0 ? trie[0].sym : trie[trie[0].left].sym);
            return off;
        }

        const std::uint32_t stream_bits = readU32(in, off);
        std::vector<std::uint8_t> stream = unpackBits(in, off, stream_bits);
        int node = 0;
        out.reserve(out.size() + count);
        for (std::uint8_t bit : stream) {
            int child = bit ? trie[node].right : trie[node].left;
            if (child < 0) throw std::runtime_error("StaticHuffmanCoder: trie underflow");
            node = child;
            if (trie[node].leaf) {
                out.push_back(trie[node].sym);
                node = 0;
                if (out.size() >= count) break;
            }
        }
        if (off > in_bytes) throw std::runtime_error("StaticHuffmanCoder: ran past buffer");
        return off;
    }

private:
    template <typename NodeVec>
    static void buildCodes(const NodeVec& nodes, int idx, std::vector<std::uint8_t>& prefix,
                           std::map<std::uint32_t, std::vector<std::uint8_t>>& out) {
        const auto& n = nodes[idx];
        if (n.left < 0 && n.right < 0) { out[n.sym] = prefix; return; }
        if (n.left >= 0) {
            prefix.push_back(0u);
            buildCodes(nodes, n.left, prefix, out);
            prefix.pop_back();
        }
        if (n.right >= 0) {
            prefix.push_back(1u);
            buildCodes(nodes, n.right, prefix, out);
            prefix.pop_back();
        }
    }

    static void writeU32(std::vector<std::uint8_t>& o, std::uint32_t v) {
        for (int i = 0; i < 4; ++i) o.push_back(static_cast<std::uint8_t>(v >> (8 * i)));
    }
    static void writeU16(std::vector<std::uint8_t>& o, std::uint16_t v) {
        for (int i = 0; i < 2; ++i) o.push_back(static_cast<std::uint8_t>(v >> (8 * i)));
    }
    static std::uint32_t readU32(const std::uint8_t* in, std::size_t& off) {
        std::uint32_t v = 0;
        for (int i = 0; i < 4; ++i) v |= static_cast<std::uint32_t>(in[off + i]) << (8 * i);
        off += 4;
        return v;
    }
    static std::uint16_t readU16(const std::uint8_t* in, std::size_t& off) {
        std::uint16_t v = 0;
        for (int i = 0; i < 2; ++i) v |= static_cast<std::uint16_t>(in[off + i]) << (8 * i);
        off += 2;
        return v;
    }
    static void packBits(std::vector<std::uint8_t>& o, const std::vector<std::uint8_t>& bits) {
        std::size_t bytes = (bits.size() + 7) / 8;
        for (std::size_t i = 0; i < bytes; ++i) {
            std::uint8_t b = 0;
            for (std::size_t j = 0; j < 8 && (i * 8 + j) < bits.size(); ++j)
                if (bits[i * 8 + j]) b |= static_cast<std::uint8_t>(1u << j);
            o.push_back(b);
        }
    }
    static std::vector<std::uint8_t> unpackBits(const std::uint8_t* in, std::size_t& off,
                                                std::size_t nbits) {
        std::vector<std::uint8_t> out(nbits);
        std::size_t bytes = (nbits + 7) / 8;
        for (std::size_t i = 0; i < nbits; ++i)
            out[i] = static_cast<std::uint8_t>((in[off + i / 8] >> (i % 8)) & 1u);
        off += bytes;
        return out;
    }
};

}  // namespace pmsf

#endif  // PMSF_ENTROPY_STATIC_HUFFMAN_CODER_H
