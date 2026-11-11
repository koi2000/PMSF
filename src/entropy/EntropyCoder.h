/*
 * @file EntropyCoder.h
 * @brief Pluggable entropy-coding backend, lifted out of SymbolCollectOperator.
 *
 * Today every algorithm embeds its coding choices (range coder / static
 * Huffman / context-adaptive arithmetic) inline. Factoring into a framework
 * hook enables:
 *   - cross-algorithm coder comparison at benchmark time ("MMSE with rANS
 *     vs. MMSE with context-AC"),
 *   - minimal-LOC new algorithms that only specify their coder by name,
 *   - deterministic round-trip — encoder/decoder share the same concrete
 *     coder instance constructed via the Builder.
 *
 * Phase 10 addition. Legacy SymbolCollectOperator codepaths keep using
 * their embedded coders; this hook is opt-in.
 */
#ifndef PMSF_ENTROPY_ENTROPY_CODER_H
#define PMSF_ENTROPY_ENTROPY_CODER_H

#include <cstddef>
#include <cstdint>
#include <vector>

namespace pmsf {

/// Minimal symmetric encode / decode interface. Symbols are uint32_t so
/// the same coder can accept codebook indices, tag ids, quantized
/// residuals, or packed bit-plane cells. Implementations are responsible
/// for any modelling state (alphabet size, context tables).
class EntropyCoder {
public:
    virtual ~EntropyCoder() = default;

    /// Reset internal state. Call before every encode() or decode() run.
    virtual void reset() = 0;

    /// Encode a sequence of symbols into a byte buffer. Implementations
    /// append to `out`; callers may pre-reserve.
    virtual void encode(const std::vector<std::uint32_t>& symbols,
                        std::vector<std::uint8_t>& out) = 0;

    /// Decode `count` symbols out of `in`. Returns the number of bytes
    /// consumed, so the caller can chain multiple blocks in a round.
    virtual std::size_t decode(const std::uint8_t* in,
                               std::size_t in_bytes,
                               std::size_t count,
                               std::vector<std::uint32_t>& out) = 0;
};

}  // namespace pmsf

#endif  // PMSF_ENTROPY_ENTROPY_CODER_H
