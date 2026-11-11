#ifndef PMSF_STORAGE_CANONICAL_SYMBOL_ENCODER_H
#define PMSF_STORAGE_CANONICAL_SYMBOL_ENCODER_H

#include "core/Symbol.h"
#include "core/SymbolStreamSpec.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace pmsf {

/// One entropy-encoded scalar stream for a single round.
struct CanonicalEncodedStream {
    std::string name;
    SymbolStreamRole role = SymbolStreamRole::SideChannel;
    std::int32_t roundId = 0;
    std::int32_t streamId = -1;
    std::int32_t symbolCount = 0;
    std::int32_t alphabetBits = 0;  ///< 0 means raw / opaque bytes
    std::vector<std::uint8_t> bytes;
};

/// Encoded geometry for a single round.
/// coordFormat: 0 = raw float triples, 1 = quantized integer coordinates
/// (bit-packed, coordBits per component), 3 = signed residual integer
/// bit-plane (used with prediction/quantization).
struct CanonicalEncodedGeometry {
    std::int32_t roundId = 0;
    std::int32_t pointCount = 0;
    std::vector<std::uint8_t> pointBytes;          ///< format byte + count + data
    std::int32_t vertexPointCount = 0;
    std::vector<std::uint8_t> vertexPointBytes;    ///< format byte + count + data
    std::int8_t coordFormat = 0;
    std::int8_t coordBits = 0;
};

/// Encoded side-channel for a single round.
struct CanonicalEncodedSideChannel {
    std::string name;
    SideChannelSpec::Type cellType = SideChannelSpec::Type::Int32;
    SideChannelSpec::Shape shape = SideChannelSpec::Shape::Scalar;
    std::int32_t roundId = 0;
    std::int32_t cellCount = 0;
    std::int32_t alphabetBits = 0;
    std::vector<std::uint8_t> bytes;
};

/// Complete encoded payload for a (group of) round(s).
struct CanonicalSymbolPayload {
    std::int32_t groupId = 0;
    std::int32_t numRounds = 0;
    std::vector<CanonicalEncodedStream> bitStreams;        ///< per round, in round-major order
    std::vector<CanonicalEncodedGeometry> geometry;         ///< one entry per round
    std::vector<CanonicalEncodedSideChannel> sideChannels;  ///< per round, in round-major order
    std::vector<std::uint8_t> headerBlob;                   ///< optional algorithm header (for legacy reassembly)
};

/// Encodes a sequence of `PaperRoundSymbols` into a canonical byte payload.
/// The payload can then be wrapped into Arrow/Parquet tables by the codec=4
/// writer, or written directly to disk.
class CanonicalSymbolEncoder {
public:
    explicit CanonicalSymbolEncoder(const std::vector<SymbolStreamSpec>& specs);

    /// Append one round. The encoder internally maps facet/edge/flip/vertex
    /// symbols and coordinates according to the registered specs.
    void addRound(const PaperRoundSymbols& round);

    /// Finish encoding and return the payload. The encoder may be reused after
    /// calling finish() by calling clear().
    CanonicalSymbolPayload finish();

    void clear();

    /// When quantization is enabled, store geometry as bit-packed unsigned
    /// integers instead of raw floats. This typically cuts the residual table
    /// size by 2-3x while keeping the decoded geometry bit-identical.
    void setQuantization(int bits, MCGAL::Point bboxMin, float quantStep);

    /// When prediction is enabled, store geometry as signed integer residuals
    /// (relative to the decoder-side barycenter predictor) instead of absolute
    /// coordinates. This matches the legacy RD path and yields the smallest BPV.
    void setUseResidual(bool use);

    const std::vector<SymbolStreamSpec>& specs() const { return specs_; }

private:
    std::vector<SymbolStreamSpec> specs_;
    CanonicalSymbolPayload payload_;

    bool useQuantization_ = false;
    int quantBits_ = 0;
    MCGAL::Point quantBboxMin_;
    float quantStep_ = 0.0f;
    bool useResidual_ = false;
    int roundsAdded_ = 0;

    // Per-round buffers. Each round's symbols / geometry / side channels are kept
    // separate so the decoder can reconstruct per-round PaperRoundSymbols. This is
    // required for non-Class-I algorithm reassembly (RD_TAG, CODEC_016, etc.).
    std::vector<std::unordered_map<SymbolStreamRole, std::deque<SymbolCode>>>
        scalarBuffers_;
    std::vector<std::deque<MCGAL::PointInt>> removedResidualBuffers_;
    std::vector<std::deque<MCGAL::PointInt>> shiftedResidualBuffers_;
    std::vector<std::deque<MCGAL::Point>> removedCoordBuffers_;
    std::vector<std::deque<MCGAL::Point>> shiftedCoordBuffers_;
    std::vector<std::unordered_map<std::string, std::vector<SideCell>>>
        sideBuffers_;

    void encodeScalarStream(const SymbolStreamSpec& spec,
                            const std::deque<SymbolCode>& symbols,
                            std::int32_t roundId);
    void flushScalarStreams();
    void encodeGeometry(std::int32_t roundId,
                        const std::deque<MCGAL::PointInt>& removedResiduals,
                        const std::deque<MCGAL::PointInt>& shiftedResiduals,
                        const std::deque<MCGAL::Point>& removedCoords,
                        const std::deque<MCGAL::Point>& shiftedCoords);
    void encodeSideChannel(const SymbolStreamSpec& spec,
                           const std::vector<SideCell>& cells,
                           std::int32_t roundId);
    void flushSideChannels();
};

/// Decodes a canonical payload back into `PaperRoundSymbols`.
class CanonicalSymbolDecoder {
public:
    CanonicalSymbolDecoder(const CanonicalSymbolPayload& payload,
                           const std::vector<SymbolStreamSpec>& specs);

    /// Provide quantization parameters so that format=1 geometry can be
    /// dequantized back to floating-point coordinates.
    void setQuantization(int bits, MCGAL::Point bboxMin, float quantStep);

    std::vector<PaperRoundSymbols> decode() const;

private:
    const CanonicalSymbolPayload& payload_;
    std::vector<SymbolStreamSpec> specs_;

    bool useQuantization_ = false;
    int quantBits_ = 0;
    MCGAL::Point quantBboxMin_;
    float quantStep_ = 0.0f;

    void decodeScalarStream(const SymbolStreamSpec& spec,
                            const CanonicalEncodedStream& encoded,
                            std::deque<SymbolCode>& out) const;
    void decodeGeometry(const CanonicalEncodedGeometry& geom,
                        PaperRoundSymbols& round) const;
    void decodeSideChannel(const SymbolStreamSpec& spec,
                           const CanonicalEncodedSideChannel& encoded,
                           PaperRoundSymbols& round) const;
};

}  // namespace pmsf

#endif  // PMSF_STORAGE_CANONICAL_SYMBOL_ENCODER_H
