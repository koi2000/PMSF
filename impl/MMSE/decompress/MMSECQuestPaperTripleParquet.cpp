// SPDX-License-Identifier: Apache-2.0
//
// MMSECQuestPaperTripleParquet.cpp — Paper-backend decode triple for `mmse_cq`
// (M5-2, task-32 MMSE half).
//
// WHY THIS FILE EXISTS
// --------------------
// `algorithm=mmse_cq` is the faithful MMSE row (A&D valence-conquest backbone +
// MMSE paper §2.2 predictor stack + §2.3 mode trailer).  Its encoder already runs
// on the Paper backend (M5-0 probe: 59,750 B artifact), but decode SIGSEGV'd:
// `makeDecompressOptions(Backend::Paper, ...)` found no Paper registration for the
// name `mmse_cq`, fell through to the framework Default paper triple
// (`OptimizedSymbolReadOperator`), and that reader parsed the CQST layout-15 blob
// as an Optimized stream.  The columnar path was fine because
// MMSECQuestAlgorithm.cpp registers Binary/Columnar/Unified triples.
//
// WHAT IT REGISTERS
// -----------------
// `PaperDeserializeOperator` + the SAME native reader/reconstructor pair the
// columnar triple uses:
//   * MMSECQuestSymbolReadOperator : splits [u32 trailerLen][MMT1 mode trailer][CQST v2]
//     and hands the CQST part to the verified conquest reader; keeps the per-layer
//     2-bit mode stream in memory for the reconstructor;
//   * MMSECQuestReconstructOperator : installs the layer's mode stream, then calls
//     pmsf::decodeLayer with the shared MMSECQuestGeomState hook (paper §2.2
//     dual-ring/LS replay) — i.e. the exact encoder mirror.
//
// Paper v2 decode path (M4-4): PaperDeserializeOperator::initV2 rebuilds the
// legacy compact bin from the artifact base bundle, then
// ColumnarDeserializeOperator::initPaperV2Native() seeds header/quant/blobLayout
// from the V2CP compact payload and feeds the whole-frame native blob verbatim to
// the reader.  For layout 15 that is the conquest dispatch branch
// (`kPaperBlobLayoutConquestRC`), which already does:
//     dynamic_cast<ConquestSymbolReadOperator*>(reader)->setBufferSize(blob.size())
//     reader->importFromRangeCodedBuffer(blob.data(), 0, 0)
// Our reader derives from ConquestSymbolReadOperator, so its trailer-first
// override runs and then delegates the CQST payload to the base class.
//
// PAYLOAD CONTRACT (also asserted in impl/MMSE/M5_2_MMSE_CQ_PAPER_ADAPTER.md)
//   * blob_layout = 15 (kPaperBlobLayoutConquestRC), codec_version = 2;
//   * native frame = **Whole** (the first 4 bytes are our trailer length, NOT
//     "CONN") — storage must pass the bytes verbatim, must NOT slice CQST layers,
//     and must NOT strip our trailer prefix;
//   * the SBC1 wrapper is optional (initPaperV2Native already unwraps it), so the
//     storage side may compress the payload to reach the ≤54,601 B target.
//
// ENV GATE
//   PMSF_MMSE_CQ_PAPER_PAIR=0 disables the registration (A/B escape hatch that
//   reproduces the pre-M5-2 SIGSEGV baseline).  Default: enabled.  This change is
//   inert for every other algorithm/backend row: it only replaces the paper triple
//   lookup for the name `mmse_cq`, which had no paper registration before.
//
// Compiled only into the Arrow/Parquet-capable tools (PMSFCompress/PMSFDecompress)
// — the `*Parquet*` filename follows the repo convention documented in CLAUDE.md
// and mirrors src/decompress/PaperDecompressFactoryRegistrationParquet.cpp.

#include <cstdlib>

#include "core/DecompressOperatorRegistry.h"

#if defined(COLUMNAR_SUPPORT)
#include "MMSE/decompress/MMSECQuestSymbolReadOperator.h"
#include "MMSE/reconstruct/MMSECQuestReconstructOperator.h"
#include "decompress/PaperDeserializeOperator.h"
#include "predict/MMSEConquestHooks.h"

namespace pmsf {
namespace {

/// Same composition as makeMMSECQuestTriple() in MMSECQuestAlgorithm.cpp, but
/// wrapped in PaperDeserializeOperator (single-file `_paper.parquet` container).
/// The reconstructor MUST hold the same reader instance (the layer cursor lives
/// on the reader), and both MUST share one MMSECQuestGeomState (the decoder hook
/// replays the encoder's paper-predictor stack).
DecompressOperatorTriple makeMMSECQuestPaperTriple() {
    auto reader = std::make_shared<MMSECQuestSymbolReadOperator>();
    auto state = std::make_shared<mmse_pred::MMSECQuestGeomState>();
    auto reconstructor = std::make_shared<MMSECQuestReconstructOperator>(reader, state);
    auto deserializer = std::make_shared<PaperDeserializeOperator>(reader, reconstructor);
    return {deserializer, reader, reconstructor};
}

struct MMSECQuestPaperRegistrations {
    MMSECQuestPaperRegistrations() {
        // A/B escape hatch only; the faithful path is the registered triple.
        if (const char* env = std::getenv("PMSF_MMSE_CQ_PAPER_PAIR"); env && env[0] == '0') {
            return;
        }
        NamedDecompressOperatorRegistrar("mmse_cq", Backend::Paper,
                                         makeMMSECQuestPaperTriple);
    }
} g_mmseCqPaperRegistrations;

}  // namespace
}  // namespace pmsf

#endif  // COLUMNAR_SUPPORT
