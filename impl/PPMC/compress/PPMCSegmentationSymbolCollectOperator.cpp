#include "PPMC/compress/PPMCSegmentationSymbolCollectOperator.h"
#include "core/PMSFBoundarySideChannel.h"
#include "PPMC/boundary/PPMCPMSFBoundary.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include "BufferUtils.h"
#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <iostream>
#include <stdexcept>
#include <utility>
#include <vector>

namespace {
bool segmentedPPMCDebugLogs() {
    static const bool enabled = [] {
        const char* env = std::getenv("PMSF_SEGMENTED_PPMC_DEBUG");
        return env && env[0] != '\0' && std::strcmp(env, "0") != 0;
    }();
    return enabled;
}

bool segmentedPPMCEnvOn(const char* name) {
    return pmsf::pmsfStorageEnvEnabled(name);
}
}

void PPMCSegmentationSymbolCollectOperator::init(std::shared_ptr<MCGAL::Mesh> mesh,
                                                 std::vector<MCGAL::Halfedge*> seeds,
                                                 PMSFHeader header) {
    SymbolCollectOperator::init(mesh, seeds, header);
    ensureCollectors(seeds.size());
}

void PPMCSegmentationSymbolCollectOperator::ensureCollectors(std::size_t count) {
    if (collectors_.size() == count) {
        return;
    }
    collectors_.clear();
    collectors_.reserve(count);
    for (std::size_t i = 0; i < count; ++i) {
        auto collector = std::make_unique<PPMCSymbolCollectOperator>();
        collector->setEnablePrediction(prediction_);
        collector->setConnectivityPredictionFaces(facePrediction_);
        collector->setConnectivityPredictionEdges(edgePrediction_);
        collector->setUseTriangleMeshConnectivityPrediction(triangleBalance_);
        collector->setGroupFilter(static_cast<int>(i));
        collector->init(mesh_, seeds_, header_);
        collectors_.push_back(std::move(collector));
    }
}

void PPMCSegmentationSymbolCollectOperator::collect(MCGAL::Halfedge* seed) {
    if (collectors_.empty()) {
        ensureCollectors(1);
    }
    collectors_[0]->collect(seed);
    const int op = collectors_[0]->getDecimationCount() - 1;
    if (op >= 0 && collectors_[0]->hasOperationData(op)) {
        pmsf::PMSFSymbolRecord& row =
            pmsf::appendPMSFSymbolRecord(paperTable_, 0, 0, static_cast<std::int32_t>(op));
        pmsf::appendPPMCPaperContext(
            row.delta,
            1,
            static_cast<std::int32_t>(facePrediction_ ? 1 : 0),
            static_cast<std::int32_t>(edgePrediction_ ? 1 : 0),
            static_cast<std::int32_t>(triangleBalance_ ? 1 : 0),
            0,
            0,
            0,
            0,
            0,
            0);

        const auto& faceSym = collectors_[0]->getConnectFaceSym();
        if (op < static_cast<int>(faceSym.size())) {
            pmsf::appendPPMCPaperFaceSymbols(row.delta, faceSym[op]);
        }

        const auto& edgeSym = collectors_[0]->getConnectEdgeSym();
        if (op < static_cast<int>(edgeSym.size())) {
            pmsf::appendPPMCPaperEdgeSymbols(row.delta, edgeSym[op]);
        }

        const auto& geomSym = collectors_[0]->getGeometrySym();
        if (op < static_cast<int>(geomSym.size())) {
            pmsf::appendPPMCPaperGeometrySymbols(row.delta, geomSym[op]);
        }

    }
    roundsCollected_ = std::max(roundsCollected_, collectors_[0]->getDecimationCount());
}

void PPMCSegmentationSymbolCollectOperator::collect(std::vector<MCGAL::Halfedge*> seeds) {
    if (seeds.empty()) {
        return;
    }
    seeds_ = seeds;
    ensureCollectors(seeds.size());
    for (std::size_t i = 0; i < seeds.size(); ++i) {
        if (seeds[i] == nullptr) {
            continue;
        }
        collectors_[i]->collect(seeds[i]);
        const int op = collectors_[i]->getDecimationCount() - 1;
        if (op >= 0 && collectors_[i]->hasOperationData(op)) {
            pmsf::PMSFSymbolRecord& row = pmsf::appendPMSFSymbolRecord(
                paperTable_, 0, static_cast<std::int32_t>(i), static_cast<std::int32_t>(op));
            pmsf::appendPPMCPaperContext(
                row.delta,
                1,
                static_cast<std::int32_t>(facePrediction_ ? 1 : 0),
                static_cast<std::int32_t>(edgePrediction_ ? 1 : 0),
                static_cast<std::int32_t>(triangleBalance_ ? 1 : 0),
                0,
                0,
                0,
                0,
                0,
                static_cast<std::int32_t>(i));

            const auto& faceSym = collectors_[i]->getConnectFaceSym();
            if (op < static_cast<int>(faceSym.size())) {
                pmsf::appendPPMCPaperFaceSymbols(row.delta, faceSym[op]);
            }

            const auto& edgeSym = collectors_[i]->getConnectEdgeSym();
            if (op < static_cast<int>(edgeSym.size())) {
                pmsf::appendPPMCPaperEdgeSymbols(row.delta, edgeSym[op]);
            }

            const auto& geomSym = collectors_[i]->getGeometrySym();
            if (op < static_cast<int>(geomSym.size())) {
                pmsf::appendPPMCPaperGeometrySymbols(row.delta, geomSym[op]);
            }

        }
        roundsCollected_ = std::max(roundsCollected_, collectors_[i]->getDecimationCount());
    }
}

int PPMCSegmentationSymbolCollectOperator::exportToBuffer(char* buffer,
                                                          bool enableQuantization,
                                                          bool enableCompress) {
    if (pmsf::boundaryFlagOn(pmsf::ppmc_boundary::kPaperExactSpec)) {
        throw std::runtime_error(
            "[SegmentedPPMC] requested paper-exact replay, but curvature "
            "prediction is still disabled/fail-fast until the encoder and "
            "decoder formulas are proved symmetric.");
    }
    (void)enableQuantization;
    (void)enableCompress;
    const int groupCount = static_cast<int>(collectors_.size());
    const int roundCount = roundsCollected_;
    const size_t headerBytes = 2 * sizeof(int);
    const size_t startTableBytes = static_cast<size_t>(groupCount) * sizeof(int);
    const size_t roundTableBytes = static_cast<size_t>(groupCount) * static_cast<size_t>(roundCount) * sizeof(int);
    const size_t metaBytes = headerBytes + startTableBytes + roundTableBytes;

    std::vector<int> groupStart(groupCount, 0);
    std::vector<std::vector<int>> roundEnd(groupCount, std::vector<int>(roundCount, 0));
    size_t offset = metaBytes;
    std::uint64_t removedConnectivityBytes = 0;
    std::uint64_t insertedConnectivityBytes = 0;
    std::uint64_t residualBytes = 0;
    std::uint64_t removedStreamBytes = 0;
    std::uint64_t insertedStreamBytes = 0;
    std::uint64_t operations = 0;

    for (int group = 0; group < groupCount; ++group) {
        const size_t groupDataStart = offset;
        groupStart[group] = static_cast<int>(groupDataStart - metaBytes);
        for (int writeIdx = 0, op = roundCount - 1; op >= 0; --op, ++writeIdx) {
            int bytes = 0;
            if (collectors_[group] && collectors_[group]->hasOperationData(op)) {
                bytes = collectors_[group]->exportOperationToBuffer(op, buffer + offset);
                if (segmentedPPMCDebugLogs()) {
                    const auto& faceSym = collectors_[group]->getConnectFaceSym();
                    const auto& edgeSym = collectors_[group]->getConnectEdgeSym();
                    const auto& geomSym = collectors_[group]->getGeometrySym();
                    std::cerr << "[PMSF_SEGMENTED_PPMC_DEBUG] encode"
                              << " group=" << group
                              << " write_idx=" << writeIdx
                              << " op=" << op
                              << " bytes=" << bytes
                              << " face_symbols=" << faceSym[op].size()
                              << " residuals=" << geomSym[op].size()
                              << " edge_symbols=" << edgeSym[op].size()
                              << "\n";
                }
                const auto& breakdown = collectors_[group]->getLastEncodedByteBreakdown();
                removedConnectivityBytes += breakdown.removedConnectivityBytes;
                insertedConnectivityBytes += breakdown.insertedConnectivityBytes;
                residualBytes += breakdown.residualBytes;
                removedStreamBytes += breakdown.removedStreamBytes;
                insertedStreamBytes += breakdown.insertedStreamBytes;
                operations += breakdown.operations;
                offset += static_cast<size_t>(bytes);
            }
            roundEnd[group][writeIdx] = static_cast<int>(offset - groupDataStart);
        }
    }

    size_t metaOffset = 0;
    writeInt(buffer, metaOffset, groupCount);
    writeInt(buffer, metaOffset, roundCount);
    for (int group = 0; group < groupCount; ++group) {
        writeInt(buffer, metaOffset, groupStart[group]);
    }
    for (int group = 0; group < groupCount; ++group) {
        for (int round = 0; round < roundCount; ++round) {
            writeInt(buffer, metaOffset, roundEnd[group][round]);
        }
    }
    const std::uint64_t connectivityBytes =
        removedConnectivityBytes + insertedConnectivityBytes;
    const std::uint64_t payloadBytes = static_cast<std::uint64_t>(offset - metaBytes);
    std::cout << "[PPMC_SEGMENTED_RC_BREAKDOWN]"
              << " total_bytes=" << offset
              << " meta_bytes=" << metaBytes
              << " payload_bytes=" << payloadBytes
              << " connectivity_bytes=" << connectivityBytes
              << " geometry_bytes=" << residualBytes
              << " removed_connectivity_bytes=" << removedConnectivityBytes
              << " inserted_connectivity_bytes=" << insertedConnectivityBytes
              << " removed_stream_bytes=" << removedStreamBytes
              << " inserted_stream_bytes=" << insertedStreamBytes
              << " operations=" << operations
              << std::endl;
    return static_cast<int>(offset);
}

const pmsf::PMSFSymbolPayloadTable*
PPMCSegmentationSymbolCollectOperator::paperSymbolPayloadTable() const {
    paperPayloadTable_.clear();
    for (const auto& collector : collectors_) {
        if (!collector) {
            continue;
        }
        const auto* payloads = collector->paperSymbolPayloadTable();
        if (!payloads) {
            continue;
        }
        paperPayloadTable_.insert(
            paperPayloadTable_.end(), payloads->begin(), payloads->end());
    }
    return paperPayloadTable_.empty() ? nullptr : &paperPayloadTable_;
}

void PPMCSegmentationSymbolCollectOperator::setEnablePrediction(bool v) {
    prediction_ = v;
    facePrediction_ = v;
    edgePrediction_ = v;
    for (auto& collector : collectors_) {
        collector->setEnablePrediction(v);
    }
}

void PPMCSegmentationSymbolCollectOperator::setConnectivityPredictionFaces(bool v) {
    facePrediction_ = v;
    for (auto& collector : collectors_) {
        collector->setConnectivityPredictionFaces(v);
    }
}

void PPMCSegmentationSymbolCollectOperator::setConnectivityPredictionEdges(bool v) {
    edgePrediction_ = v;
    for (auto& collector : collectors_) {
        collector->setConnectivityPredictionEdges(v);
    }
}

void PPMCSegmentationSymbolCollectOperator::setUseTriangleMeshConnectivityPrediction(bool v) {
    triangleBalance_ = v;
    for (auto& collector : collectors_) {
        collector->setUseTriangleMeshConnectivityPrediction(v);
    }
}
