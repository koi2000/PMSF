#include "VQMMCPostProcessor.h"
#include "VectorQuantizer.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Point.h"
#include "Vertex.h"
#include "common/VQMMCQuantization.h"
#include <cstdlib>
#include <cstring>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <limits>
#include <unordered_map>
#include <vector>

namespace {

bool vqmmcUsePredictiveResidual() {
    const char* absEnv = std::getenv("VQMMC_ABS_GEOM");
    if (absEnv && *absEnv && std::strcmp(absEnv, "0") != 0) {
        return false;
    }
    const char* env = std::getenv("VQMMC_USE_PREDICTIVE_RESIDUAL");
    if (env && *env) {
        return std::strcmp(env, "0") != 0;
    }
    return true;
}

bool vqmmcDisableCodebook() {
    const char* env = std::getenv("VQMMC_DISABLE_CODEBOOK");
    return env && *env && std::strcmp(env, "0") != 0;
}


}  // namespace

MCGAL::Point vqmmcBarycentricPrediction(MCGAL::Facet* facet) {
    MCGAL::Halfedge* h_start = facet->proxyHalfedge();
    MCGAL::Halfedge* h = h_start;
    MCGAL::Point sum(0, 0, 0);
    int count = 0;
    do {
        MCGAL::Vertex* v = h->vertex();
        if (v && !v->isRemoved()) {
            sum = sum + v->point();
            count++;
        }
        h = h->next();
    } while (h != h_start);

    if (count == 0) return MCGAL::Point(0, 0, 0);
    return sum / (double)count;
}

namespace {

std::vector<int> boundaryVertexIds(MCGAL::Facet* facet) {
    std::vector<int> ids;
    if (!facet || !facet->proxyHalfedge()) return ids;

    MCGAL::Halfedge* start = facet->proxyHalfedge();
    MCGAL::Halfedge* h = start;
    int guard = 0;
    do {
        if (!h || ++guard > 4096) {
            ids.clear();
            return ids;
        }
        MCGAL::Vertex* v = h->vertex();
        if (!v || v->isRemoved()) {
            ids.clear();
            return ids;
        }
        ids.push_back(v->globalVertexId());
        h = h->next();
    } while (h != start);
    return ids;
}

MCGAL::Point averageShadowPoints(
    const VQMMCPostProcessor::FacetTrace& trace,
    const std::unordered_map<int, MCGAL::Point>& shadow,
    bool* complete) {
    MCGAL::Point sum(0, 0, 0);
    int count = 0;
    bool ok = true;
    for (int id : trace.boundaryVertexIds) {
        auto it = shadow.find(id);
        if (it == shadow.end()) {
            ok = false;
            continue;
        }
        sum = sum + it->second;
        ++count;
    }
    if (complete) *complete = ok && count > 0;
    return count > 0 ? sum / static_cast<double>(count)
                     : MCGAL::Point(0, 0, 0);
}

// Mirror the wire quantization without modifying the codebook sent to any
// serializer. Constant axes always have q=0, independent of the stored step.
std::vector<MCGAL::Point> wireCodebook(
    const std::vector<MCGAL::Point>& codebook) {
    if (codebook.empty()) return {};
    const int bits = pmsf::vqmmcConfiguredCodebookQuantBits();
    const int quantMax = (1 << bits) - 1;
    float minimum[3] = {
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max()};
    float maximum[3] = {-minimum[0], -minimum[1], -minimum[2]};
    for (const auto& point : codebook) {
        const float values[3] = {point.x(), point.y(), point.z()};
        for (int k = 0; k < 3; ++k) {
            minimum[k] = std::min(minimum[k], values[k]);
            maximum[k] = std::max(maximum[k], values[k]);
        }
    }
    float step[3];
    for (int k = 0; k < 3; ++k) {
        const float range = maximum[k] - minimum[k];
        step[k] = (range > 0.0f ? range : 1e-9f) /
                  static_cast<float>(quantMax);
    }
    std::vector<MCGAL::Point> decoded;
    decoded.reserve(codebook.size());
    for (const auto& point : codebook) {
        const float values[3] = {point.x(), point.y(), point.z()};
        float coord[3];
        for (int k = 0; k < 3; ++k) {
            const int q = std::clamp(
                static_cast<int>(std::round((values[k] - minimum[k]) / step[k])),
                0, quantMax);
            coord[k] = minimum[k] + static_cast<float>(q) * step[k];
        }
        decoded.emplace_back(coord[0], coord[1], coord[2]);
    }
    return decoded;
}

MCGAL::Point wireBasePoint(const std::shared_ptr<MCGAL::Mesh>& mesh,
                           const MCGAL::Point& source) {
    if (!mesh->enableQuantization) return source;
    MCGAL::PointInt quantized = mesh->floatPosToInt(source);
    const int maxValue = (1 << mesh->i_nbQuantBits) - 1;
    for (int k = 0; k < 3; ++k) {
        quantized[k] = std::clamp(quantized[k], 0, maxValue);
    }
    return mesh->intPosToFloat(quantized);
}


}  // namespace

void VQMMCPostProcessor::process(std::shared_ptr<MCGAL::Mesh> mesh) {
    mesh->garbage_collection();

    std::vector<MCGAL::Point> residuals;
    std::vector<MCGAL::Facet*> splittableFacets;
    RoundVQData roundData;
    for (MCGAL::Facet* facet : mesh->faces()) {
        if (facet->isRemoved() || !facet->isSplittable()) continue;
        MCGAL::Point actual = facet->getRemovedVertexPos();
        MCGAL::Point prediction(0, 0, 0);
        if (vqmmcUsePredictiveResidual()) {
            prediction = vqmmcBarycentricPrediction(facet);
            residuals.push_back(actual - prediction);
        } else {
            residuals.push_back(actual);
        }
        FacetTrace trace;
        trace.facet = facet;
        auto removedId = removedVertexIds_.find(facet);
        trace.removedVertexId =
            removedId == removedVertexIds_.end() ? -1 : removedId->second;
        trace.boundaryVertexIds = boundaryVertexIds(facet);
        trace.actualPosition = actual;
        trace.sourcePrediction = prediction;
        roundData.traces.push_back(std::move(trace));
        splittableFacets.push_back(facet);
    }

    if (residuals.empty()) {
        roundData_.push_back(roundData);
        return;
    }

    if (vqmmcDisableCodebook()) {
        roundData.codebook = residuals;
        for (size_t i = 0; i < residuals.size(); i++) {
            roundData.facetToIndex[splittableFacets[i]] = (uint16_t)i;
            roundData.traces[i].codebookIndex = static_cast<uint16_t>(i);
            splittableFacets[i]->setRemovedVertexPos(residuals[i]);
        }
        std::cout << "[VQMMC] Round " << roundData_.size() << ": "
                  << residuals.size()
                  << " residuals, codebook=disabled(identity)" << std::endl;
        roundData_.push_back(std::move(roundData));
        return;
    }

    int requestedCBSize = std::min(codebookSize_, (int)residuals.size());

    VectorQuantizer vq(requestedCBSize, 3);
    vq.trainCodebook(residuals, 300, 1e-5);

    const auto& cb = vq.getCodebook();
    int actualCBSize = (int)cb.codewords.size();
    roundData.codebook.resize(actualCBSize);
    for (int i = 0; i < actualCBSize; i++) {
        roundData.codebook[i] = cb.codewords[i];
    }
    // Paper alignment: VQM uses a full-rate codebook plus a progressive
    // transmission tree over that codebook. trainCodebook() already builds
    // the tree; keep it in the round model so later bit-plane allocation /
    // equalization can use the paper structure without changing today's
    // leaf-index byte stream.
    roundData.progressiveTree = cb.progressiveTree;

    for (size_t i = 0; i < residuals.size(); i++) {
        int nearestIdx = 0;
        double minDist = std::numeric_limits<double>::max();
        for (int j = 0; j < actualCBSize; j++) {
            MCGAL::Point diff = residuals[i] - roundData.codebook[j];
            double dist = diff.dot(diff);
            if (dist < minDist) {
                minDist = dist;
                nearestIdx = j;
            }
        }
        roundData.facetToIndex[splittableFacets[i]] = (uint16_t)nearestIdx;
        roundData.traces[i].codebookIndex = static_cast<uint16_t>(nearestIdx);

        MCGAL::Point decodedResidual = roundData.codebook[nearestIdx];
        splittableFacets[i]->setRemovedVertexPos(decodedResidual);
    }

    std::cout << "[VQMMC] Round " << roundData_.size() << ": "
              << residuals.size() << " residuals, codebook=" << actualCBSize << std::endl;

    // P16（2026-09-25）：binary 口径的**逐轮体量分解**。产物是「8 字节头 + 单个
    // 不透明 blob」，读不出来，只能在这里累计各类载荷的**未压缩**体量：
    //   · 残差向量数（几何条目数）
    //   · 码本索引位数 = 残差数 × ceil(log2(cbSize))   ← 索引流的下界
    //   · 码本浮点数 = cbSize × 3
    // 仅 env 门控、只打印，不改行为。
    
    roundData_.push_back(std::move(roundData));
}

void VQMMCPostProcessor::recordRemovedVertex(MCGAL::Facet* facet, int vertexId) {
    if (facet) removedVertexIds_[facet] = vertexId;
}

void VQMMCPostProcessor::finalizeDecoderEquivalentGeometry(
    const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (!mesh || !vqmmcUsePredictiveResidual() || roundData_.empty()) return;

    std::unordered_map<int, MCGAL::Point> shadow;
    shadow.reserve(mesh->vertices().size() + 1024);
    for (MCGAL::Vertex* vertex : mesh->vertices()) {
        if (!vertex || vertex->isRemoved()) continue;
        shadow[vertex->globalVertexId()] = wireBasePoint(mesh, vertex->point());
    }

    for (int round = static_cast<int>(roundData_.size()) - 1; round >= 0;
         --round) {
        RoundVQData& data = roundData_[static_cast<std::size_t>(round)];
        if (data.traces.empty()) continue;

        std::vector<MCGAL::Point> residuals;
        residuals.reserve(data.traces.size());
        std::vector<MCGAL::Point> predictions;
        predictions.reserve(data.traces.size());
        for (const FacetTrace& trace : data.traces) {
            bool complete = false;
            MCGAL::Point prediction =
                averageShadowPoints(trace, shadow, &complete);
            if (!complete || trace.removedVertexId < 0) {
                throw std::runtime_error(
                    "[VQMMC] decoder-state geometry replay is incomplete");
            }
            predictions.push_back(prediction);
            residuals.push_back(trace.actualPosition - prediction);
        }

        if (vqmmcDisableCodebook()) {
            data.codebook = residuals;
        } else {
            const int requestedCBSize =
                std::min(codebookSize_, static_cast<int>(residuals.size()));
            VectorQuantizer vq(requestedCBSize, 3);
            vq.trainCodebook(residuals, 300, 1e-5);
            const auto& trained = vq.getCodebook();
            data.codebook = trained.codewords;
            data.progressiveTree = trained.progressiveTree;
        }

        const std::vector<MCGAL::Point> decodedCodebook =
            wireCodebook(data.codebook);
        if (decodedCodebook.empty()) {
            throw std::runtime_error(
                "[VQMMC] decoder-state geometry replay produced empty codebook");
        }

        std::vector<MCGAL::Point> reconstructedPositions;
        reconstructedPositions.reserve(data.traces.size());
        for (std::size_t i = 0; i < data.traces.size(); ++i) {
            int nearest = 0;
            double best = std::numeric_limits<double>::max();
            for (std::size_t j = 0; j < decodedCodebook.size(); ++j) {
                MCGAL::Point diff = residuals[i] - decodedCodebook[j];
                const double distance = diff.dot(diff);
                if (distance < best) {
                    best = distance;
                    nearest = static_cast<int>(j);
                }
            }
            FacetTrace& trace = data.traces[i];
            trace.sourcePrediction = predictions[i];
            trace.codebookIndex = static_cast<uint16_t>(nearest);
            data.facetToIndex[trace.facet] = trace.codebookIndex;
            MCGAL::Point decodedResidual =
                decodedCodebook[static_cast<std::size_t>(nearest)];
            const MCGAL::Point reconstructed =
                predictions[i] + decodedResidual;
            reconstructedPositions.push_back(reconstructed);
            if (trace.facet) trace.facet->setRemovedVertexPos(decodedResidual);
        }
        // The decoder inverse-processes the complete round before any of the
        // reconstructed vertices become visible to the next prediction.
        // Keep the replay shadow on the same round boundary; updating it
        // inside the trace loop makes later facets observe encoder-side
        // insertions that the real decoder cannot observe yet.
        for (std::size_t i = 0; i < data.traces.size(); ++i) {
            shadow[data.traces[i].removedVertexId] = reconstructedPositions[i];
        }
    }
}


void VQMMCPostProcessor::processWithGraph(std::shared_ptr<MCGAL::Mesh> mesh, std::shared_ptr<Graph> graph) {}

void VQMMCPostProcessor::inverseProcess(std::shared_ptr<MCGAL::Mesh> mesh,
                                         std::vector<MCGAL::Facet*>& facets) {
    for (MCGAL::Facet* facet : facets) {
        if (facet->isRemoved()) continue;
        MCGAL::Point stored = facet->getRemovedVertexPos();
        MCGAL::Point prediction(0, 0, 0);
        if (facet->getProtocolHint() == 21) {
            facet->setRemovedVertexPos(stored);
        } else {
            prediction = vqmmcBarycentricPrediction(facet);
            facet->setRemovedVertexPos(prediction + stored);
        }
            }
}
