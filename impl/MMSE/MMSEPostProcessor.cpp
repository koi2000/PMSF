/*
 * MMSEPostProcessor.cpp (Phase B.3 — framework predictor migration)
 *
 * The dualRingPrediction function has been promoted to
 * src/predict/DualRingPredictor.h as a framework-level Predictor hook.
 * This file now delegates the prediction math to that header, keeping only
 * the residual offset / facet iteration / quantization shift logic which
 * is specific to MMSE's residual encoding convention.
 *
 * Byte-identical with the pre-migration implementation: the predictor port
 * is a 1:1 copy of the float operations.
 */
#include "MMSEPostProcessor.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Point.h"
#include "predict/DualRingPredictor.h"

#include <vector>

static MCGAL::Point computeResidualOffset(std::shared_ptr<MCGAL::Mesh> mesh) {
    float half_range = (1 << (mesh->i_nbQuantBits - 1)) * mesh->f_quantStep;
    return MCGAL::Point(mesh->bboxMin.x() + half_range,
                        mesh->bboxMin.y() + half_range,
                        mesh->bboxMin.z() + half_range);
}

void MMSEPostProcessor::process(std::shared_ptr<MCGAL::Mesh> mesh) {
    mesh->garbage_collection();
    const bool useQuantization = mesh->enableQuantization;
    pmsf::DualRingPredictor predictor(alpha_);

    for (MCGAL::Facet* facet : mesh->faces()) {
        if (facet->isRemoved() || !facet->isSplittable()) continue;

        MCGAL::Point actual = facet->getRemovedVertexPos();
        MCGAL::Point prediction = predictor.predictForFacet(facet);
        MCGAL::Point residual = actual - prediction;

        if (useQuantization) {
            // Quantization branch historically kept `shifted = residual` —
            // the offset was never applied in process(). Preserve that exact
            // behaviour so compressed bytes don't drift.
            facet->setRemovedVertexPos(residual);
        } else {
            facet->setRemovedVertexPos(residual);
        }
    }
}

void MMSEPostProcessor::processWithGraph(std::shared_ptr<MCGAL::Mesh> /*mesh*/,
                                          std::shared_ptr<Graph> /*graph*/) {}

void MMSEPostProcessor::inverseProcess(std::shared_ptr<MCGAL::Mesh> mesh,
                                       std::vector<MCGAL::Facet*>& facets) {
    const MCGAL::Point offset = computeResidualOffset(mesh);
    const bool useQuantization = mesh->enableQuantization;
    pmsf::DualRingPredictor predictor(alpha_);

    for (MCGAL::Facet* facet : facets) {
        if (facet->isRemoved()) continue;

        MCGAL::Point stored = facet->getRemovedVertexPos();
        MCGAL::Point prediction = predictor.predictForFacet(facet);
        MCGAL::Point residual = useQuantization ? (stored - offset) : stored;
        facet->setRemovedVertexPos(prediction + residual);
    }
}
