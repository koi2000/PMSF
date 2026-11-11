/*****************************************************************************
 * @Author: koi
 * @Description: 算法专属的 default paper-boundary 配置函数实现。
 * 阶段 F：把 src/core/DefaultPMSFBoundary.h 中的算法相关 inline 配置下沉到 impl/，
 * 使 src/core/DefaultPMSFBoundary.h 只剩通用 helper。
 *****************************************************************************/
#ifndef IMPL_COMMON_DEFAULT_PMSF_BOUNDARY_IMPL_H
#define IMPL_COMMON_DEFAULT_PMSF_BOUNDARY_IMPL_H

#include "core/DefaultPMSFBoundary.h"

#include "QEM/boundary/QEMPMSFBoundary.h"
#include "APTM/boundary/APTMPMSFBoundary.h"
#include "PCATM/boundary/PCATMPMSFBoundary.h"
#include "PCLTTM/boundary/PCLTTMPMSFBoundary.h"
#include "MMSE/boundary/MMSEPMSFBoundary.h"
#include "RD/boundary/RDPMSFBoundary.h"

namespace pmsf {

inline void configureQemPMSFBoundary(CompressPipelineConfig& cfg) {
    if (pmsf::boundaryFlagOn(qem_boundary::kPaperExactSpec)) {
        throw std::runtime_error(
            "PMSF_QEM_REQUIRE_PMSF_EXACT=1 requested Garland-Heckbert "
            "paper-exact QEM, but current PMSF QEM still uses the shared "
            "vertex-removal reconstruction path instead of strict "
            "edge-contraction / close-pair replay.");
    }

    addDefaultPaperSidecarValues(
        cfg,
        {{"qem_paper_exact", 0},
         {"qem_paper_exact_required", 0},
         {"qem_quadric_cost_exact", 1},
         {"qem_q_matrix_refresh_exact", 1},
         {"qem_boundary_guard_exact", 1},
         {"qem_edge_contraction_exact", 0},
         {"qem_close_pair_exact", 0},
         {"qem_simplification_rate_exact", 0},
         {"qem_manifold_guard_stricter", 1}});
}

inline void configureAptmPMSFBoundary(CompressPipelineConfig& cfg) {
    if (pmsf::boundaryFlagOn(aptm_boundary::kPaperExactSpec)) {
        throw std::runtime_error(
            "APTM paper-exact replay requested, but default APTM currently "
            "uses anchor-aware removal constraints over the shared "
            "vertex-removal reconstruction path; APTM polar geometry and "
            "anchor-aware reconstruction are not implemented yet.");
    }

    addDefaultPaperSidecarValues(
        cfg,
        {{"aptm_default_storage", 1},
         {"aptm_anchor_preprocess", 1},
         {"aptm_degree_policy", 1},
         {"aptm_paper_exact", 0},
         {"aptm_paper_exact_required", 0},
         {"aptm_anchor_aware_reconstruct", 0},
         {"aptm_anchor_reconstruct_exact", 0},
         {"aptm_anchor_reconstruct_required", 0},
         {"aptm_polar_geometry", 0},
         {"aptm_polar_geometry_exact", 0},
         {"aptm_polar_geometry_required", 0}});
}

inline void configurePcatmDefaultPMSFBoundary(CompressPipelineConfig& cfg) {
    if (pmsf::boundaryFlagOn(pmsf::pcatm_boundary::kPaperExactSpec)) {
        throw std::runtime_error(
            "PCATM paper-exact replay requested, but default PCATM uses the "
            "shared optimized topology stream; exact color-only Z-shape "
            "reconstruction requires a PCATM-specific strict collector.");
    }

    addDefaultPaperSidecarValues(
        cfg,
        {{"pcatm_default_storage", 1},
         {"pcatm_shared_topology_storage", 1},
         {"pcatm_independent_set", 1},
         {"pcatm_paper_exact", 0},
         {"pcatm_paper_exact_required", 0},
         {"pcatm_z_shape_exact", 0},
         {"pcatm_z_shape_exact_required", 0},
         {"pcatm_color_stream_exact", 0}});
}

inline void configurePclttmDefaultPMSFBoundary(CompressPipelineConfig& cfg) {
    if (pmsf::boundaryFlagOn(pmsf::pclttm_boundary::kAnyPaperOrFig9Spec)) {
        throw std::runtime_error(
            "PCLTTM paper-exact replay requested, but default PCLTTM uses the "
            "shared optimized topology stream instead of the valence/null tag "
            "paper protocol and Fig.9 table replay.");
    }

    addDefaultPaperSidecarValues(
        cfg,
        {{"pclttm_default_storage", 1},
         {"pclttm_shared_topology_storage", 1},
         {"pclttm_paper_exact", 0},
         {"pclttm_paper_exact_required", 0},
         {"pclttm_topology_replay_exact", 0},
         {"pclttm_topology_replay_exact_required", 0},
         {"pclttm_geometry_replay_exact", 0},
         {"pclttm_geometry_replay_exact_required", 0},
         {"pclttm_fig9_exact", 0},
         {"pclttm_fig9_exact_required", 0}});
}

inline void configureMmseDefaultPMSFBoundary(CompressPipelineConfig& cfg) {
    if (pmsf::boundaryFlagOn(pmsf::mmse_boundary::kPaperExactSpec)) {
        throw std::runtime_error(
            "MMSE paper-exact replay requested, but default MMSE uses the "
            "shared optimized topology stream plus postprocess metadata; "
            "strict dual-ring storage and mode-decision replay are not the "
            "default binary/parquet pipeline.");
    }

    addDefaultPaperSidecarValues(
        cfg,
        {{"mmse_default_storage", 1},
         {"mmse_shared_topology_storage", 1},
         {"mmse_postprocess_dual_ring", 1},
         {"mmse_paper_exact", 0},
         {"mmse_paper_exact_required", 0},
         {"mmse_strict_dual_ring", 0},
         {"mmse_strict_dual_ring_required", 0},
         {"mmse_mode_decision_exact", 0},
         {"mmse_mode_decision_exact_required", 0}});
}

inline void configureRdDefaultPMSFBoundary(CompressPipelineConfig& cfg) {
    if (pmsf::rdPaperExactRequired()) {
        throw std::runtime_error(
            "RD paper-exact replay requested, but default RD uses the shared "
            "optimized topology stream with longest-edge seeding, not the "
            "prioritized gate / curvature / KLT paper protocol.");
    }

    addDefaultPaperSidecarValues(
        cfg,
        {{"rd_default_storage", 1},
         {"rd_shared_topology_storage", 1},
         {"rd_longest_edge_seed", 1},
         {"rd_paper_exact", 0},
         {"rd_paper_exact_required", 0},
         {"rd_prioritized_gate_selection_exact", 0},
         {"rd_prioritized_gate_selection_exact_required", 0},
         {"rd_curvature_prediction_exact", 0},
         {"rd_curvature_prediction_exact_required", 0},
         {"rd_klt_exact", 0},
         {"rd_klt_exact_required", 0}});
}

}  // namespace pmsf

#endif  // IMPL_COMMON_DEFAULT_PMSF_BOUNDARY_IMPL_H
