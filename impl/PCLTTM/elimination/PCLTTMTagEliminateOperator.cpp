// SPDX-License-Identifier: Apache-2.0
//
// PCLTTMTagEliminateOperator.cpp — see header for design contract.

#include "PCLTTMTagEliminateOperator.h"

#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Vertex.h"
#include "common/Define.h"
#include "common/PCLTTMTagTable.h"

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <vector>

namespace {

inline pmsf::pclttm::Tag toCommonTag(MCGAL::Vertex::TagFlag t) {
    switch (t) {
        case MCGAL::Vertex::Plus:  return pmsf::pclttm::Tag::Plus;
        case MCGAL::Vertex::Minus: return pmsf::pclttm::Tag::Minus;
        default:                   return pmsf::pclttm::Tag::Untagged;
    }
}

inline MCGAL::Vertex::TagFlag toMcgalTag(pmsf::pclttm::Tag t) {
    switch (t) {
        case pmsf::pclttm::Tag::Plus:  return MCGAL::Vertex::Plus;
        case pmsf::pclttm::Tag::Minus: return MCGAL::Vertex::Minus;
        default:                       return MCGAL::Vertex::Untagged;
    }
}

bool envOn(const char* key) {
    const char* v = std::getenv(key);
    return v && *v && std::strcmp(v, "0") != 0;
}

int faceDegree(MCGAL::Facet* f) {
    if (!f) return 0;
    int n = 0;
    for (MCGAL::Halfedge* h : *f) {
        if (h) ++n;
    }
    return n;
}

bool faceContainsVertex(MCGAL::Facet* f, MCGAL::Vertex* v) {
    if (!f || !v) return false;
    for (MCGAL::Halfedge* h : *f) {
        if (h && h->vertex() == v) return true;
    }
    return false;
}

MCGAL::Halfedge* incomingHalfedgeTo(MCGAL::Facet* f, MCGAL::Vertex* v) {
    if (!f || !v) return nullptr;
    for (MCGAL::Halfedge* h : *f) {
        if (h && h->end_vertex() == v) return h;
    }
    return nullptr;
}

}  // namespace

bool PCLTTMTagEliminateOperator::triangulateWithTagTable(
    MCGAL::Facet* polyFace,
    const std::vector<MCGAL::Vertex*>& polyVerts) {

    if (!polyFace) return false;
    const int N = static_cast<int>(polyVerts.size());
    if (N < 3) return false;
    if (N == 3) {
        polyFace->setSplittable();
        return true;
    }

    std::vector<pmsf::pclttm::Tag> tagSeq;
    tagSeq.reserve(polyVerts.size());
    for (MCGAL::Vertex* v : polyVerts) {
        tagSeq.push_back(v ? toCommonTag(v->getTag())
                           : pmsf::pclttm::Tag::Untagged);
    }

    const auto retri = pmsf::pclttm::computeTagGuidedRetriangulation(tagSeq);
    if (!retri.valid ||
        static_cast<int>(retri.diagonals.size()) != N - 3 ||
        static_cast<int>(retri.triangles.size()) != N - 2) {
        std::cerr << "[PCLTTMTag] tag-table retriangulation invalid N="
                  << N << std::endl;
        return false;
    }

    const int groupId = polyFace->groupId();
    polyFace->setSplittable();
    std::vector<MCGAL::Facet*> patchFaces;
    patchFaces.push_back(polyFace);

    for (const auto& diag : retri.diagonals) {
        if (diag.va < 0 || diag.vb < 0 || diag.va >= N || diag.vb >= N) {
            return false;
        }
        MCGAL::Vertex* va = polyVerts[static_cast<std::size_t>(diag.va)];
        MCGAL::Vertex* vb = polyVerts[static_cast<std::size_t>(diag.vb)];
        MCGAL::Facet* splitFace = nullptr;
        for (MCGAL::Facet* candidate : patchFaces) {
            if (candidate && !candidate->isRemoved() &&
                faceDegree(candidate) > 3 &&
                faceContainsVertex(candidate, va) &&
                faceContainsVertex(candidate, vb)) {
                splitFace = candidate;
                break;
            }
        }
        if (!splitFace) {
            std::cerr << "[PCLTTMTag] no split face for tag-table diagonal "
                      << diag.va << "-" << diag.vb << " N=" << N << std::endl;
            return false;
        }

        MCGAL::Halfedge* ha = incomingHalfedgeTo(splitFace, va);
        MCGAL::Halfedge* hb = incomingHalfedgeTo(splitFace, vb);
        if (!ha || !hb || ha == hb) {
            std::cerr << "[PCLTTMTag] no incoming halfedges for tag-table diagonal "
                      << diag.va << "-" << diag.vb << " N=" << N << std::endl;
            return false;
        }

        MCGAL::Halfedge* hd = mesh_->split_facet(ha, hb);
        if (!hd || !hd->opposite()) {
            std::cerr << "[PCLTTMTag] split_facet failed for tag-table N="
                      << N << std::endl;
            return false;
        }
        hd->setAdded();
        hd->opposite()->setAdded();
        if (va && !va->isRemoved() && !va->is_degree_dirty()) {
            va->incCachedDegree(1);
        }
        if (vb && !vb->isRemoved() && !vb->is_degree_dirty()) {
            vb->incCachedDegree(1);
        }

        MCGAL::Facet* fNew = hd->opposite()->face();
        if (fNew) {
            fNew->setGroupId(groupId);
            fNew->setSplittable();
            patchFaces.push_back(fNew);
        }
        if (splitFace) {
            splitFace->setGroupId(groupId);
            splitFace->setSplittable();
        }
    }
    return true;
}

MCGAL::Vector3i PCLTTMTagEliminateOperator::computeRemovedVertexResidualInt(
    MCGAL::Halfedge* entry, MCGAL::Halfedge* patch,
    const MCGAL::Point& point, const MCGAL::PointInt& target) {
    if (frenetPatches_) {
        using namespace pmsf::pclttm_frenet;
        auto* face=patch?patch->face():nullptr;
        const auto ring=patchCells(face,*mesh_,header_.isEnableAdjustment());
        const auto code=projectPatch(makeFrame(ring),toCell(target));
        if(!frenetPatches_->emplace(face,code).second)
            throw std::runtime_error("PCLTTM Frenet duplicate patch before collection");
    }
    return VertexRemovalEliminateOperator::computeRemovedVertexResidualInt(entry,patch,point,target);
}

bool PCLTTMTagEliminateOperator::eliminate(HalfedgeErrorEntry& entry) {
    MCGAL::Halfedge*& h = entry.edge;
    if (h == nullptr) return false;

    // 1) adjustment 逻辑（同父类）
    if (header_.isEnableAdjustment()) {
        h->vertex()->setAdjusted();
        h->vertex()->setVertexPoint(h->vertex()->point());
        h->vertex()->setPoint(entry.p);
    }

    // 2) remove_point —— polyFace 现在是 N-gon polygon
    CHECK_TRUE(VertexRemovalEliminateOperator::remove_point(h));

    // 3) Phase 2 Step 3：在 polygon 状态下 normalize polyFace.proxyHalfedge
    //    到 vertex **量化整数坐标**字典序最小的 outgoing halfedge —— encoder/
    //    decoder 共用规则，bit-exact 对齐。后续 tag 演化 + triangulate 都基于
    //    normalize 后的 polyVerts 顺序。
    MCGAL::Facet* polyFace = h->face();
    if (polyFace) {
        polyFace->normalize_proxy_to_min_position_vertex(
            mesh_->bboxMin, mesh_->f_quantStep);
    }

    // 4) tag 演化（基于 normalize 后的 polyVerts）
    if (polyFace) {
        std::vector<MCGAL::Vertex*> polyVerts = polyFace->getVertices();
        const int N = static_cast<int>(polyVerts.size());
        if (N >= 3) {
            const auto tagLeft  = toCommonTag(polyVerts[0]->getTag());
            const auto tagRight = toCommonTag(polyVerts[N - 1]->getTag());
            pmsf::pclttm::applyTagEvolution<MCGAL::Vertex>(
                polyVerts, tagLeft, tagRight,
                pmsf::pclttm::AnchorKind::V0First,
                /*getTagFn=*/[](MCGAL::Vertex* v) {
                    return toCommonTag(v->getTag());
                },
                /*setTagFn=*/[](MCGAL::Vertex* v, pmsf::pclttm::Tag t) {
                    v->setTag(toMcgalTag(t));
                },
                /*poolIdFn=*/[](MCGAL::Vertex* v) { return v->poolId(); },
                roundTagEvolutions_);
        }

        // CODEC_016 BlobLayout=16/sv=3 remains Cartesian-residual only.
        // Frenet residual experiments must use a layout that serializes and
        // advertises that mode explicitly; the collector fail-fasts otherwise.
    }

    // 5) triangulate 用 normalize 后的 proxyHalfedge 作起点。
    // 默认仍走历史 fan，以保护现有 BPV/时间基线；显式打开时使用共享的
    // tag-guided descriptor，向论文 Figure 9 tag-table 几何收敛。
    if (polyFace && envOn("PMSF_PCLTTM_USE_TAG_TABLE_TRIANGULATION")) {
        CHECK_TRUE(triangulateWithTagTable(polyFace, polyFace->getVertices()));
        CHECK_TRUE(VertexRemovalEliminateOperator::postprocess(polyFace->proxyHalfedge()));
        entry.edge = polyFace->proxyHalfedge();
        return true;
    }

    MCGAL::Halfedge* triH = polyFace ? polyFace->proxyHalfedge() : h;
    CHECK_TRUE(VertexRemovalEliminateOperator::triangulate(triH));
    CHECK_TRUE(VertexRemovalEliminateOperator::postprocess(triH));

    entry.edge = (polyFace ? polyFace->proxyHalfedge() : h);
    return true;
}
