// SPDX-License-Identifier: Apache-2.0
//
// PCLTTMTagReconstructOperator.cpp — see header for design contract.

#include "PCLTTMTagReconstructOperator.h"
#include "PCLTTM/PCLTTMFrenetPatch.h"
#include "PCLTTM/decompress/PCLTTMTagSymbolReadOperator.h"

#include "Facet.h"
#include "Mesh.h"
#include "Vertex.h"
#include "common/PCLTTMTagTable.h"

#include <cstdint>
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

}  // namespace

void PCLTTMTagReconstructOperator::reconstruct(
    std::vector<MCGAL::Vertex*>& vertices,
    std::vector<MCGAL::Halfedge*>& halfedges,
    std::vector<MCGAL::Facet*>& facets) {

    // 第一阶段：与 BasicReconstructOperator 完全一致的 vertex point 写回 +
    // halfedge merge_face。
    for (MCGAL::Vertex* v : vertices) {
        if (v) v->setPoint(v->vertexPoint());
    }
    for (MCGAL::Halfedge* hit : halfedges) {
        merge_face(hit);
    }

    // 第二阶段：拆开父类的 insert_point 循环，在每个 polyFace 上：
    //   1) 在 insert_point 之前读 polyVerts （N-gon 状态）；
    //   2) 演化 tag（与 encoder 完全对称：相同 evolve(...) 函数）；
    //   3) 调父类 insert_point(f) 做 v0-fan 几何反演。
    const unsigned denominator=reader_?reader_->currentFrenetDenominator():0;
    for (MCGAL::Facet* f : facets) {
        if (!f) continue;

        std::vector<MCGAL::Vertex*> polyVerts = f->getVertices();
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

        // CODEC_016 BlobLayout=16/sv=3 is a Cartesian-residual storage path.
        // Frenet residual experiments must use a layout that serializes and
        // advertises that mode explicitly; the encoder now fail-fasts otherwise.

        if (denominator) {
            using namespace pmsf::pclttm_frenet;
            if (!mesh_ || !mesh_->enableQuantization || N!=f->getPolyValence())
                throw std::runtime_error("PCLTTM Frenet reconstructed patch valence/grid differs");
            const auto frame=makeFrame(patchCells(f,*mesh_,false));
            auto target=restore(frame,toCell(f->getRemovedVertexPosInt()),denominator);
            for (int axis=0;axis<3;++axis) {
                if (target[axis]<0||target[axis]>(1u<<mesh_->i_nbQuantBits))
                    throw std::runtime_error("PCLTTM Frenet reconstructed cell outside grid");
                target[axis]-=static_cast<std::int32_t>(frame.barycenter[axis]);
            }
            f->setRemovedVertexPosInt(MCGAL::PointInt(target[0],target[1],target[2]));
        }
        insert_point(f);
    }
}
