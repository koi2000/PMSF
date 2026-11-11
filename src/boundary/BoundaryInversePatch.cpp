#include "boundary/BoundaryCompressionModule.h"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <iostream>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "common/BoundaryPatchCodec.h"

namespace pmsf {

bool applyBoundaryInversePayload(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                 const std::vector<int>& res) {
    if (!mesh || res.size() < 6) return false;
    struct Rec {
        int round = -1;
        int d = 0;
        MCGAL::PointInt vPos{0, 0, 0};
        std::vector<MCGAL::PointInt> ringQ;
        std::vector<int> ringG;
        std::vector<std::array<MCGAL::PointInt, 3>> overrideFaces;
        std::vector<int> overrideGroups;
    };
    std::vector<Rec> records;
    std::size_t ri = 0;
    const int payloadVersion = res[ri++];
    int opCount = payloadVersion;
    int anchorInts = 0;
    if (payloadVersion == 2) {
        if (ri >= res.size()) return false;
        anchorInts = res[ri++];
        if (anchorInts < 0 || ri + static_cast<std::size_t>(anchorInts) >= res.size()) return false;
        ri += static_cast<std::size_t>(anchorInts);
        opCount = res[ri++];
    }
    if (opCount <= 0 || opCount > 256) return false;
    for (int oi = 0; oi < opCount; ++oi) {
        if (ri + 7 > res.size()) break;
        Rec rec;
        rec.round = res[ri++];
        (void)res[ri++];  // interface id
        (void)res[ri++];  // boundary index
        rec.d = res[ri++];
        if (rec.d < 4 || rec.d > 12) break;
        rec.vPos = MCGAL::PointInt(res[ri], res[ri + 1], res[ri + 2]);
        ri += 3;
        if (ri + static_cast<std::size_t>(4 * rec.d) > res.size()) break;
        rec.ringQ.resize(static_cast<std::size_t>(rec.d));
        rec.ringG.resize(static_cast<std::size_t>(rec.d));
        for (int i = 0; i < rec.d; ++i) {
            rec.ringQ[static_cast<std::size_t>(i)] =
                MCGAL::PointInt(rec.vPos.x() + res[ri], rec.vPos.y() + res[ri + 1],
                                rec.vPos.z() + res[ri + 2]);
            ri += 3;
            rec.ringG[static_cast<std::size_t>(i)] = res[ri++];
        }
        if (ri < res.size()) {
            const int nOverride = res[ri++];
            for (int oi = 0; oi < nOverride && ri + 9 < res.size(); ++oi) {
                std::array<MCGAL::PointInt, 3> tri{};
                for (int k = 0; k < 3; ++k) {
                    tri[static_cast<std::size_t>(k)] =
                        MCGAL::PointInt(res[ri], res[ri + 1], res[ri + 2]);
                    ri += 3;
                }
                rec.overrideFaces.push_back(tri);
                rec.overrideGroups.push_back(res[ri++]);
            }
        }
        records.push_back(std::move(rec));
    }
    auto applyRecord = [&](const Rec& rec) -> bool {
        const int d = rec.d;
        std::vector<MCGAL::Vertex*> ringV(static_cast<std::size_t>(d), nullptr);
        for (int i = 0; i < d; ++i) {
            for (MCGAL::Vertex* v : mesh->vertices()) {
                if (!v || v->isRemoved()) continue;
                auto q = mesh->floatPosToInt(v->point());
                if (q.x() == rec.ringQ[static_cast<std::size_t>(i)].x() &&
                    q.y() == rec.ringQ[static_cast<std::size_t>(i)].y() &&
                    q.z() == rec.ringQ[static_cast<std::size_t>(i)].z()) {
                    ringV[static_cast<std::size_t>(i)] = v;
                    break;
                }
            }
        }
        for (auto* v : ringV) if (!v) return false;
        std::set<MCGAL::Vertex*> ringSet(ringV.begin(), ringV.end());
        std::vector<MCGAL::Facet*> fillFaces;
        for (MCGAL::Facet* f : mesh->faces()) {
            if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
            MCGAL::Vertex* fv[3]; int n = 0;
            MCGAL::Halfedge* h0 = f->proxyHalfedge();
            MCGAL::Halfedge* hh = h0; int guard = 0;
            do {
                if (hh && hh->vertex() && n < 3) fv[n++] = hh->vertex();
                hh = hh->next();
            } while (hh && hh != h0 && ++guard < 64);
            if (n != 3) continue;
            if (ringSet.count(fv[0]) && ringSet.count(fv[1]) && ringSet.count(fv[2]))
                fillFaces.push_back(f);
        }
        for (MCGAL::Facet* f : fillFaces) {
            if (f && !f->isRemoved()) pmsf::removeFaceForPatch(mesh, f);
        }
        MCGAL::Vertex* nv = MCGAL::contextPool.allocateVertexFromPool(
            mesh->meshId(), mesh->intPosToFloat(rec.vPos));
        if (!nv) return false;
        nv->halfedges().clear();
        nv->setGroupId(rec.ringG[0]);
        mesh->vertices().push_back(nv);
        for (int i = 0; i < d; ++i) {
            std::vector<MCGAL::Vertex*> vs{
                nv, ringV[static_cast<std::size_t>(i)],
                ringV[static_cast<std::size_t>((i + 1) % d)]};
            MCGAL::Facet* nf = MCGAL::contextPool.allocateFaceFromPool(mesh->meshId(), vs);
            if (!nf) return false;
            nf->setGroupId(rec.ringG[static_cast<std::size_t>(i)]);
            mesh->add_face(nf);
        }
        for (std::size_t oi = 0; oi < rec.overrideFaces.size() &&
                                oi < rec.overrideGroups.size(); ++oi) {
            std::array<std::array<int, 3>, 3> want{};
            for (int k = 0; k < 3; ++k)
                want[static_cast<std::size_t>(k)] = {
                    rec.overrideFaces[oi][static_cast<std::size_t>(k)].x(),
                    rec.overrideFaces[oi][static_cast<std::size_t>(k)].y(),
                    rec.overrideFaces[oi][static_cast<std::size_t>(k)].z()};
            std::sort(want.begin(), want.end());
            for (MCGAL::Facet* f : mesh->faces()) {
                if (!f || f->isRemoved() || !f->proxyHalfedge()) continue;
                std::array<std::array<int, 3>, 3> have{};
                int n = 0;
                MCGAL::Halfedge* h0 = f->proxyHalfedge();
                MCGAL::Halfedge* hh = h0; int guard = 0;
                do {
                    if (hh && hh->vertex() && n < 3) {
                        auto q = mesh->floatPosToInt(hh->vertex()->point());
                        have[static_cast<std::size_t>(n++)] = {q.x(), q.y(), q.z()};
                    }
                    hh = hh->next();
                } while (hh && hh != h0 && ++guard < 64);
                if (n != 3) continue;
                std::sort(have.begin(), have.end());
                if (have == want) { f->setGroupId(rec.overrideGroups[oi]); break; }
            }
        }
        return true;
    };
    int applied = 0;
    for (int i = static_cast<int>(records.size()) - 1; i >= 0; --i) {
        if (applyRecord(records[static_cast<std::size_t>(i)])) ++applied;
    }
    if (std::getenv("PMSF_BOUNDARY_DEBUG"))
        std::cerr << "[boundary-module-apply] records=" << records.size()
                  << " applied=" << applied << std::endl;
    return applied > 0;
}

}  // namespace pmsf