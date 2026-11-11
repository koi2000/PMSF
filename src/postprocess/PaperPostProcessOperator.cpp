#include "PaperPostProcessOperator.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <set>
#include <string>

namespace pmsf {

namespace {
struct Bounds {
    double lo[3]{0, 0, 0};
    double hi[3]{0, 0, 0};
    bool valid = false;
};

Bounds computeBounds(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    Bounds b;
    const double inf = std::numeric_limits<double>::infinity();
    for (int a = 0; a < 3; ++a) { b.lo[a] = inf; b.hi[a] = -inf; }
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        for (int a = 0; a < 3; ++a) {
            b.lo[a] = std::min(b.lo[a], static_cast<double>(v->point()[a]));
            b.hi[a] = std::max(b.hi[a], static_cast<double>(v->point()[a]));
        }
        b.valid = true;
    }
    return b;
}
}  // namespace

int PaperPostProcessOperator::apply(const std::shared_ptr<MCGAL::Mesh>& mesh,
                                    const std::string& mode) {
    if (!mesh || mode.empty() || mode == "none") return 0;
    // Per-paper entry points: every local paper evaluates 12-bit-quantized
    // meshes, so the reconstruction-side normalization is a snap to that grid.
    static const std::set<std::string> kPaperModes = {
        "aptm", "mmse", "pcatm", "pclttm", "pomar", "ppmc", "qem", "rd", "vqmmc"};
    std::string effective = mode;
    if (kPaperModes.count(mode)) effective = "snap12";
    if (effective != "snap12") {
        std::fprintf(stderr, "[PaperPostProcess] unknown mode '%s' (ignored)\n", mode.c_str());
        return 0;
    }
    const Bounds b = computeBounds(mesh);
    if (!b.valid) return 0;
    const double qmax = 4095.0;  // 12-bit grid
    double scale[3];
    for (int a = 0; a < 3; ++a) {
        const double extent = b.hi[a] - b.lo[a];
        scale[a] = extent > 0.0 ? extent / qmax : 0.0;
    }
    int changed = 0;
    for (MCGAL::Vertex* v : mesh->vertices()) {
        if (!v || v->isRemoved()) continue;
        bool did = false;
        for (int a = 0; a < 3; ++a) {
            if (scale[a] <= 0.0) continue;
            const double snapped = b.lo[a] +
                std::round((v->point()[a] - b.lo[a]) / scale[a]) * scale[a];
            if (snapped != v->point()[a]) {
                v->point()[a] = snapped;
                did = true;
            }
        }
        if (did) ++changed;
    }
    return changed;
}

}  // namespace pmsf
