/*
 * @file MeshRuntime.cpp
 * @brief 见 MeshRuntime.h。
 */
#include "core/MeshRuntime.h"

#include "ContextPool.h"
#include "Mesh.h"
#include "Global.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <limits>

namespace pmsf {

void resetRoundElementState(const std::shared_ptr<MCGAL::Mesh>& mesh) {
    if (!mesh) {
        return;
    }
    const int meshId = mesh->meshId();

    // 顶点：池里 [0, cursor) 已分配。已移除的顶点同样要复位 —— 它下一轮可能
    // 因相邻面被重建而重新进入 BFS。
    MCGAL::Vertex* vpool = MCGAL::contextPool.getVertexPoolByIndex(meshId);
    const int vcount = MCGAL::contextPool.getVindex(meshId);
    for (int i = 0; i < vcount && vpool != nullptr; ++i) {
        vpool[i].resetState();
    }

    MCGAL::Halfedge* hpool = MCGAL::contextPool.getHalfedgePoolByIndex(meshId);
    const int hcount = MCGAL::contextPool.getHindex(meshId);
    for (int i = 0; i < hcount && hpool != nullptr; ++i) {
        hpool[i].resetState();
    }

    MCGAL::Facet* fpool = MCGAL::contextPool.getFacetPoolByIndex(meshId);
    const int fcount = MCGAL::contextPool.getFindex(meshId);
    for (int i = 0; i < fcount && fpool != nullptr; ++i) {
        fpool[i].resetState();
    }

    // perf（2026-11-10 QEM 解码实测：MCGAL::Mesh::resetState 占 4.24% cycles）：
    // 上面三个池循环已经对 [0, cursor) 的每个 Vertex/Halfedge/Facet 调用了同一个
    // resetState()（池覆盖 ⊇ Mesh::resetState 的活面/活顶点覆盖，且对象函数相同），
    // 再走一遍 Mesh::resetState 是纯重复工作。删除后复位后的对象状态不变。
    // 保留注释说明覆盖关系：Mesh::resetState 只覆盖活面触及的元素，池循环是超集。
}

void ensureMeshRuntimeInitialized() {
    ensureMeshRuntimeInitializedWithEstimate(0, 0, 0);
}

namespace {
bool& meshRuntimePoolInitializedFlag() {
    static bool initialized = false;
    return initialized;
}
}  // namespace

void ensureMeshRuntimeInitializedExact(int vertices, int halfedges, int facets) {
    if (meshRuntimePoolInitializedFlag()) return;
    meshRuntimePoolInitializedFlag() = true;
    MCGAL::contextPool.initPoolSize(1);
    MCGAL::contextPool.registerPool(vertices, halfedges, facets);
}

void ensureMeshRuntimeInitializedWithEstimate(int estimatedVertices,
                                              int estimatedHalfedges,
                                              int estimatedFacets) {
    if (meshRuntimePoolInitializedFlag()) return;
    meshRuntimePoolInitializedFlag() = true;
    {
        // env 显式设置且 >0 时优先（兼容旧脚本）；否则取 max(估计值, 内置默认)。
        // 内置默认：V/F=150k 是 Cheap 档（顶点/面片构造 ~3ns/个）；半边逐元素
        // 构造 ~33ns/个，地板从 2.5M 降到 1.25M（≈41ms），实测五模型解码峰值
        // H ≤ 5.2×F（horse ≈ 504k），1.25M 覆盖 horse/bunny 级；更大模型的
        // 产物都带 origTotals（2026-10-11 起写入侧已修），会走估计值而非地板。
        auto pick = [](const char* name, int estimate, int fallback) {
            const char* raw = std::getenv(name);
            if (raw && raw[0] != '\0') {
                const long v = std::strtol(raw, nullptr, 10);
                if (v > 0) {
                    return static_cast<int>(v);
                }
            }
            return estimate > fallback ? estimate : fallback;
        };
        MCGAL::contextPool.initPoolSize(1);
        MCGAL::contextPool.registerPool(
            pick("PMSF_POOL_V", estimatedVertices, 150000),
            pick("PMSF_POOL_H", estimatedHalfedges, 1250000),
            pick("PMSF_POOL_F", estimatedFacets, 150000));
    }
}


bool prescanOffMeshCounts(const std::string& path, int& v, int& f) {
    std::ifstream in(path);
    if (!in) {
        return false;
    }
    std::string token;
    if (!(in >> token) || token != "OFF") {
        return false;
    }
    long long nv = 0, nf = 0, ne = 0;
    // 跳过行内注释（OFF 头允许 # 开头的注释行）
    while (in >> token) {
        if (!token.empty() && token[0] == '#') {
            in.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            continue;
        }
        break;
    }
    try {
        nv = std::stoll(token);
        if (!(in >> nf >> ne)) {
            return false;
        }
    } catch (...) {
        return false;
    }
    if (nv <= 0 || nf < 0) {
        return false;
    }
    v = static_cast<int>(nv);
    f = static_cast<int>(nf);
    return true;
}

void ensureMeshRuntimeInitializedForMeshSize(int originalVertices,
                                             int originalFaces) {
    // 标定乘数（见 MeshRuntime.h 注释）：V×2、F×3、H=F×8。
    ensureMeshRuntimeInitializedWithEstimate(originalVertices * 2,
                                             originalFaces * 8,
                                             originalFaces * 3);
}

void ensureMeshRuntimeInitializedForInputFile(const std::string& path) {
    int inV = 0, inF = 0;
    if (prescanOffMeshCounts(path, inV, inF)) {
        ensureMeshRuntimeInitializedForMeshSize(inV, inF);
    } else {
        ensureMeshRuntimeInitialized();
    }
}

}  // namespace pmsf
