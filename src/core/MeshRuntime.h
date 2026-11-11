/*
 * @file MeshRuntime.h
 * @brief 网格运行时的进程级初始化（MCGAL ContextPool）。
 *
 * 背景：MCGAL 的顶点/半边/面片内存池必须在加载任何网格之前初始化，池容量由
 * `PMSF_POOL_V` / `PMSF_POOL_H` / `PMSF_POOL_F` 环境变量决定。历史上这件事
 * 由 `PMSF` 对象的构造函数顺带完成，于是「必须先构造一个 PMSF 实例」变成了
 * 一条隐式前置条件——新算法接入时极易踩空（忘记构造 ⇒ 加载网格时 SIGSEGV）。
 *
 * Phase 1 把它显式化为框架职责：`pmsf::runCompress` / `pmsf::runDecompress` 在
 * 入口处调用 `ensureMeshRuntimeInitialized()`，幂等且线程安全。算法侧不再需要
 * 知道它的存在。
 */
#ifndef PMSF_CORE_MESH_RUNTIME_H
#define PMSF_CORE_MESH_RUNTIME_H

#include <memory>
#include <string>

namespace MCGAL {
class Mesh;
}

namespace pmsf {

/// @brief 初始化 MCGAL ContextPool。幂等：重复调用只有第一次生效。
///        必须在设置 `PMSF_POOL_*` 环境变量之后、加载网格之前调用。
void ensureMeshRuntimeInitialized();

/// @brief 以**精确**的池容量注册（忽略 PMSF_POOL_* 环境变量）。
/// 第一个初始化调用生效；用于算法按产物/输入规模自算安全容量、避免
/// 配置里过度预留的池（构造 48 万 halfedge ≈ 40 ms 的固定开销）。
void ensureMeshRuntimeInitializedExact(int vertices, int halfedges, int facets);

/// @brief 带容量估计的初始化：池容量 = max(估计值, 内置默认)，
///        `PMSF_POOL_*` 环境变量显式设置时优先（兼容旧脚本）。
///
/// 为什么需要它：`new Halfedge[N]` 逐元素构造，N=18M 时单进程约 +600ms
/// （实测 bunny 压缩 wall 1499ms 中 ~780ms 在管线之外）。压缩侧在加载网格前
/// 预读 OFF 头、解码侧在读 metadata 后调用本函数，即可按真实规模注册，
/// 不再需要脚本统一塞巨型池（小模型白白付出大池初始化，默认池又会在
/// horse 及以上规模的解码中直接耗尽抛异常）。
void ensureMeshRuntimeInitializedWithEstimate(int estimatedVertices,
                                              int estimatedHalfedges,
                                              int estimatedFacets);

/// @brief 按「原始网格规模」初始化：内部套用标定乘数（顶点 V×2、面片 F×3、
///        半边 F×8）后转调 ensureMeshRuntimeInitializedWithEstimate。
///
///   压缩侧峰值 ≈ V×1.0 / F×2.08 / H×5.16；解码侧 ≈ V×1.0 / F×1.93 / H×5.2。
/// 所有调用方共享这一处标定，不要在调用点各自写乘数。
void ensureMeshRuntimeInitializedForMeshSize(int originalVertices,
                                             int originalFaces);

/// @brief 预读 OFF 头（"OFF\nV F E"，允许 # 注释行）拿输入网格规模并转调
///        ensureMeshRuntimeInitializedForMeshSize；解析失败回落内置默认池。
void ensureMeshRuntimeInitializedForInputFile(const std::string& path);

/// @brief 仅预读 OFF 头拿 (V, F)，不初始化任何运行时；失败返回 false。
///        供调用方复用同一解析逻辑（避免重复打开文件）。
bool prescanOffMeshCounts(const std::string& path, int& v, int& f);

///       （峰值用量诊断；仅读游标，零副作用）。

/// @brief 把一轮解码结束后残留的元素状态复位干净（解码主循环每轮调用）。
///
/// 为什么不能直接用 `MCGAL::Mesh::resetState()`：它只遍历**活着的面**所触及的
/// 顶点与半边（`deps/MCGAL/src/Mesh.cpp`：
/// `for (face : faces_) { if (face->isRemoved()) continue; ... }`）。于是
/// **只被已移除的面包裹的元素永远不被复位**，它们的 `addedFlag_` /
/// `splittableFlag_` 会跨轮残留；解码端下一轮的 BFS 再走到同一个元素时，
/// `setAdded()` / `setSplittable()` 直接撞 `assert(... == Original|Unknown)`
/// 中断。实测样本：某算法二进制路径（轮 4 撞 Facet::setSplittable，
/// 修掉后轮 5 撞 Halfedge::setAdded）。
///
/// 本函数按 ContextPool 的分配游标把**已分配的全部**顶点/半边/面片复位一遍，
/// 补齐 `Mesh::resetState()` 漏掉的那部分。不改 `deps/MCGAL`。
void resetRoundElementState(const std::shared_ptr<MCGAL::Mesh>& mesh);

}  // namespace pmsf

#endif  // PMSF_CORE_MESH_RUNTIME_H
