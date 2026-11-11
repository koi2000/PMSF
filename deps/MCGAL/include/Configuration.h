#ifndef CONFIGURATION_H
#define CONFIGURATION_H

// ─── PPMC P0 对齐补丁 ─────────────────────────────────────────────────────────
// USE_BIJECTION 控制 PPMC 几何残差是否走 Frenet 双射旋转后再分两组（α,β + γ）建模
// 熵编码。这是 PPMC 论文（Maglo & Courbet 2011 / 2013）几何熵下降的核心手段。
// 原版 impl/OpenMesh/PPMC/configuration.h 默认开启；PMSF 复现下若不定义，所有
// `#ifdef USE_BIJECTION` 代码块（编/解码两侧）都会失效，几何编码退化到 3 维同分布
// 单模型，BPV 显著上升。这里全局开启以便编/解码两侧对称。
// 2026-11-08：重新开启（PPMC 几何熵编码核心；仅 PPMC 四个文件消费此宏）。
// 开启后编/解码两侧对称走 Frenet(α,β)-γ 双模型，参考实现默认值。
#define USE_BIJECTION 1

namespace MCGAL {

// #define VERTEX_POOL_SIZE 50000 * 1024
// #define HALFEDGE_POOL_SIZE 100000 * 1024
// #define FACET_POOL_SIZE 80000 * 1024

#define VERTEX_POOL_SIZE 20000 * 1024
#define HALFEDGE_POOL_SIZE 30000 * 1024
#define FACET_POOL_SIZE 26000 * 1024

#define VID_TO_POOLID 24000 * 1024

#define BUCKET_SIZE 4096
#define SMALL_BUCKET_SIZE 32

#define HALFEDGE_IN_VERTEX 24
#define HALFEDGE_IN_FACE 24
#define VERTEX_IN_FACE 24

#define MESHID_OFFSET 25

#define DEFAULT_MESH_ID 0

// #define DECODE

#ifdef DECODE
#    define USE_SEGMENT
#    define SEGMENT_SIZE 50

#else
#    define SEGMENT_SIZE 1

#endif

// #define COLOR
// #define LOAD_COLOR

#define ENCODE_BOUNDARY

// #define OLD_DUMP_TYPE

#define BUFFER_SIZE 600 * 1024 * 1024
}  // namespace MCGAL

#endif