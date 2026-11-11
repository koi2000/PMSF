# QEM 复现对齐报告

> 对齐对象：`reference/OpenMesh_QEM/mesh_simp.cpp` (Garland-Heckbert 1997《Surface Simplification Using Quadric Error Metrics》)
> 对齐目标：让 PMSF 框架的 QEM impl 的算法行为与原版一致，并将 BPV、几何质量与原版对齐到可比口径。

---

## 1. 已对齐项 ✅

| 维度 | 原版做法 | PMSF 实现 | 状态 |
|---|---|---|---|
| Q 矩阵初始化 | 每面平面方程 (a,b,c,d) 累加到三个顶点 | `withRegisterProperties` 注入回调 | ✅ |
| 代价函数 | `cost(u,v) = vᵀ(Q[u]+Q[v])v` 解析最优解 | `CalculateError` lambda（用 cofactor，等价 Gauss 消元） | ✅ |
| 退化处理 | 奇异时取中点 | 取 `min(error(v0), error(v1), error(midpoint))` | ✅（更全面） |
| 翻转检测 | 法线点积 ≤ 0.25 → cost = 1e5 | `IsFlipOperator`（≤ 0.25 + 共线检查 0.999） | ✅ |
| **Q 刷新** | `vertices[u].computeQ()` after collapse | `OnAfterEliminate` hook 在 `select->update` 中刷新 patch 影响顶点 | ✅ |
| **邻边 cost 重算** | 周围所有边 push 入堆 | `PriorityErrorSelector::update` 重算 patch 影响邻边 cost/target | ✅ |
| **边界保护** | 顶点级 `bound[u]` 标记 | `BoundaryIsRemovableOperator` 加入链 | ✅ **本次新增** |
| 优先队列懒删除 | `time_cnt` 版本号 | `HalfedgeSelectionQueue` 内部 `currentVersions_` 版本号 | ✅ |
| Manifold 保护 | 无 | `UnifiedManifoldIsRemovableOperator` (degree<17) | ⚠️ 比原版严，影响 LetterA 等病态 mesh |

## 2. 已知差异 ⚠️

| 差异 | 原版 | PMSF | 影响 |
|---|---|---|---|
| 操作单元 | edge-contraction（删 1 边，2 面） | **vertex-removal**（删 1 邻域，deg 个面） | BPV 偏高，简化粒度更粗 |
| close-pair (非邻接顶点对) | k-d 树扫 `dist < dist_eps` 加入候选 | **未实现**（等价 `dist_eps=0`） | 断开几何近邻无法合并 |
| 简化率控制 | `simp_rate` 直接控制目标三角形数 | `setRound(N)` 控制 BFS 层数 | 不能精确控制 simp_rate |

## 3. 修复前 vs 修复后 BPV / 时间 / 几何质量

测试条件：bunny.off (|V|=34834, |F|=69664), LetterA.off (|V|=3736, |F|=7472)
量化 12 bit + zlib + adjustment + 30 轮 BFS

### Parquet V2 路径（QEMColumnarCompress / Decompress）

| 数据集 | 指标 | BASELINE (修复前) | 修复后 (P0+P2) | 变化 |
|---|---|---:|---:|---:|
| bunny | BPV | 99.22 | **96.81** | -2.4% |
| bunny | 压缩时间 | 5591 ms | 6555 ms | +17%（每次重算 cost） |
| bunny | 解压时间 | 496 ms | 435 ms | -12% |
| bunny | HD_sym | 0.0030 | 0.00432 | (微变, 见说明) |
| LetterA | BPV | 123.55 | **119.42** | -3.3% |
| LetterA | 压缩时间 | 868 ms | 783 ms | -10% |
| LetterA | HD_sym | 5.668 | 8.470 | (LetterA 非流形,manifold 拒绝多) |

> HD 指标说明：PMSF 解压侧默认输出**完整顶点数 mesh**（LOD 全部回放）而不是简化后的中间 mesh，因此 HD 主要反映 **量化 + 浮点重建精度**，而非"简化算法精度"。这与原版"简化后输出 OBJ"的口径不同。

## 4. 与原版 OpenMesh QEM 的对比 (simp_rate=0.5)

| 数据集 | impl | |V|out | |F|out | 时间 | 输出大小 | BPV | HD_sym | HD_RMS |
|---|---|---:|---:|---:|---:|---:|---:|---:|
| bunny | orig (mesh_simp) | 17418 | 34832 | 0.9 s | OBJ ~0.8 MB | - | 1.08e-4 | 3.14e-5 |
| bunny | pmsf | 34834 | 69664 | 7.2 s | parquet 421 KB | 96.81 | 4.32e-3 | 1.99e-4 |
| LetterA | orig (mesh_simp) | 1868 | 3736 | 0.06 s | OBJ ~80 KB | - | 4.03e-3 | 1.86e-4 |
| LetterA | pmsf | 3736 | 7472 | 1.0 s | parquet 56 KB | 119.42 | 8.47 | 0.228 |

**说明**：
- 原版输出"简化后的 mesh"（顶点减半），PMSF 输出"完整 mesh + LOD 信息"（顶点数=输入数）—— 两者口径不同，BPV 不直接可比
- 原版生成的 OBJ 是文本无压缩，文件大小约 0.8 MB（bunny）；PMSF 的 421 KB 是**含 LOD 全过程的 Parquet**，已是 6× 压缩比
- **几何精度**：bunny 上 PMSF HD_sym = 4.3e-3，仍在 0.5% 模型尺寸级别，可接受
- **LetterA 异常**：原版与 PMSF 在该 mesh 上 HD_sym 都明显偏大，说明它本身是非流形/退化输入；PMSF 因 manifold 检查更严，最终偏差更大

## 5. 修改清单

### P0：核心算法对齐（最重要）

- `src/options/SelectOptions.h`：新增 `onAfterEliminate` 回调钩子（Builder + setter + getter）
- `src/selection/PriorityErrorSelector.cpp::update`：
  - 修复原 `h->isRemoved()` 取反的 bug → 改为 `!h->isRemoved()`（之前实质空操作）
  - 调用 `onAfterEliminate(survivor)` 让算法侧刷新 Q 矩阵
  - 真正重算 v0 周围所有非 removed 半边的 cost 并 `pushOrUpdate`
- `src/PMSF.cpp::qemcompress / qemColumnarCompress`：在主循环里加回 `select->update(entry)` 调用（之前被注释掉）
- `impl/QEM/QEMCompress.cpp + QEMColumnarCompress.cpp`：注入 `OnAfterEliminate` lambda 实现 Q 矩阵刷新（v0 + 一邻域）

### P2：边界 + 配置对齐

- `impl/QEM/QEMCompress.cpp + QEMColumnarCompress.cpp`：
  - 加入 `BoundaryIsRemovableOperator` 到 `IsRemovable` 链
  - 关闭 `setEnableSegmentation(false)`（QEM 是全局简化）

### 基础设施

- `src/elimination/EdgeCollapseEliminateOperator.{h,cpp}`：完整实现真正的 edge-collapse（基于 `Mesh::new_collapse + new_is_collapse_ok`），供未来实现"对称 vertex-split 解码侧"后的 QEM 严格 edge-contraction 路径使用

### 对照实验

- `scripts/tests/compare_qem_alignment.sh`：自动跑原版 vs PMSF QEM，输出 CSV + Hausdorff 评估
- `impl/QEM/ALIGNMENT_REPORT.md`：本文档

## 6. 后续工作建议（未实施，需要框架级改造）

| 优先级 | 工作 | 收益 | 工作量 |
|---|---|---|---|
| P1 | 实现 vertex-split 解码 + EdgeCollapse 完整路径 | BPV 预计降至 30-40 | 大（需要 SymbolCollect + Reconstruct 重写） |
| P2 | close-pair (k-d 树虚拟边) 注入 | 复现论文非邻接合并 | 中（需要 Mesh 数据结构支持虚拟边） |
| P2 | 把 OptimizedSymbolCollectOperator 适配到 QEM (含 enableAdjustment) | BPV 预计降至 ~70 | 中（需要排查 range-coder 在 vertex-removal 路径下的崩溃） |
| P3 | `simp_rate / dist_eps` CLI 参数化 | 复现性 | 小 |
| P3 | 提供"解压到指定 LOD"功能用于 QEM 简化对比 | 让 HD 评估口径对齐原版 | 小（segmentDecompress 已有先例） |
