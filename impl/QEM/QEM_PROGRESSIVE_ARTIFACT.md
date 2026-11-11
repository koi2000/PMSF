# QEM 250-face progressive artifact (WS-B5, task-20) — **PASS**

Date: 2026-10-22. Owner: qem-repro. Scope: `impl/QEM/**`,
`tools/QEMFaceBudget.cpp`, `scripts/tests/qem*`.

## Verdict

阶段一 QEM E250 行此前只有 250-face 快照（论文无码流）。本任务在框架自有的
reference-style 独立引擎（`impl/QEM/QEMRefStyleEngine.h`，无 MCGAL
link-condition、不依赖 reference 二进制）之上实现了**配对的 progressive
encoder/decoder**，产出可 prefix / maxLod 解码的 progressive artifact：

| 项 | 结果 |
|---|---|
| encode bunny（seeded 引擎，base=250 faces / 127 v） | artifact **1,793,880 B = 411.984 bpv**（f32 geometry，默认）；f64 geometry 模式 2,626,848 B = 603.284 bpv |
| base LOD (maxLod=0) | **250 faces / 127 v，与 phase-1 快照逐字节相同**；E250 = **1.31833754428e-06**（目标 ≤1.732377043668e-06，unpinned） |
| full LOD (maxLod=all) | **34,834 v / 69,664 f**；MeshCompare(原 bunny, decoded)：`counts_ok / topology_ok / connectivity_ok / nearest_geometry_ok / coordinate_remap_complete / manifold_profile_ok` **全 true**；MeshAudit 闭合 2-manifold |
| f64 geometry 模式 | full decode 与原 bunny **所有 34,834 个顶点坐标逐位相同、定向面集合完全相同**（bit-exact） |
| 20 个 LOD（0..19） | 每个 LOD 与**正向引擎在相同 face budget 的快照**全部内容一致（`remapped_same_unordered_faces == ref_faces`、`remapped_edges_only_*=0`、counts/nearest true）；**0 failures** |
| prefix 解码 | 19 个 round 边界截断 + mid-chunk 截断 + 空/半 header 全部行为正确（mid-chunk 只丢不完整 round）；`maxLod=0/3/7/19` 与等价 prefix 一致 |
| 默认 legacy 回归 | 六遍 500→…→250 E250 **9.14274932589e-06 精确不变**；enable recorder 前后 base 快照逐字节相同；phase-1 `qem_ref_engine_e250.sh` 全行不变 |

复现（~29 s，PASS 才 exit 0，重生成 `impl/QEM/QEM_PROGRESSIVE_EVIDENCE.csv`）：

```bash
scripts/tests/qem_progressive_artifact.sh
```

## 用法（全部 env-gated，默认路径不受影响）

```bash
# encode: base 快照 + artifact
PMSF_QEM_REF_ENGINE=1 PMSF_QEM_PROG_ARTIFACT=/tmp/qem.bin \
    build/QEMFaceBudget datasets/Stanford/bunny.off /tmp/base250.off 250
#   可选 PMSF_QEM_PROG_GEOM64=1 -> f64 geometry（bit-exact 重建模式）

# decode: artifact -> OFF at maxLod (0=base, -1/all=full)
PMSF_QEM_PROG_DECODE=1 build/QEMFaceBudget /tmp/qem.bin /tmp/out.off 10
```

新增 `impl/QEM/QEMProgressiveArtifact.h`（~1,100 行）：`buildLogicalStreams` /
`serializeOwnFormat` / `encodeProgressiveArtifact` /
`decodeProgressiveArtifact`（前后缀、maxLod、LOD 表、truncated 语义）。
`impl/QEM/QEMRefStyleEngine.h` 增加可选的 `CollapseRecorder` hook（记录每个
collapse 的 survivor/victim 原始 id、两者 collapse 前位置、victim 的定向
`(victim,a,b)` incident fan）以及 `SimplifyStats::outputVertexOriginalIds`。
recorder 为 null 时引擎行为与产物逐字节不变（已由脚本与 phase-1 快照对比证明）。

## Artifact v1 own format（encoding-independent 语义流 + 自有容器）

```
header 64 B  : "QEMPROG1", version=1, flags(geomF64), idWidth(2/4),
               geomBytes(4/8), originalV/F, baseV/F, roundCount,
               totalCollapses, headerBytes
base mesh    : baseV x (original id + f64 x,y,z)  +  baseF x 3 original ids
chunks       : 每轮一个，**最后简化轮在前**（prefix = 更粗 LOD）
               u32 payloadBytes, u32 collapseCount,
               topology: (survivor, victim, u16 fanCount, fanCount x (a,b))
               geometry: (oldU[3], oldV[3]) f32 或 f64
```

- 记录为什么足够：forward collapse `(u survivor, v victim)` 杀掉含 `u,v` 的面
  （record 里 `a/b == u` 的 fan），其余 v 的面 `(v,a,b)` 被改写成 `(u,a,b)`
  （reverse 时按定向 canonical rotation 找回并改回 v），并移动 u 到最优位置、
  v 死亡。reverse 时恢复 u/v 的 collapse 前位置 → 精确逆操作。顶点 id 用
  **original id**（union-find 代表恒为 original id），全流稳定。
- prefix 语义：按 chunk 整体切片；数据在 chunk 中途截断时只丢弃不完整轮，
  返回最长完整前缀（`truncated=true`）。
- 中间 LOD 与正向引擎状态**逐内容一致**，其中个别中间 LOD（如 LOD17）会继承
  引擎在该 LOD 的临时 duplicate face（reference-style 无 link-condition 的
  固有属性），decoder 忠实复现；base 与 full LOD 均干净。

## bytes / bpv 分量（bunny 34,834 vertices；QEM 无 paper bpv，仅存储账本）

| section | bytes | bpv |
|---|---:|---:|
| header | 64 | 0.015 |
| base mesh (f64 位置 + ids) | 4,802 | 1.103 |
| round framing (19 chunks) | 152 | 0.035 |
| topology (fan/ids) | 955,894 | 219.531 |
| geometry f32（默认，量化等价） | 832,968 | 191.300 |
| **total f32** | **1,793,880** | **411.984** |
| geometry f64（`GEOM64=1`，bit-exact） | 1,665,936 | 382.600 |
| **total f64** | **2,626,848** | **603.284** |

## PaperStream seam（M4 / task-22 就绪后接入）

`buildLogicalStreams()` 产出 **四个语义流**，own-format 容器只做「chunk framing +
header」：

| seam stream | PaperStream M4 映射（计划） |
|---|---|
| `baseMeshStream` | base mesh 表（vertex id + f64 位置、face id） |
| `roundStream` | per-round metadata 表（round id / collapseCount / resulting faces） |
| `topologyStream` | topology symbol 列（survivor/victim/fanCount/fan ids，可 RC/Huffman） |
| `geometryStream` | geometry residual 列（collapse 前位置；可加 predictor/量化） |

接入时只需替换 `serializeOwnFormat()` / `parseOwnFormatHeader()`（容器层），
引擎与 `decodeProgressiveArtifact` 的逐记录逆操作逻辑不变；decoder 已按
「读取全部 chunk → 逆序回放」设计，PaperStream 侧只需按同一顺序提供 chunk。
未接 M4 前**不要**把本 artifact 当作 PaperStream 产物；它只是 QEM 行的
progressive 能力证据。

## 证据

- `scripts/tests/qem_progressive_artifact.sh`（gate，28.6 s）
- `impl/QEM/QEM_PROGRESSIVE_EVIDENCE.csv`（70 行：artifact ledger / decode
  flags / 20-LOD forward cross-check / prefix / regression / seam）
  — `.gitignore` 忽略 `*.csv`，提交需 `git add -f`
- `build/qem_progressive_artifact/`（artifact、各 LOD OFF、logs、measured CSV）
- 既有 `scripts/tests/qem_ref_engine_e250.sh` 复跑 PASS（E250 行未变）
- 工具：`build/QEMFaceBudget`（encode/decode 模式）、`MeshCompare`、
  `MeshAudit`、`quality_metrics.py`

## 已知限制

- own format v1 尚未接 M4 PaperStream schema（按任务要求先自有格式 + seam）。
- 默认 geometry 为 f32（量化等价，full decode 坐标偏差 <1e-8）；需要 bit-exact
  时用 `PMSF_QEM_PROG_GEOM64=1`（体积 +46%）。
- 单文件一模型；base 必须完整才能解码（header/base 截断会报错），rounds 可
  prefix 截断。
- 中间 LOD 可能继承引擎自身的 transient duplicate/non-manifold face（见上）；
  这不是 decoder 缺陷，gate 以「与正向引擎状态逐内容一致」为判据，并记录
  MeshAudit 结果。
