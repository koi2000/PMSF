# task-43：RD route A（canonical deterministic traversal + decoder replay）最终证据

日期：2026-10-22；worktree `task38-rd-pgs-p1p2`@3de2e54（+task-38/40/41 instrumentation）
env 开关：`PMSF_RD_CANON=1`（默认关闭；默认 rd_tag/PCATM/PCLTTM 路径零字节影响）
run dir：`/tmp/wsb43/`（`canon/` = route A artifact+decode，`default/` = rd_tag 守卫，`r2/` = rounds=2 对拍）
复现：`scripts/tests/test_rd_canonical.sh [outdir]`
patch：`/tmp/wsb43/task43_rd_route_a.patch`（base 3de2e54；仅新增 impl/RD 文件 + `impl/common/RetriOverride.h` env 分支）

## 0. 结论

| 项 | 结果 | gate |
|---|---|---|
| **B1 artifact** | **52,644 B / 21.215657 bpv**（N=19,851） | 硬顶 54,491 B / 21.96 ⇒ **PASS**（margin 1,847 B / 0.744 bpv） |
| **B2 decode** | **六项全 true**（counts/test_valid/manifold/connectivity/topology/nearest）；`coord_match_max_dist=3.78036582603e-05` | 六项 true + coord ≤3.78e-5 ⇒ **PASS**（与冻结 rd_tag 解码值逐位相同） |
| **质量** | `rms_max_norm=5.1239746868e-05`（rd_tag baseline 5.1241859673e-05；矩阵 RD 质量行 5.1240e-05） | 与 rd_tag 同口径/同量级（略优 4.1e-10） |
| **默认守卫** | rd_tag 64,604 B / md5 `d1fde1ae…` / compact `98e5c511…` 不变；PCLTTM 68,160 B A/B md5 逐字节相同；PCATM 默认行 63,420 B 复现 | 不变 |
| **确定性** | 第二次 encoder artifact sha 完全相同；full 50 与 rounds=2 解码坐标/面集逐位一致 | — |

未达理想：理想 50,297 B / 20.27 bpv，本次 52,644 B（+2,347 B）。ratio vs paper 16.89 = **1.256×**（<1.3 hard cap，>1.2 ideal，按矩阵 notes 需披露）。

## 1. 实现（全部新代码在 `impl/RD`，不 include/不 alias `src/conquest` codec）

| 文件 | 作用 |
|---|---|
| `impl/RD/RDLift.h` | canonical ring frame（Newell 法向 + 量化格字典序 anchor，order-independent）+ RD §3.1 Eq.(3) 局部 Frenet 系的可逆整数 lifting（3-lift shear） |
| `impl/RD/RDCanonicalTraversal.h` | frontier FIFO gate queue + valence/null 符号 + sign/type/flag 传播 + `encodeLayer`（编码端删点/重三角化）与 `decodeLayer`（解码端 merge fan→polygon→插中心→fan 三角化）；canonical seed（全网格量化格字典序最小边，无 rand）；两端同源的 ring/预测/反 Frenet |
| `impl/RD/RDCanonicalBlobFormat.h` | sv=71 三流 framing + 两端同一份 RC 模型（符号 5 元 4-ctx；残差自适应位宽 plane-major；seed 6×int32） |
| `impl/RD/compress/RDCanonicalCollector.{h,cpp}` | 每轮一层 traversal，导出 sv=71 三流（blob layout=18 由 `RDTagSymbolCollectOperator` 基类自动命中，不改 layout 插件） |
| `impl/RD/elimination/RDCanonicalNoopEliminate.h` | 框架侧空 eliminate（遍历自带删点） |
| `impl/RD/decompress/RDCanonicalSymbolReadOperator.{h,cpp}` | sv=71 blob 解析；container round r=encoder 层 r 反序映射成 decoder 层序 |
| `impl/RD/reconstruct/RDCanonicalReconstructOperator.{h,cpp}` | 每轮按存储 seed 找回 gate，重放同一 traversal（base→fine） |
| `impl/common/RetriOverride.h` | RD_TAG 分支 env `PMSF_RD_CANON=1`：compress 装 Collector+Noop+canonical seed；decompress 换 reader/reconstructor。默认（env off）逐字走原路径 |

忠实性：遍历 = RD 论文 §2.1（建立于 A&D valence-conquest）/§3.3 的 gate-queue + null-patch + 解码 replay；几何 = RD §3.1 局部 Frenet 预测残差。机制对照框架内逐层验证过的 A&D conquest 语义在 impl/RD **重新实现**（新 sv=71、新 blob/几何编码），未 include / 未 alias conquest codec，也不复用其 artifact/布局。

## 2. B1 证据（encoder-only，env `PMSF_RD_CANON=1`）

```
artifact_bytes    = 52,644 B（art_residual.parquet 52,559 + art_compact.bin 85）
bpv               = 21.215657
scheme_version    = 71 ; blob_layout = 18（RD_TAG）；streams = 3；rounds = 50
blob_bytes        = 52,567
component_bytes   = conn(valence+null) RC 11,812 / residual RC 38,735 / seed 1,400 / container 620
statistics        = valence syms 19,848 ; null_patch 38,033 ; residuals 19,848 ; base mesh 3V/2F
sha256            = parquet 6b534529b47e291840042c13aa8dc80704152530a5540c4357cb033c3dbaf19a
                    compact de460a61941d91eeb42ef5a69d4ee89b56cb895ed95add8111d398c12c559f7e
determinism       = 连跑两次 sha 相同
```

## 3. B2 证据（同一 sv=71 artifact，env `PMSF_RD_CANON=1`，未改格式/未加 side）

```
decoder_status          = OK（50/50 层 ok=1，failed_rounds=0；rounds=2 也对拍 ok）
decode_six              = counts_ok=true test_valid=true manifold_profile_ok=true
                          connectivity_ok=true topology_ok=true nearest_geometry_ok=true
coord_match_max_dist    = 3.78036582603e-05（= 冻结 rd_tag 解码同值；≤3.78e-5）
nn_ref_to_test_over_tol = 0 ; nn_test_to_ref_over_tol = 0
remapped_same_unordered_faces = 39,698 = ref 面数（full face match after coord remap）
decoded_sha256          = 6b5c5cabc5b12b93a3a3f3348a9e0272010fee04534bfcbbf5cb4f539e91657d
canonical_decoded_sha256= 33bded31a5b7e0ede41b3f962eb2739cc1dab2441ca50c29d447a20e9aa77b76
                          （冻结 rd_tag artifact 的默认解码，env off；同二进制）
quality rms_max_norm    = 5.1239746868161346e-05（rd_tag baseline 5.1241859673182866e-05）
rounds=2 对拍           = artifact 128,913 B，decode 2/2 层 ok，六项 true，coord 同值；
                          full-r50 与 r2 解码的顶点坐标/面集逐位相同（round 划分不变性）
```

## 4. 默认/守卫复跑（byte-level，env off / 无关路径）

| 守卫 | 复跑结果 | 与冻结比对 |
|---|---|---|
| rd_tag 默认 r50 q12 | 64,604 B / 26.0356 bpv / compact+residual sha256 `3c5789c6…`/`bb3356ce…` | **与 `PHASE1_MEASUREMENTS/rd_faithful_opt_q12_r50` 冻结 sha256 逐字节相同** |
| rd_tag 默认 decode | 六项 true、rms_max_norm 5.1241859673e-05、sha `33bded31…` | 与矩阵 RD 质量行一致 |
| PCLTTM 默认 r50 | 68,160 B；sha256 `3be9057e…`/`801159c1…` | **与 `pclttm_horse19k_q12_r50` 冻结 sha256 逐字节相同** |
| PCLTTM 矩阵行 r44 eb_z22 | 66,755 B；sha256 `171f448e…`/`70c18d46…` | **与 `pclttm_q12_r44_eb_z22` 冻结 sha256 逐字节相同** |
| PCATM strict r50 | 63,420 B；sha256 `ce9d9e11…`/`316c3e04…` | **与 `pcatm_strict_hq12_r50` 冻结 sha256 逐字节相同** |
| paper matrix 8/8 | `check_paper_matrix_expected.py` → `PASS algorithms=8 pinned=8` | 通过 |
| fusion（conquest→paper, bunny r16） | `test_conquest_paper_fusion.sh` → PASS，decode bit-exact；`paper_bytes=90,489` vs 冻结记载 90,490（**-1 B**） | 门禁脚本只断言 decode bit-exact；-1 B 来自 worktree base 与 live 当前未提交树在容器/parquet 侧的字节差（本 patch 不含该路径 diff），如实披露 |
| v1 四件套 | **引用 targets-quality live 复核（全一致）**；本任务未重跑（patch 无 VQMMC 路径） | 一致 |
| MMSE 62,633 / APTM 84,496 默认值 | **未在本 patch base 复跑**：其当前修复是 live tree 未提交文件（algo-bpv 报告的 58,781/55,496 漂移来源与本 patch 无关）；以 live 全量门禁一次性复核为准 | 待 targets-quality/Lead |

## 4b. decoded sha `6b5c5c…` vs rd_tag canonical `33bded…` 差异原因（非几何/拓扑错误）

1. **顺序差异**：route A 的 vertex/face 分配顺序由 canonical traversal（gate FIFO 消除序 + base 3V/2F 的插入序）决定，与 rd_tag 的 BFS/selector 序不同 ⇒ OFF 文件字节序不同。MeshCompare 与原始网格 **remapped_same_unordered_faces = 39,698/39,698**（全量面匹配）、`remapped_edges_only_*=0`，说明面集完全一致。
2. **1/19,851 顶点的 1 个量化步**：`full vs rd_tag baseline` 的顶点坐标集合中 19,850 个逐位相同，1 个 bbox 角点（x 轴）差 4.4763e-05（恰 1 个量化步）。原因是 route A 的量化格号与 base-mesh 序列化的 clamp 同口径（q 夹到 2^bits−1），rd_tag 路径在该角点用了相邻格；两者对原始网格的 `coord_match_max_dist` 完全相同（3.78036582603e-05），六项均 true，质量指标同量级（5.1239747e-05 vs 5.1241860e-05）。
3. 结论：sha 不同来自**编码顺序 + 1 个角点量化约定**，不是错解/伪造。可复现证据：`/tmp/wsb43/{canon,default}/meshcompare.log`、`quality_canon.csv`、`quality_default.csv`。

## 5. 偏差与风险披露（不掩盖）

1. **null_patch 远高于设计估计**：实测 38,033（≈1.9/删除、占符号 66%），设计估计 ~5%（~1 KB）。conn RC 因此 11,812 B（下界估计 5,102 B，+6.7 KB）。总字节仍在硬顶内（52,644 ≤ 54,491），但这是本路线最大的效率偏差；PGS/priority 类改动可再压，但不在本任务范围。
2. **base mesh 3V/2F**（全网格删至 3 顶点）：合法（V−E+F=2）且 replay 正常；但使其后若干层只剩 null 符号（r38–r49 每层 2 个 null），即最后 ~12 层零删除。
3. **与 rd_tag baseline 解码的 1 个顶点差 1 个量化步**（x 轴 4.4763e-05）：19,851 个顶点中 19,850 逐位相同；差异顶点在 bbox 边界格上的 clamp 约定不同（我们的量化格号与 base-mesh 序列化的 clamp 同口径）。两者对原始网格 coord max 相同（3.780366e-05），六项均 true。已如实记录，未把它说成 bit-identical。
4. 本任务只跑 LOD 全量解码（rounds=50 与 rounds=2）；未做逐 LOD 截断/随机访问验证（超出 B2 gate）。
5. build 计数：B1 = 1 次（`--target PMSFCompress`）；B2 = 1 次（`--target PMSFDecompress`，含 2 次增量修复重链：include path、`~SymbolReadOperator` 非虚、sign 写入 `setSign`）。

## 6. 证据文件

- CSV：`/tmp/wsb43/RD_ROUTE_A_EVIDENCE.csv`（同内容副本 `impl/RD/RD_ROUTE_A_EVIDENCE.csv`）
- 日志：`/tmp/wsb43/{canon,default,r2,canon_repeat}/`
- patch：`/tmp/wsb43/task43_rd_route_a.patch`
- 复现脚本：`scripts/tests/test_rd_canonical.sh`
