# APTM P3 报告：A&D valence/null_patch connectivity（P3-P1，已实测）

日期：2026-10-22 / 负责人：WS-B3（aptm-repro）/ task-14（多轮）
前置：
- `impl/APTM/P1_APTM_TAG_2026-10-22.md`（P1：tag-frenet 29.164 可解码；polar 25.095 decode FAIL）
- `impl/APTM/P2PRE_SPIKE_2026-10-22.md`（P2-pre：根因=坐标依赖拓扑推断；Arch-2 建议）
- `impl/APTM/ARCH2_SPIKE_PLAN_2026-10-22.md`（conquest 路径的 Arch-2 spike 方案）
证据脚本：`scripts/tests/test_aptm_p3_conquest.sh`
证据 CSV：`impl/APTM/APTM_P3_CONQUEST_EVIDENCE.csv`
原始产物：`/tmp/wsb3/aptm_p3/`（baseline + rounds_r8/12/16/24/32/150）

---

## 0. 结论（先读）

**P3-P1：PASS（本轮所有验收项达成）。**

| 项 | 结果 |
|---|---|
| **C（connectivity）** | **4.639 bpv**（r16）← 论文 APTM C=4.9，**达标**（任务目标 4.9–5.5）；对比 tag 8.955 = **−4.32 bpv** |
| G（geometry） | 16.516 bpv（AD 一阶预测水平；论文 APTM G=12.9 → 差 −3.6，留给 polar/P3-P2） |
| container | 0.637 bpv（r16） |
| **artifact 总 bpv** | **22.907058（r16）/ 22.918745（rounds=150 task 口径）** |
| 比值 vs 论文 17.80 | **1.2869× / 1.2876× ⇒ 过硬顶 1.30（PASS）**；理想 1.20（21.36）未达，需 polar G |
| decode | counts/connectivity/topology/nearest/remap/test_valid **六项全 true**；A1 逐字 **19,851 / 39,698**；coord_match_max=3.78e−5（= 量化无损水平） |
| anchors | **139**（APTM §3 曲率+hysteresis，与 default APTM 一致）；null_patch 42,125（r16）= 锚点保锚的符号代价 |
| 默认 APTM | **84,496 B / 34.052088 bpv / 4 个 artifact md5 与冻结审计逐字节一致**（P3 env 未设时零影响） |
| conquest paper fusion | `test_conquest_paper_fusion.sh` PASS：90,490 B / bit-exact（P3 未影响共享路径） |

**Lead 裁决（2026-10-22）：本行按 labeled variant 记录（APTM-Conquest P3）。**
它用的是 A&D connectivity + AnchorVeto + **conquest 通用几何残差（G 16.516）**，
不是论文 §4.2 的 polar θ/φ coder ⇒ 不进阶段一 bpv 门禁；矩阵 APTM 仍保持
**34.052 FAIL**，等 polar 完成后由 Lead 决定是否把 P3 作为 faithful 行。
（graph 上的诚实口径：P3 = 论文「AD baseline」部分；polar = 论文增益部分，P3-P2 做。）

**阶段一失真证据（P3 r16/r150 decode 网格，`SurfaceMetrics --samples 5000`）：**
`surface_rms_max=9.23407766308e-06`、`surface_mse_sym=8.39796182256e-11`、
`bbox_diag=0.253028554246`、`normalized_rms=3.64942118512e-05`（= 0.00365%），
与 default APTM / tag-frenet 同量级（量化无损口径）；命令见 §6。polar 完成后再测一次。

**关键架构结论：P3 不需要重写 1.5–2.5k LOC。** 框架已有逐符号验证过的 A&D
valence-conquest 原生 codec（`FrontierConquestEngine` + `decodeLayer`），P3 =
**集成（~180 LOC）+ APTM 锚点注入**（`AnchorVetoPredicate` 就是为 APTM 语义准备的
hook：锚点 veto ⇒ 留 base mesh + 引擎发 null_patch）。默认 APTM 路径、tag 路径、
conquest fusion 均未受影响。

**下一步（P3-P2）**：G 从 16.52 → polar 目标 13–14 ⇒ 总 **19.4–20.4 bpv
（1.09–1.15×，过 1.20 理想）**；C 还可用「null_patch 后处理去冗余」（论文 §4.1：
≥10%）再降。Arch-2 spike 方案已就绪（`ARCH2_SPIKE_PLAN_2026-10-22.md`，500–800 LOC）：
conquest 拓扑重放与坐标无关，Arch-2 只需处理几何，比 tag 路径简单一个量级。

---

## 1. 忠实性论证（为什么这是论文的 connectivity，而不是「Conquest 变体」）

| 论文 APTM §4.1 原文 | 本实现 |
|---|---|
| connectivity = Alliez-Desbrun 2001 valence-driven decimating/cleaning conquest | `pmsf::FrontierConquestEngine::decimateLayer`（AD 迁移实现；`tools/AdConquestVerify` 三网格 30,000/30,000 逐符号 0 差异） |
| "The connectivity encoding step is not changed in our approach" | 未改 AD 规则（引擎是已验证资产，只复用不重写） |
| "respect anchors" | `AnchorVetoPredicate` veto（APTM §3 `CurvatureHysteresisDetector` 1.7/1.2，实测 139 anchors，与 default APTM 一致） |
| "send more null_patch codes to encode them" | 引擎对被 veto 的前端点发 null_patch=4（r16 = 42,125 个；符号流可见、已计入 C） |
| geometry = Frenet + polar θ/φ（§4.2） | **P3-P1 暂用引擎自带 AD 一阶预测残差**（G 16.52 ≈ AD 论文 G 16.2）；polar 为 P3-P2 |

与旧「Conquest 变体」的区别：旧口径是 *用 `algorithm=conquest` 冒充 RD/APTM*；
现在是 **`algorithm=APTM` 内部走 AD 族连通性 + APTM 锚点**（论文结构本身）。
四舍五入地说：**P3-P1 复现出了论文的「AD baseline」行**（C≈4.6、G≈16.5），
论文的增益（G→12.9）由 polar 几何 coder 承担。

---

## 2. 实现（P3-P1，全部在 `impl/APTM/`，未改共享代码）

新增 `impl/APTM/compress/APTMConquestCollector.h`（~200 行，继承
`ConquestSymbolCollectOperator`）：
1. `init()`：在**原始网格**（吸附前）跑 `CurvatureHysteresisDetector` 得 anchor poolId 集；
2. 通过既有 hook 接口 `PMSF_CONQUEST_ANCHOR_IDS` 注入 `AnchorVetoPredicate`；
3. `exportToBufferWithRangeCoder()`：父类导出后解析 conquest_blob 原始字节，输出逐分量
   `[APTM-Conquest bpv] layers= symbols= nullPatch= connectivityBytes= geometryBytes=
   containerBytes= totalBlob= verts= faces= anchors=`。

`impl/APTM/APTMAlgorithm.cpp` env-gated P3 分支（`PMSF_APTM_USE_CONQUEST_P3=1`，
优先级 conquest > tag > default）：
- `collectorFactory()` → `APTMConquestCollector`；
- `eliminate()` → `ConquestNoopEliminateOperator`（引擎自己删点，框架不得叠第二遍）；
- `pipelineTuner()` → 空（与 `algorithm=conquest` 完全一致；q12 由框架标准
  `preInitPreparation` + `PMSF_DEFAULT_QUANT_BITS=12` 落地）；
- `removability()`/`preProcessOperator()`/`boundaryConfigurator()`/
  `beforeCompressLoop()`/`afterBaseMeshHook()` → 默认/空；
- `makeDecompressOptions()` → conquest 三元组（`ConquestSymbolReadOperator` +
  `ConquestReconstructOperator`（共享同一 reader）+ `ColumnarDeserializeOperator`）；
- `defaultRounds()` → 16（P13.4 已验证运行点）。
产物：BlobLayout = `kPaperBlobLayoutConquestRC (15)`，codecVersion=2（整条 blob 进
rc_blob + compact bin）。

**build 说明**：共享锁单目标增量；上一轮 clean rebuild 与我的编辑存在 1 次竞态
（`APTMAlgorithm.cpp.o` 在源码落盘前开始编译），已 `touch` 强制重编并确认
`strings PMSFCompress | grep APTM-Conquest` = 4。

---

## 3. 实测（q12、horse19k、columnar、anchors=139）

### 3.1 rounds 扫描（artifact bpv / 逐分量）

| rounds | artifact B | **bpv** | ratio/17.8 | C B (bpv) | G B (bpv) | container B | null_patch | decode |
|---:|---:|---:|---:|---:|---:|---:|---:|---|
| 8 | 63,874 | 25.741373 | 1.446 | 10,777 (4.342) | 38,066 (15.340) | 812 | 37,066 | 六项 true |
| 12 | 57,894 | 23.331419 | 1.311 | 11,332 (4.566) | 40,412 (16.284) | 1,196 | 40,053 | 六项 true |
| **16** | **56,841** | **22.907058** | **1.2869** | **11,512 (4.639)** | **40,983 (16.516)** | **1,580** | **42,125** | **六项 true** |
| 24 | 56,849 | 22.910282 | 1.2871 | 11,627 (4.685) | 41,173 (16.593) | 2,348 | 46,103 | 六项 true |
| 32 | 56,850 | 22.910685 | 1.2871 | 11,715 (4.721) | 41,293 (16.641) | 3,116 | 50,103 | 六项 true |
| 150（task 口径） | 56,870 | 22.918745 | 1.2876 | 13,013 (5.244) | 43,063 (17.354) | 14,444 | 109,103 | 六项 true |

说明：
- r16 是 P13.4 已验证运行点（base 74 顶点）；r≥24 base 只剩 13/4 顶点，但
  **每层重复 anchor null_patch + 每层 seed/header** 让 container 增大，总率进入平台。
  因此 **r16 作为主证据点**，r150 作为任务口径点（两者 artifact bpv 差 0.012）。
- C 从 r8 的 4.34 升到 r150 的 5.24：增量几乎全是「锚点每层被重新 null_patch」
  （r16 null_patch 42,125 ≈ 68% 符号；r150 109,103）。论文 §4.1 也承认 null_patch
  随锚点增加，并说末尾模拟解码可去掉 ≥10% —— 这就是 P3-P2 的 C 优化项。
- container 在 r16 只有 1,580 B（0.637 bpv）；r150 因 150 层 seed/header 涨到
  14,444 B（但被 zlib/parquet 摊销，artifact 几乎不变）。

### 3.2 decode / 质量（每个 rounds 点都验）

- `test_vertices=19,851`、`test_faces=39,698`（A1 逐字）；
- `counts_ok=true`、`connectivity_ok=true`、`topology_ok=true`、
  `nearest_geometry_ok=true`、`coordinate_remap_complete=true`、`test_valid=true`、
  `manifold_profile_ok=true`；
- `coord_match_max_dist=3.78023068196e-05`、`nn_ref_to_test_rmse=2.2411e−05`
  —— 与 P1 无损 tag 路径逐位同量级（量化无损口径）。
- 解码 rc=0（所有 rounds 点）。

### 3.1b 阶段一失真（SurfaceMetrics，5000 samples）
| 网格 | surface_rms_max | surface_mse_sym | bbox diag | normalized RMS |
|---|---:|---:|---:|---:|
| P3 r16 decode | 9.23407766308e-06 | 8.39796182256e-11 | 0.253028554246 | 3.64942118512e-05 |
| P3 r150 decode | 9.23407766308e-06 | 8.39796182256e-11 | 0.253028554246 | 3.64942118512e-05 |
| tag-frenet（P1） | 9.23242365323e-06 | 8.3314536793e-11 | 同上 | 3.6488e-05 |
| default APTM | 9.23242365323e-06 | 8.43925758853e-11 | 同上 | 3.6488e-05 |

命令：`build/SurfaceMetrics datasets/PaperModels/horse19k.off <dec.off> --samples 5000`。
⇒ P3 的量化级 lossless 口径与其它路径一致（差异在浮点末位噪声内）。

### 3.3 归因（对比表）

| 配置 | bpv | 说明 |
|---|---:|---|
| 默认 APTM（历史忠实路径） | 34.052088 | baseline（本轮复核字节/md5 不变） |
| P1 tag-frenet（此前可解码最好） | 29.164072 | layout 16；C 8.955 |
| **AD baseline（无锚点，`algorithm=conquest` r16）** | **22.150200** | 同一引擎、无 APTM 锚点（预证据，六项 true） |
| **P3-P1（APTM 锚点，r16）** | **22.907058** | 锚点代价 **+0.757 bpv**（base mesh +null_patch） |
| 论文 APTM horse | 17.80 | C 4.9 + G 12.9 |

⇒ P3-P1 已把 **C 打到论文水平（4.639 ≤ 4.9）**；剩余差距全在 **G（16.52 vs 12.9）**
和 P4 容器开销。C 若再做 null_patch 去冗余可到 ~4.0–4.5。

---

## 4. 缺口 / P3-P2 计划

| # | 项 | 现状 | 计划 | LOC | 轮次 |
|---|---|---|---|---|---|
| 1 | **polar 几何（论文 §4.2）** | G 16.52（AD 一阶） | 走引擎 H3/H4 hook：Frenet 帧 + θ/φ 3-bit + 半径；复用 P1 已验证的量化/展开代码 + Arch-2 位置对称（conquest 拓扑坐标无关，Arch-2 只做几何） | 400–700 | 1–2 |
| 2 | **null_patch 去冗余** | 42k–109k 个，C 4.6–5.2 bpv | 编码末尾模拟 decode 去掉可省 null_patch（论文 ≥10%） | 150–300 | 1 |
| 3 | **cleaning conquest 未接** | 只调 `decimateLayer`（`regulateLayer` 未接线） | 交替 decimate/clean + 解码端按层类型重放（层类型 flag） | 200–400 | 1 |
| 4 | **container/rounds 口径** | r16 最优；r150 也可解 | P4 一并处理（层表/seed 压缩） | 100–300 | 0.5 |
| 5 | 质量口径 | coord_max 3.78e−5（量化无损级） | polar 有损后按论文 Metro/bbox% 登记 | — | 随 1 |

**下一轮建议**：先做 **Arch-2 spike（1 轮，500–800 LOC）**——因为 C 已达目标，瓶颈
在 G；conquest 路径拓扑坐标无关，Arch-2 只需位置对称的几何编码，成功后直接接
polar（P3-P2），目标 **19.4–20.4 bpv（1.09–1.15×，过 1.20 理想）**。若 Lead 更想
先做 null_patch 去冗余（1 轮、稳、−0.5~−1 bpv），也可先做；两者不冲突。

---

## 5. 验收（本轮，全部 PASS）

1. 默认 APTM：84,496 B / 34.052088 bpv / md5 与 `/tmp/wsb2/aptm_audit/base` 逐字节一致 ✓
2. `test_conquest_paper_fusion.sh` PASS：90,490 B / bit-exact ✓
3. P3 路径：rounds∈{8,12,16,24,32,150} 的 bpv / C / G / container / null_patch /
   anchors 全部入 CSV ✓
4. decode 六项 + A1 计数：全 true / 19,851 / 39,698 ✓
5. 证据脚本 + CSV + 报告；默认/tag 路径零改动 ✓
6. 未用 RETRI/Conquest 别名（`algorithm=conquest` 仅作 AD baseline 对照行）、未降 q ✓
