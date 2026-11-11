# P3 connectivity spike (task-15 / WS-B4, 2026-10-22 round 2)

**Question.** MMSE P1 已证明 predictor 栈最多 −71 B、P2 残差熵现实 headroom 只有
−0.3…−0.8 KB；硬顶 22.10 bpv（54,838 B）需要的 −7.8 KB 只能来自 connectivity 的
**表示重建**。本轮先做 1 轮设计/可行性 spike（不写大实现），回答：valence-conquest
replay 到底要多少 LOC、多大风险、几轮、预期 bpv，以及值不值得投入 1.5–2.5k LOC。

**Verdict: GO —— 但 P3 不是"从零写 1.5–2.5k LOC 的 conquest coder"，而是
"把论文 §2 自己指定的 connectivity/decimation 引擎（框架里已有）接成 MMSE 的几何"。
实测该 backbone 单独已经落在 21.81 bpv（硬顶 22.10，余量 708 B，decode 六项全
true，q12 质量与现 MMSE 同尺度）。P3 实现 ~870–1,130 LOC、2–3 轮、中风险，
**全部在写作用域内（impl/MMSE/** + src/predict/**，零 CMake 修改）**；预期落地
**21.8–22.1 bpv（硬顶大概率 PASS，理想 20.40 不可达）**。

---

## 1. 决定性发现：论文要求的 connectivity/decimation 引擎已在仓库里

MMSE 论文 §2（`docs/pdfs/MMSE.txt` p.906）原文：

> "By iterating the vertex decimation conquest [2], a mesh is decomposed into a
> base mesh and several refinement layers. … the connectivity data of each layer
> is compressed by the valence-driven coder in [2]."

即论文的 connectivity 与 decimation 都是 **Alliez & Desbrun valence-conquest
[2]**。仓库里已有该机制的**框架原生、算法无关**实现：

| 组件 | 位置 | 状态 |
|---|---|---|
| 前沿式顶点删减引擎（编码） | `src/conquest/FrontierConquestEngine.{h,cpp}` | 与验证器三网格**逐符号对拍一致**（fandisk 30000/30000、cow_simp 669/669、cow_open 695/695） |
| 解码镜像（一层） | `src/conquest/ConquestDecodeLayer.{h,cpp}` | 四网格 × 8 层链式 `topoGeomOk=1`，与编码端**严格互逆** |
| 中性别名/纯函数（唯一来源） | `src/conquest/ConquestTraversal.h` | 两侧共用 |
| 流布局（含量化网格） | `src/conquest/ConquestBlobFormat.h` | v2：每层 `symbols`（3-bit RC）+ `deltas`（逐分量 lo/span 精确字母表 RC） |
| 5 个引擎级 hook（S5） | `src/scheme/EngineHooks.h` | H1 gate 次序 / H2 可删判据 / **H3 几何预测子** / H4 残差量化 / H5 符号后处理；**解码端用同一份 hook** |
| 现成算法外壳 | `impl/Conquest/**`（`algorithm=conquest`） | 编码薄壳 + reader + reconstructor + layout15 解码分支 |

关键：**H3 `GeometryPredictor` 就是论文 dual-ring/MMSE 预测子该挂的位置**，而且
`ConquestDecodeLayer::decodeLayer(..., const EngineHooks&)` 与
`ConquestReconstructOperator::setHooks()` 已经把**同一份 hook 实例**接到解码端 ——
预测子只影响残差熵、不影响重建精度（`位置 = 预测 + 残差`，残差在量化整数域）。
所以 P3 的几何 fidelity 是"往已有接缝里插 hook"，不是重写 traversal。

## 2. 实测：backbone 的 bpv 地板（本 spike 的核心证据）

复现：`impl/MMSE/mmse_p3_backbone_probe.sh`（只跑现有二进制，不 build）；
证据 CSV：`impl/MMSE/MMSE_P3_BACKBONE_EVIDENCE.csv`；原始 run
`/tmp/mmsep1/p3probe/cq_r*/`。配置：`algorithm=conquest`（仓库自带
`configs/algorithms/RD.json`）、horse19k、q12、`PMSF_BASE_EDGEBREAKER_NATIVE=1`；
硬顶 22.10 bpv = **54,838 B**，理想 20.40 bpv = 50,620 B；现 MMSE canonical =
62,633 B / 25.241247 bpv。

| rounds (=层数) | artifact B | bpv | ratio/17.0 | 硬顶余量 | decode 六项 | base 顶点 |
|---:|---:|---:|---:|---:|---|---:|
| 3 | 54,454 | 21.9451 | 1.2909 | +384 B | 全 true | 13,158 |
| 4 | 54,141 | 21.8190 | 1.2835 | +697 B | 全 true | 6,295 |
| **5** | **54,130** | **21.8145** | **1.2832** | **+708 B** | 全 true | 3,118 |
| 6 | 54,175 | 21.8327 | 1.2843 | +663 B | 全 true | 1,658 |
| 8 | 54,315 | 21.8891 | 1.2876 | +523 B | 全 true | 396 |
| 10 | 54,388 | 21.9185 | 1.2893 | +450 B | 全 true | 117 |
| 12 | 54,468 | 21.9507 | 1.2912 | +370 B | 全 true | 66 |
| 14 | 54,532 | 21.9765 | 1.2927 | +306 B | 全 true | 62 |
| **16** | **54,604** | **22.0055** | **1.2944** | **+234 B** | 全 true | **74** |
| 18 | 54,668 | 22.0313 | 1.2960 | +170 B | 全 true | 46 |
| 20 | 54,741 | 22.0608 | 1.2977 | +97 B | 全 true | 20 |
| 24 | 54,856 | 22.1071 | 1.3004 | −18 B | 全 true | 2 |

- **r16 = conquest 迭代到收敛的"论文式"工作点**（base 只剩 74 顶点，与 A&D 语义
  一致）：22.0055 bpv，仍 PASS 硬顶；r4–r8 是 bpv 最优带（余量 0.5–0.7 KB）。
- decode：所有行的 MeshCompare 六项（counts / connectivity / topology /
  nearest_geometry / manifold_profile / coordinate_remap）**全 true**。
- 质量（SurfaceMetrics 5000 samples, r5 & r16 同值）：
  `rms_max = 9.23407766308e-06`、`mse_sym = 8.43–8.45e-11`；现 MMSE canonical
  `rms_max = 9.23242365323e-06`、`mse_sym = 8.47382702311e-11` —— **同尺度
  （差 <0.3%），不构成质量退化**。q12 未变、无 q 下调、无 48k 模型。

### 分量拆分（CQST blob 逐层解析，SBC1 解包后）

| 工作点 | CQST raw | connectivity symbols | geometry deltas | 固定头 | compact(base) | parquet |
|---|---:|---:|---:|---:|---:|---:|
| r5 | 42,782 B | **9,443 B (3.81 bpv)** | 32,815 B (13.22 bpv) | 524 B | 10,884 B（base 3,118 v） | 43,246 B |
| r16 | 54,438 B | **11,344 B (4.57 bpv)** | 41,514 B (16.73 bpv) | 1,580 B | 468 B（base 74 v） | 54,136 B |
| 现 MMSE r50 | 60,743 B | valence 4,087 + isPoly 12,386 = **16,473 B (6.64 bpv)** | residual 43,650 B (17.59 bpv) | container 620 B | 598 B（base 54 v） | 62,035 B |

两个可直接写进 P3 归因的事实：

1. **r16 的每层 connectivity = 11,344 B = 4.572 bpv，几乎等于 A&D 原论文公布的
   4.61 bpv** ⇒ 这套 backbone 就是论文 [2] 的 coder，不是近似；相对现 MMSE 的
   16,473 B **省 5,129 B（−2.07 bpv）**。
2. 现 MMSE 的 isPoly 流（12.4 KB）不是编码不够好，而是**显式 gate 子集**的信息量
   本身：每轮 BFS 访问 ~5,600 面只标 ~400 个门，log2 C(5600,400) ≈ 265 B/轮，
   正好 ≈ 实际 12.4 KB（见 `MMSE_BPV_AUDIT.md §P1.4`）。只有"让门集由 conquest
   隐式决定"才能消掉它 —— 这正是 P3。

## 3. P3 实现设计（在写作用域内，零 CMake 修改）

### 3.1 接入方式（已验证可行）

- 新算法 **`impl/MMSE/MMSECQuestAlgorithm.cpp`**：继承 `ISimplificationAlgorithm`
  并 `PMSF_REGISTER_SIMPLIFICATION_ALGORITHM`。它同时在本 TU 内用
  `NamedDecompressOperatorRegistrar(name, Backend::Columnar, ...)` 注册解压三件套。
  两个 target 都编译 `${PMSF_ENTRY_SOURCES}`（含 `impl/*/*Algorithm.cpp`），因此
  **压缩与解压两侧都自动带上，零 CMake 修改**（`cmake -S . -B build` 的 GLOB 会
  捡到 `impl/MMSE/{compress,decompress,reconstruct}/*.cpp`）。
- **默认 MMSE 路径完全不动**：新算法是独立注册名（建议 `mmse_cq`，避免与现有
  `algorithm=MMSE` 的 RetriOverride 桥撞名），canonical 62,633 B 继续逐字节不变。
- 复用 layout 15 的框架解码分支（`ColumnarDeserializeOperator` 对 conquest layout
  的分支会 `dynamic_cast<ConquestSymbolReadOperator*>`）⇒ 我们的 reader **继承**
  `ConquestSymbolReadOperator`、reconstructor **继承**（或薄封装）
  `ConquestReconstructOperator`，即可复用已验证的"装填 → 逐轮 collectFromDecoded
  → reconstruct"驱动，不需要 src/ 改动。

### 3.2 几何：MMSE predictor 栈怎么插进去（H3 hook + 层后重编码）

难点：论文 §2.3 的 mode decision 是**每 cluster** 选最优模式，而 H3 hook 是
**逐顶点**被调用（无前瞻）。解法（无需两遍 traversal）：

1. **编码端 H3 hook** 返回引擎默认的"邻居算术平均"（= 论文 dual-ring 的 α=0
   / mode 0），于是引擎算出的 `delta = target_cell − mean_cell` 被 `GeometrySink`
   收下；同一次 hook 调用里同时算出 4 个候选预测子（mode0 barycentric / mode1
   dual-ring α=0.5 / mode2 α=1 / mode3 MMSE-LS）的 cell 值并暂存。
2. **层后处理**（`exportToBufferWithRangeCoder` 之前，仍在 `impl/MMSE`）：
   `target = delta + mean` ⇒ 已知每个顶点的真值与 4 个候选 ⇒ 按论文 §2.3 eq.(9)
   以 **BFS 顺序连续块为 cluster**（`PMSF_MMSE_CLUSTER`，默认 32）选 Σ‖x−f_k(x)‖²
   最小者 ⇒ 残差 `= target − pred_mode` **覆写进该层 CQST deltas**，并把每 cluster
   的 2-bit mode id 写进 MMSE 自描述 trailer。
3. **解码端**：reader 继承 `ConquestSymbolReadOperator`，解析
   `[i32 cqstSize][CQST][u32 clusterSize][u32 numClusters][packed modes]` 后把该层
   mode 序列装进全局 replay state；reconstructor 继承
   `ConquestReconstructOperator` 并 `setHooks()` 一个 H3 hook，hook 按 replay state
   返回 `pred_mode`（cell 域）⇒ `decodeLayer` 的 `pred + 残差` 还原精确位置。
4. **对称性**：所有预测子都在**量化 cell 域**用 `PatchView.neighborPos`（两侧
   引擎已保证同一串坐标）与 `neighborIds`(poolId) 计算；dual-ring 的 secondary
   ring 用"注册的 mesh + poolId → 顶点 → 受控环游"取得。encoder 侧删除中被调、
   decoder 侧插入中被调，二者是引擎保证的镜像状态。

### 3.3 文件 / LOC / 风险

| 文件（全在写作用域内） | 内容 | LOC 估计 | 风险 |
|---|---|---:|---|
| `impl/MMSE/MMSECQuestAlgorithm.cpp` | 算法自注册 + 解压三件套注册 + 配置 | 90–130 | 低 |
| `impl/MMSE/compress/MMSECQuestSymbolCollectOperator.{h,cpp}` | 驱动 engine（照 `impl/Conquest` 薄壳改）+ GeometrySink 暂存 + 层后 mode 决策/重编码 + trailer 写出 | ~380–460 | 中 |
| `impl/MMSE/decompress/MMSECQuestSymbolReadOperator.{h,cpp}` | 继承 Conquest reader：解析 trailer + 装 replay state | 90–120 | 低 |
| `impl/MMSE/reconstruct/MMSECQuestReconstructOperator.{h,cpp}` | 继承 Conquest reconstructor：`setHooks(MMSE H3)` | 110–140 | 中 |
| `src/predict/MMSEPredictor.h`（扩展） | cell 域 barycentric/dual-ring/LS 候选 + H3 `GeometryPredictor` 实现 + mode replay | +200–280 | 中 |
| 证据脚本/文档 | probe + gate 复现 + fidelity 对照 | （不计入代码 LOC） | 低 |
| **合计** | | **~870–1,130 LOC** | **中** |

**无需任何写作用域外改动**。两个可选的降风险项（都要 Lead 批准）：
(a) `src/scheme/EngineHooks.h` 的 `PatchView` 增加 `centerId`（~15–20 LOC，涉及
`src/conquest/{FrontierConquestEngine.cpp,ConquestDecodeLayer.cpp}` 填字段）——
dual-ring 需要把"被删中心点"从邻居 l 的环里排除，有 centerId 最稳；没有也能做
（中心点是"同时邻接全部 ring 顶点"的唯一候选，P1 已用类似判据）；
(b) 若最终要求 artifact metadata 里 `algorithm=MMSE`，需要在
`impl/common/RetriOverride.h` 加 ~25 LOC 的 env 分支（当前写作用域外）。

## 4. 预期 bpv（含敏感性）

- backbone 单独：r4–r8 = **21.82–21.89 bpv**（余量 523–708 B），r16（论文式收敛
  工作点）= **22.0055 bpv**（余量 234 B）。
- 加上 MMSE 预测子栈：P1 已实测 dual-ring/LS 相对 barycentric 的残差熵收益
  ≤0.1–0.3%（本 backbone 的几何是 32.8–41.5 KB ⇒ **−0…−120 B**）；per-cluster
  mode 流 = 3 B/层 + 2 bit/cluster（cluster 32 时 ~600 clusters × 2 bit ≈ 150 B
  + 头）⇒ **+50…+300 B**。净 **−100…+300 B**。
- **预期落地：21.8–22.1 bpv**：
  - r4–r8 工作点：21.82–22.0（PASS，余量 ~200–700 B）；
  - r16 论文式工作点：22.0–22.1（PASS/临界，取决于 mode 流大小）；
  - 理想 20.40（50,620 B）**不可达**：还差 ~3.4 KB，需要把 base-mesh coder
    （r16 的 468 B 已不是问题）或几何 coder（16.73 bpv vs 论文 13.1）压到论文级；
    后者是框架级问题，不是 MMSE 单行能解决的。
- 敏感性最大的两个未知数：① 我们的几何残差熵 coder 沿用 backbone 的"逐层
  逐分量 lo/span + 精确字母表自适应 RC"是否与论文 bit-plane coder 有系统性
  差异（若差 >0.5%，需改回 MMSE bitplane coder 或 hybrid）；② 低层数工作点的
  base mesh 较大（r5 base=3,118 v，compact 10.9 KB 里含 base 几何+连通）。两者
  都会在 P3-B1/B2 用实测回答，工作点/report 口径由 Lead 定。

## 5. 里程碑（2–3 轮）

- **P3-B1（1 轮）**：新算法骨架接上 backbone（先不做预测子，H3=默认 mean），
  端到端 compress/decompress + 六项 decode flags；默认 MMSE 字节不变复核。
  预期 ≈ 21.82–22.01 bpv PASS；产出"MMSE-on-conquest 骨架"证据。
- **P3-B2（1 轮）**：H3 hook 接入 dual-ring α∈{0,0.5,1} + MMSE-LS + per-cluster
  mode decision + mode trailer；逐项消融；验证 decoded mesh 与 encoder 量化网格
  逐点一致（对称性硬判据）。预期 21.8–22.1。
- **P3-B3（0.5–1 轮）**：工作点选择（paper-like r16 vs bpv-opt r4–r8）、运输
  （EB-native base 已含；zstd22/zlib 已测无额外收益）、fidelity 对照文档、把
  `faithful_path`/runner 口径交 Lead/T10 更新。

## 6. 风险与缓解

| 风险 | 级别 | 缓解 |
|---|---|---|
| H3 拓扑型预测子 encode/decode 不对称（encoder 删除中 vs decoder 插入中） | 中 | 先用"decoded q12 网格逐点 == encoder q12"作硬判据；不对称则退回仅 mode 0（barycentric）并如实标注 fidelity 缺口；P1 的锚定半边经验可复用 |
| mode decision 的前瞻（cluster 级） | 中→低 | 已用"层后重编码"消解，不需两遍 traversal |
| 硬顶余量小（r16 仅 234 B） | 中 | rounds 扫（r4–r8 余量 0.5–0.7 KB）；必要时选 bpv-opt 工作点并如实登记 |
| 几何 coder 与论文 bit-plane 有差异 | 中 | B2 实测两种 coder，取小者；文档如实写"等价自适应算术编码" |
| artifact metadata 算法名/runner 口径 | 低 | 独立注册名 `mmse_cq`，默认 MMSE 零影响；口径由 Lead/T10 决定（可选 (b) 25 LOC） |
| 与现有 conquest 算法混淆（用户禁用 conquest 变体当忠实行） | 低→中 | P3 的落点必须是"论文 §2 的 connectivity/decimation + MMSE 几何预测子"的**新实现**，不是把 `algorithm=conquest` 的数字当 MMSE；报告时把 backbone 实测仅作为**可行性证据**，最终交付行必须带 MMSE 预测子栈与明确 fidelity 说明 |

## 7. 建议

**GO**，按 P3-B1 → B2 → B3 共 2–3 轮推进；不需要先批 1.5–2.5k LOC 的 from-scratch
预算（实际 ~870–1,130 LOC）。若 Lead 认可，下一轮从 P3-B1 开始；同时请裁决两件事：
1) 是否允许新增 `PatchView.centerId`（~15–20 LOC，写作用域外，de-risk dual-ring）；
2) 最终验收口径用独立算法名 `mmse_cq` 还是要求 `algorithm=MMSE`（后者需 ~25 LOC
   `impl/common/RetriOverride.h` 的 env 分支）。

## 8. 复现

```bash
# 只跑现有二进制，不 build
impl/MMSE/mmse_p3_backbone_probe.sh /tmp/mmsep1/p3probe
# 单点复现（r5/r16）
python3 - <<'EOF'   # 见脚本内 c.json 模板；algorithm=conquest rounds=5/16 q12 horse19k
EOF
```

证据：`impl/MMSE/MMSE_P3_BACKBONE_EVIDENCE.csv`、
`impl/MMSE/mmse_p3_backbone_probe.sh`、原始 run `/tmp/mmsep1/p3probe/`。
backbone 引擎本身的验证记录：`src/conquest/FrontierConquestEngine.h` 文件头 +
`docs/refactor/P13_3_ENGINE_HOOKS_2026-09-22.md`、`P13_4_DECODER_BLUEPRINT_2026-09-22.md`。
