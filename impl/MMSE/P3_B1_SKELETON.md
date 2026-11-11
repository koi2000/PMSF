# P3-B1: `mmse_cq` 骨架（valence-conquest backbone 接成 MMSE）— **PASS**

> 状态：**已完成并验证**（2026-10-22，build window clear 后 rebuild；压缩/解压/回归
> 全绿，见下）。上游设计见 `P3_CONNECTIVITY_SPIKE.md`。

## 1. 这是什么

`mmse_cq` 是把 **MMSE 论文 §2 的 connectivity/decimation**（= Alliez-Desbrun
valence-conquest [2]，框架 `src/conquest/` 已有且逐符号/逐层验证过）接成 MMSE
的一次实现尝试，用独立算法名注册（不触碰 canonical `algorithm=MMSE` 路径）：

- 编码：`ConquestNoopEliminateOperator`（框架不施加遍历）+ `ConquestSymbolCollectOperator`
  （引擎自己跑前沿 conquest 并按层收符号/几何残差）+ `setHooks(MMSE H3)`；
- 解码：`ConquestSymbolReadOperator` + `ConquestReconstructOperator`（框架 layout15
  分支自带的逐轮驱动，解码实体只有 `decodeLayer` 一处）+ 同一份 MMSE H3 hook；
- **B1 的 H3 = 论文 §2.2.1 eq.(1) barycentric（dual-ring α=0）**，算术与引擎默认
  predictor 逐字相同 ⇒ 产物应与 `algorithm=conquest` **逐字节一致**（plumbing 检验，
  不是最终行）。

新文件（全部在写作用域内，无 CMake 改动，现有 GLOB 自动收集）：

| 文件 | 作用 |
|---|---|
| `impl/MMSE/MMSECQuestAlgorithm.cpp` | 自注册 `mmse_cq`（`PMSF_REGISTER_SIMPLIFICATION_ALGORITHM`）+ 解压三件套 `NamedDecompressOperatorRegistrar`（Binary/Columnar/Unified） |
| `src/predict/MMSEConquestHooks.h` | H3 hook：B1 = `ConquestBarycentricPredictor`；`makeMMSECQuestHooks()` 两侧共用 |
| `impl/MMSE/mmse_cq_probe.sh` | 探针：mmse_cq vs conquest 的 bytes/bpv/md5/decode/quality |

## 2. 压缩侧实测（已通过，07:41 build）

q12 horse19k、columnar、`PMSF_BASE_EDGEBREAKER_NATIVE=1`、硬顶 22.10 bpv =
54,838 B、理想 20.40 = 50,620 B、现 MMSE canonical = 62,633 B / 25.241247 bpv。

| rounds | mmse_cq artifact | bpv | 硬顶余量 | conquest 同 rounds | payload md5 == conquest |
|---:|---:|---:|---:|---:|---|
| 5 | **54,130 B** | **21.814518** | +708 B | 54,130 B / 21.814518 | **是** |
| 16 | **54,604 B** | **22.005541** | +234 B | 54,604 B / 22.005541 | **是** |

⇒ B1 的显式 barycentric H3 hook 与引擎默认**字节等价**，hook 注入链路（编码 + 解码
两侧构造）验证成立；`mmse_cq` 的产物就是 backbone 地板。

## 3. 解压侧（已通过）

首轮 build 时发现并已修：`MMSECQuestDecompressRegistrations` 结构体漏了**全局实例**
（conquest 的对应文件是 `} g_conquestRegistrations;`），导致按名查不到三件套 →
回落 Default 解压器 → `OptimizedSymbolReadOperator` 读 CQST → SIGSEGV(139)。补
`} g_mmseCqDecompressRegistrations;` 后 rebuild 验证：

| rounds | decode_rc | counts | connectivity | topology | nearest | manifold | remap | rms_max | mse_sym | decoded == conquest |
|---:|---:|---|---|---|---|---|---|---:|---:|---|
| 5 | 0 | true | true | true | true | true | true | 9.234078e-06 | 8.4463e-11 | — |
| 16 | 0 | true | true | true | true | true | true | 9.234078e-06 | 8.4297e-11 | **逐字节一致**（dec.off md5 `d856290b…`） |

证据：`impl/MMSE/MMSE_P3_B1_EVIDENCE.csv`、`impl/MMSE/mmse_cq_probe.sh`、
`/tmp/mmsep1/cq_b1v2/{r5,r16}/`。

## 4. 回归不变量（每次 build 后必查）

1. canonical `algorithm=MMSE`：62,633 B / 25.241247 bpv，md5
   `9744d99c65407e1379b3dff1b67d7a54`（compact）/ `06a635313084b51c9ed6ae886ff821aa`
   （residual），decode 六项 true —— 新算法是**额外注册**，默认分派不变。
2. `algorithm=conquest` r5/r16 产物 md5 与 centerId 改动前一致
   （r5 `7e700ee0…`/`508f6f2a…`，r16 `15403a9b…`/`a3054310…`）。
3. `scripts/tests/test_conquest_paper_fusion.sh`：paper_bytes 90,490 / bpv 20.7820 /
   bit-exact（由 storage-architect 的 PaperStream restore 决定，Lead 协调）。

## 5. 下一步（B2）

在同一个 H3 hook 上扩展：dual-ring α∈{0.5,1}（`PatchView.centerId` 用于把中心点从
邻居环排除）、MMSE-LS、per-cluster mode decision；编码端用
`GeometrySink` 收 `delta=target−mean` 反推 `target` 后在**层后**重编码残差
（`residual = target − pred_mode`，覆写该层 CQST deltas），2-bit mode 写进 MMSE
自描述 trailer；解码端 reader 解析 trailer 装 replay state，H3 hook 按 mode 返回预测。
预期 21.8–22.1 bpv（硬顶 PASS 余量 200–700 B），理想 20.40 不可达。

## 6. B2 设计要点（本轮静态分析的结论，供下一轮实现）

**关键发现：paper 的 dual-ring 公式不能在"层内渐进状态"上直接算。** conquest 每层
删除的是**独立集**，故被删点 j 的 1-ring 全是"本层存活"顶点（coarse 多边形环，两侧
一致，B1 的 mean 预测即依赖此性质）。但 dual-ring 的 secondary ring 需要 l∈N_j 的
1-ring `N_l`：在 encoder 的层内状态里 `N_l` 含**本层后续才删**的顶点，在 decoder 的
层内状态里含**本层先前已插回**的顶点 ⇒ 两侧不同。论文公式里 `Σ_{k∈N_l,k≠j}` 若按
各自层内状态实现，encode/decode 预测会分叉（残差写错 → 位置错格）。

**正确做法（无需 closed-loop、也不扩 scope）：把预测子定义在 `coarse mesh`（本层
全部删点后的网格）上，两侧都可用：**

- encoder：引擎跑完一层后 mesh 恰好是 coarse mesh（层内顶点已 erase）。H3 在层内
  只返回 mean（引擎记录 `delta_mean = toCell(target) − toCell(mean)`），同时把每个
  gate 的 `neighborIds`(ring poolIds)、`centerId`、层序号、gate 序号记下；**层后**在
  coarse mesh 上算 4 个候选预测子（cell 整数域），`target_cell = delta_mean +
  toCell(mean)`，`residual_mode = target_cell − pred_mode_cell` 覆写该层 deltas。
- decoder：H3 在 decodeLayer 的插入时刻计算预测；用 `centerId` + **本层已插点集合**
  （hook 自己按调用序记录 centerId）从 l 的环里排除，等价于 coarse ring ⇒ 与 encoder
  coarse mesh 上的公式逐项一致。
- 全程 **cell 整数域**：邻居坐标先 `toCell` 再进公式，求和/均值与顺序无关；H3 返回
  `toWorld(pred_cell)`，引擎 `toCell(toWorld(q)) == q`（默认 H4 下）⇒ 两侧 bit-exact。
- mode trailer（2-bit/cluster）放在 CQST 之后：`[u32 trailerLen][CQST][trailer]`，
  trailer = `[magic 'MMT1'][u32 layers][per layer: u32 gateCount, u32 clusterSize,
  u32 modeBytes, packed modes]`；reader 解析 trailer 装 per-layer replay state。
- **验收硬判据**：decoded mesh 必须与 `algorithm=conquest` 同 rounds 的 dec.off
  **逐字节一致**（lossless q12）；若不一致 = 预测子不对称，先查 inserted-set 排除。
