# PCATM task-62 LEDGER — 2-pass static/quantized-context residual coder（sv14）

日期：2026-10-29；owner：aptm-repro（handoff from rd-canonical，`PCATM_TASK62_HANDOFF.md` sha `12bb4df8…`）
工作区：`/tmp/task62` → `/tmp/task61/src`（**live 树的隔离 copy**，14:20 snapshot；live 只新增本 CSV/MD，无代码/默认改动）
build：**1 build**（`cmake --build -j32 --target PMSFCompress PMSFDecompress MeshCompare`，rc=0，日志 `/tmp/t62_build.log`）
证据：`impl/PCATM/PCATM_TASK62_EVIDENCE.csv`（53 行）、patch `/tmp/pcatm_task62_sv14.patch`（4 文件，`git apply --check` OK，**未合入**）、runs `/tmp/task62_runs/`

---

## 0. 结论（先读）

**FAIL（bytes）/ decode PASS / guards PASS —— 不标 PASS，不伪造。**

| 项 | 目标 | 实测 sv14 | 判定 |
|---|---|---|---|
| artifact bytes | ≤ 64,895（理想 ≤64,000） | **69,257 B = 27.910735 bpv** | **FAIL（+4,362）** |
| residual stream | ≤ 40,000（理想 ≤39,000） | **44,947 B**（payload 44,067 + header 880） | **FAIL（+4,947）** |
| vs sv13 对照（同 build） | 应显著更小 | sv13 = **68,027 B / 27.415042**（residual 43,711） | sv14 **更差 +1,230 B** |
| decode | 逐字节不变 | rc=0；decoded sha `c3382783…` == canonical strict decode（bit-identical） | PASS |
| decode 六项 | 全 true | counts/connectivity/topology/nearest/remap/test_valid 全 true；coord_max 3.78e-05 | PASS |
| guards | 不变 | strict 71,550/md5、PURE_K3 69,187/md5、PCLTTM 68,160、phase1 PASS(9/1/1, violations 0)、phase2 paper 8/8 | PASS |
| fused M5-9 | ≤64,895 | 68,216 B（decode bit-identical） | FAIL（+3,321；且 > sv13 canonical 68,027） |

**根因（结构性，实测量化）**：handoff 的 37,786 B payload（+579 B table）预测**不可复现**。独立重算
同一条 `residuals.bin`、同一 handoff context `(roundGroup=8, axis, prevBucket=cap2, slot)`：
empirical cross-entropy = **44,047 B**，与本实现实测 payload **44,067 B**（+20 B，range-coder/8-bit 量化开销）一致。
扫描更丰富 context 的 CE：最佳可实际落地模型（groups=40, pb8, fb16）= **43,465 B**（仅比 sv13 的 43,711 好 246 B）；
**过拟合上限**（groups=40, pb64, 无 fallback）= **42,466 B**。即使拿到过拟合上限，也还差目标
（residual ≤40,579）**1,887 B**。⇒ **2-pass static residual recoding 在本流上不存在 ~5.9 KB 的 headroom，
task-62 的 ≤64,895 目标无法靠 residual 重编码单独达成。**

---

## 1. 实现（sv14，全部 env-gated，sv13/default 字节不变）

4 文件（patch `/tmp/pcatm_task62_sv14.patch`）：

1. `impl/common/strict/PCATMStrictAdaptiveResidual.h`：新增 `namespace scr`（sv13 `ard` 不动）
   - 上下文：roundGroup = r*8/numRounds，axis，prevBucket=min(bitlen(prevSameAxisInRound),1)（2 桶），slot ∈ [0,29)（15 length-tree node + 14 mantissa bit）；fine 1392 表项；coarse 87 `(axis,slot)` fallback。
   - pass0：统计每个二进制决策的 n0/n1；`p=(n1+0.5)/(n+1)` 8-bit 量化 1..254；fine n<16 → 写 0（fallback to coarse）。
   - header：varint numRounds、per-round counts、varint fineModels=1392、varint fb=16、varint len + **zlib(fine 1392B)**、varint coarseModels=87 + coarse raw、varint payloadBytes。
   - pass1：**静态** range coder（TOT=4096，f1=q*16 clamp 1..4095；bit0 = encode_shift(f0,0,12)，bit1 = encode_shift(f1,f0,12)），无自适应更新；解码对称（`decode_culshift`/`decode_update`）。本 header 与独立 smoke test（40 轮/4800 随机值）roundtrip OK。
2. `impl/common/PaperBlobAlgorithmSpecs.h`：`kPCATMStrictPaperStaticContextResidualVersion = 14` + IsStrictPaper/IsColorOnly/UsesGlobalColor 分支 + 新谓词 `UsesStaticContextResidual`。
3. collector export：env `PMSF_PCATM_STRICT_STATIC_CTX_RESIDUAL=1` → sv14 + `scr::encodeStaticContextRounds`；color/elimination/容器逻辑不变；diag 打印 header/payload。
4. reader import：sv14 → `scr::decodeStaticContextRounds`；其余路径不变。

验证：decode sha 与 canonical 严格解码**逐字节相同** ⇒ 位移值/重建语义完全未变（只是熵编码层改变），实现本身正确。

---

## 2. 实测（同 build，isolated copy）

命令见 §5。核心数字：

- sv13 对照（`PMSF_PCATM_STRICT_ADAPT_RESIDUAL=1`）：**68,027 B / 27.415042**，residual 43,711，decoded `c3382783…`（复现冻结值）。
- sv14（`PMSF_PCATM_STRICT_STATIC_CTX_RESIDUAL=1`）：**69,257 B / 27.910735**；blob 67,559 = color 22,272 + **residual 44,947**（header 880 + payload 44,067）+ container 340；framing 839 + compact 859。
- 目标预算：artifact ≤64,895 ⇒ blob ≤63,191 ⇒ **residual ≤40,579**（需 −3,132 / −7.2%）；实际 residual 44,947 反而 +5,368。
- decode：rc=0，six flags 全 true，coord_max 3.78023068196e-05，decoded sha == canonical。
- guards（同 build）：default strict **71,550 B md5 `ac222c20…/52b17d0c…`**；PURE_K3 **69,187 B md5 `e91bb4bc…/5d8a5d98…`**；PCLTTM r50 **68,160**；phase1 gate **PASS**（rows=10 pass=9 assumption=1 fail=0 na=1 violations=0 NOT_GREEN=none）；phase2 paper matrix **PASS 8/8**。
- fused M5-9（paper v2 blob native + SBC1-zstd，sv14 env）：**68,216 B**（fused 相对同 canonical 69,257 省 1,041，与 handoff 估计 ≈−1,036 一致），decode bit-identical；但 **> 64,895**，并且 > sv13 canonical 68,027（+189）。

---

## 3. 归因：为什么 handoff 的 37,786 B 不可达

handoff 的模型描述 = 本实现模型（roundGroup=8 × axis × prevBucket cap2 × fallback n<16→coarse）。用
`/tmp/task59/standalone/residuals.bin`（19,694 残差 / 59,082 值，sv13 dump）独立重算二进制决策的
empirical CE（p=(n1+.5)/(n+1)，fallback 按 n<16）：

| 模型 | payload CE (B) | 说明 |
|---|---:|---|
| **handoff 模型（g8, pb2, fb16）** | **44,047** | 与实现实测 payload 44,067 相差 +20 B ⇒ 实现无浪费 |
| fb=4 / fb=64 | 44,017 / 44,123 | 阈值影响 ≤100 B |
| groups=40（每轮独立，pb2, fb16） | 43,369 | 静态上限接近 sv13 adaptive 43,711（−342 B） |
| groups=40, pb8, fb16 | **43,465** | 最佳“可落地”扫描点（仍只 −246 vs sv13） |
| groups=40, pb64, **无 fallback（过拟合）** | **42,466** | 本 context 家族绝对下限 |
| handoff 声称 | **37,786 (+579 table)** | 与上述全部相差 ≥4,680 B，**不可复现** |

补充：真实产品 coder 还要为 fine 表付 header 成本（本次 880 B，压缩后），这只会让可落地值更高。
任务-61 ledger 的 “H(residual|…)=38,020 B” 同样无法复现（我们同族最好 42,466）。

**因此 task-62 的目标缺口（residual 需 −3,132 B）在 residual 熵编码层面不存在可实现空间**（真实可落地
headroom ≈ 0–250 B，甚至可能为负）。sv14 实测 +1,230 B 是 context 分裂/量化开销大于收益的直接体现。

---

## 4. 后续（供 Lead 决策，未执行）

1. 接受 PCATM 现状：live matrix 已把 sv13 B1-alt 记为 **PASS 27.415042 / 1.2970×**（phase1 gate 9/1/1 已确认）；task-62 目标（≤64,895/26.153）**需要 color 流或 mode2 的结构性改进**，不是 residual 熵编码。
2. 若仍要更低 bpv：color stream 22,272 B 是最大剩余块（信息论 floor ~22,254 已接近；task-44/45/46 实测显式 patch/mask 路线失败）；mode2/Z-shape 已 FINAL BLOCKER（task-46/47/48/49/50）。剩余可行方向只有 decoder-reconstruction 重写（task-50 估 ≈600–1,250 LOC / ≥3 builds）或接受现状。
3. sv14 代码不合并：patch 保留 `/tmp/pcatm_task62_sv14.patch`（`git apply --check` OK、env-gated、默认字节不变），若 Lead 想要“负结果可复现”可合入测试但无收益。

---

## 5. 复现

```bash
# 隔离 copy 已建于 /tmp/task62 -> /tmp/task61/src（live snapshot）
cd /tmp/task62/build && cmake --build . -j32 --target PMSFCompress PMSFDecompress MeshCompare  # 1 build, rc=0
BUILD=/tmp/task62/build bash /tmp/task62_measure.sh   # 见 /tmp/task62_runs/{sv13,sv14,guards,paper}
# phase guards
cd /tmp/task62 && MESH=<live>/datasets/Stanford/bunny.off BUILD_DIR=/tmp/task62/build \
  OUT_ROOT=/tmp/t62_phase2 scripts/tests/test_paper_matrix.sh            # PASS 8/8
scripts/tests/run_phase1_gate.sh --no-build --report /tmp/t62_phase1_report2.md   # PASS 9/1/1
```

## 6. 诚实记录

- 1 build 已用完（rc=0）；未追加 build（无安全收益、Lead 限制）。
- live 树未改任何代码/默认；仅新增 `impl/PCATM/PCATM_TASK62_EVIDENCE.csv`、本 MD；patch 在 `/tmp` 未合入。
- 未降 q / 未降 codebook / 未用 conquest/alias；未改 displacement 值/color/elimination/decoder 语义（decoded sha 逐字节证明）。
- 不标“超过 CIKM”、不标 PASS；PCATM 矩阵行由 Lead 按既有 27.415042（sv13）口径维护。
