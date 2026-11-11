# PCATM task-61 residual-context ledger（无 build 消耗；未改 live）

## 结论
prev-magnitude/round/axis residual context **未能在 1-build 预算内兑现**理论 headroom。

- 现状：canonical 68,027 B / 27.415042 bpv（color 22,272 + residual 43,711 + container 340 + framing 1,704）；official fused 66,991 B / 26.997532 bpv；CIKM 26.153 → target 64,895 B（需 −2,096 fused / −3,132 canonical）。
- 理论：H(residual | round, axis, prev-magnitude-bucket8) = **38,020 B**（59,082 值），确实存在约 3.4 KB context headroom → residual 可达 ~38–40 KB、canonical ~62.5–64.5 KB。
- 实测（standalone range-coder harness，/tmp/task59/standalone，与生产同 primitives，roundtrip OK）：
  1. 整数混合模型（A=round×axis×role，B=axis×prevBucket×role，权重 2:1，cap4）：**43,729 B**（baseline sv13 pooled-axis 43,690 B）→ +39 B，无收益；
  2. 直接扩展上下文（round×axis×prevBucket 自适应 qsmodel，cap4/5/8）：**44,283–44,430 B** → 更差。
- 归因：59,082 个值 / 40 轮下，context 分裂导致的模型自适应开销（~5 KB）超过 context 增益（理论 ~3.4 KB）。要兑现 headroom 需要更强方案（如 2-pass 静态 + context 量化/共享表、或 logistic mixing/SSE），超出本 1-build 简单实现范围。

## 状态
- **0 build 消耗**（仅 standalone 测试；live/artifact/矩阵未动）。
- PCATM 矩阵保持：fused 66,991 / 26.997532，canonical 68,027 / 27.415042；**未超过 CIKM，未标 PASS**。
- 建议：本路线转 ledger；若仍要追 CIKM，建议另立任务做静态两遍 context 量化（或换 PCATM 其它流），并在计划中预留 ≥1 build 做 coder 迭代。

## 复现
```
/tmp/task59/standalone/residuals.bin                  # 19,694 条残差（sv13 encoder dump）
/tmp/task59/standalone/mix_test residuals.bin        # 43,729 B，roundtrip OK
/tmp/task59/standalone/ctx_test residuals.bin        # 变体扫描（43,690–44,476 B）
```
