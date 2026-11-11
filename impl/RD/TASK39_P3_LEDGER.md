# task-39：RD PGS P3（sv=70 decoder replay）— **FAIL / BLOCKED 归因 ledger**

日期：2026-10-22（worktree `task38-rd-pgs-p1p2`，base `3de2e54`）
结论：**不能在不增加 sideinfo 的前提下解码 sv=70 的 48,680 B 产物；P3 本轮判 FAIL（blocked-by-design）。**
encoder-only P1+P2 证据保留（48,680 B / 19.618155 bpv，PGS moved 99.0%，默认守卫不变）。
本 ledger 给出三项**实测**blocker 与量化后的 sideinfo 下界；未伪造 decode/PASS。

## 交付内容（本任务增量，全部 env-gated，默认路径零影响）
- `impl/RD/elimination/RDTagEliminateOperator.cpp`：env `PMSF_RD_PGS_DUMP_ELIM` 逐消除 journal
  （seq / removed poolId / ring poolIds）。
- `impl/RD/compress/RDTagSymbolCollectOperator.cpp`：同 env 写 per-round 边界；P2 export 时输出
  `[P3-FEAS]` 可行性诊断（final mesh 面度数分布、首 decode 步候选数）。
- 默认 rd_tag 守卫复核：64,604 B / residual md5 `d1fde1ae…` / compact `98e5c511…` **不变**。
- 证据：`/tmp/wsb38/elim_journal.txt`、`/tmp/wsb38/p1p2_sv7/log39b`、脚本见 task38。

## Blocker 1（结构性）：base mesh 已全部三角化 ⇒ patch 位置不可从 mesh 识别
实测（`[P3-FEAS] final_mesh`）：`faces=162, ngons(degree>3)=0, triangles=162`。
sv=70 删掉 isPoly locator 后，decoder 拿到 base mesh 全是三角形，**无法判断哪些面是
本轮被消顶点留下的 polyFace**，也无法知道 patch 的 ring/中心点。locator 不是"冗余结构"，
而是解码所需的最小定位信息。

## Blocker 2（信息论）：同轮删除彼此**独立** ⇒ 顺序无拓扑约束，顺序 sideinfo 下界 > locator
- journal 实测：19,768 次消除，**同轮"ring 含更早被删顶点"的比例 = 0.000**；
  每轮 roots≈395、最长依赖链=1 ⇒ 同轮删除是一个**独立集**（这是好消息：插入顺序不影响
  拓扑/预测），但反过来也意味着**删除顺序完全没有拓扑约束**。
- 顺序数下界：每轮 k 个删除 → k! 种顺序，`Σ log2(k!) ≈ Σ k log2 k`。
  实测 50 轮、19,768 删除（平均 395.4/轮）⇒ **≈170,600 bits ≈ 21.3 KB**，
  显著大于原 isPoly locator 的 13,154 B（gateq 实测实际花费 12,803 B）。
  ⇒ 想靠"把 selection 顺序当 sideinfo 存下来"来救 locator-free，**体积上严格变差**
  （21.3 KB ≈ +8.6 bpv），不是可行路线。
- 每个残差还与具体 patch 配对：即使顺序已知，也需要 per-gate patch 身份（同一 locator 问题）。

## Blocker 3（算法状态）：PGS selection 依赖 intra-round 演化状态，不可从 coarse 状态重算
- 实测 P1 priority histogram：`p6=6,250 / p4=3,152 / p2=10,366`——这些级别来自
  ⊕/⊖ tag evolution + conquered-incident 计数，而 conquered 集合在**同轮内随每次消除演化**；
  decoder 在反向插入时只有 coarse 状态（conquered 计数与 encoder 删除时不同），
  无法复现同一 gate 优先级。
- GreedySelector 的 seed 为 `rand()` 回退（非内容 canonical）；gate queue 的扩张/更新
  (`GreedySelector::update`) 也依赖 intra-round mesh 状态。
- ⇒ 现有 sv=70 的 selection 顺序 **不可由 decoder 从产物重放**；这正是 RDStrict
  "sideinfo-free replay 只有 23/39,698 faces" 的同类根因。

## 可行路线（建议，需 Lead 决策）
1. **改用已验证 decoder replay 的 conquest 引擎做 P1/P2**（同 `rd_cq` 路径）：
   `src/conquest/FrontierConquestEngine`/`decodeLayer` 已有 4 网格 × 8 层符号级 replay 验证；
   把 PGS 优先级接到引擎 H1（GatePriorityPolicy；task-19 已做 additive hook，但 decoder
   mirror 曾 hung → 需先修 H1 双侧出队一致性）。该路线可望在保持可解码前提下逼近 19.6–21.1。
2. **接受 locator（回到 isPoly）**：可解码，但 artifact 回到 62,490/25.18（sv7）附近，
   不满足本任务 bpv 目标。
3. **接受 BFS 顺序 + locator-free 的 decode-capable 变体**（sv=71）：
   encoder 已在 A/B 实测 `moved=0` 形态 = 49,060 B / 19.771296 bpv（ratio 1.171× paper，
   ≤1.2 ideal）。但 P3-FEAS 显示 base mesh 无 N-gon ⇒ 仍缺 patch 身份 locator；
   仅在 **base mesh 保留未拆 polyFace**（改 base 写出语义）时才可行——属于容器/编码协议改动，
   需单独立项评估。
4. **加权残差+读流顺序隐式配对**不可行：同轮独立 ⇒ 无拓扑约束可推导配对。

## 交付/状态
- P3：FAIL（blocked-by-design）；未实现 decoder，未声称 RD PASS；encoder-only 证据保留。
- 默认守卫：rd_tag 64,604 / 26.035565 / md5 不变（本任务 instrumentation 全 env-gated）。
- 阶段一仍 7/2/1；RD 行保持 `rd_tag` 26.036 FAIL（targets-quality notes 已加 encoder-only variant）。
