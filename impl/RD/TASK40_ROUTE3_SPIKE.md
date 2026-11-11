# task-40：RD route-3（BFS-order P2 + N-gon base patch identity）— **SPIKE 结论：FAIL / blocked-by-design**

日期：2026-10-22（worktree `task38-rd-pgs-p1p2`@3de2e54）
本轮 1 次增量 build 完成 route-3 可行性 spike；结论：**"base mesh 保留 N-gon" 本身不能提供
decoder 所需的 patch identity（不是开销问题，而是信息缺口）**；按 Lead 预设判 FAIL，未伪造 decode。

## 1. 实测事实
1. **base mesh 序列化格式已支持 N-gon**：`ColumnarSerializeOperator::serializeBaseMeshWithSeed`
   按 `faceDegrees` 逐面写（`faceVertices` + `faceDegrees`，含 seed 半边坐标），
   并不强制三角化。⇒ 路线 3 的"最小容器改动"在格式层**不需要**。
2. **但 encoder 端 mesh 在 eliminate 就 fan-triangulate**：
   `VertexRemovalEliminateOperator::eliminate` = `remove_point` → `triangulate`（fan，V−2 三角形）
   ⇒ expose 给 collector/base 的 mesh 始终是三角形（patch 只以 fan 三角形 + `removedVertexPos`
   属性存在，不是 N-gon）。实测 export 时：`final_mesh faces=162, ngons(degree>3)=0, triangles=162`。
3. **route-3 的核心缺口：base 只有"终态"，而 decoder 需要**每一轮**的 patch identity**。
   encoder 第 r 轮的 patch 在后续轮次会被消费/合并；decoder 反向还原时，状态序列为
   `base(第 R 轮后) → 插入第 R 轮 patch → 第 R−1 轮后 → …`，**中间各轮 patch 集合并不存在于 base**。
   只保留终态 N-gon 只能识别最后一轮 patch（实测最后 5 轮删除数 `[5,8,6,8,5]`，即终态最多
   ~5 个 patch identity），对 19,768 个删除中的绝大多数无用。
4. 每轮删除规模（journal 实测 50 轮）：`sum=19,768`、`max=3,584/轮`、`min=5/轮`；
   每轮 patch 集合需要被 decoder 知道（位置/环）才能插入。
5. 之前 P3 ledger 的"同轮独立集"结论经二次校核成立：既无"ring 含同轮更早被删顶点"（结构上
   不可能），也无"ring 含同轮更晚被删顶点"（实测 0.000；等价于同轮删除两两不相邻 ⇒ independent set）
   ⇒ 顺序不影响拓扑，但**不影响 patch identity 缺口**（identity 仍需每轮传输）。
   因此 ledgers 的 24.5KB 顺序 sideinfo 下界依然成立（若要存顺序的话）。

## 2. 为什么这使 route 3 FAIL（不是"开销超 21.96"）
route-3 的目标"base N-gon ⇒ patch identity 零成本"假设不成立：identity 是**逐轮**信息，
不在终态 base 里。要补上它，等价于逐轮 side stream：
  - 若写每轮 patch 多边形/环：约等于把 isPoly locator 换个编码（原 locator 13,154 B），
    48,680/49,060 的 bpv 优势即消失（回 ~25 bpv），不满足 ≤21.96；
  - 若靠 deterministic replay：需要 decoder 复现 PGS/Greedy gate 序（task-39 已证
    intra-round 演化 + rand seed 不可复现），同样 FAIL。
⇒ 在"faithful（禁止 conquest 变体）+ 无 sideinfo + ≤21.96"三重约束下，route 3 无法闭合。

## 3. 保留与建议
- encoder-only 证据保留：48,680 B / 19.618155 bpv（selection-order sv=70）与
  49,060 B / 19.771296 bpv（BFS-order sv=70 变体；ratio 1.171× ≤1.2 ideal）；均**无 decoder**。
- 默认守卫复核：rd_tag 64,604 B / 26.035565 / residual md5 `d1fde1ae…` / compact `98e5c511…` 不变。
- 建议（需 Lead 裁决其一）：
  (a) 接受 encoder-only labeled variant（RD 行保持 FAIL 26.036，不重开）；
  (b) 放宽 faithful 约束用 conquest 引擎（有已验证 decoder replay；可望 ≤21.96 且可解码）；
  (c) 立项"每轮 patch identity 侧流"并从信息论下界重估：需把 isPoly 换更紧凑编码，
      先证明 ≤13.15KB（否则 bpv 不可能 ≤21.96）；
  (d) 只做阶段二：把 encoder-only sv=70 作为 PaperStream 实验输入，不做 phase-1 PASS。
- task-40 按 FAIL 收口；phase-1 仍 7/2/1、RD FAIL；未碰 live/M4/M5/PCATM。
