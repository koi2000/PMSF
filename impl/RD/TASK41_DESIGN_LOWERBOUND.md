# task-41：RD 结构 R&D — 可解码 patch identity / canonical replay 设计 + 下界报告（read-only）

日期：2026-10-22（worktree `task38-rd-pgs-p1p2`@3de2e54）
状态：**设计/下界完成；未 build、未改代码。结论：只有路线 A（canonical deterministic traversal + decoder replay）可行；B/C/D 下界均超硬顶。**
以下数字全部用已有实测 artifact 与 journal 推导（来源见 §5）。

## 0. 预算修正（重要）
- 硬顶 21.96 bpv = **54,491 B**；理想 20.27 bpv = **50,297 B**（N=19,851，q12 horse19k）。
- 相对 task-38 encoder-only P2：
  | P2 变体 | artifact | side 预算（到硬顶） | side 预算（到理想） |
  |---|---|---|---|
  | selection-order sv70 | 48,680 B | **5,811 B** | 1,617 B |
  | BFS-order sv70 | 49,060 B | **5,431 B** | 1,237 B |
- ⇒ task 文本里 "side ≤13,150 B" 太宽：13,150 B 会得到 **25.18 bpv**（= sv7 原值），不是 ≤21.96。
  真正可用 side 预算只有 **5.4–5.8 KB**（硬顶）/ ~1.2–1.6 KB（理想）。

## 1. 候选路线评估（摘要）
| 路线 | sideinfo bytes（下界/实测） | decoder replay | bpv 估算 | 判定 |
|---|---|---|---|---|
| **A canonical deterministic traversal** | **0**（不写 locator/顺序 side） | ✅ 反向可 replay（状态只依赖当前 coarse mesh） | **~49.2 KB / 19.82**（理论下界）→ 实际 20.1–21.0 | **✅ 唯一可行** |
| B 逐轮 patch identity 紧凑 side | **12.77 KB**（间隙组合熵下界，见 §2） | ✅ | 48.7–49.1 + 12.8 = **61.5–61.9 KB / 24.8–25.0** | ❌ 超硬顶 3.5 KB |
| C 层级 patch base | ≥12.8 KB identity + 每轮 base/父面开销（≥数 KB） | ✅（形式） | ≥25 bpv | ❌ |
| D 其他新格式 | D1=A；D2 几何 canonical 规则 identity 熵同 B；D3 仅"歧义 patch"补 side 仍需 ≥B 的量 | 部分 | ≥25 | ❌ |
| （既有）default rd_tag | isPoly 13,154 B | ✅ | 26.036 | ❌ |
| （既有）encoder-only P2 | 0 | ❌ | 19.618 / 19.771 | 非 PASS |

## 2. 路线 B 下界（为什么 13.15KB 也不够）
- 19,768 次删除；每次删除产生 fan patch = (valence−2) 个三角形；实测 valence 分布
  `{3:4164, 4:4693, 5:5638, 6:5273}` ⇒ patch 面 = **51,556**；默认 rd_tag BFS 访问面 = **326,656**
  ⇒ 非 patch 面 Z = **275,100**，平均 gap = **13.92**。
- patch identity 序列（BFS 顺序）等价的**间隙组合**数 ≥ `C(Z+K−1, K)`，下界
  `log2 C(300,868, 19,768) ≈ 104,605 bits ≈ 12.77 KB`。
- 这还没包括：patch 的**轮次边界**、顺序/配对信息；也假设所有 0-run 长度可由 valence 推出
  （实际 fan 面在 BFS 里不保证连续，真实熵更高）。
- ⇒ B 的**理论最优**已超 5.4–5.8 KB 预算 >2×，**不可行**（不需要实现验证）。

## 3. 路线 C（层级 patch base）为什么不行
- "每轮保留 patch 多边形/父面"本质是把 identity 从终态 base 挪到每轮 base；
  identity 熵下界同 §2（≥12.77 KB），再叠加每轮 base 的父面/多边形坐标编码（≥2–4 KB）。
- 另：decoder 反向还原的每轮 base 状态是**算出来的中间态**，若要"层级 base"显式存储，
  等于每轮存一份 mesh 增量；字节量级 ≥10 KB 起步 ⇒ 同样超硬顶。
- 若只存"终态"，task-40 已证只覆盖最后一轮 ~5 个 identity（失败）。

## 4. 推荐路线 A：canonical deterministic traversal + decoder replay（faithful RD §3.3 本义）
### 4.1 编码器（取代当前 PGS/GreedySelector + 逐 face locator）
- 每轮一个 **frontier-gate** 遍历（论文 A&D 式）：
  - **canonical seed**：全网格中「proxy 顶点量化整数坐标字典序最小」的边（无 rand、与进程历史无关）；
  - FIFO gate queue；每 gate 取 `front = tgt(nxt(g))`；
  - 可删判据 = **只依赖当前 mesh 拓扑/几何**：valence∈[3,6]、非边界、流形、link condition；
    **不使用** conquered/tag/intra-round 状态（这与 task-38 的 PGS 不同，是 A 路线的关键）；
  - 可删 ⇒ 发 **valence 符号**（dval−3）+ 记录该 gate 的残差（Frenet inverse）；
    不可删 ⇒ 发 **null-patch 符号**（论文 §3.3 必需，当前 PGS 版靠候选过滤规避）；
  - 前沿扩张：删点后把 ring 对边入队；队列空 ⇒ 本层结束。
- **decoder replay**：base mesh 上重跑**同一** canonical traversal（seed/判据/扩张全一致），
  按符号流顺序：valence ⇒ merge fan region 回 polygon + 插中心点 + 反 Frenet + fan 三角化；
  null ⇒ 跳过并扩张。状态只依赖当前 mesh ⇒ 无 poolId/堆序/rand 依赖，可确定性 replay。
- **无 isPoly / 无顺序 sideinfo**（这是与 B 的本质差别：traversal 本身就是 locator）。

### 4.2 下界与 bpv 估算
- connectivity：valence 0 阶熵 `H=1.991 bit/sym × 19,768 = 4,919 B`；null-patch marker
  （按 5% gates 估）≈0.2 KB ⇒ **≈5.1 KB**。
- residual（Frenet，task-38 实测同口径）≈ **41.9 KB**；容器/parquet 固定开销 ≈ **2.2 KB**。
- 理论合计 ≈ **49.2 KB / 19.82 bpv**；给 entropy coder/边界上下文留 1–2 KB 余量 ⇒ 预计
  **20.1–21.0 bpv**，在硬顶 21.96 内（理想 20.27 需 residual 保持 Frenet 增益，边缘）。
- 实测参照：同引擎模式的 `rd_cq`（conquest + Frenet lifting）artifact 52,471 B / 21.146
  （不同 residual 编码与容器），说明该量级可达；A 路线用 RD 自己的 residual/Frenet 流
  预计不差于它。

### 4.3 实现量与 spike 计划（需 Lead 批 build）
- 编码器：canonical seed + frontier 遍历 + valence/null 符号 + sv=71 容器（~400–600 LOC）。
- 解码器：replay 循环（queue/判据/扩张镜像）+ merge/fan/proxy normalize 逆向 + 反 Frenet
  + base 重建（~500–700 LOC，可复用 RDTagSymbolReadOperator 的插入/反投影原语）。
- faithful：全部在 `impl/RD` 内新代码（新 sv=71，env-gated），**不引用/不 alias conquest 引擎**；
  默认 rd_tag 与所有守卫路径不变。
- **build 计划（诚实）**：1 次 build 能完成「编码器 + artifact bpv 验证」，但 **decoder replay
  六项 true 在 1 次 build 内风险极高**（需 1000+ LOC 一次写对，且要与 encoder 镜像对齐）。
  建议 2-build spike：
  - **B1**：encoder + sv=71 artifact（gate：bytes ≤54,491 / bpv ≤21.96）；
  - **B2**：decoder replay（gate：decode 六项 true + coord ≤3.78e-5 + 默认守卫不变）。
  若 Lead 坚持 1 build 全 PASS，我按 1 build 做但请预期风险 ~20–30%。
- 主要风险：canonical 判据与 decoder 镜像的**状态演化一致性**（merge/fan/proxy 归一化顺序）、
  null-patch 符号在真实 mesh 上的频率（当前 removability 链曾测 null_patch=0，但 frontier
  canonical 版本可能新增）、Frenet inverse 的 lifting 对称性（task-19 已有可逆 lifting 原型）。

## 5. 证据来源（全部已有实测，无新 build）
- `/tmp/wsb38/p1p2_sv7/`：48,680 B / 19.618155（selection sv70；valence 4,188 / residual 41,898 / container 2,174）。
- `/tmp/wsb38/gateq_nopgs/`：49,060 B / 19.771296（BFS sv70）。
- `/tmp/wsb38/default/`：64,604 B / 26.035565 / md5 守卫。
- `/tmp/wsb38/elim_journal.txt`：50 轮 / 19,768 删除 / valence 分布 / 每轮规模。
- 已交付 ledger：`impl/RD/TASK39_P3_LEDGER.md`（24.5 KB 顺序 sideinfo 下界）、
  `impl/RD/TASK40_ROUTE3_SPIKE.md`（base N-gon 不可提供逐轮 identity）。

## 6. 结论与请求
- **B/C/D 不可行**（下界超预算 2×+）；**A 是唯一同时满足 faithful + 可解码 + ≤21.96 的路线**。
- 请 Lead 批准 **2-build spike**（B1 encoder artifact、B2 decoder six-flag）；若只批 1 build，
  我按"B1 encoder + artifact"先跑并在第二 build 前给中间证据。
- 默认守卫（rd_tag 64,604/26.035565/md5、PCATM/PCLTTM/matrix/v1）不受影响；未碰 live/M4/M5。
