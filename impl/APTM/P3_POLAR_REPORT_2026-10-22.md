# APTM polar θ/φ 几何 coder + Arch-2 报告（P3-P2，2026-10-22）

负责人：WS-B3（aptm-repro）/ task-14（多轮）
前置：`impl/APTM/P3_CONQUEST_PLAN_2026-10-22.md`（P3-P1：A&D connectivity + anchors，
G 16.516，22.907 bpv，strict 六项 true）、`impl/APTM/ARCH2_SPIKE_PLAN_2026-10-22.md`、
`impl/APTM/P2PRE_SPIKE_2026-10-22.md`（tag 路径根因）。
证据脚本：`scripts/tests/test_aptm_polar_final.sh`（+ `test_aptm_p3_polar.sh` 中间证据）
证据 CSV：`impl/APTM/APTM_POLAR_FINAL_EVIDENCE.csv`
原始产物：`/tmp/wsb3/aptm_polar_final/`（gate/阶梯/对齐/日志）

---

## 0. 结论（先读）

**Lead 裁决 A 的 final 行候选**（同一 polar coder 的精度参数 levels=48 + Arch-2）：

| 项 | 值 |
|---|---|
| **bpv（rounds=16，P13.4 工作点）** | **21.309556**（artifact 52,877 B；ratio **1.1972×** ≤ 1.20 理想） |
| **bpv（rounds=150，矩阵口径）** | **21.311571**（artifact 52,882 B；ratio **1.1973×**） |
| 分量（r16） | C 11,512 B = **4.639 bpv**（≤ 论文 4.9）；G 37,059 B = **14.93 bpv**（论文 12.9 → 1.16×）；container 1,580 B = 0.637 bpv |
| decode | strict MeshCompare **六项 true**：counts / connectivity / topology / nearest_geometry / coordinate_remap_complete / manifold_profile(+test_valid) |
| A1 | 19,851 / 39,698 逐字；anchors=139；null_patch=42,125 |
| surface | `SurfaceMetrics --samples 5000`：RMS max **2.319e-05**、MSE symmetric **5.272e-10**、normalized RMS **9.17e-05**（= 0.0092%）；default APTM 为 9.23e-06 / 8.43e-11 / 3.65e-05（MSE 6.2× baseline） |
| 拓扑 | decoded 面列表与 native P3 decode **逐字节相同**（顶点序 = base 容器序 + 逐层插入序）；索引对齐顶点位移 rms 5.382e-05 / max 8.504e-04（r16）、5.462e-05 / 8.445e-04（r150），0 个顶点 >1e-3 |
| 默认 APTM | **84,496 B / 34.052088 bpv / 4 md5 与冻结审计逐字节一致** |

**诚实标注（必须进矩阵/报告 notes）：**
- 论文 §4.2 的 **θ/φ 各 3-bit 设定 = levels=8**：**16.4880 bpv / 0.9263×**，
  但几何有损（max 顶点误差 6.7e-3）导致 MeshCompare 的 `coordinate_remap` 判据不过
  （strict 六项 false）；其**拓扑与 native decode 逐字节相同**（见 §3）。
- **levels=48 是同一 polar coder 的精度参数，不是论文 3-bit 设定**；选它是因为在
  strict 六项 + 合理 surface MSE 的验收口径下，只有 levels≥48 才能让全网格坐标
  remap 通过（max 误差 8.5e-4 < 1e-3）。不得把它写成“论文 3-bit 的忠实复现”。
- P3-P1 的 **APTM-Conquest P3 = 22.907 bpv / 1.2869× 保留为 labeled variant**
  （A&D connectivity + anchors + conquest 通用几何残差，不是 polar coder）。
- 矩阵 APTM 的 PASS 行由 Lead/矩阵 owner 按本报告裁决；本轮不自行改矩阵数字。

---

## 1. 实现（只走既有 hook 接缝，未改 `src/conquest/*` / `src/scheme/*`）

### 1.1 几何接缝 = conquest 引擎 H4 `ResidualQuantizer`
- 编码：`FrontierConquestEngine::recordGeometry` 算 `delta = toCell(src) - toCell(pred)` 后
  调 `transform(delta, out, PatchView)`；
- 解码：`decodeLayer` 用**同一个 PatchView**（同一 `ringNeighbors` 环序）调
  `inverseTransform(stored, d, pv)`，再 `cell = toCell(pred) + d`。
⇒ polar 只需在 `impl/APTM/hooks/APTMConquestGeometryHook.h` 实现两个对称函数。

### 1.2 polar θ/φ coder（论文 §4.2）
1. 由 PatchView 环邻居位置构造局部 Frenet 系：n = 环的 Newell 法向；
   t1 = 最长环边投影到切平面；t2 = n×t1（双端同一公式/环序）。
2. `(c_n, c_t, c_b) = (δ·n, δ·t1, δ·t2)`；
3. `R=|δ|`，`θ=acos(c_n/R)`，`φ=atan2(c_b,c_t)`；**θ/φ 各量化到 levels 级**
   （论文 3-bit ⇒ levels=8；默认 8），R 保全精度；存 `(R, θq, φq)` 进 3 分量流。
4. 解码端 inverse：用同一 frame 展开回 Cartesian。

### 1.3 Arch-2（S2）：encoder-side decoder 精确模拟（单趟）
动机（P2PRE_SPIKE）：有损几何 ⇒ decoder 重建立方位置 ≠ encoder 原位置 ⇒
预测/frame 不同步 ⇒ 误差跨层漂移（无 Arch-2 的 polar_l8 顶点 max 误差 2.2e-2）。

正确做法（**单趟 LIFO 重放，无需迭代**）：decoder 对每个插点 k 的动作可完全复现：
`ring = 环邻居（解码序中都已先处理）`、`pred = mean(ring 位置)`、
`d = inverseTransform(stored, PatchView(ring 位置))`、`pos = bm + (toCell(pred)+d+0.5)*step`。
编码端从 base mesh 出发按解码顺序重放拓扑（decodeLayer 只用来建环/给池号），维护
`posMap: 池号 → decoder 重建立方坐标`，逐点重算
`raw = target_cell - toCell(pred_from_posMap)`、`stored = transform(raw)`、
`pos = 上式`。因每步 pred/ring/frame 与 decoder 逐位相同，**单趟即固定点**；
第 2 趟 `changed=0` 验证（`converged=1`）。
- 诊断：`[APTM-Conquest arch2] iter=1 changed=18109 geomErrRmsCells=4.33 geomErrMaxCells=95`；
  `iter=2 changed=0 converged=1`（l8/r16）；l48/r16：rms 0.7091 cells、max 18。
- 旧实现（迭代 Jacobi）为何错：用 cur-delta 的 decodeLayer 重建位置算 pred，
  换新 delta 后位置又变 ⇒ 帧不同步、出现漂移尖峰（i2/i3 反而更差），已废弃。

---

## 2. 实测阶梯（horse19k q12，polar+Arch-2，rounds=16）

| levels | bpv | ratio/17.80 | C | G | container | strict 六项 | surface RMS max / MSE / norm RMS |
|---:|---:|---:|---:|---:|---:|---|---|
| 8（论文 3-bit） | 16.4880 | 0.9263 | 4.639 | 10.12 | 0.637 | ✗ remap（有损漂移） | 1.553e-04 / 1.906e-08 / 6.14e-04 |
| 16 | 18.1738 | 1.0210 | 4.639 | 11.80 | 0.637 | ✗ remap | 6.252e-05 / 3.775e-09 / 2.47e-04 |
| 32 | 20.1167 | 1.1301 | 4.639 | 13.74 | 0.637 | ✗（仅 4 顶点>1e-3） | 3.171e-05 / 1.003e-09 / 1.25e-04 |
| **48** | **21.309556** | **1.1972** | 4.639 | 14.935 | 0.637 | **✓ 全 true** | **2.319e-05 / 5.272e-10 / 9.17e-05** |
| 64 | 22.1534 | 1.2444 | 4.639 | 15.77 | 0.637 | ✓ 全 true | 1.808e-05 / 3.268e-10 / 7.15e-05 |

另测（矩阵口径 rounds=150）：**l48 = 21.311571 bpv / 1.1973×，strict 六项全 true，
surface RMS max 2.322e-05 / MSE 5.373e-10 / norm 9.18e-05**；
l64 = 22.153（六项 true）。r150 的 C=13,013（5.24 bpv）、G=39,108（15.76 bpv）
——container 随层数增大但 blob 被 zlib 摊销，artifact 与 r16 基本相同。

**索引对齐拓扑证据**（`scripts/tests/aptm_lossy_topology_check.py` + 最终脚本内联；CSV 内 `*_topology_faces_identical_to_native` / `*_disp_vs_native_*` 行）：
decode 顶点顺序 = base mesh 容器序 + 逐层插入序（与 native 完全一致）⇒
所有 polar/Arch-2 变体的 **decoded 面列表与同 rounds 的 native P3 decode 逐字节相同**；
顶点位移（vs native，索引对齐）：l8 rms 2.617e-04/max 6.685e-03；l16 1.585e-04/2.121e-03；
l32 8.141e-05/1.434e-03；**l48 5.382e-05/8.504e-04（0 个 >1e-3）**；l64 4.221e-05/7.091e-04；
l48 r150 5.462e-05/8.445e-04。⇒ “拓扑没坏”是**逐字节**证据，不是容差判断。

---

## 3. 归因 / 与论文的差距

| 项 | 论文 horse | 本实现（l48） | 说明 |
|---|---:|---:|---|
| C | 4.9 | **4.639** | A&D valence + anchor null_patch（P3-P1 已 ≤ 论文） |
| G | 12.9 | 14.93 | polar θ/φ 48-level；论文 θ/φ 3-bit 点 = l8 的 G 10.12（但几何有损） |
| 总 | 17.80 | **21.3096（r16）/ 21.3116（r150）** | 1.1972× / 1.1973×（≤1.20 理想）；l8 点为 16.49 / 0.926× |
| 几何 | 论文 Fig.7 MSE 曲线 | MSE 5.27e-10（0.0092% bbox RMS） | levels 越大越接近量化无损 |

**为什么 l8 不能直接当 final**：3-bit 角度量化误差 ∝ 残差长度，horse 上最坏顶点误差
6.7e-3（0.36 个量化步长级的绝对量，但远超 strict tol），MeshCompare 的
`coordinate_remap` 要求坐标近精确匹配（顶点平均间距 ~1.8e-3），因此 strict 六项必 false；
这是**验收工具口径**问题，不是拓扑错误（拓扑已证逐字节相同）。**不采用“lossy-aware
check 替代六项 true”的方案 B**（Lead 裁决）。

**风险/限制**：
- levels=48 的 choose 是为了“strict 六项 true”的验收口径；若矩阵口径要求论文 3-bit，
  则应报 l8（16.488/0.926×）并注明 strict remap 失败、拓扑逐字节相同。
- 论文的 MSE/rate 曲线无逐点数值可比，§3 的“几何”列是我们自己的口径（Metro/MESH 式
  surface sampling, 5000 samples；`SurfaceMetrics`），与论文 Fig.7 只能定性对齐。
- Arch-2 在编码进程内多跑一遍解码模拟（~2× 压缩时间，16 层 horse ~0.6s），
  池容量需 PMSF_POOL_H≥2M（脚本已设；不是 q/变体改动）。

---

## 4. 复现

```bash
# 门禁 + 阶梯 + SurfaceMetrics + 索引对齐拓扑（全部）
scripts/tests/test_aptm_polar_final.sh /tmp/wsb3/aptm_polar_final
# 说明：若 build/ 正被其他队友 relink（会短暂不可执行），可先快照二进制并用
#   BUILD_DIR=/tmp/aptm_bin scripts/tests/test_aptm_polar_final.sh ...（证据等价）
# 单点（l48 r150）：
PMSF_APTM_USE_CONQUEST_P3=1 PMSF_APTM_CONQUEST_GEOM=polar \
PMSF_APTM_CONQUEST_POLAR_LEVELS=48 PMSF_APTM_CONQUEST_ARCH2_ITERS=2 \
PMSF_POOL_V=400000 PMSF_POOL_H=2000000 PMSF_POOL_F=800000 \
build/PMSFCompress <c.json>   # c.json: algorithm=APTM, rounds=150, q12, prediction=false/adjustment=false
```

## 5. 验收（本轮）
1. 默认 APTM 84,496 / 34.052088 / md5 冻结一致 ✓
2. conquest paper fusion 90,490 bit-exact ✓；PCLTTM 默认 r50 68,160 / 27.468641 ✓
3. final 候选 l48+Arch-2：21.3096（r16）/ 21.3116（r150），ratio 1.1972/1.1973 ≤1.20，
   strict 六项 true，surface MSE 5.27e-10，A1 19,851/39,698，anchors 139 ✓
4. 论文 3-bit 点 l8 数据与拓扑证据保留（诚实标注）✓
5. 未编辑 `src/conquest/*`、`src/scheme/*`、MMSE/PCLTTM/RD 文件 ✓
