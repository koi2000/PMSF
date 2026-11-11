# APTM · Arch-2 spike 方案（位置对称编码，conquest 路径）— 2026-10-22

> 目的：给 Lead 的「Arch-2 最小 spike（P3-P1 后交错）」提供可执行的方案。
> 前置结论见 `P2PRE_SPIKE_2026-10-22.md`：有损几何 + 位置相关预测/帧 ⇒ 漂移；
> tag 路径必须先做 P2-pre（坐标无关拓扑推断）或 Arch-2（编码端解码模拟）。
> **本方案的关键新事实**：P3 的 conquest 路径里 **拓扑重放完全不依赖坐标**
> （`decodeLayer` 只消费符号流），所以 Arch-2 在 conquest 上只需处理**几何**，
> 比 tag 路径简单一个量级。

---

## 0. 关键架构事实（决定 spike 形态）

1. **conquest 的拓扑 = 符号流驱动**：`decodeLayer` 按 `symbols`（valence-3 / null）
   做 retriangulation + 插点；位置只影响几何，**不影响拓扑结构/流形性/计数**。
   ⇒ 有损几何在 conquest 路径上**不会**像 tag 路径那样把 decode 弄成结构无效
   （tag 的 normalize/proxy 推断依赖坐标；conquest 不需要坐标推断）。
2. **几何 = H3/H4 hook**：
   - H3 `GeometryPredictor::predict(PatchView, out[3])`：默认 = 一环邻居算术平均；
   - H4 `ResidualQuantizer`：默认 = 均匀网格量化 + 恒等变换；
   - 两者都可以在 `impl/` 注入（`EngineHooks`），**编码端 collector 与解码端
     `ConquestReconstructOperator::setHooks()` 必须用同一份实例**。
   - APTM 论文 §4.2 的 Frenet/polar 几何 coder 未来就挂在 H3（Frenet 帧）+
     H4（polar θ/φ 量化）上，不需要动引擎/不需要重写解码器。
3. **闭环预测已经做过一半**：`FrontierConquestEngine::recordGeometry` 里邻居坐标
   取的是"解码端会拿到的那串"（网格已吸附到格中心 + 解码端 float 公式复刻），
   所以**无损整数残差**时两侧 pred 逐位一致（P13.4 A2 已验证）。
   有损残差（polar 3-bit）破坏的只是"解码端位置 = 原位置"这个前提。

---

## 1. spike 目标（1 轮，可判 PASS/FAIL）

**S1（因果 + 边界）**：证明 conquest 路径上"有损几何"不破坏拓扑，只造成几何漂移。
- 实现一个 env 门控的 **lossy H4 hook**（`impl/APTM/hooks/QuantizedResidualHook.h`）：
  在默认量化之后把 cellDelta 的每个分量**舍掉低 k 位/或量化到 ±d 网格**
  （`PMSF_APTM_ARCH2_LOSSY_BITS=k`，k=1/2/3）。编码端 collector 注入该 hook；
  解码端**不注入**（decodeLayer 用默认 H4 恢复到格中心）——即真实"有损码流"。
- 判据：decode 的 counts/connectivity/topology/manifold 应**全 true**（拓扑不依赖坐标）；
  `nearest_geometry` 变差、`coord_match_max` 随 k 增长；记录漂移量 vs k。
- 若 S1 判据成立：conquest 路径不需要 P2-pre，Arch-2 只需修几何质量。

**S2（不动点收敛）**：实现最小 encoder-side decode simulation + 一次 re-encode：
1. Pass 1：正常 encode（lossy H4）→ 得到 symbols + deltas；
2. 模拟 decode：`decodeLayer` LIFO 重放（**在编码进程内**，可直接调
   `pmsf::decodeLayer`），得到每层/每顶点的 decoder 位置；
3. Pass 2：重跑同一消除序列的**几何编码**，预测子输入换成第 2 步的 decoder 位置
   （H3 注入一个"位置覆盖"预测器），得到新 deltas；
4. 再模拟 decode → 比较两次 decoder 位置；**收敛判据**：max |Δ| ≤ 1 格 或
   两遍位置集合稳定（bit-identical）。
- 产出：收敛轮数、每轮 max/rms 漂移、最终 decode 六项 + 几何 RMS。
- 风险：Pass 2 需要"编码端当前网格位置 = decoder 到期位置"的按层映射；
  spike 可先做**单层/两轮网格**（rounds=2, 4）验证方法，再上 16 层。

**S3（hook 公共层验证）**：把 S2 的 H3 覆盖预测器换成 APTM 论文 §4.2 的
Frenet 帧 + polar θ/φ 3-bit（复用 P1 已验证的量化/展开代码），测 G 分量
（预期 13–14 bpv）与 decode 质量；若 S2 收敛成立，则这步直接给出
**P3+P2 合并后的 bpv**（预期 15–18 bpv）。

---

## 2. 实现规模（spike 预算）

| 件 | 内容 | LOC |
|---|---|---:|
| H4 lossy hook | `impl/APTM/hooks/QuantizedResidualHook.h`（继承 ResidualQuantizer，env 门控） | 80–120 |
| collector/reconstructor 注入 | `APTMConquestCollector` + `makeDecompressOptions` 里 setHooks（同一实例语义） | 40–60 |
| 编码端 decode-sim 编排 | 在 `APTMAlgorithm::compress` 外层加"pass2 重编码"？→ 更稳的接缝：`APTMConquestCollector::exportToBufferWithRangeCoder` 里已拿到全部 layers，可在此**内部**做 sim + 二次几何编码（不碰框架 round 循环）；但 prediction 用的是 encoder mesh 状态，需要保留每层 mesh 快照或 replay decimation。spike 先做 replay 版本 | 150–250 |
| Frenet/polar H3/H4（S3，可复用 P1 代码） | 帧构造 + θ/φ 量化 + 展开 | 150–250 |
| 证据脚本/CSV | `test_aptm_arch2_spike.sh` | 80–120 |
| **合计** | | **500–800** |

---

## 3. 判据与止损

- **PASS**：S1 拓扑全 true（证明 conquest 路径鲁棒）；S2 在 ≤3 次迭代内位置收敛
  （max |Δ| ≤ 1 格）+ decode 六项 true + 几何 RMS ≤ 论文量化口径；S3 的 G ≤ 14.5 bpv。
- **FAIL/止损**：若 S2 迭代振荡不收敛（位置 max |Δ| 不下降），记录 ≤3 次迭代轨迹并
  报告；回退方案 = 只用 S1+S3（polar 但保留一次漂移，质量口径登记为有损）；
  若 S1 也失败（拓扑坏）→ conquest 路径的流形假设有问题，需查引擎（报告给 Lead）。
- 全程：默认 APTM 84,496 B / 34.052088 逐字节不变；不用 RETRI/Conquest 别名/降 q。
- 依赖：P3-P1 的 conquest 路径先出 bpv/C/G/decode（本轮）；若 P3-P1 的 decode 未通，
  Arch-2 spike 暂停，先修 conquest 路径。

---

## 4. 与 P2-pre / tag 路径的关系

- **tag 路径**（P1/P2-pre）：拓扑依赖坐标 ⇒ 必须 P2-pre + Arch-2 两件套（~1.2k LOC）。
- **conquest 路径**（P3）：拓扑坐标无关 ⇒ Arch-2 只剩几何（~500–800 LOC），
  且 H3/H4 hook 把"换几何 coder"变成干净接缝。
- ⇒ 建议资源顺序：P3-P1 收口（本轮）→ Arch-2 spike on conquest（1 轮）→
  P3-P2（polar 几何 + null_patch 后处理）→ 若仍要 tag 路径，才补 P2-pre。

---

## 结果（2026-10-22，已实测）

- S1（lossy-H4 因果）：`lossy_b1` 20.281 bpv / strict 六项 true、surface MSE 3.15e-09；
  `lossy_b2` 17.470 bpv / 六项 true。⇒ **有损几何不破坏拓扑**（与符号驱动重放一致）。
- S2（encoder decode 模拟**单趟精确解**）：见报告 §1.3。`changed` 第 1 趟 18,109、
  第 2 趟 0 ⇒ `converged=1`（固定点验证）；几何误差 vs 无 Arch-2 降 ~30×。
- S3（接 polar θ/φ）：ladder 8/16/32/48/64 → 16.488 / 18.174 / 20.117 / 21.310 / 22.153 bpv；
  **l48 = 21.3096 bpv（1.1972×，≤1.20）+ strict 六项 true + surface MSE 5.27e-10**
  （r150 口径 21.3116 / 1.1973×）。
- 诚实标注：论文 3-bit 点 = l8（16.488 / 0.926×，strict remap 因子步有损漂移不过；
  拓扑与 native decode 逐字节相同）；l48 是同一 polar coder 的精度参数。
- 交付：`scripts/tests/test_aptm_polar_final.sh`、`impl/APTM/APTM_POLAR_FINAL_EVIDENCE.csv`、
  `impl/APTM/P3_POLAR_REPORT_2026-10-22.md`。
