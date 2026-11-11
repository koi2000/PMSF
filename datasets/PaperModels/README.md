# 论文原始模型集（用于 bpv 门禁的同模型对照）

基准网格集（`bunny` / `LetterA`）是框架自身的历史 bench 网格，但 9 篇论文里
只有 RD 报告过 bunny（34,834 v，逐点吻合）。其余算法的 `bpv_original` 来自论文
自己的模型集，与基准网格不是同一个模型 —— 这类行的比值只能作指示性指标。

本目录收集**能与论文数值逐点对齐**的模型，用于补充严格同模型的对照行。

## 已收集

| 文件 | V | F | 对齐的论文数值 |
|---|---:|---:|---|
| `fandisk.off` | 6475 | 12946 | APTM 论文 Table 1 fandisk 6475 v ✅ 逐点吻合；PCLTTM 论文 Table 1 fandisk 6475 v ✅ 逐点吻合 |
| `venus.off` | 8268 | 16532 | **RD 论文 Table 2 Venus 8,268 v ✅ 逐点吻合**；注意 APTM Table 1 的 venus body/head 是 **11,217 / 13,407 v**（不同版本，待收集）；VQMMC 用的是 67,170 面的 venus **head**（不是本文件） |

来源：`datasets/PaperModels/venus.off` 来自 `https://github.com/abbaselmas/3D-Model-Dataset`（`Venus/venus.off`，经 jsDelivr 抓取：`https://cdn.jsdelivr.net/gh/abbaselmas/3D-Model-Dataset@master/Venus/venus.off`），sha256 `9b75e069dd15399fd67e9fe4a30740d65f5b3f7fabd9cb60785e3334779963e9`；

来源：`https://github.com/alecjacobson/common-3d-test-models`（`data/fandisk.obj`），
OBJ → OFF 转换见 `scripts/tests/fetch_paper_models.sh`。
许可证见该仓库。

## 待收集（下一步）

| 模型 | 论文中的 V | 出现在 |
|---|---:|---|
| horse | 19,851 | APTM / MMSE / PCATM / PCLTTM / RD（五篇共同模型） |
| venus | 11,217 / 8,268 | APTM / PCLTTM / RD（**8,268 已收集 ✅**；11,217 版本待收集） |
| tiger | 2,738 | MMSE / RD |
| triceratops | 2,832 | PCATM / RD |
| torus | 36,450 | APTM / MMSE / PCLTTM |
| nefertiti | 3,407 / 4,492 | PCLTTM / RD |
| rabbit | 67,039 | APTM / RD |

注意：`common-3d-test-models` 里的 `horse.obj` 是 48,485 v、`nefertiti.obj` 是
49,971 v，**都不是论文用的版本**（论文的 horse 是 19,851 v 的简化版）。
需要找 INRIA/MPEG-4 时代的原始测试集镜像。

---

## horse 19,851 v 收集尝试记录（2026-09-17，**未成功**）

按 MASTER_PLAN §4 Phase 3 的要求执行收集，**本轮失败**，逐条记录已尝试的手段，
供后续接手直接排除这些路径。

### 已尝试

| # | 途径 | 结果 |
|---|---|---|
| 1 | 跑 `scripts/tests/fetch_paper_models.sh` | ❌ 该脚本从 `raw.githubusercontent.com` 拉取；本机**无外网**（curl exit 7，连接失败） |
| 2 | 全盘搜索本地已有的 horse 模型 | ❌ 只找到**非论文版**，见下表 |
| 3 | 检查仓库内是否已 vendored | ❌ 无 |

本地找到的三个 horse **全部是 48,484–48,485 v**，即论文**未**使用的那个版本：

| 路径 | V | F |
|---|---:|---:|
| `datasets/Stanford/horse.off` | 48,484 | 96,964 |
| `datasets/Stanford/single/horse.off` | 48,484 | 96,964 |
| `/tmp/models/horse.obj`（工作区缓存，非仓库内容） | 48,485 | 96,966 |

这与 §5.1 的提示一致：`common-3d-test-models` 的 horse 就是 48,485 v，
**不是**论文的 19,851 v 简化版。

### 结论与影响

- **mmse / pcatm / vqmmc 维持监控行（不门禁）** —— 它们没有 exact 对照，
  按 R4 §5.1「收集成功前全部行仅监控」执行。已反映在
  `bpv_budget_baseline.csv` 的 `gate` 列（三者的行均为 `monitor`）。
- 这三行同时记录着**量化位宽不符**（mmse 论文 12 / 实际 10，
  pcatm 论文 12 / 实际 11，见 `BPV_BUDGET_REPORT.md` 的"论文q / 产物q"两列）——
  即便将来收集到 horse，建 exact 行前也**必须先对齐量化位宽**，
  否则新行一出生就是不可比的。

### 下一步（需要外网或人工取件）

目标是 **INRIA / MPEG-4 时代的原始测试集**（Gotsman 组的 horse 简化版，
19,851 v）。可行的取件途径：

1. 论文作者主页的 supplementary material（APTM / MMSE 两篇的年代主页）；
2. `3D Mesh Compression` 综述类仓库里常附的测试模型包；
3. 学术镜像站（如 `3d-model-repo` 类）；
4. 直接向论文作者索取。

**口径提醒**：取到后必须**先核对 V=19,851** 再入库，并在上表登记来源与许可证。
若拿到的是别的简化级别（例如 5k 或 40k），不能拿来当 exact 对照 ——
那会把 closest 行伪装成 exact 行，方向正是"静默放宽"。

## 2026-11-10：QEM 原论文 foot 模型已取得

`foot_qem_4204.off` 来自 Michael Garland 原作者网站的
`https://www.mgarland.org/dist/models/bones.smf.gz`，2,154 顶点 / 4,204 面，
用于 QEM Fig.7/Fig.14 的 250-face 工作点核对。源SMF以 `v`/`t` 记录点/三角形；
转换只把1-based索引变为0-based，保留十进制坐标文本及点/面顺序。

- 压缩包 SHA256：`f8809daeba2274df8cb2be1d18b73ed706e75f5cd15ff85aae98aed966503071`。
- OFF SHA256：`ee5375d566866b20e97ee303559487788e72a784bcc5df6b1730af209e3e55e7`。
- 完整来源/转换记录：`foot_qem_4204.provenance.json`。
- 源文件与转换脚本：`docs/memory/FULL_REPRO_QEM_2026-11-10/round_001/`。

这不改变旧 bunny E250 作为参考回归的身份；原Fig.7的模型尺度、曲线与
采样协议仍需严格核对。没有因模型到位就把失真自动改为 PASS。
