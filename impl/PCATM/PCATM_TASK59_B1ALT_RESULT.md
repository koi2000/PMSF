# PCATM task-59 B1-alt — paper-faithful displacement recode（B1 结果 + mode2 缺口 ledger）

日期：2026-10-23；owner rd-canonical；独立 copy `/tmp/task59/src`（task49 worktree 派生），**未触碰 live/M4/M5**；1 build（`-DPROFILE=OFF` Release，build rc=0）。
交付：本 MD、`PCATM_TASK59_B1ALT_EVIDENCE.csv`、`PCATM_TASK59_B1ALT.patch`（4 文件）、run dir `/tmp/task59/run/b1alt`。

## 0. 结论（B1-alt 数据，不标 PASS）

| 项 | 值 |
|---|---|
| artifact | **68,027 B = 27.415042 bpv**（B3 字节 gate ≤68,185/≤27.48 **PASS**，margin 158 B） |
| 分量 | blob 66,323 = color **22,272（未动）** + residual **43,711**（原 47,298，**−3,587**）+ container 340；framing 1,704 |
| scheme | 新 **sv=13**：位移值不变、重建语义不变，仅把位移流换成 per-axis 自适应 bit-plane 熵编码（paper §4 本来就对 displacement 熵编码；非 conquest/alias/降 q/降 codebook） |
| decode | 7/7 **PASS**：decoded sha `c3382783…46a91` **== 冻结 canonical strict decode（逐字节相同 mesh）**；MeshCompare --tol 0.001：counts/connectivity/topology/coordinate_remap/nearest 全 true（remapped_same_unordered_faces=39698, coord_max=3.78e-05, nn_rmse=2.24e-05） |
| guards（同 build） | 默认 strict **71,550 B md5 `ac222c20…/52b17d0c…` 逐字节不变**；PURE_K3 **69,187 B md5 `e91bb4bc…/5d8a5d98…` 不变**；新 binary decode 冻结旧 sv12 artifact **`c3382783…`**（旧解码路径不变） |
| **mode2Rounds** | **0 / mode4Rounds=40** ← **未满足 B3 gate（>0）；故不标 PASS，PCATM 矩阵保持 FAIL 28.834819 / 1.3644x** |

## 1. 实现（patch 摘要）

1. 新增 `impl/common/strict/PCATMStrictAdaptiveResidual.h`：per-axis 自适应二进制 bit-plane 编码（zigzag → 4-bit length tree + mantissa，qsmodel scaleBits=12/rescale=64），编码/解码对称；自描述 wire format（varint numRounds/counts/payload）。值集合与顺序完全沿用原 residual 队列，**解码位姿逐字节不变**（证明：decoded sha == canonical）。
2. `impl/common/PaperBlobAlgorithmSpecs.h`：新增 `kPCATMStrictPaperAdaptiveResidualVersion = 13` + 相应谓词（IsStrictPaper/IsColorOnly/UsesGlobalColor/UsesAdaptiveResidual）。sv1..12 语义冻结。
3. collector export：`PMSF_PCATM_STRICT_ADAPT_RESIDUAL=1` 时用 sv13 + 新 residual 编码；color 流保持原 global 4-colour 逻辑（bytes 不变）。
4. reader import：sv13 → adaptive residual 解码；color/merge/insert 路径零改动。

## 2. mode2 缺口（诚实 ledger）

- 任务要求的 B3 gate 含 `mode2Rounds>0`（paper Z / 2-color 模式实际启用）。本 build **未达成**：0 轮。
- 实测不可直接用：
  - plain `PMSF_PCATM_STRICT_ENABLE_MODE2=1` → mode2Rounds=4 但 **decode 出完全不同 mesh**（dec.off 1,639,927 B ≠ canonical；MeshCompare remap 0）→ 不可接受（复现 task47/48 的 matcher 问题）。
  - `ENABLE_MODE2 + STRICT_2COLOR_SIDE=1` → mode2Rounds=**0** 且 artifact 74,951（+6.9KB）→ 不可用。
- 已有 encoder 侧 `decoderSelectionMatchesEncoderComponents()`（镜像 decoder 的 ones BFS + `paperMode2Zeros` + comp 校验）在 plain mode2 路径已启用且返回 true，但实际 decode 仍偏 → 说明 encoder 校验与 decoder 实际恢复之间存在未定位差异（疑似 proxy normalize/BFS ones 顺序或 decoder bboxMin_ 差异）。**根因定位 + 安全化需要单独 build 预算**；本次 B2 未消耗第二 build（避免半成品伪造）。
- 若 Lead 判定可接受：B1-alt 已满足「字节 ≤68,185 / ≤27.48 + decode 7/7 + guards」，可作为 **paper-faithful 熵编码改进** 写进矩阵（PCATM bpv 28.8348→27.4150, ratio 1.3644x→1.2970x），mode2 验证项另立任务；否则 PCATM 保持 FAIL + 出 ledger。

## 3. 复现

```bash
# copy: rsync task49 worktree → /tmp/task59/src (datasets/reference symlink)
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DPROFILE=OFF \
  -DCMAKE_DISABLE_FIND_PACKAGE_Arrow=ON -DCMAKE_DISABLE_FIND_PACKAGE_Parquet=ON \
  -DPython3_EXECUTABLE=<venv>/bin/python
cmake --build build -j32 --target PMSFCompress PMSFDecompress MeshCompare
# c.json = /tmp/task49_runs/guard_strict/c.json + extra: PMSF_PCATM_STRICT_ADAPT_RESIDUAL=1
build/PMSFCompress /tmp/task59/run/b1alt_c.json
build/PMSFDecompress /tmp/task59/run/b1alt_d.json
build/MeshCompare datasets/PaperModels/horse19k.off run/b1alt/dec.off --tol 0.001
```
