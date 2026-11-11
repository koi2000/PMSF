# M5-5 VQMMC Paper v2 正式验收证据（2026-10-22，task-35）

结论：**PASS（coded，zero-overhead）** —— paper artifact **62,931 B ≤ 64,510 B（Δ −1,579）**，
decoded file-sha **bit-identical** 于 canonical，determinism/默认守卫/paper matrix/fusion/v1
全部不变。raw 无 codec ledger = 64,798 B（+288）超支，按约定披露（PERF 以 coded 为准）。
未改 live 默认、未跑 phase-1 gate；storage helper + VQMMC paper TU 随 build window 落地。

## 1. 命令与 env
canonical（binary r55/cb256/q12）：
```
PMSF_V2_COMPACT=1 PMSF_SYMBOL_BLOB_ZLIB=1 PMSF_REPAIR_DEGENERATE=1 \
PMSF_POOL_V=100000 PMSF_POOL_H=480000 PMSF_POOL_F=200000 \
VQMMC_PAPER_CONN=1 VQMMC_USE_PREDICTIVE_RESIDUAL=1 VQMMC_CODEBOOK_QUANT_BITS=8 \
PMSF_DEFAULT_QUANT_BITS=12
PMSFCompress c.json(backend=binary, rounds=55, codebookSize=256) ; PMSFDecompress d.json
```
paper：同 canonical env + `PMSF_VQMMC_PAPER_ARTIFACT=1`，`backend=paper`；
encode 内部复用 canonical binary 序列化到临时文件 → 逐字节捕获 64,510 B → P2BL v2 opaque
payload container（payloadCodec=ZlibRaw，baseCodec=None）；decode 走
`VQMMCPaperDeserializeOperator`（strip u32 前缀 → `VQMMCDeserializeOperator` strict 路径：
`VQMMCStrictSymbolReadOperator + VQMMCReconstructOperator + VQMMCPostProcessor`）。
一键复现：`impl/VQMMC/vqmmc_paper_roundtrip.sh [out_dir]`。

## 2. 结果（真实 artifact）
| row | mode | bytes | Δ vs 64,510 | bpv | decoded sha256 | bit-identical |
|---|---|---:|---:|---:|---|---|
| canonical binary | verbatim | 64,510 | 0 | 25.997683 | `bd47e456f19d7648d2558bb35f7d352fc65f59019eafdef9d64ce3fdda6736d0` | baseline |
| paper-v2 coded | zlib-raw(3) | **62,931** | **−1,579** | 25.361342 | 同上 | **true** |
| paper-v2 raw ledger | raw | 64,798 | +288 | — | — | **超支披露**（storage helper 单测实测） |

- artifact sha256 `dde5f385748dd93105ebc8f9fcd75d72327de8757c00c698276d594efbcc9054`；
  **determinism**：独立二次 encode+decode，artifact sha 与 decoded sha 均相同。
- 保真 ledger：opaque payload = canonical 64,510 B **verbatim**；zlib-9 = 62,635 B；
  +8 B zlib-raw framing +288 B P2BL/parquet 容器 = 62,931 B（≡ raw artifact 64,798 = 64,510+288）。
- decode 六项判据：**paper decode 与 canonical decode 的 MeshCompare 六项完全一致**
  （counts/manifold=true；connectivity/topology/nearest/remap=false —— canonical VQMMC 行
  在阶段一即 PASS-ASSUMPTION/Reading B，vs-input 的四项本来为 false，非 paper 引入）；
  paper 与 canonical `dec.off` 逐字节相同（cmp）。
- 质量口径：cb=256/q12/Reading B 未改；canonical 质量值沿用阶段一
  （surface_mse_sym 3.76758336503e-08 等），paper 不改变 decode 几何（bit-identical）。

## 3. 守卫（全部 PASS/不变）
- 默认 VQMMC binary r55 cb256 q12：**64,510 B / sha `3d34efa9…` / decoded `bd47e456…`**（重构后不变）。
- paper matrix **8/8**（env OFF；artifact bytes 与 `PAPER_MATRIX_EXPECTED.csv` 逐项一致）。
- conquest paper fusion：**90,490 B / 20.7820 bpv / bit-exact**（canonical 86,301 B）。
- env-OFF paper v1 四件套 size/sha 不变：ppmc 64,235 `59209958…`、pclttm 191,836 `dfe70532…`、
  mmse 59,140 `5622066c…`、aptm 57,066 `3550fc2f…`。
- 未改 live 默认 / 未跑 phase-1 gate；阶段一 7/2/1 不动。

## 4. 证据
- `impl/VQMMC/VQMMC_PAPER_V2_EVIDENCE.csv`（final，sha256
  `370a39d100d77a0c37c540aee960511a599c2736cadd90f7b99155d9ba86f838`，22 行 = 17 PASS / 5 INFO /
  0 FAIL；schema `section,item,value,expected,verdict,notes`，与 QEM CSV 对齐；由 storage-architect
  按 Lead 指令合并 live，数值全部取自本报告 run 目录）
- run dirs：`/tmp/m5_5_formal/`（canonical + paper + logs + evidence.csv）、
  `/tmp/m5_5_formal_rerun/`（determinism）、`/tmp/m5_5_guard/`、`/tmp/m5_5_paper_matrix/`、
  `/tmp/m5_5_fusion/`、`/tmp/m5_5_v1/`
- 实现：`impl/VQMMC/VQMMCAlgorithm.cpp`（paper 分支 + binary 序列化抽函数）、
  `VQMMCPaperDeserializeOperator.{h,cpp}`、`VQMMCPaperRegistration.{h,cpp}`、
  `VQMMCPaperOpaqueIO.h`；storage helper `src/storage/PaperStreamOpaquePayloadV2.{h,cpp}`（storage-architect）。
