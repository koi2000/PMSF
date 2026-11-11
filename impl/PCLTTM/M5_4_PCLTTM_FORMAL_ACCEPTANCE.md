# M5-4 PCLTTM PaperStream v2 正式验收证据（2026-10-22，task-34）

结论：**PASS（zero-overhead）** —— fused 65,801 B（zstd）/ 65,882 B（sbc1）≤ canonical
66,755 B；decoded file-sha bit-identical；base/payload 逐字节 canonical；默认/矩阵/fusion/v1
守卫全部不变。未改 live 默认、未 build（纯 env/wiring 行）。

## 1. 命令与 env
canonical（r44/q12）：`impl/PCLTTM/pclttm_best_bpv_roundtrip.sh`（EB+Zstd22 口径）。
paper-v2 encode（canonical env 之上追加）：
```
PMSF_PAPERSTREAM_V2=1 PMSF_PAPERSTREAM_V2_PAYLOAD=native \
PMSF_PAPERSTREAM_V2_PAYLOAD_CODEC=zstd|sbc1 PMSF_PAPERSTREAM_V2_CONTAINER=blob \
PMSF_PAPERSTREAM_V2_BASE_SOURCE=canonical PMSF_PAPER_RETRI=1
```
paper-v2 decode：`PMSF_PAPERSTREAM_V2=1 PMSF_PAPER_RETRI=1`（**漏 RETRI 会 SIGSEGV**）。
一键复现：`impl/PCLTTM/m5_4_paper_roundtrip.sh [out_dir] [zstd|sbc1]`。

## 2. 结果（真实 artifact）
| row | codec | bytes | Δ vs 66,755 | bpv(fused) | decoded sha256 | bit-identical |
|---|---|---:|---:|---:|---|---|
| canonical | n/a | 66,755 | 0 | 26.902423 | `089911398d2b6b63fe308ae9ca387d60c6d5d4e949957162ce850341d6c445a3` | baseline |
| paper-v2 | zstd | **65,801** | **−954** | 26.517959 | 同上 | **true** |
| paper-v2 | sbc1(zlib) | **65,882** | **−873** | 26.550602 | 同上 | **true** |

- fused 单文件 `art_paper.parquet`；decode rc=0；MeshCompare 六项（counts/connectivity/
  topology/nearest/manifold/remap/test_valid）全 true。
- **base** 832 B == canonical `art_compact.bin`（sha `171f448e…`）逐字节相同。
- **payload** = canonical CONN payload（65,277 B，sha `a8d50283…`）的 SBC1 无损封装：
  zstd rawLen 65,277/compLen 64,657；sbc1(zlib) rawLen 65,277/compLen 64,738；
  sbc1 unwrap 后逐字节 == canonical（zstd 由 decoded sha 等价证明）。
- **determinism**：两次 paper(zstd) encode 的 `art_paper.parquet` sha256 相同
  `b38227e4c78ba60ae79c38c4616dbe48c7c634d10dc188b9038ff1446bd667bb`。

## 3. 守卫（全部 PASS/不变）
- 默认 PCLTTM r50 canonical：**68,160 B / 27.468641**；`art_compact.bin` 1,051 B sha
  `3be9057e…`、`art_residual.parquet` 67,109 B sha `801159c1…`（与冻结一致）；decode 六项 true。
- paper matrix：**8/8 PASS**，各 artifact bytes 与 `PAPER_MATRIX_EXPECTED.csv` 逐项一致
  （335396/335398/98656/307453/340564/375371/338189/324673）。
- conquest paper fusion：**90,490 B / 20.7820 bpv，decode bit-exact**（canonical 86,301 B）。
- v1 bridge 四件套（env OFF）大小与 probe 冻结值一致，sha256 首次记录：
  ppmc 64,235 `59209958…`；pclttm 191,836 `dfe70532…`；mmse 59,140 `5622066c…`；aptm 57,066
  `3550fc2f…`。
- 未改 live 默认/未 build；阶段一 7/2/1 不动。

## 4. 证据
- `impl/PCLTTM/PCLTTM_PAPER_V2_EVIDENCE.csv`（canonical/paper zstd/sbc1/guards/v1 共 11 行）
- run dirs：`/tmp/m5_4_formal/`（zstd）、`/tmp/m5_4_formal_sbc1/`、`/tmp/m5_4_guard/`、
  `/tmp/m5_4_paper_matrix/`、`/tmp/m5_4_fusion/`、`/tmp/m5_4_v1/`；配置/日志同名目录内。
