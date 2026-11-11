# M5-4 PCLTTM PaperStream v2 审计（只读，2026-10-22）

Owner: algo-bpv（task-34）。未改 live 代码、未 build；用现有 `build/` 二进制在 `/tmp/audit_m5_4/` 复跑。
结论：**配方已打通**，fused ≤ canonical 且 decoded bit-identical；probe 78,715 B 的根因是
paper encode 缺 `PMSF_PAPER_RETRI=1`。

## 1. Canonical PCLTTM（r44/q12，可复现 66,755 B）
命令：`impl/PCLTTM/pclttm_best_bpv_roundtrip.sh`（backend=columnar，rounds=44，q=12）。
必需 env：
`PMSF_V2_COMPACT=1 PMSF_DEFAULT_QUANT_BITS=12 PMSF_PCLTTM_QUANT_BITS=12
PMSF_REPAIR_DEGENERATE=1 PMSF_POOL_V=100000 PMSF_POOL_H=480000 PMSF_POOL_F=200000
PMSF_BASE_EDGEBREAKER_NATIVE=1 PMSF_PARQUET_ZSTD_LEVEL=22`

| 产物 | bytes | sha256 |
|---|---:|---|
| `art_compact.bin` | 832 | `171f448e7582238f8568902d18449b477dcf3481b7cb0a43249c4c73c7e55956` |
| `art_residual.parquet` | 65,923 | `70c18d46c94fdfff057f20f83a6c2638d8644b18654f5ff5af9cc5a69612cf79` |
| payload = `residual_data` 列（CONN blob） | 65,277 | `a8d502839a970ef3230e3a61e1b7926e…` |
| 合计 | **66,755** | |

布局 = CODEC_016（layout 16）；没有 rc_blob/symbols parquet 产物，权威 payload 是
`art_residual.parquet` 的 `residual_data` BLOB（magic `CONN\x10\x00\x03\x00`）。

## 2. Paper v2 encode（M5-2 blob 容器配方）
在 canonical env 基础上追加：
`PMSF_PAPERSTREAM_V2=1 PMSF_PAPERSTREAM_V2_PAYLOAD=native
PMSF_PAPERSTREAM_V2_PAYLOAD_CODEC=sbc1|zstd PMSF_PAPERSTREAM_V2_CONTAINER=blob
PMSF_PAPERSTREAM_V2_BASE_SOURCE=canonical PMSF_PAPER_RETRI=1`

- **`PMSF_PAPER_RETRI=1` 是关键**：缺省时 paper 走 generic bridge（legacy layout=1、base 957 B、
  payload 73,049 B）→ fused 74,304–78,715 B，decoded sha 不同（= M5-0 probe 现象）。
  加上后走 CODEC_016 retri override（layout=16、base 832 B、canonical CONN payload）。
- decode 必须 `PMSF_PAPERSTREAM_V2=1` **且** `PMSF_PAPER_RETRI=1`；漏 RETRI 实测 SIGSEGV。
  decoder 的 input 写 base 路径（`.../art`，reader 自动加 `_paper.parquet`）。

## 3. 保真与体积（P2BL v2 blob 容器解析结果）
| 项 | 值 |
|---|---|
| fused `art_paper.parquet`（`_PAYLOAD_CODEC=zstd`） | **65,801 B**（Δ −954 vs 66,755） |
| fused（`sbc1`=zlib） | 65,882 B（Δ −873） |
| base（partition baseMesh） | 832 B，**逐字节 == canonical `art_compact.bin`** ✓ |
| native payload | SBC1 无损封装：zstd rawLen 65,277 / compLen 64,657；zlib rawLen 65,277 / compLen 64,738 |
| unwrap 后 payload | **逐字节 == canonical `residual_data`**（sha `a8d50283…`）✓（非重编码/非 strip） |
| decoded `dec.off` sha256 | **`089911398d2b6b63fe308ae9ca387d60c6d5d4e949957162ce850341d6c445a3`** =
  canonical decoded sha（bit-identical，两种 codec 均验证），decode rc=0 |

即：**fused 65,801 B（zstd，推荐）/ 65,882 B（sbc1）≤ 66,755 → PASS margin 954/873 B；
decoded file-sha bit-identical。**

## 4. 证据/复现
`/tmp/audit_m5_4/`：
- `canonical.json` / `canonical_d.json`：canonical encode/decode；`canonical/`（66,755 + dec.off sha 08991139…）
- `paper2.json` / `paper2_d.json`：sbc1 变体；`paper2/art_paper.parquet` 65,882 B + dec.off sha 相等
- `paper3.json` / `paper3_d.json`：zstd 变体；`paper3/art_paper.parquet` 65,801 B + dec.off sha 相等
- `paper*_c.log` / `paper*_d.log`：含 `[RetriOverride] preset=Default proto=CODEC_016(16)`、
  `layout=16`、`compact bin ... bytes=832`、payload/codec 证据。

## 5. 正式验收待办（实现落地后）
- storage-architect 按 M5-2 blob 容器落 M5-4 存储半侧（无需改容器格式）；algo-bpv 做 PCLTTM
  接线 + 证据 CSV（fused bytes/Δ/decoded sha/bit-identical/六项/命令/run dir）。
- 默认守卫：PCLTTM default canonical **68,160 B / 27.468641 / md5 不变**；env OFF paper matrix 8/8、
  v1 四 sha256 不变；阶段一 7/2/1 不动。
