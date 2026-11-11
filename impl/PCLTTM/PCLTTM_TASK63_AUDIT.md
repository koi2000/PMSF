# PCLTTM task-63 审计 — CONN stream / 容器 headroom（read-only，N/A，no build）

日期：2026-10-29；owner：aptm-repro；Lead 任务：**先只读审计，headroom <1.5KB → ledger N/A，不 build**。
证据：`impl/PCLTTM/PCLTTM_TASK63_AUDIT.csv`（45 行）+ 可复现脚本 `scripts/tests/pclttm_task63_audit.py`。
工作方式：**未 build、未改 live 代码/默认**；仅读真实 artifact（官方 page-RA / 历史 semantic / canonical）。

---

## 0. 结论

**N/A —— 不 build。** 对**真实 artifact** 重算得到的**可落地 transport/容器 headroom**：

| 基线 | 当前 bytes | bpv | 最好可落地估计 | headroom | CIKM 目标 64,364 需省 | 缺口 |
|---|---:|---:|---:|---:|---:|---:|
| **官方 page-RA fused**（66,752） | 66,752 | 26.901214 | **66,504** | **248 B** | **2,388 B（3.58%）** | 差 2,140 B |
| 历史 semantic fused（65,801） | 65,801 | 26.5173 | **65,593** | **208 B** | **1,437 B（2.18%）** | 差 1,229 B |

两者都 **< 1.5 KB stop 线**，按 Lead 规则 **ledger N/A、不进入 build**。即使把「拆流 + offset delta + framing + stream0 重编码」所有理论项全部加满，上限也 **< 1.2 KB**，仍达不到 semantic 的 1,437 B 门槛，更远低于 page-RA 的 2,388 B。

**根因（实测）**：PCLTTM CONN payload 是**已熵编码**的 range-coded 流（3 个 stream 的 order-0 熵 7.435/7.959/7.992 bits/byte；占 69.7% 的最大 stream2 完全不可压缩，zlib/zstd/brotli/xz 全部 ≥raw）。也就是**想再省 2%，必须改 PCLTTM 算法符号编码本身**（会改 bitstream/需多 build + decode 对齐），不是 transport/容器层能解决的。

---

## 1. 真实 artifact 分量（已逐一解析，非估算）

### 1.1 canonical（`impl/PCLTTM/pclttm_best_bpv_roundtrip.sh`，q12 rounds=44）
| 分量 | bytes |
|---|---:|
| `art_compact.bin`（base compact） | 832 |
| `art_residual.parquet` → `residual_data`（CONN payload） | 65,277 |
| parquet framing（residual parquet 65,923 − payload） | 646 |
| **合计** | **66,755 / 26.902423 bpv** |

CONN payload = **nativeHeader 548**（magic `CONN`、44 rounds × 3 streams、528 B = 44×3 个 u32 end-offset 表）**+ body 64,729**。
body 按 44 轮 × 3 streams 切分（offset 表可完整重构）：
| stream | raw B | order-0 熵 | zlib-9 | zstd-19 | xz-9 | brotli-11 |
|---|---:|---:|---:|---:|---:|---:|
| s0 | 4,070 | 7.4352 | 3,709 | **3,669** | 3,740 | 3,675 |
| s1 | 15,497 | 7.9590 | 15,420 | 15,408 | 15,560 | 15,409 |
| s2 | 45,162 | 7.9919 | 45,169 | 45,176 | 45,224 | 45,166 |

⇒ s1/s2 **已熵编码、无压缩余量**；s0 有 ~400 B 余量（4,070→3,669）但仅此。

### 1.2 历史 semantic fused（`archive_semantic/pclttm_semantic_65801.parquet`，sha `b38227e4…`）
| 分量 | bytes |
|---|---:|
| P2BL header（payloadCodec=2/zstd, algorithm etc.） | 89 |
| base（== canonical compact，sha `171f448e…`） | 832 |
| payload stored = SBC1(zstd frame 64,657 + 13 envelope) | 64,670 |
| parquet framing | 210 |
| **合计** | **65,801 / 26.5173 bpv** |

### 1.3 官方 page-RA fused（`docs/memory/PHASE2_PAGE_RA/pclttm_ra.parquet`，sha `bdd75cb0…`）
| 分量 | bytes |
|---|---:|
| header row：固定 28 + nativeHeader 548 + base 832 | 1,408 |
| 5 chunks（block10，zlib）：comp 1,332+3,799+11,579+29,787+17,965 + 5×8 len 头 | 64,502 |
| parquet framing（6 RGs：header + 5 chunks；UNCOMPRESSED/no dict/no stats） | 842 |
| **合计** | **66,752 / 26.901214 bpv**（≤canonical 66,755，margin 3 B / page-RA 已 promote） |

---

## 2. Headroom 逐项（全部实测）

| 优化项 | 可省 | 计算/证据 |
|---|---:|---|
| page-RA payload：chunk zlib → zstd-19 | **~245 B** | body raw 64,729：zlib 分块总 64,462 → body zstd19 64,207（+5 帧 ≈ 64,25x） |
| page-RA：nativeHeader 548 压缩（zstd 354 / brotli 345） | ~180 B | 现在 header row 里 raw 存 548 |
| page-RA 固定头/offset 表 delta+varint（528 B offsets） | ≤380 B | 只对 header row 有效；semantic 里已在 payload 内被 zstd 压过 |
| semantic payload：zstd(64,657) → brotli-11(64,487) | **170 B** | whole payload raw 65,277：zstd19 64,661 / brotli11 64,487 / xz9 65,272 |
| semantic P2BL header 89 → 最小 ~60 | ~30 B | 头部字段可精简 |
| parquet framing（page-RA 842） | ≤431 B 但**丢 page RA** | 单 RG framing 实测 411 B（pyarrow 同内容模拟）；官方 page-RA 必须保留逐 chunk RG；page-index 替代方案估 ≤300 B |
| stream 拆分（native header+3 streams 分别 zstd） | ~0 | 354+64,607（+帧）≈ whole zstd 64,661；拆流无收益 |
| stream0 重新熵编码（更强 coder） | ≤400 B | 4,070→3,669 已是 zstd；order-0 上限 ~3,780（7.4352 bits）→ 实得有限 |
| **所有理论项加满（不可同时成立版）** | **< 1,200 B** | 仍 < semantic 门槛 1,437 B，远 < page-RA 2,388 B |

**所需**（target 64,364）：
- semantic：container 最小 60+832+4+210 = 1,106 ⇒ payload 需 ≤63,258；最好实测 64,487 ⇒ **还差 1,229 B**（且这已是 brotli-11 极限）。
- page-RA：更差，需 2,388 B。

---

## 3. 为什么不是“换个 entropy coder”就能过 CIKM

- stream2（45,162 B，69.7% payload）order-0 熵 7.9919 bits/byte ⇒ 理论下限仅比 raw 少 ~50 B；实测 zstd/brotli/xz 均 ≥45,166。
- stream1（15,497 B）order-0 7.959 ⇒ zstd 只省 89 B。
- stream0（4,070 B）order-0 7.435 ⇒ 上限 ~3,780，zstd 实得 3,669（省 401 B）。
⇒ payload 层面**已接近信息论下限**；再省 2,388 B（3.58%）在 transport 层不可能。任何“更大”的节省都必须改 **PCLTTM 编码器/预测/符号语义**（新 bitstream、decode 对齐多 build），超出本任务「1 build + decode sha 不变 + 不碰算法」的约束。

## 4. 结构选项的取舍（未 build）

- **优先保 page-RA**（Lead 要求）：page-RA 66,752 的 2,388 B 缺口无法由容器解决；block10 5 chunks 的页粒度、header row、6 RG framing 是官方验收的一部分，不能为省 400-800 B 丢掉 RA。
- 若退回 **semantic 65,801**：可省 951 B，但仍差 1,437 B（semantic 已 zstd，重压只再 ~200 B）⇒ 同样过不了 64,364。
- 因此**不申请 build**；也没有“1 build 可达 CIKM”的可行方案。

## 5. 复现

```bash
# read-only；使用 docs/memory 官方 artifact + /tmp/m5_4_formal canonical
scripts/warehouse/.venv/bin/python scripts/tests/pclttm_task63_audit.py
scripts/warehouse/.venv/bin/python scripts/tests/pclttm_task63_audit.py --json | head -80
# canonical 若不存在，可用 impl/PCLTTM/m5_4_paper_roundtrip.sh 生成（本审计未 build/未改 live）
```

解析口径：page-RA PRA1（`src/storage/PaperStreamPageRAV1.h`：header row = magic/kind/codec/block/rounds/numChunks/nativeHeaderLen/baseLen/topoCount + nativeHeader + base；chunk = [u32 rawLen][u32 compLen][zlib]）；semantic P2BL（`src/storage/PaperStreamBlobContainerV2.h`，payloadCodec=2 ⇒ SBC1-zstd envelope）；CONN 容器（nativeHeader：magic + 44×3 end-offsets，body 按 offsets 还原 3 streams）。

## 6. 诚实记录

- **0 build、0 代码改动、0 live 默认改动**；官方 page-RA row 66,752（26.901214）语义不变；阶段一 9/0/1、phase2 8/8、PCATM 27.415042 均未触碰。
- 所有数字来自真实 artifact 解析/重压缩实测；未套用任何其它算法（PCATM）的预测模型。
- 结论按 Lead 规则：headroom < 1.5 KB ⇒ **N/A ledger，不 build**；PCLTTM 官方 fused 保持 66,752 / 26.901214（PASS 行不变，CIKM 数值线 +2,388 仍未过——如实记录，不伪造）。
