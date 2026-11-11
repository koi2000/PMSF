# task-64 MMSE transport / entropy audit (read-only, 0 build) — verdict: N/A

2026-10-29. Owner: mmse-repro. Live tree and `build/` untouched (only read
artifacts + standalone `/tmp` tool). Baseline artifacts from task-63-adjacent run
`/tmp/mmsep1/m52b/run3` (no rebuild in this task).

## 0. Baseline (unchanged, verified from existing artifacts)

| item | value |
|---|---|
| canonical `mmse_cq` r16 | **54,601 B / 22.004332 bpv**, payload md5 `3ebb7197…`, decode six true |
| official fused paper-v2 | **54,214 B / 21.848370 bpv**, decoded sha256 `3a4d07e7d95530cfdd434828459aadf0eb59113720f12a31a046c20564ec2b33` (bit-identical to canonical) |
| mode hist | bary=300 / α0.5=222 / α1=103 / LS=0 (625 clusters) |
| quality (canonical) | rms_max 9.23407766308e-06 |
| CIKM target | 20.710 bpv → **51,389 B** (gap from fused **+2,825 B**) |

## 1. Real artifact decomposition

Fused artifact 54,214 B = P2BL blob 54,004 B + parquet single-cell framing 210 B.
P2BL 54,004 = canonical compact base **468** + payload **53,447** + P2BL framing **89**.
Payload = SBC1-zstd (`ZSTD_compress` level **22**, already the strongest practical
configuration) of the raw algorithm blob **54,567** = `[u32 trailerLen]` + mode
trailer **371** + CQST **54,196**.

CQST 54,196 = RC connectivity symbols **11,344** + RC geometry deltas **41,272** +
container headers **1,580** (44 global + 16 layers × (48 seedPos + 8 counts + 4 nd +
3×(12 lo/span/bytes))).

Raw overhead (mode+headers) = 371 + 1,580 = 1,951 B. **Measured marginal compressed
cost** (zstd-22 level, exact deltas of compressed size):
- mode stream: `zstd22(raw) − zstd22(CQST only)` = 53,435 − 53,176 = **259 B**
- CQST headers: `zstd22(CQST) − zstd22(headers-zeroed)` = 53,176 − 52,740 = **436 B**
- floor with all headers+mode removed: payload 52,686 → artifact **53,453 B**
  ⇒ **total transport-overhead headroom ≤ 761 B** (matches 54,214 − 53,453).

## 2. Stream entropy audit (decoded with the same RC coder; `impl/MMSE/mmse_cq_entropy_audit.cpp`)

| stream | stored B | ideal 0th-order H0 (per layer/component) | gap | KT-adaptive floor (no tables) | gap |
|---|---:|---:|---:|---:|---:|
| connectivity symbols (57,040 syms) | 11,344 | 11,161 | 183 | 11,199 | **145** |
| geometry deltas (59,331 vals) | 41,272 | 38,816 | 2,456 | 40,471 | **801** |
| mode stream (625 clusters) | 371 raw / 259 comp. | 114.6 B (1.467 bit/mode; per-layer static 106.2; cond-on-prev 112.6) | — | ~115 | **~144** |

The 2,456 B delta `H0` gap is **not** transport-recoverable: realizing per-layer
static entropy needs 48 frequency tables (Σ spans) ≈ ≥2.4 KB of table bytes ⇒ net
≈0 (that is exactly why the current design uses adaptive qsmodel with per-layer
reset). The realistic delta gain is only the coder-adaptation gap **≤801 B**.

### Context / transform experiments (all measured, all negative)

| scheme | cost | vs stored 41,272 |
|---|---:|---:|
| per-layer adaptive order-0 (current design emulation) | 40,471 | +801 |
| order-1 (prev value, KT per context) | 44,605 | −3,333 |
| order-1 blended (α=8 / α=32) | 43,310 / 41,371 | −2,038 / −99 |
| order-1 class-context (prev magnitude bucket) | ~+50 | ≈0 |
| static global per-component tables (incl. 1,810 B tables) | 49,819 | −8,547 |
| **joint (x,y,z) entropy** | 27,232 ideal | −11,585 ideal, **but** table cost ≈33.5 KB ⇒ net −21,953 |
| table-free context coding `z | binned(x,y)` (buckets 1–12) | 41,725–44,789 | −453…−3,517 |
| reversible axis transforms (y−x, z−x / z−y / 2nd diff) | 43,728–46,108 | −2,456…−4,836 |
| gate-differencing (delta-of-delta, incl. after axis transform) | 44,418–47,644 | −3,146…−6,372 |
| cross-layer prior init (previous coarser layer stats) | 41,387 | −115 |

Symbols: stored 11,344 vs KT 11,199 (alphabet 5) → ≤145 B. Ideal global static
(11,520 with no table cost) is already worse — no gain.

## 3. Headroom ledger

| item | realizable headroom |
|---|---:|
| transport overhead (compact headers + entropy-coded mode + framing) | **≤ 761 B** (realistic 400–600) |
| delta coder adaptation gap (KT init / cross-layer init) | **≤ 801 B** (realistic 300–600) |
| symbol coder | ≤ 145 B |
| mode recode | ≤ 144 B (counted in transport) |
| fixed base compact 468 / P2BL 89 / parquet 210 | 0 (artifact contract) |
| **upper bound (all ideal)** | **≈ 1.7 KB** (54,214 → ≥52,507) |
| **realistic** | **≈ 0.9–1.3 KB** (54,214 → 52.9–53.3 KB) |

Even the physically absolute upper bound (transport+model all ideal simultaneously)
is **1,706 B < the 2,825 B gap**; realistic is 0.9–1.3 KB. Therefore
**CIKM 20.710 bpv (51,389 B) is unreachable by stream/transport entropy work** on
this artifact; residual bulk is the paper-level geometry cost (16.63 bpv fused
deltas + 4.57 bpv A&D connectivity, already ≈ the A&D reference 4.61 bpv).
Closing it needs *symbol-level* modeling changes (better prediction/contexts of
the residual values), which the audit shows consistently lose with the current
bottom-up transforms — i.e., not a 1-build transport task.

## 4. Verdict

**N/A (ledger, 0 build).** Per task-64 rule (<1.5 KB headroom → N/A, no build):
audit gives ≤1.7 KB ideal / ~1.0–1.3 KB realistic headroom and **structurally
cannot reach 51,389 B**; keeping official fused artifact **54,214 B / 21.848370 bpv**
and canonical 54,601 B untouched.
Guards unaffected (zero code changes, zero build): default MMSE 62,633 B/md5
`9744d99c…`, PURE_K3 69,187, PCLTTM 68,160, PCATM 66,991, QEM/RD page-RA,
phase1 9/1/1, phase2 8/8 — all as previously locked.

## 5. Reproduce

```bash
# artifact (existing, task-63 window): /tmp/mmsep1/m52b/run3/paper/art_paper.parquet
# 1) unwrap P2BL payload / SBC1-zstd -> raw blob, split trailer/CQST  (see MMSE_TASK64_AUDIT.csv `command`)
# 2) standalone audit tool (NOT part of the project build):
gcc -O2 -c -o /tmp/rangecod.o src/rangecoder/rangecod.c
gcc -O2 -c -o /tmp/qsmodel.o src/rangecoder/qsmodel.c
g++ -O2 -std=gnu++20 -I src -c -o /tmp/mmse_audit.o impl/MMSE/mmse_cq_entropy_audit.cpp
g++ -o /tmp/mmse_audit /tmp/mmse_audit.o /tmp/rangecod.o /tmp/qsmodel.o
/tmp/mmse_audit /tmp/cqst_section.bin /tmp/mmse_seq
```
