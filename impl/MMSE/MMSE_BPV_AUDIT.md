# MMSE faithful horse19k q12 audit (WS-B, task-6) — **FAIL / not faithful**

Date: 2026-10-22. Owner: algo-bpv. Scope: read-only probe + diagnosis; no impl change.

## Verdict

- Canonical row: **25.241247 bpv** (`r50`, q12, horse19k) = 1.485x paper 17.0 -> **hard cap FAIL**.
- Best measured variant (lossless transport only): **24.696388 bpv** (`r40` + native
  EdgeBreaker base + Parquet ZSTD-22) = 1.453x -> still **FAIL** (hard cap 22.10 =
  54,375 B; best artifact 61,281 B -> need **-6,906 B / -11.3%**).
- Root cause is **structural**: the MMSE paper (Lee et al., ICIP 2011) algorithm is
  largely **not implemented in the serialized path**; the current row is an
  RDTag-compatible stream carrying an MMSE label. It cannot be closed by context
  modelling (documented ceiling 2.85%), transport tuning, rounds or quality tricks.

## Paper contract (docs/pdfs/MMSE.txt)

Total bitrates Table 1: Horse (19,851) = **17.0 bpv** (AD baseline 20.9, gain 18.7%).
Geometry-only Table 2: barycentric 13.9 -> proposed **13.1** bpv (gain 5.6%).
=> implied connectivity ~= 17.0 - 13.1 = **3.9 bpv** (AD/A&D's own connectivity is
4.61 bpv per Ahn et al. Table 3, so MMSE improves both sides).

Paper algorithm (§2):
1. §2.1 Segmentation: split each layer's vertices into clusters (split until cluster
   size < threshold), mode decided **per cluster**.
2. §2.2.1 Dual-ring prediction: `p = (u_j + alpha*Su)/w`, `alpha ∈ {0, 0.5, 1}`
   (alpha=0 is barycentric); 1-ring + alpha-weighted 2-ring.
3. §2.2.2 MMSE prediction (mode 3): linear minimum-MSE predictor with weights from the
   per-cluster autocorrelation matrix `R_j`, trained on the cluster neighbourhood.
4. §2.3 Mode decision: for each cluster choose best of modes {0,1,2,3}; encode mode
   indices with an arithmetic coder.

## Implementation audit (current gate path)

| paper component | implementation status | evidence |
|---|---|---|
| dual-ring predictor math | exists (`src/predict/DualRingPredictor.h`) but **not used by the serialized residual** | `alpha` probe: alpha=0/0.5/1 -> artifact byte-identical 61,281 B |
| MMSE LS predictor (mode 3) | **missing** | no `MMSEPredictor`/autocorrelation code anywhere in `impl/`+`src/` |
| per-cluster segmentation + mode decision | **missing in gate path** | `MMSEDualRingSymbolCollectOperator::collect()` delegates to `RDTagSymbolCollectOperator::collect()`; `gateModeQueues_` never filled => `modeDecisionPresent=0`; sv=3 comment: "V3 无 mode 流（原为 useModeDecision ? WithMode : plain，恒取 plain）"; `PMSF_MMSE_USE_MODE_DECISION` only survives in comments |
| residual stream | RDTag curvature-predicted bitplane RC (not MMSE's prediction) | `[MMSE-DR bpv sv=3] ... residual=...` identical machinery to RD |
| decoder | RDTag-compatible (`sv=1/3/4/8` readers), no mode stream in sv=3 | `MMSEDualRingSymbolReadOperator.cpp` |
| strict paper path | `PMSF_MMSE_STRICT=1` compresses (24.61 bpv) but **decode is structurally wrong** | MeshCompare: connectivity_ok=false, topology_ok=false, remap=false |

So: **dual-ring/MMSE prediction and mode decision are dead or absent**; the
MMSEPostProcessor dual-ring residual rewrite does not reach `getRemovedVertexPosInt`
(the serialized value), which the `alpha` probe proves.

## Measured decomposition (q12, horse19k)

Canonical `rounds=50` (62,633 B = 25.241247):
`valence=4,087 B (1.65 bpv) + isPolyBit=12,386 B (4.99) + residual=43,650 B (17.59)
+ blob container=620 B` = blob 60,743; + `art_compact.bin` 598 + Parquet 62,035
(=1,292 B Parquet overhead).
Best transport `rounds=40` + EB + ZSTD22 (61,281 B = 24.696388):
`valence=3,939 + isPolyBit=12,239 + residual=43,272 + container=500` = blob 59,950;
compact 685; residual.parquet 60,596.

Against paper: our G-side = 17.44 bpv vs 13.1 (-4.3 needed); our C-side
(valence+isPoly+container) = 6.72 bpv vs 3.9 (-2.8 needed). **Both sides must be
rebuilt** to reach 22.10.

## Structural experiments run (all lossless, q12 unchanged)

| probe | result |
|---|---|
| rounds 30/40/50/60/70/100 | optimum **r40 = 25.200** (r50 25.241, r30 25.612, r100 25.300) |
| r40 + `PMSF_BASE_EDGEBREAKER_NATIVE=1` + `PMSF_PARQUET_ZSTD_LEVEL=22` | **24.696388 bpv**, decode all true (counts/topology/manifold/remap) |
| config `alpha` = 0.0 / 0.5 / 1.0 | artifact **byte-identical** 61,281 B -> dual-ring predictor not serialized |
| `PMSF_MMSE_STRICT=1` | 61,068 B / 24.6105 bpv compress, but decode connectivity/topology **false** -> unusable |
| `PMSF_MMSE_USE_RETRI_TAG=1` / `PCLTTM_TAG` | cross-algorithm storage aliasing (RD/PCLTTM pipeline), not faithful; not used |

## Closure plan (requires new implementation, not tuning)

Phase A — wire the paper predictor stack into `impl/MMSE` (+ `src/predict`):
1. In the MMSE collector (do **not** delegate to RDTag): after patch collection,
   recompute each patch's residual with `DualRingPredictor(alpha)` for alpha in
   {0, 0.5, 1} (mode 0 = barycentric) plus a mode-3 linear predictor; choose the best
   mode per **cluster** (or per face as a cheaper first step); emit a mode bitstream.
2. Decoder must recompute the same predictor at reconstruction time (custom
   reconstruct hook for layout 22); needs a symmetric, bit-identical predictor.
3. Expected gain from predictors alone: paper shows only **+5.6% geometry**
   (13.9 -> 13.1), i.e. ~-0.8..-1.0 bpv here (-2..-2.5 KB) — **not enough alone**.

Phase B — connectivity rebuild: replace the `isPolyBit` patch stream (~5.0 bpv) with
a paper-style connectivity stream (~3.9 bpv target): ~-2.7 KB.

Only A+B together reach ~22.1. Estimated effort: multi-day (new codec path in
`impl/MMSE/{compress,decompress}` + layout22 scheme version + decoder symmetry tests),
not closable in the M2 window.

## Recommendation to Lead

- Classify the MMSE row as **NOT FAITHFUL (RDTag-compatible stream with MMSE label)**
  and report it as `FAIL / blocked` rather than a bpv-only gap; 25.241 (1.485x) is the
  number to carry, with 24.696 as the best lossless-transport variant.
- Do not spend further M1/M2 budget on context/rounds/transport; either scope a
  dedicated MMSE codec task (Phase A+B above) or accept the row as an open defect.

## Evidence

- `/tmp/wsb/runs/mmse_hq12_r{30,40,50,60,70,100}/`
- `/tmp/wsb/runs/mmse_hq12_r40_eb_z22/` (61,281 B, decode true)
- `/tmp/wsb/runs/mmse_audit_{alpha0,alpha05,alpha1,strict}/`
- `/tmp/wsb/runs/results.csv`; this directory's `MMSE_BPV_EVIDENCE.csv`.

---

# P1 (2026-10-22, WS-B4, task-15): 论文 predictor 栈进码流 — **FAIL（归因已量化）**

Owner: mmse-repro. Scope: `impl/MMSE/**`, `src/predict/**` (no q/variant tricks).
Target for the row: hard cap ≤ 22.10 bpv (54,375 B) / ideal ≤ 20.40 bpv.
Baseline carried: **62,633 B = 25.241247 bpv** (r50, q12, horse19k, decode six flags true).

## P1.1 Gate-path audit (corrects one earlier guess)

Measured with `PMSF_CFD_TRACE=1` + `PMSF_C2_DIAG=1` + `PMSF_STREAM_DUMP=1`:

- `[cfd] RDTag::collect ENTER isEnableAdjustment=0` — the MMSE config's
  `"adjustment": true` does **not** reach the header; the gate path takes
  `RDTagSymbolCollectOperator::collect` → `PCLTTMTagSymbolCollectOperator::collect`
  → `PCATMColorSymbolCollectOperator::collect`.
- Therefore `gateResidualQueues_` is filled with raw `r1 = vrPos_quant −
  floatPosToInt(barycenter(ring))` by `PCATMEliminateOperator::computeRemovedVertexResidualInt`,
  then the PCLTTM second BFS rewrites slot = `r1 − pred_c2`
  (`MCGAL::rdCurvaturePrediction`, mean of neighbours' `kVPropRDR1`).
  `pred_c2` is **nonzero** but tiny (mean |pred_c2| ≈ 1.9 units vs mean |r1| ≈ 17.6;
  the RD audit measured the same no-op). Probe: encoder with the paper's mode-0
  (barycentric, correction 0) gives residual stream 43,681 B vs the gate path's
  43,650 B ⇒ the curvature predictor is worth **−19 B**, i.e. the serialized
  residual is effectively the *barycentric* r1.
- Export emits exactly 3 streams (valence 4,087 B / isPoly 12,386 B /
  adaptive bit-plane residual 43,650 B) + 620 B blob container. No mode stream,
  no cluster segmentation; `alpha` never reaches the residual. Confirms the
  M2 verdict: the row is **not a faithful MMSE implementation**.
- Decoder reconstruction (`BasicReconstructOperator::insert_point` →
  `reconstructBarycenterPointNoAlloc`) uses the *integer* barycenter
  `trunc(mean(ring ints)) + r1`, which is what makes a symmetric, integer-domain
  predictor stack implementable (see P1.2).

## P1.2 What was implemented (env-gated, default byte-identical)

New `src/predict/MMSEPredictor.h` + a hook in
`src/predict/RDPredictor.h::rdCurvaturePrediction()` (decoder replay) +
`impl/MMSE/compress/MMSEDualRingSymbolCollectOperator.cpp` post-pass:

1. **Dual-ring prediction** (paper §2.2.1 eq.(3)), quantized int domain, both
   α = 0.5 and α = 1:
   `xhat = (u_j + α·Σ_{l∈N_j} (1/v_l²)(v_l·x_l − Σ_{k∈N_l,k≠j} x_k)) / w`.
   In the coarse mesh j is absent, so the paper's `v_l` (= |N_l| in the finer
   mesh, which contains j) is `(coarse valence of l) + 1`, and the inner sum is
   over l's coarse neighbours. (Dropping the `+1` shrinks the prediction by a
   constant factor on planar regions — measured residual stream +19,821 B; this
   is why the naive "coarse valence" form is wrong.)
   All ring walks use **halfedge circulation anchored at the gate polygon**
   (`h → h->opposite()->next()`), never `Vertex::proxyHalfedge()`: an eliminated
   vertex's proxy can point at a dead halfedge, so the encoder walked a
   different umbrella than the decoder (measured decode connectivity/topology
   **FAIL**). After anchoring: all six decode flags true.
2. **MMSE LS prediction** (paper §2.2.2 eq.(4)–(8), ρ=0.997, 1-ring ∪ 2-ring,
   R_kl = ρ^{d_kl}, C_k = ρ^{d_kj}, w = R^{-1}C, tiny ridge 1e-9).
3. **Per-cluster mode decision** (paper §2.3 eq.(9)): minimise Σ‖x_j − f_k(x_j)‖²
   over clusters of consecutive polyFaces in BFS order (threshold
   `PMSF_MMSE_CLUSTER`, default 32; the paper's connectivity-driven recursion is
   approximated by BFS order — deterministic and decodable with no side info).
4. **Mode stream in the code stream**: a self-describing 2-bit/cluster table
   appended inside the existing residual stream (flag `0x20000000` on the
   `numCoords` word): fixed mode = 1 B/round, per-cluster = 3 B + 2 bit/cluster.
   No blob-layout/container change; off-state artifact is byte-identical to the
   frozen baseline (`md5 06a635…/9744d9…`, 62,633 B).
5. `PMSF_MMSE_PREDICTOR` = `bary|dr05|dr1|ls|mode` (env-gated, encoder side only;
   the decoder reads the self-describing stream).

## P1.3 Measured ablation (q12, horse19k, r50, canonical transport)

Evidence: `impl/MMSE/MMSE_P1_PREDICTOR_EVIDENCE.csv`,
`impl/MMSE/mmse_p1_predictor_ablation.sh`, raw runs `/tmp/mmsep1/ab2/`.

| config | artifact B | bpv | residual RC B | mode hdr B | predictor net | decode 6 flags |
|---|---:|---:|---:|---:|---:|---|
| baseline (off) | 62,633 | 25.241247 | 43,650 | 0 | — | all true |
| bary (mode 0) | 62,664 | 25.253740 | 43,631 | 50 | +31 B | all true |
| **dr05 (α=0.5)** | **62,562** | **25.212634** | **43,529** | 50 | **−71 B (−0.029 bpv)** | all true |
| dr1 (α=1) | 62,652 | 25.248904 | 43,619 | 50 | +19 B | all true |
| ls (mode 3, fixed) | 65,864 | 26.543348 | 46,831 | 50 | +3,231 B | all true |
| mode decision, cluster=32 | 62,730 | 25.280339 | 43,468 | 279 | +97 B | all true |
| mode decision, cluster=8 | 63,092 | 25.426225 | 43,334 | 775 | +459 B | all true |
| mode decision, cluster=16 | 62,851 | 25.329102 | 43,423 | 445 | +218 B | all true |
| mode decision, cluster=64 | 62,676 | 25.258576 | 43,502 | 191 | +43 B | all true |
| mode decision, cluster=128 | 62,636 | 25.242456 | 43,415 | 138 | +3 B | all true |
| mode decision, no LS | 62,730 | 25.280339 | 43,468 | 279 | +97 B | all true |

Mode histograms (cluster=32, 649 clusters): bary 191, α=0.5 177, α=1 277, LS 4.
Decoded-mesh check vs the baseline decoded OFF: 0 differing vertices for bary,
43–50 (of 19,851) for the dual-ring/LS rows, max |Δ| = 8.95e-5 (≈1.4 quant
steps) — a ≤1-unit, 0.25%-of-vertices predictor asymmetry that survives the six
MeshCompare flags (documented, not hidden).

## P1.4 Why the paper's predictor does not transfer (quantified)

Offline analysis of the per-face `r1` + the four paper corrections
(`PMSF_MMSE_PRED_DUMP`, 19,797 faces, `/tmp/mmsep1/pd`):

- correction magnitude: mean |corr| = **1.79 units (α=0.5) / 3.41 (α=1)** vs
  mean |r1| = **15.71 units** — the paper correction is 11–22% of the residual
  scale;
- optimal scalar multiple of the correction: β = 1.88 (α=0.5 form) / 0.96 (α=1),
  and the **maximum removable residual energy is only 3.8–4.0% of SSE**;
- entropy effect: per-round-per-axis H0 of the residual values changes
  40,204 B → 40,107 B (α=0.5) / 40,192 B (α=1): the residual is heavy-tailed, so
  a 4% MSE cut buys only ~0.05% of total bpv;
- LS (mode 3): with ρ=0.997, R is near-singular (cond ~2×10⁶) and the exact
  weights collapse to (≈) the 1-ring average, i.e. mode 3 ≈ mode 0; when the
  solve is allowed to produce the paper's nn weights it is net-harmful
  (+3.2 KB fixed). Its measured selection rate is 0.6%.

**Conclusion P1**: the faithful predictor stack (dual-ring + LS + per-cluster
mode decision) delivers at most **−121 B of residual payload / −71 B artifact
(−0.029 bpv)**; the paper's 5.6% *geometry* gain (13.9→13.1 bpv) does **not**
transfer to this gate/BFS patch structure because our residual is
detail-dominated (the decimation keeps high-frequency detail vertices, whose
positions are unpredictable geometrically). Hard cap 22.10 needs **−8,258 B**, a
factor **116×** beyond what the predictor can give. → **FAIL**, structural.
Also measured: `isPoly`'s information content is already at its combinatorial
floor — a round visits ~5,600 faces to flag ~400 gates, and
log2(C(5600,400)) ≈ 265 B/round ≈ 13.2 KB ≈ the actual 12.4 KB. The gate set is
irreducible *as an explicit subset*; only making it implicit via a conquest can
cut connectivity.

## P1.5 Remaining gap attribution (where the −8.2 KB can come from)

| side | ours | paper (implied) | gap | what is needed |
|---|---:|---:|---:|---|
| geometry residual | 17.59 bpv | 13.1 bpv | −4.49 | not prediction: the value entropy of *our* residuals is 16.24 bpv (H0) and the decimation detail is irreducible; a residual-aware decimation order would change quality (previously measured +0.49 bpv at quality cost, rejected as unfaithful) |
| connectivity (valence+isPoly) | 6.64 bpv | 3.90 bpv | −2.74 | a paper-style *valence-driven conquest* connectivity coder (gate identity derived from the conquest instead of a per-visited-face isPoly bit) |
| residual entropy coding | 43,250 B RC vs 42,099 B (same symbolization floor); value H0 40,240 B unreachable | — | −0.3…−0.8 KB realistic | bitplane-model warm start across rounds; direct value/Rice/gamma/Huffman probes are all worse (see P1.6) |
| transport | 1,890 B (1,292 parquet + 598 compact) | — | −1,300…−1,800 | r40 + EdgeBreaker base + zstd22 measured 24.696 bpv (61,281 B) earlier; phase-2 schema work |

Stream entropies measured this round (`PMSF_STREAM_DUMP`, per-round models):
isPoly symbols 280,620 → H0 11,954 B vs actual RC 12,186 B (232 B headroom);
valence symbols 19,797 → H0 3,192 B vs actual 3,687 B (495 B). Connectivity
coding is also near its entropy floor ⇒ the 2.7 bpv connectivity gap is a
**representation** problem, not an entropy-coder problem.

## P1.6 P2/P3 plan (multi-round, hard cap 22.10 = 54,375 B)

- **P2 (cheap, low risk): residual entropy coder, 200–400 LOC, 1 round.**
  Keep the bit-plane symbolization; warm-start/group the adaptive models across
  adjacent rounds and add a magnitude-class context. Probe results:
  value-level H0 (per-round-axis, oracle tables) = 40,240 B, but the *current
  symbolization's* oracle floor is 42,099 B (per-round bitplane contexts) and
  shared tables per 2-round group lose to 42,354 B; actual = 43,250 B.
  Direct Rice (−/+ 260 B), Elias-gamma (+14.8 KB), bit-length-Huffman (+2.1 KB)
  and direct value-adaptive KT (+13.7 KB) all lose — the bitplane coder is
  already near its symbolization's entropy. Realistic gain = **−0.3…−0.8 KB
  (−0.12…−0.32 bpv)**, not the naive −3.0 KB. Env-gated; default bytes
  unchanged.
- **P3 (main lever, high risk): connectivity rebuild, 1,500–2,500 LOC, 3–5
  rounds.** Replace the per-visited-face isPoly stream (12,386 B / 280,620 BFS
  visits for 19,797 gates) with a valence-driven conquest replay: the decoder
  derives gate faces from the valence/conquest order instead of reading an
  isPoly bit per visited face. Target connectivity 16,473 B → ≈9,700 B
  (paper-implied 3.9 bpv) ⇒ **−6.0…−6.8 KB (−2.4…−2.7 bpv)**. This is the same
  traversal-rebuild problem as RD's PGS plan (shared risk with task-13).
- **P4 (polish): keep fixed α=0.5 dual-ring (`PMSF_MMSE_PREDICTOR=dr05`), −71 B;
  per-cluster segmentation/mode polish ≤ −0.1 bpv.**
- Expected landing: 25.241 − (0.12…0.32) − (2.1…2.7) − 0.03 ≈ **22.2–23.0 bpv**
  (transport r40+EB+z22, previously measured −1,352 B / −0.55 bpv, would give
  **21.6–22.4 bpv**). Hard cap 22.10 is reachable only if P3 lands at its upper
  bound (−6.8 KB) *and* the transport path is adopted; the ideal 20.40 line
  additionally needs a below-3.9-bpv connectivity coder.

## P1.7 Reproduce

```bash
MAKE_J=32 scripts/tests/build.sh                      # or the flock target build
impl/MMSE/mmse_p1_predictor_ablation.sh build/phase1/mmse_p1
# baseline byte-identity (off state):
PMSF_MMSE_PREDICTOR= ./build/PMSFCompress <canonical c.json>  # md5 06a635…/9744d9…
```
Artifacts: `impl/MMSE/MMSE_P1_PREDICTOR_EVIDENCE.csv`,
`impl/MMSE/mmse_p1_predictor_ablation.sh`, raw `/tmp/mmsep1/ab2/`.

---

# P3 spike (2026-10-22 round 2, WS-B4): connectivity 表示重建 — **GO**

设计/可行性 spike 全文见 **[`P3_CONNECTIVITY_SPIKE.md`](P3_CONNECTIVITY_SPIKE.md)**；
证据 `MMSE_P3_BACKBONE_EVIDENCE.csv` + `mmse_p3_backbone_probe.sh`。结论摘要：

1. MMSE 论文 §2 的 connectivity/decimation 就是 **Alliez-Desbrun valence-conquest
   [2]**，而该机制已在仓库里（`src/conquest/FrontierConquestEngine` +
   `ConquestDecodeLayer`，逐符号/逐层验证过；5 个 EngineHooks 中 **H3
   GeometryPredictor 是 MMSE dual-ring/LS 的正确接缝，且解码端用同一份 hook**）。
2. 实测该 backbone（q12 horse19k，`algorithm=conquest` 仅作测量载体，
   `PMSF_BASE_EDGEBREAKER_NATIVE=1`）：**r5 = 54,130 B / 21.8145 bpv**（硬顶
   22.10 = 54,838 B，余量 **+708 B**），**r16（迭代到收敛、base 74 v 的论文式
   工作点）= 54,604 B / 22.0055 bpv**（+234 B）；r3–r20 全部 PASS 硬顶，
   decode 六项全 true，质量 `rms_max=9.2341e-06` 与现 MMSE `9.2324e-06` 同尺度。
3. 分量：r16 每层 connectivity **11,344 B = 4.57 bpv ≈ A&D 论文 4.61 bpv**
   （现 MMSE C-side 16,473 B ⇒ −5,129 B）；r5 connectivity 9,443 B + geometry
   32,815 B（base 3,118 v 在 compact 10,884 B）。
4. P3 落点 = **把 backbone 接成 MMSE（新自注册算法 `mmse_cq`，零 CMake 修改、
   零写作用域外改动）+ 论文 dual-ring α∈{0,0.5,1}/MMSE-LS/per-cluster mode
   decision 经 H3 hook + 层后重编码（解决 cluster 前瞻）**；~870–1,130 LOC、
   2–3 轮、中风险；预期 **21.8–22.1 bpv（硬顶大概率 PASS，理想 20.40 不可达）**。
   默认 MMSE canonical 62,633 B / 25.241247 继续逐字节不变。

---

# P3-B2 (2026-10-22 round 2, WS-B4): paper predictor stack on conquest backbone — **DONE**

`mmse_cq` = A&D valence-conquest backbone (paper [2]) + MMSE paper §2.2 dual-ring
(α=0/0.5/1, cell domain) + §2.2.2 MMSE-LS (ρ=0.997 on coarse-graph distances) +
§2.3 per-cluster mode decision (2-bit/cluster=32, BFS order), independently
named/registered (`impl/MMSE/MMSECQuestAlgorithm.cpp`, layout plugin self-registers
layout=15 + codecVersionOverride=2). Zero `src/` change beyond the already-approved
`PatchView::centerId`; zero `src/conquest`/`src/scheme`/storage edits.

**Symmetry contract (non-obvious, was the only real bug):** the paper predictor is
evaluated on the layer's **coarse graph**, not on the intra-layer progressive state.
Encoder: H3 returns the plain mean during `decimateLayer` (engine records
`delta_mean = cell(target) − cell(mean)`), then a post-layer pass rebuilds every
residual with the paper stack on the now-coarse mesh and rewrites the layer's CQST
deltas + emits the mode trailer. Decoder: excludes the current center
(`PatchView::centerId`) and every vertex inserted earlier in the same layer. The
coarse adjacency is snapshotted from **face loops** — proxy/halfedge umbrella walks
can stop early on halfedges whose faces were merged, which made encoder/decoder
predict different secondary rings (measured: decoded topology/geometry FAIL,
position drift ≈0.0058 before the fix). All arithmetic is integer-cell domain, so
sums are order-independent ⇒ bit-exact replay.

## Measured (q12 horse19k, columnar, `PMSF_BASE_EDGEBREAKER_NATIVE=1`, 19851v)

| rounds | mmse_cq | bpv | conquest ref | Δ | decode six | decoded mesh vs conquest |
|---|---|---|---|---|---|---|
| r5 | 54,119 B | 21.810085 | 54,130 B / 21.814518 | −11 B | all true | **byte-identical** (`24de7feb…`) |
| **r16 (final)** | **54,601 B** | **22.004332** | 54,604 B / 22.005541 | −3 B | all true | **byte-identical** (`d856290b…`) |

Hard cap 22.10 → r16 margin **237 B** (r5 margin 719 B); ideal 20.40 not reached
(needs 50,620 B; gap 3,981 B = 1.60 bpv, C-side/delta dominated).

Components r16: compact 468 + parquet 54,133; raw blob 54,567 = trailer 371
(0.1495 bpv) + CQST 54,196 (conn 11,344 + delta 41,272 + container 808).
Net geometry gain −374 B vs conquest, trailer cost +371 B ⇒ net −3 B (r5: −11 B).
**Mode hist r16: bary 300 (48.0%), dr05 222 (35.5%), dr1 103 (16.5%), LS 0**
(r5: 264/182/79/0). LS is implemented and evaluated per cluster but never wins on
this data — consistent with P1's LS ablation being worse than barycentric.

## Byte-identity vs baseline (default paths untouched)

- canonical MMSE r50: 62,633 B / 25.241247 bpv, `art_compact.bin` md5 `9744d99c…`,
  `art_residual.parquet` md5 `06a63531…` — identical to the frozen baseline
  (⚠️ this row must NOT set `PMSF_BASE_EDGEBREAKER_NATIVE`; setting it changes the
  compact bin to 378 B — probe usage error, not a regression).
- default conquest r5/r16 payload md5 identical to the B1 baseline
  (`7e700ee0…/508f6f2a…`, `15403a9b…/a3054310…`).

## Reproduce

```bash
flock /tmp/pmsf_build.lock -c 'cmake -S . -B build && cmake --build build -j32 --target PMSFCompress PMSFDecompress'
BUILD_DIR=/tmp/mmsep1/bin bash impl/MMSE/mmse_cq_probe.sh /tmp/mmsep1/b2final 5 16
# run dirs: /tmp/mmsep1/b2final/{r5,r16}; canonical: /tmp/mmsep1/verify_b2/noenv
```
