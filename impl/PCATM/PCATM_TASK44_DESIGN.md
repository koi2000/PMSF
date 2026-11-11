# PCATM task-44 二轮结构 R&D — read-only design / lower-bound report

Date: 2026-10-22. Owner: algo-bpv. **No build in this turn** (Lead reviews first). Stage-1 now
8 PASS(assumption 1)/1 FAIL/N/A: PCATM is the only FAIL (28.834819; hard cap 68,185 B / 27.48 bpv).

## 0. Measured stack (all real full-40-round artifacts)

| attempt | total | color/side | residual | overhead | rounds/removed | decode |
|---|---:|---|---:|---:|---|---|
| strict faithful baseline r40 q12 | 71,550 | color 22,272 (rawColorBits 462,644 over 231,322 face symbols) | 47,298 | 1,980 | 40 / 19,694 | 7/7 |
| v7 per-round side (task-36) | 74,293 | color+side 24,799 | 47,621 | 1,873 | 40 / 19,694 | 7/7 |
| task-42 A+B global-mixed (PHASE=4) | 75,325 | mixed color 24,348 + side 1,344 (+8) | 47,621 | 2,012 | 40 / 19,683 | 7/7 |

Task-42 conclusion stands: fragmentation fix alone cannot pass — corrected route-A lower bound
(ideal baseline coder) 71,453 B > 68,185. We now need a format that removes the **per-face color
symbols**, not just recodes them.

## 1. Patch statistics (from the strict log / deg distribution)

removed patches **19,694**: deg3 6,203 / deg4 2,737 / deg5 3,610 / deg6 6,474 / other 670.
Σvalence ≈ 91,476 → **inner (child) triangles Σ(v−2) = 52,088**, versus **231,322 emitted color
symbols** (77.5 % of the color symbols are *not* patch labels — they are per-face background
colors re-emitted every round).

## 2. Candidate A+ — patch-record color emission (RECOMMENDED for the 1-build spike)

Idea: replace the per-face color map with **one record per patch**, emitted every round in the
decoder's patch order (the same order the residual stream already uses), one global LZ pass:

per patch: `[root face delta varint][valence/compSize 4b][labels: (v−2)×1b (mode2) or ×2b
(mode4)][zero-pair rank: B-lite rank0-first unary ≈1.1b]` + header counts.

Quantitative estimate (raw bits): root deltas 19,694×3.5 b ≈ 68.9 kb; valence H≈2.06 b/patch
≈40.6 kb; labels 52,088 symbols ×0.8–1.3 b ≈ 41.7–67.7 kb; zero ranks ≈21.7 kb →
**raw 172.9–198.9 kb = 21.6–24.9 KB**; with a context/LZ coder at ratio 0.45–0.65 →
**color ≈9.7–16.2 KB**. Replacing color 22,272 in the baseline:

| coder ratio | new total | bpv | hard cap |
|---|---:|---:|---|
| 0.45 | 59.0–60.5 KB | 23.8–24.4 | **PASS (margin 7.7–9.2 KB)** |
| 0.55 | 61.2–63.0 KB | 24.7–25.4 | **PASS (margin 5.2–7.0 KB)** |
| 0.65 | 63.3–65.4 KB | 25.5–26.4 | **PASS (margin 2.7–4.9 KB)** |

- Full 40 rounds / removed ≈ 19,694; faithful (same elimination, no q/codebook/alias/Conquest).
- **mode2Rounds>0**: keep the existing PHASE=4 lever (measured full 40 rounds, removed 19,683 ≈
  canonical) and give mode-2 rounds 1-bit labels → mode2Rounds = 4/40.
- Also removes the **side ambiguity dependency**: zeros are transmitted per patch with the B-lite
  rank (1.1 b/patch), so no global candidate-table sync.
- Decoder replay: root face + valence + labels + zero pair fully determine the polygon
  triangulation; the existing fan/Z reconstruction (`collectPatchTreeDiagonals` semantics) can be
  driven from the record. Faces are then re-tagged/created exactly as today. Deterministic.
- Implementation: new color-stream format v3 + encoder record emission (reuse existing per-round
  patchOrders/residual loops) + decoder record-driven recovery (~250–350 LOC across the two strict
  TUs). Risk: **medium-high** for a single build (runtime ordering/edge-case risk), but
  compile-risk is low (syntax-check before build) and the size margin is large even in the worst
  coder-ratio scenario.
- Why not just sparse-color the existing map: nonzero density is ~39 % and a mask entropy
  ≥0.96 b/symbol (≥27.8 KB raw for the mask alone) already exceeds the current 22.3 KB color
  stream, so the per-patch record is the only way to cut the emitted symbols.

## 3. Candidate C — deterministic zero / Z-shape+alternating invariant (not recommended for 1 build)

Removes side (1.34 KB) and reuses the paper's 2-color Z-shape rule so the decoder derives zero
pairs. Lower bound (task-42 design): **60,737 B / 24.47 bpv**; with paper-level residual
54,472 / 21.95. But it requires elimination-time Z-shape/alternating-invariant enforcement plus a
decoder deterministic matcher; v2–v6 showed the ambiguity/blue-conflict failure modes, and the
elimination rewrite is a multi-build effort. 1-build success probability low (≈30–40 %), and it
does not remove the per-face color map (color saving uncertain). → keep as the follow-up route to
21–22 bpv after A+.

## 4. Candidate B+ — residual + color entropy specialization (low-risk fallback)

Self-contained codec changes (new codec ids; no format/order change):
- residual: 47,621 B → target 44.3–44.8 K (5–7 % via per-axis/context range coding over the 544-symbol
  alphabet); - color: 24,348 (task-42 mixed) / 22,272 (baseline) → -8–15 % via order-2 context model.
On the task-42 PHASE=4 full-40-round stack: PASS requires (color×0.85, residual×0.90) or better
(66.9 K / 26.96) — (color×0.90, residual×0.93) is already 68,120 B = marginal PASS. Realistic
combined gain is uncertain and the margin is thin (≤1 KB), so a single build is a coin flip.
→ fallback if Lead prefers minimal-risk coder work; also a natural **add-on** after A+.

## 5. Lower-bound summary (full 40 rounds, faithful)

| route | lower bound (est) | bpv | 1-build risk |
|---|---:|---:|---|
| A+ patch-record color (PHASE=4, B-lite zeros) | **59.0–65.4 KB** (coder 0.45–0.65) | 23.8–26.4 | medium-high |
| C deterministic Z-shape/no side | 60.7 KB (paper residual 54.5 KB) | 24.5 (21.95) | high (multi-build) |
| B+ residual+color entropy | 64.3–68.3 KB (needs ×0.87–0.90 + ×0.85–0.90) | 25.9–27.5 | low-medium, thin margin |
| D other (container/parquet only) | 70,570 | 28.4 | FAIL alone |

**Recommendation: A+ patch-record color as the 1-build spike** — it is the only route whose
estimated lower bound is below the cap in *all* coder-ratio scenarios and it removes the
per-face-symbol overhead (the actual root cause). C stays as the next multi-build route; B+ is a
fallback/add-on.

## 6. Spike plan (after Lead approval only; exactly 1 build)

1. Derive worktree from the existing `.agents/worktrees/task42-pcatm-global-color` (branch
   `task44-pcatm-patch-color`) — already mirrors live + v7/task-42 code; no live/M4/M5 edits.
2. New env `PMSF_PCATM_STRICT_PATCH_COLOR=1` + format v3 in the color stream:
   `[magic/version][numRounds][per-round: patchCount, mode][records...]` one LZ pass; env off keeps
   existing bytes exactly.
3. Encoder: build per-patch records from the existing comps/patchOrders, emit in the residual order
   already used by the strict path; reuse the measured rank-unary zero coding (B-lite).
4. Decoder: record-driven patch reconstruction using the existing fan/Z construction; fail-closed
   with per-round checksums; fall back per round to the legacy path if a record is invalid (the
   legacy color map is still emitted? no — we replace it; fallback = hard fail, no silent corruption).
5. Same build runs: PHASE=4 (mode2Rounds=4) and PHASE=0 control; guards: PCATM PURE_K3
   69,187/27.882525/md5, strict 71,550/28.834819/md5 + 7/7, PCLTTM 68,160/sha, paper matrix 8/8,
   fusion 90,490, v1 four (all frozen, re-verified values).
6. PASS: artifact ≤68,185 B / ≤27.48 bpv, **full 40 rounds / removed≈19,683**, decode 7/7,
   mode2Rounds>0. Else FAIL + ledger. No additional build (Lead may re-approve only).

## 7. Stop conditions (honest)
- Any decode mismatch/tree/checksum failure → FAIL (no relaxation).
- If the first build's artifact >68,185 B → FAIL + per-component ledger (no second format tweak).
- PHASE>4 or any 6-round artifact is invalid and will not be reported as PASS.
- No live default / M4 / M5 changes; no conquest/alias/q/codebook changes.

## 8. Evidence
- `impl/PCATM/PCATM_TASK44_OPTIONS.csv` (route/size/lower-bound/risk table)
- measured inputs: `impl/PCATM/PCATM_TASK42_{EVIDENCE.csv,SPIKE_RESULT.md,DESIGN.md}`; baseline
  strict/v7 logs `/tmp/pcatm_v7_*`, `/tmp/task42_runs/`.
- No build was run for this report.
