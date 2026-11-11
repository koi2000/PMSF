# PCATM task-45 route C (decoder-derivable implicit patch/zero set) — read-only design / lower bound

Date: 2026-10-22. Owner: algo-bpv. **No build in this turn** (Lead reviews first).
Context: PCATM is the last failing row (strict faithful 28.834819 / 71,550 B; hard cap 68,185 B).
task-42 (mixed global color) and task-44 (explicit masked/patch-only color) both measured FAIL on
full-40-round artifacts; this report evaluates route C, whose task-42 design lower bound was
≈60.7 KB / 24.47 bpv.

## 1. What route C must remove (measured motivation)

| stream (strict r40 q12) | bytes | note |
|---|---:|---|
| color (`rc_blob` colorLZ) | 22,272 | 462,644 raw bits / 231,322 face symbols; 77.5 % are repeated background faces |
| zero side (v7/task-42) | 1,344–3,785 | per-patch candidate zero pair indices |
| residual | 47,298 | one 3-D residual per patch (geometry), unchanged by C |
| CONN + compact + parquet | 1,980 | container/artifact overhead |
| **total** | **71,550** | need −3,365 B for the hard cap |

Target of C: replace the per-face color map and the explicit zero side with **decoder-derivable
patch/zero sets**, leaving only residuals + tiny headers.

## 2. What the decoder actually needs (from the current strict code)

Per round the existing decoder derives the fine-mesh recovery from the color stream in three
implicit ways:
1. **patch membership** — flood-fill of faces with the same nonzero color (mode-2/3/4 grouping);
2. **patch root** — the first face of each flood-filled component; it becomes the `splittable`
   polygon root (`setPolyValence(v)`), all other component faces are its children;
3. **patch orientation / zero pair** — mode-2 Z-shape uses the explicit candidate pair (v7 side),
   mode-4 uses the same-color child grouping; child order/expanded fan is derived from faces marked
   `isAdded()/paperLevel()` by the encoder.

The encoder-side component definition itself is **history-dependent**: `comps` are connected
components of faces linked by *encoder-added half-edges at the current paper level*
(`isInternalPatchEdge`), and roots are encoder faces with `isSplittable()`. Those flags are **write
side state**: the decoder only learns them by consuming the color stream (it sets `setAdded()` on
the diagonals it recovers). In other words, the current format's connectivity information is carried
by the per-face color labels; there is **no independent topology signal** in the decoder's coarse
mesh from which the next round's patch face set / root / zero pair could be re-derived.

Deterministic replay with no color stream therefore requires one of:
- **(C1) elimination-time invariant**: enforce the paper's alternating 2-colour Z-shape rule at
  elimination so that every patch is a deterministic Z/fan of its root and the decoder can
  reconstruct it from the coarse polygon ring alone (no per-face bits, no side). Multi-build:
  elimination operator + collector assignment + decoder rule + validation.
- **(C2) decoder restructure**: keep arbitrary patches but transmit per-patch records that the
  decoder consumes in a deterministic order (root-index/valence/label sequence/zero rank) — the
  A+ idea, but with per-face-identity-free ordering. Multi-build decoder rewrite; measured mask
  variant (task-44) already failed on size.

## 3. Ambiguity evidence (why C is not a 1-build spike)

Measured on real full-40-round strict artifacts / v7 dumps:
- per-phase-patch zero candidates: 6,655 patches, candidate-pair count distribution
  {n=3:1,094; n=4:4,953; n=5:397; n=6:114; n=7:56; n=8:26; n=9:11; n=10:3; n=12:1}; mean ≈3.98 →
  **~82 % of patches have ≥2 valid zero pairs**; choosing pair (0,1) happens 62 % of the time and
  idx0==0 96.9 % of the time, i.e. a deterministic "first candidate" rule is wrong for ~38 % of
  patches;
- v2–v6 history: geometric Z-matcher ambiguity/foreign-blue conflicts made all rounds fall back to
  mode 4 (mode2Rounds=0); task-36 v7 needed explicit indices to get 4/40 mode-2 rounds;
- patches are not Z-shaped in general: same-color components come from the fan created by
  `VertexRemovalEliminateOperator`; the added-diagonal pattern must be enforced at elimination to
  satisfy the paper's alternating invariant. That enforcement changes which vertices are removable
  and was explicitly out of scope for the 1-build attempts (v2–v6 and task-42/44 all avoided it
  because it is a multi-build effort).

Therefore **C1 cannot be validated in one build**; the ambiguity is structural (0.38 error mass on
the zero choice, plus component membership unknown without adding an equivalent signal).

## 4. Lower bounds (full 40 rounds, faithful residual 47,298 / overhead 1,980)

| variant | color/side estimate | total estimate | bpv | 1-build feasibility |
|---|---:|---:|---:|---|
| C1 fully implicit (paper Z invariant; no color, no side; only a tiny per-round root mask ≈19.7 kb raw → ~1.0 KB compressed) | ~1.0 KB | **≈50.3 KB** | ≈20.3 | **multi-build** (elimination rewrite + decoder rule) |
| C1 + ambiguity side fallback (root mask + zero rank ~2.7 KB raw) | ~3.3 KB | **≈52.6 KB** | ≈21.2 | multi-build |
| C2 per-patch records (root delta/valence/labels/zero rank; no full color map) — task-44 estimate | 9.7–16.2 KB | 59.0–65.4 KB | 23.8–26.4 | ≥2 builds (decoder restructure); 1-build success low |
| keep full color, remove only side (task-42 v7) | 24,799 | 74,293 | 29.94 | measured FAIL |
| explicit mask/patch-only (task-44) | 23,459–25,818 | 73,754–75,119 | 29.7–30.3 | measured FAIL |

Important: the C1 lower bound (~50 KB) is *below the task-42 estimate* because it removes both the
per-face color stream and the zero side, but it is **not achievable in 1 build**: it requires the
elimination-time Z/alternating invariant to be proven and enforced first (the paper itself spends a
full section on this), plus a decoder reconstruction rule and a validation/fallback path.

## 5. Verdict

**FAIL / blocker for a 1-build spike.** Route C is the right long-term route (its implicit lower
bound ≈50–53 KB would also beat the ideal 25.37 bpv), but it cannot be delivered in the approved
single-build window: the decoder has no independent signal for patch membership/root/zero choice,
and the measured ambiguity (≈38 % of patches choose a non-first zero pair; v2–v6 confirmed
matcher conflicts) makes a one-shot deterministic rule fail without elimination enforcement.
No build was run for this report; no PASS claim; matrix PCATM stays FAIL 28.834819.

If Lead wants to invest multi-build effort, the recommended sequence is:
1. **Build A (instrumented, no format change)**: count, on the strict full-40 flow, how many patches
   are already paper-Z/alternating-compliant; measure the ambiguity that survives a candidate
   deterministic rule (report rates only, no artifact change).
2. **Build B**: elimination-time alternating/Z-shape enforcement for patches, keeping the existing
   color stream as a safety channel; verify decode 7/7 + byte identity of defaults + measure how
   many patches become color-free.
3. **Build C**: remove the color map for the compliant subset (hybrid C1/C2), verify ≤68,185 B.
This is ≥3 builds and is therefore outside task-45's 1-build bound.

## 6. Evidence
- `impl/PCATM/PCATM_TASK45_OPTIONS.csv` (lower-bound table)
- measured inputs: `PCATM_STRICT_FIX.md`, `PCATM_2COLOR_TASK18.md` §v2–v7, `PCATM_V7_EVIDENCE.csv`,
  `PCATM_TASK42_EVIDENCE.csv`, `PCATM_TASK44_EVIDENCE.csv`; candidate distributions from
  `/tmp/pcatm_v7_side_dump/compress.log` (6,655 phase patches) and v7 phase dumps.
- No build was run in this turn; live defaults/M4/M5 untouched.
