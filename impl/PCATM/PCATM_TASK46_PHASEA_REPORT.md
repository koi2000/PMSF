# PCATM task-46 Phase A (instrumentation) — result: **compliance too low → BLOCKER, no Phase B**

Date: 2026-10-22. Owner: algo-bpv. **1 build (instrumentation only)**, isolated worktree
`.agents/worktrees/task46-pcatm-phaseA` (derived from task-44); no live/M4/M5 changes, no
artifact/format/default-path changes.

## 1. Instrumentation integrity (bytes unchanged)
- Env-gated stats only (`PMSF_PCATM_TASK46_STATS=1`, CSV/append + stderr); encoder logic untouched.
- Strict r40 q12 artifact with stats **byte-identical** to the frozen strict row and to the no-stats
  control: `art_compact.bin` md5 `ac222c20299abd2ea2eb2fd2c418eda2`,
  `art_residual.parquet` md5 `52b17d0c4e1e1ecdd7b0a7934291cca0` (removed=19,694, rounds=40).
- PHASE=4 run: 40 rounds, removed=19,683, artifact 73,635 B (matches the earlier v7 control),
  mode2Rounds=4. No default bytes changed.
- Instrumentation patch: `/tmp/pcatm_task46_phaseA_instrument.patch` (1 TU, stats block only).

## 2. Measured compliance / ambiguity (full 40 rounds, strict flow)

| metric | strict r40 | PHASE=4 r40 |
|---|---:|---:|
| patches (comps) | 19,694 | 19,683 |
| high-valence patches (k≥3, valence≥5) | 10,754 (**54.6 %**) | 10,580 (**53.8 %**) |
| **paper-Z reproducible** (geometric matcher pair == actual zero pair) | **0 / 10,754 = 0.00 %** | **0 / 10,580 = 0.00 %** |
| matcher returns any pair (zOk) | 33.1 % | 29.1 % |
| candidate pairs per high-valence patch (mean) | 13.06 | 13.70 |
| first-candidate correctness (rank0 fraction) | 12.2 % | 12.2 % |
| rounds where all patches are high-valence | **0 / 40** | **4 / 40** (117,216 of 271,338 color symbols = 43.2 %) |
| full paper-Z eligible rounds (all patches compliant) | **0** | **0** |
| ambiguity side lower bound (if explicit side for all high-valence patches) | 3.69 bits/patch ≈ **4,958 B** | 3.75 bits/patch ≈ **4,966 B** |

Key readings:
- **The current strict elimination produces no paper-Z-compliant patches at all**; the geometric
  Z-matcher only returns a pair for ~30 % of high-valence patches, and it never matches the actual
  zero-pair. The first-candidate rule is correct only 12.2 % of the time (v7 side dumps had the same
  order of ambiguity) — so route C's implicit derivation would need a full elimination-time
  Z-shape/alternating rewrite, not a one-build change.
- Under the strict schedule there is **no round whose patches are all high-valence**, so C1 cannot
  even start without the PHASE-style re-scheduling (which changes removed count to 19,683 and is
  accepted as ≈canonical). PHASE=4 gives 4 such rounds (43.2 % of color symbols) but still **0**
  compliant/eligible rounds today.

## 3. Projection (hypothetical Phase B) and verdict

Even assuming Phase B could (a) enforce 100 % paper-Z compliance and (b) make the patch set
implicit for those 4 all-high-valence rounds, removing their color bits + side gives:

    PHASE=4 base 73,635 B
    − color of the 4 all-high rounds (24,145 × 117,216/271,338)  ≈ −10,430 B
    − side for those rounds (B-lite)                              ≈ −0.4 KB
    + residual schedule delta                                    ≈ +0.3 KB
    ≈ 68,093 B / 27.44 bpv   (margin only ~92 B before any enforcement/mask/membership cost)

That is **not credible** (92 B margin) and it requires a multi-build elimination rewrite
(Z-shape/alternating invariant) plus an unproven decoder-side implicit patch-set derivation.
For the strict schedule the projection saves 0 (no all-high-valence rounds).

**Verdict: BLOCKER — compliance rate is 0 %, the hypothetical lower bound is only 68,093 B
(92 B margin) and requires ≥3 builds. Per Lead stop condition, Phase B is NOT started.**
PCATM stays **FAIL 28.834819 / 1.3644x**; matrix unchanged; no PASS claim.

## 4. Evidence
- `impl/PCATM/PCATM_TASK46_PHASEA_STATS.csv` (80 per-round rows: strict + PHASE=4: comps,
  highValence, zOk, zMatchActual, candidate/rank stats, color symbols, eligibility flags)
- `impl/PCATM/PCATM_TASK46_DESIGN` context: `PCATM_TASK45_DESIGN.md` (route C multi-build bound)
- run dirs: `/tmp/task46_strict/`, `/tmp/task46_strict_nostats/`, `/tmp/task46_phase4/`;
  stats CSVs `/tmp/task46_stats_{strict,phase4}.csv`; build log `/tmp/t46_build.log`
- build accounting: 1 isolated configure + 1 build of `PMSFCompress` (instrumented); no decoder
  build was needed; **no further build** was run.
