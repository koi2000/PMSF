# PCATM task-49 Phase B2b — fan→paper-Z ordering retriangulation: **FINAL BLOCKER (compliance 13.40 %)**

Date: 2026-10-22. Owner: algo-bpv. Isolated worktree `.agents/worktrees/task49-pcatm-phaseB2b`
(task-48 derived). **1 configure + 1 build** (`PMSFCompress PMSFDecompress MeshCompare`); no
live/M4/M5 changes; default env off byte-identical.

## Implementation
Env `PMSF_PCATM_ZSHAPE_RETRI=1` + `PMSF_PCATM_ZSHAPE_FAN2Z=1` (plus B1 `..._ENFORCE=1`): for every
high-valence patch whose structural candidates are not paper-Z recovered, the patch's ones run is
fed to the paper matcher in deterministic **fan→Z path orderings** — canonical, reversed, and all
cyclic rotations (len 3..16) — while the **patch-exact complement zero pair is kept unchanged**
(decode-safe: no zero re-targeting, unlike B2). Compliance counts only patches where
`paperMode2Zeros(ordered ones)` returns exactly the true complement pair.

## Measured (full 40 rounds, q12 horse19k, PHASE=4)

| metric | control | FAN2Z run |
|---|---:|---:|
| compliance (matcher == true complement) | 892/6,655 = 13.40 % | **892/6,655 = 13.40 %** |
| per round | — | 321/3,014; 322/2,249; 204/593; 45/799 (10.65 % / 14.32 % / 34.40 % / 5.63 %) |
| ordering fixes (fan2z) | — | **0** |
| non-compliant / no matcher pair | 5,763 | **5,763 (86.60 %)** |
| artifact bytes | 73,635 | **73,635 (byte-identical sha)** |
| color / residual | 24,145 / 47,621 | 24,145 / 47,621 |
| rounds / removed / mode2 | 40 / 19,683 / 4 | 40 / 19,683 / 4 |
| decode | 7/7 true | **7/7 true** (`360e78b9…`, ≠ canonical `c3382783…`) |
| quality | — | MeshCompare 7/7 true |

## Verdict — FINAL BLOCKER (stop condition compliance < 95 %)

Deterministic fan→Z **ordering** of the same ones set does not change the matcher result at all
(fan2z fixes = 0; artifact byte-identical to control), and the true patch complement still does not
match `paperMode2Zeros` for 86.6 % of patches. The B2b hypothesis (re-orderings ≈ retriangulation)
is therefore falsified: making the matcher and the real complement agree requires **physically
changing the patch triangles at elimination** (new fan→Z retriangulation) **and a decoder change to
replay that Z shape** — a multi-build mesh+decoder rewrite, not a single-build collector change.
Per Lead stop condition (compliance < 95 %), this is the **FINAL BLOCKER; no further B2/C build**.

**PCATM stays FAIL 28.834819 / 1.3644x**, matrix unchanged, phase-1 8/1/1, phase-2 double PASS,
no PASS claim.

## Guards (same build; all unchanged PASS)
PCATM PURE_K3 69,187/27.882525 md5 `e91bb4bc…/5d8a5d98…`; strict 71,550/28.834819 md5
`ac222c20…/52b17d0c…` + 7/7; PCLTTM 68,160 sha `3be9057e…/801159c1…`; paper matrix 8/8;
fusion 90,490 bit-exact; v1 four frozen (64,235/191,836/59,140/57,066 with sha).

## Evidence
- `impl/PCATM/PCATM_TASK49_PHASEB2B_EVIDENCE.csv` (16 rows: control, FAN2Z run, 4 round stats,
  9 guards, attribution/blocker)
- `impl/PCATM/PCATM_TASK48_PHASEB2_{EVIDENCE.csv,REPORT.md}` (B2 baseline)
- patch `/tmp/pcatm_task49_phaseB2b_fan2z.patch` (delta over B2; env-gated, 1 TU)
- run dirs `/tmp/task49_runs/{guard_purek3,guard_strict,phase4_control,phase4_enforce}`,
  `/tmp/t49_{pclttm,matrix,fusion,v1}/`; stats `/tmp/task49_zshape_stats.csv`,
  `/tmp/task49_stats_phase4.csv`; build log `/tmp/t49_build.log`
- build accounting: 1 configure + 1 build; no further build (Lead stop condition).
