# PCATM task-48 Phase B2 — paper-Z retriangulation attempt: **FINAL BLOCKER (decode fails)**

Date: 2026-10-22. Owner: algo-bpv. Isolated worktree `.agents/worktrees/task48-pcatm-phaseB2`
(task-47 derived). **1 configure + 1 build** (`PMSFCompress PMSFDecompress MeshCompare`); no
live/M4/M5 changes.

## Implementation
Env `PMSF_PCATM_ZSHAPE_RETRI=1` (plus B1 `PMSF_PCATM_ZSHAPE_ENFORCE=1`, default off byte-identical):
for every high-valence patch where no structural candidate is paper-Z matched, the geometric
matcher `paperMode2Zeros(ones, colorMap, bbox, quant)` is re-run on the chosen ones run and its
returned pair is adopted as the assignment's zero pair (if external/distinct from ones). Colour
stream, decoder and format are unchanged (safety channel).

## Measured (full 40 rounds, q12 horse19k, PHASE=4)

| metric | control | RETRI run |
|---|---:|---:|
| compliance (paper-Z reproducible) | 892/6,655 = 13.40 % | **6,415/6,655 = 96.40 %** |
| per round | — | r0 3,011/3,014 (99.90 %), r1 2,182/2,249 (97.02 %), r2 565/593 (95.28 %), r3 657/799 (82.23 %) |
| first-candidate ok | 892 | 892 |
| matcher retarget fixes / fails | — | 5,523 / 240 |
| artifact bytes | 73,635 | **75,271** (compact 908 + parquet 74,363; ≤75 KB stop ✓) |
| color / residual | 24,145 / 47,621 | 25,779 / 47,621 |
| rounds / removed | 40 / 19,683 | 40 / 19,683 |
| decode | 7/7 true | **FAIL** — `[PCATMStrictPaper decoder] mode=2 ambiguous/overlapping patch round=36` |

## Verdict — FINAL BLOCKER (per Lead stop condition "decode 失败 → 不再追加 B2/C")

Compliance 96.4 % (≥95 % success bound) was reached **only by re-targeting the zero pair to the
geometric matcher output**, but that pair is not the patch-exact complement the decoder requires:
its tree/overlap validation rejects the round at 36. In other words the paper geometric matcher and
the actual patch complement differ (240 patches have no matcher pair at all; retargeting the rest
breaks decodability). A real fix requires **changing the ones geometry / triangle retriangulation at
elimination** (re-triangulate fan patches into paper-Z shape while keeping the patch complement
patch-exact), which is beyond this 1-build attempt and has uncertain payoff.

**PCATM stays FAIL 28.834819 / 1.3644x; matrix unchanged; phase-1 8/1/1; phase-2 double PASS; no
PASS claim.** Env-off/default paths byte-identical.

## Guards (same build; all unchanged PASS)
PCATM PURE_K3 69,187 / 27.882525 md5 `e91bb4bc…/5d8a5d98…`; PCATM strict 71,550 / 28.834819
md5 `ac222c20…/52b17d0c…` + 7/7; PCLTTM 68,160 sha `3be9057e…/801159c1…`; paper matrix 8/8;
conquest fusion 90,490 bit-exact; v1 four frozen (64,235/191,836/59,140/57,066 with sha).

## Evidence
- `impl/PCATM/PCATM_TASK48_PHASEB2_EVIDENCE.csv` (16 rows: control, RETRI run, 4 round stats,
  9 guards, attribution/blocker)
- `impl/PCATM/PCATM_TASK47_PHASEB1_{EVIDENCE.csv,REPORT.md}` (B1 baseline)
- patch `/tmp/pcatm_task48_phaseB2_retri.patch` (delta over B1; env-gated, 1 TU)
- run dirs `/tmp/task48_runs/{guard_purek3,guard_strict,phase4_control,phase4_enforce}`,
  `/tmp/t48_{pclttm,matrix,fusion,v1}/`; stats `/tmp/task48_zshape_stats.csv`,
  `/tmp/task48_stats_phase4_retri.csv`; build log `/tmp/t48_build.log`
- build accounting: 1 configure + 1 build; no further build (Lead stop condition).
