# PCATM task-47 Phase B1 — Z-shape/alternation enforcement attempt: **BLOCKER (compliance 13.4 %)**

Date: 2026-10-22. Owner: algo-bpv. Isolated worktree `.agents/worktrees/task47-pcatm-phaseB1`
(derived from task-46; live tree + v7/task-42/44/46 code). **1 configure + 1 build**
(`PMSFCompress PMSFDecompress MeshCompare`); no live/M4/M5 changes.

## Implementation (env-gated; default byte-identical)

`PMSF_PCATM_ZSHAPE_ENFORCE=1` in the strict 2-colour patch builder: for every high-valence
component the deterministic structural candidates are tried in order; a candidate is accepted only
if the paper geometric matcher `paperMode2Zeros(ones, colorMap, bboxMin, quantStep)` recovers
exactly the selected zero pair; otherwise the first candidate is kept (counted non-compliant).
The colour stream, decoder and format are untouched (safety channel preserved). Defaults unchanged.

## Measured (full 40 rounds, q12 horse19k, PHASE=4)

| metric | value |
|---|---|
| full-40 artifact (env on) | **73,635 B** (compact 908 + parquet 72,727), removed=19,683, rounds=40 |
| artifact sha env-on vs control | **identical** (`df418926…`/`4b8d893a…`) — enforcement could not alter the byte stream |
| patches (phase rounds) | 6,655 |
| paper-Z reproducible / compliant | **892 / 6,655 = 13.40 %** |
| per round compliant | r0 321/3,014 (10.65 %); r1 322/2,249 (14.32 %); r2 204/593 (34.40 %); r3 45/799 (5.63 %) |
| first-candidate matches | 892 (all compliant cases are first-candidate matches; alternates never matched) |
| no matching candidate | 5,763 / 6,655 = **86.60 %** |
| Phase-A metric (`zMatchActual`) | 0 / 10,580 = 0.00 % (unchanged) |
| mode2Rounds | 4 / 40 |
| decode | **7/7 true**; decoded sha `360e78b9…` (≠ canonical `c3382783…`, PHASE=4 schedule) |
| artifact size stop condition | ≤75 KB ✓ (73,635) |

**Root cause (structural, not a bug):** `paperMode2Zeros` derives the zero pair from the geometry of
the `ones` run only — it does not depend on which zero candidates are chosen. Iterating alternative
structural candidates therefore cannot change the matcher output; this is proven by
firstOk==compliant in every round and by the byte-identical env-on artifact. Raising compliance
requires **changing the ones geometry / patch triangulation at elimination** (retriangulate the fan
into the paper Z-shape and/or re-order the patch children), which changes the fine mesh and needs
decoder coordination → Phase B2+ work, outside this B1 task.

## Stop verdict

Per Lead stop condition **compliance < 80 % → BLOCKER**: 13.40 % (and 0 % on the Phase-A metric).
No B2/C plan is proposed as "success"; PCATM stays **FAIL 28.834819 / 1.3644x**, matrix unchanged,
phase-1 8/1/1, no PASS claim. A B2 proposal would have to be a mesh/decoder-level retriangulation
experiment with its own build budget, and current evidence (86.6 % of patches have no matcher-
matching assignment) makes its payoff uncertain.

## Guards (same build; all unchanged PASS)
- PCATM PURE_K3 **69,187 / 27.882525**, md5 `e91bb4bc…/5d8a5d98…`
- PCATM strict faithful **71,550 / 28.834819**, md5 `ac222c20…/52b17d0c…`, decode 7/7
- PCLTTM default **68,160**, sha `3be9057e…/801159c1…`
- paper matrix **8/8**; conquest paper fusion **90,490 B bit-exact**
- v1 four frozen: ppmc 64,235 `59209958…`, pclttm 191,836 `dfe70532…`, mmse 59,140 `5622066c…`,
  aptm 57,066 `3550fc2f…`
- default environments (env OFF) byte-identical; no phase-1 gate run.

## Evidence
- `impl/PCATM/PCATM_TASK47_PHASEB1_EVIDENCE.csv` (18 rows: control, enforce, 4 round stats,
  9 guards, attribution)
- `impl/PCATM/PCATM_TASK46_PHASEA_{STATS.csv,REPORT.md}` (Phase A baseline)
- patch `/tmp/pcatm_task47_phaseB1_zshape.patch` (83 lines, 1 TU, env-gated)
- run dirs `/tmp/task47_runs/{guard_purek3,guard_strict,phase4_control,phase4_enforce}`,
  `/tmp/t47_{pclttm,matrix,fusion,v1}/`; zshape stats `/tmp/task47_zshape_stats.csv`,
  Phase-A stats `/tmp/task47_stats_phase4_enforce.csv`; build log `/tmp/t47_build.log`
- build accounting: 1 configure + 1 build; no further build was run.
