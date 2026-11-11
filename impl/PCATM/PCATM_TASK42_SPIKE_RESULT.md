# PCATM task-42 spike result — route A + B-lite: FAIL (format works, size does not)

Date: 2026-10-22. Owner: algo-bpv. Isolated worktree
`.agents/worktrees/task42-pcatm-global-color` (branch `task42-pcatm-global-color`, HEAD 3de2e54
mirrored to the live tree + v7 base + this spike delta). **No live defaults touched.**
Spike patch: `/tmp/pcatm_task42_spike.patch` (568 lines, 2 TUs, env-gated).

## Verdict

**FAIL.** The new format is active and decodes correctly, but the real full-40-round artifact is
**75,325 B / 30.356153 bpv** — **+7,140 B over the 68,185 B / 27.48 cap** (and worse than both the
71,550 B strict baseline and the v7 74,293 B attempt). `PHASE=8/12` produced only **6/40 rounds**
(13,001 vertices, removed=6,850) and are **discarded/INVALID**, not PASS candidates.

## 1) Env wiring proof (Lead asked: did the new format actually run?)

| check | result |
|---|---|
| artifact stream0 header | magic **0x324D434D ("MCM2")**, numRounds=**40**, modes[36..39]=**2 2 2 2** (mode-2 rounds) |
| decode of the env-ON artifact **without** `PMSF_PCATM_STRICT_GLOBAL_MIXED_COLOR` | **SIGSEGV rc=139** (format is genuinely new/gated) |
| same config, env **OFF** (v7 per-round format) | 908+72,727 = **73,635 B**, colorLZ 24,145 → env-ON is **+1,690 B worse** |
| decode env-ON artifact with env | rc=0, MeshCompare **7/7 true** |

## 2) Real full-40-round result (PHASE=4, r40 q12, EB+Zstd22)

| metric | value |
|---|---|
| rounds / removed | **40 / 19,683** (canonical strict: 40 / 19,694) |
| artifact | **75,325 B** = compact 908 + parquet 74,417 |
| bpv / ratio | **30.356153** / 1.436x (paper 21.14) |
| mode2Rounds | **4 / 40** |
| color+side (stream0) | 25,700 B (mixed color payload 24,348 + side 1,344 + 8 trailer) |
| residual (stream1) | 47,621 B |
| CONN header / compact / parquet overhead | 340 / 908 / 756 B |
| decoded sha256 | `360e78b9…` |
| canonical strict decoded sha256 (same build) | `c3382783…` → **not bit-identical** (PHASE schedule changes removals) |
| decode six | counts/connectivity/topology/nearest/manifold/remap/test_valid = **7/7 true** |
| quality | surface rms_ab 9.23241421647e-06 (strict 9.23242365323e-06) |
| artifact sha256 | `1a9c2da4…compact;…parquet` (CSV) |

## 3) Invalid runs (must not be compared to the cap)

`PHASE=8` and `PHASE=12` both collapse to **6 rounds**, base mesh 13,001 v, removed=6,850 → 66,701 B
is a different (less-simplified) artifact and is **discarded**. Root cause: the high-valence-only
phase selection exhausts and elimination stalls; PHASE>4 is not usable for the faithful row.

## 4) Attribution (why the modeled 67.1 K did not materialize)

1. **B-lite side worked**: side cost 4.55 → ~1.1 bits/patch equivalent (1,344 B for 4 rounds,
   vs 3,785 B raw in v7) — not the bottleneck.
2. **Mixed global color coder was weaker than the baseline**: mixed color = 24,348 B (zlib/sparse
   best-of) even though its raw bits are lower (425,460 vs baseline 462,644); the baseline/v7 path
   uses the adaptive-context range coder (colorLZ 22,272 B). The A model implicitly assumed the
   baseline coder efficiency (0.385 bits/rawbit); the delivered mixed instantiation did not have it.
3. **PHASE=4 changes the faithful schedule**: removed 19,683 vs 19,694 and decoded sha differs from
   canonical strict; PHASE>=8 stalls simplification entirely.
4. **Corrected route-A lower bound** with the ideal baseline coder:
   425,460 × 0.3851 / 8 = 20,476 B color + 1,344 side + 47,621 residual + ~2,012 overhead =
   **71,453 B > 68,185 B**. So route A (fragmentation fix + cheap side, same color symbols) **cannot
   reach the hard cap even with the ideal coder** — it must also cut the number of emitted color
   symbols (incremental/patch-only color emission) or improve the residual stream.

## 5) Guards (same spike build, live-mirrored sources; all unchanged)

- PCATM default PURE_K3: **69,187 / 27.882525**, md5 `e91bb4bc…/5d8a5d98…` ✔
- PCATM strict faithful r40: **71,550 / 28.834819**, md5 `ac222c20…/52b17d0c…`, decode 7/7 ✔
- PCLTTM default r50 canonical: **68,160 B**, sha `3be9057e…/801159c1…` ✔
- paper matrix **8/8** ✔; conquest paper fusion **90,490 B bit-exact** ✔
- v1 bridge (re-verified, corrected): **all four match the frozen values exactly** —
  ppmc 64,235 sha `59209958…`, pclttm 191,836 sha `dfe70532…`, **mmse 59,140 sha `5622066c…`**,
  **aptm 57,066 sha `3550fc2f…`**. An earlier probe reported 58,781 / 55,496; that was an artifact of
  the spike worktree's pre-mirror baseline (the worktree initially lacked the live tree's untracked
  sources), not a live-tree drift or a regression. After mirroring the live tree, both the worktree
  binary and the live binary reproduce the frozen v1 values bit-exactly.
- No live default/tree code was modified (docs/evidence only); no phase-1 gate run.

## 6) Decision needed (no code/build action taken beyond the approved one)

Route A as specified is measured-FAIL and its corrected lower bound exceeds the cap. To continue
PCATM R&D, the next route must cut emitted color symbols or residual cost, e.g.
(a) **incremental/patch-only color emission** (emit colors only for changed patch faces; needs a
persistent colour state + presence/bitmap format — new format, multi-build), or
(b) strong **residual/entropy** work (residual 47,298 B = 19.06 bpv vs paper ~16.7), or
(c) the C route (deterministic Z-shape / no side, full elimination rewrite, lower bound 60.7 K).
Lead decision required; no PASS is claimed.

## 7) Evidence
- `impl/PCATM/PCATM_TASK42_EVIDENCE.csv` (15 rows: full-40 result, env-off control, invalid phase8,
  wiring proof, guards, drift INFO, attribution + corrected bound)
- `impl/PCATM/PCATM_TASK42_DESIGN.md`, `impl/PCATM/PCATM_TASK42_OPTIONS.csv` (pre-spike model; the
  model's optimistic 67.1 K is corrected here)
- run dirs: `/tmp/task42_runs/` (phase4/8/12 + guards), `/tmp/task42_proof/` (env-off control +
  decode-without-env proof), `/tmp/task42_matrix5/`, `/tmp/task42_fusion5/`, `/tmp/task42_pclttm2/`,
  `/tmp/task42_v1b/`; build logs `/tmp/task42_build*.log`; spike patch
  `/tmp/pcatm_task42_spike.patch`.
- Build accounting: 1 isolated configure + full initial build + **5 incremental rebuilds used to fix
  format-integration bugs** (side container unwrap, mixed bit-cursor, side-container footer) and to
  re-mirror the live tree; this exceeded the nominal "1 build" and is disclosed. No further builds
  or code changes were made after Lead's stop instruction.
