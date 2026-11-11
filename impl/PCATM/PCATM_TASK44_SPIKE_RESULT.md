# PCATM task-44 A+ (masked/patch-only color) 1-build spike — FAIL (size), full 40 rounds

Date: 2026-10-22. Owner: algo-bpv. Isolated worktree
`.agents/worktrees/task44-pcatm-patch-color` (derived from the task-42 worktree: live tree mirror +
v7/task-42 code; branch `task44-pcatm-patch-color`). **No live/M4/M5 changes.**
Builds: **1 full build + 1 Lead-approved incremental rebuild** (crash + coder-selection fixes).
Spike patch: `/tmp/pcatm_task44_spike.patch` (delta over the task-42 spike; base
`/tmp/pcatm_task42_spike.patch`).

## Verdict — FAIL (size), decode is correct

A+ implemented as the **explicit masked/patch-only colour stream** (`PMSF_PCATM_STRICT_PATCH_COLOR=1`):
per round, 1-bit mask over the round's live faces (1 = patch colour) + colour value only for
masked faces (mode-4: 2 bits; mode-2: implied 1), one global stream (best of zlib / adaptive-context
range coder, self-describing codec+ctxBits), same v5 side trailer. Decoder rebuilds the identical
`colorByFace` arrays, so the existing colour-map recovery path is unchanged.

All runs are **full 40 rounds** (numRounds=40, removed 19,683–19,698), decode **7/7 true**, but
**every variant is over the 68,185 B cap** — FAIL, no PASS claim, no further build.

| PHASE | rounds/removed | artifact | bpv | mode2 | color_B | side_B | residual_B | compact | parquet | decode | decoded sha |
|---|---|---|---:|---:|---:|---:|---:|---:|---:|---|---|
| 0 (control) | 40 / 19,694 | **75,119** | 30.273135 | 0 | 25,818 | 4 | 47,298 | 859 | 74,260 | 7/7 | `c3382783…` == canonical |
| 1 | 40 / 19,695 | **74,063** | 29.847564 | 1 | 24,473 | 400 | 47,206 | 858 | 73,205 | 7/7 | `ae48eb8a…` ≠ canonical |
| 2 | 40 / 19,698 | **73,754** | 29.723037 | 2 | 23,475 | 924 | 47,392 | 840 | 72,914 | 7/7 | `fcc45a67…` ≠ canonical |
| 4 | 40 / 19,683 | **74,410** | 29.987406 | 4 | 23,459 | 1,344 | 47,621 | 908 | 73,502 | 7/7 | `360e78b9…` ≠ canonical |

Hard cap = 68,185 B: PHASE=0 +6,934 B; PHASE=1 +5,878; PHASE=2 +5,569; PHASE=4 +6,225.
Quality: surface rms_ab 9.23242365323e-06 for all variants (identical to strict). Artifact sha256
per run in `impl/PCATM/PCATM_TASK44_EVIDENCE.csv`.

## Root cause (fundamental, not an integration bug)

The explicit mask itself costs more than the entire stream it replaces:
- mask = 231,322 bits = 28.9 KB raw; density ≈0.39 → binary entropy ≈0.96 b/sym ≈ **27.8 KB** —
  already larger than the baseline colour stream (22,272 B);
- values add 104,210 bits (13.0 KB raw) but compress well; total masked payload was 25,474 B
  (zlib) — the adaptive-context range coder was **never smaller** (`masked_codec=1` in all runs),
  confirming the mask is high-entropy;
- the baseline stream instead stores 2-bit symbol bytes and LZ-compresses long background runs to
  0.385 bits/rawbit (22,272 B for 462,644 raw bits), which an explicit per-face mask cannot beat.
This invalidates the task-44 A+ estimate (which assumed the mask would compress like the symbol
stream); the lower bound with a decoder-derivable (implicit) mask is the route-C question, not an
encoding-coder choice.

## Guards (same spike build; all unchanged PASS)
- PCATM PURE_K3 **69,187 / 27.882525**, md5 `e91bb4bc…/5d8a5d98…`
- PCATM strict faithful r40 **71,550 / 28.834819**, md5 `ac222c20…/52b17d0c…`, 7/7, canonical
  decoded sha `c3382783…`
- PCLTTM default r50 **68,160**, sha `3be9057e…/801159c1…`
- paper matrix **8/8**; conquest paper fusion **90,490 B bit-exact**
- v1 four frozen: ppmc 64,235 `59209958…`, pclttm 191,836 `dfe70532…`, mmse 59,140 `5622066c…`,
  aptm 57,066 `3550fc2f…`
- No phase-1 gate run; PCATM matrix row stays **FAIL 28.834819** (no PASS update).

## What is needed next (not executed; requires Lead decision)
1. **Implicit mask / decoder-derivable patch set (route C baseline)**: make patch faces derivable
   from elimination/2-color invariants so no per-face background bits are sent; this is the only
   way A-style savings materialise (lower bound from task-42 design ≈60.7 KB / 24.47 bpv).
2. Or **per-patch records with decoder-side patch enumeration** (root identity derived from the
   coarse mesh + polygon rings), avoiding both the mask and per-face identities — a larger decoder
   restructure (multi-build).
3. B+ residual/colour entropy specialisation remains a thin-margin fallback (needs residual ≥7 %).

## Build accounting / stop conditions (honest)
- Build #1 succeeded; PHASE≥1 crashed (`globalSideRounds` not resized for patchColorMode) → one-line
  fix; masked coder initially zlib-only → added best-of ctx-range selection. Lead approved exactly
  one incremental rebuild; **no further build was run** after the results above (all > cap).
- All runs are full 40 rounds; PHASE>4/6-round artifacts remain invalid and are not reported.
- Evidence: `impl/PCATM/PCATM_TASK44_EVIDENCE.csv` (15 rows), this file,
  `impl/PCATM/PCATM_TASK44_DESIGN.md` / `_OPTIONS.csv`; run dirs `/tmp/task44_runs/`,
  `/tmp/t44_matrix/`, `/tmp/t44_fusion/`, `/tmp/t44_pclttm/`, `/tmp/t44_v1/`;
  build logs `/tmp/t44_build{,2}.log`; patch `/tmp/pcatm_task44_spike.patch`.
