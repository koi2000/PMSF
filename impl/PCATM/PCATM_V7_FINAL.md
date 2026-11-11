# PCATM v7 (task-36) FINAL — direct zero-index side stream: FAIL on size, but mode-2 now decodes

Date: 2026-10-22. Owner: algo-bpv. Scope: `impl/common/strict/**`, `impl/PCATM/**`, build: 3 windows
(2 budgeted + 1 Lead-approved extra). No v8, no PHASE expansion. Defaults byte-identical.

## Verdict (honest)

**FAIL for the hard cap.** Real artifact = **74,293 B / 29.940255 bpv** (paper 21.14, ratio **1.4163**)
= **+2.460 bpv / +6,108 B over the 27.48 hard cap**; also **worse than the strict baseline**
28.834819 (71,550 B). Ideal 25.37 would need 62,950 B (-11,343 B). **No PASS is claimed.**

**But the v7 mechanism itself works for the first time**: `mode2Rounds = 4/40`,
`decode six flags ALL TRUE`, determinism of the stream content, defaults unchanged.

## What was implemented (env-gated, default paths untouched)

`PMSF_PCATM_STRICT_2COLOR_SIDE=1` (+ `PMSF_PCATM_STRICT_ENABLE_MODE2=1`,
`PMSF_PCATM_STRICT_2COLOR_PHASE=4`) now uses the v7 direct zero-index side stream:

1. Encoder: per patch, enumerate zero candidates **topologically** = faces adjacent (across an
   edge) to the recovered blue run `ones`, excluding `ones`, restricted to (and ordered by) the
   quantized face-centroid key over all live faces; ambiguous centroid keys invalidate the round.
2. Writes the two chosen zero faces as indices (ceil(log2(nCand)) bits each; 4.2 bits/patch over
   the 6,655 phase patches) after the retained v5 header (`hasMerged` + merged records +
   `patchCount u16` + `candSum u32` checksum), LZ-compressed; v5 footer `[sideLen][magic]` kept.
3. Decoder: enumerates the same candidate list from the recovered blue run (whole-mesh face set,
   geometry-key order — no decoded-colour dependence, no candidate-table sync, no pair-index list,
   no assignment repair), maps the two indices to faces, and **accepts the patch only after the
   existing tree check** (`collectPatchTreeDiagonals`: n faces / n-1 diagonals / connected).
   Any desync fails closed; the encoder falls the whole round back to mode 4 when a patch is not
   patch-exact (this is what keeps bad streams out).
4. Removed the v6 assignment-repair machinery (kept out of the byte path) and the duplicated
   mode-2 residual append found during integration.

## Measured (horse19k, q12, r40, EB+Zstd22, PHASE=4)

| metric | v7 side | strict baseline (mode4) | delta |
|---|---:|---:|---:|
| artifact total | **74,293 B** | 71,550 B | +2,743 |
| bpv | **29.940255** | 28.834819 | +1.105 |
| decodable | six flags ALL true | six flags true | — |
| mode2 / mode4 rounds | **4 / 36** | 0 / 40 | +4 mode2 |
| color + side stream | 24,799 B | 22,272 B (global colorLZ) | +2,527 |
| residual stream | 47,621 B | 47,298 B | +323 |
| CONN header/container | 340 B | 340 B | 0 |
| compact bin | 908 B | 859 B | +49 |
| parquet file overhead | 625 B | 700 B | -75 |

Side bits raw = 12,711+10,505+2,543+4,523 = **30,282 bits = 3,785 B** (4.2 bits/patch over 6,655
patches — inside the 4-6 bits/patch budget).

## Attribution (why FAIL)

1. **Coverage**: only 4/40 rounds are mode-2 phase (PHASE=4). The other 36 rounds stay
   2-bit/triangle mode4; one bit/face is only saved on the first four (largest) levels.
2. **Color-stream fragmentation is the dominant loss**: side mode replaces the single global
   colorLZ stream with 40 per-round PaperStream streams (CONN layout 25, S=2: stream0 =
   color+side 24,799 B, stream1 = residual 47,621 B). Per-round mode4 streams sum 15,603 B for 36
   rounds while the baseline global colorLZ is 22,272 B for all 40 rounds — the lost cross-round
   LZ context costs roughly the same as the mode-2 saving.
3. **Side index cost**: 3,785 B raw (LZ-embedded) > the net gain from 4 rounds of 1-bit colors;
   after fragmentation the color+side stream is +2,527 B vs baseline.
4. Residual unchanged in content (+323 B entropy/order only). Geometry/side channels never
   touched.

## Budget / process

- Builds: 3 (v7 build #1, duplicate-residual fix #2, candidate-set fix #3 — #3 explicitly approved
  by Lead). No v8, no PHASE expansion, no storage/M4/M5 edits.
- Defaults after every build: PURE_K3 **69,187 B / 27.882525 / md5 e91bb4bc…/5d8a5d98…**;
  strict faithful **71,550 B / 28.834819 / md5 ac222c20…/52b17d0c…** + six flags true (unchanged).

## Remaining routes (NOT executed; require a new Lead decision)

1. Mode-2 for **all** rounds while keeping the single global color stream (avoid fragmentation) —
   the only route that can plausibly recover the ~9-11 KB needed.
2. Cheaper side coding (range-coded zero index, <2 bits/patch) and/or side channel shared across
   rounds.
3. Residual context/prediction improvement (paper geometry ~15.4 bpv vs our ~19.2) — orthogonal.

## Evidence

- `impl/PCATM/PCATM_V7_EVIDENCE.csv` (measured run + 4 attribution rows + 2 default-guard rows)
- run dirs: `/tmp/pcatm_v7_final_r40/` (final FAIL run: artifacts/logs), `/tmp/pcatm_v7_dbg8/`
  (candidate-set debug), `/tmp/pcatm_v7_guard_purek3c/`, `/tmp/pcatm_v7_guard_strict2/`
- build logs: `/tmp/pcatm_v7_build{1,2,3}.log`; WIP: `/tmp/pcatm_v7_final_wip_*.patch`
- history: `impl/PCATM/PCATM_2COLOR_TASK18.md` §v7

## task-42 spike addendum (2026-10-22)
Route A+B-lite (single global mixed-width color stream + rank0-first unary side, env
`PMSF_PCATM_STRICT_GLOBAL_MIXED_COLOR=1`) was implemented in an isolated worktree and measured.
Full-40-round PHASE=4: **75,325 B / 30.356153 bpv, FAIL (+7,140 B over the 68,185 cap)**;
PHASE=8/12 collapse to 6 rounds (invalid, discarded). The format is verifiably active (MCM2 magic,
numRounds=40, modes 36..39=2; decode without the env segfaults) but the mixed stream's coder is
weaker than the baseline adaptive-ctx-range coder, and PHASE=4 also perturbs the faithful removal
schedule. Corrected route-A lower bound with the ideal coder is 71,453 B > 68,185, so fragmentation
fix + cheap side alone cannot pass. Details: `impl/PCATM/PCATM_TASK42_SPIKE_RESULT.md` +
`impl/PCATM/PCATM_TASK42_EVIDENCE.csv`; defaults/guards unchanged.
