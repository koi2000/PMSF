# PCATM 2-color Z-shape recoverability (WS-B task-18) — status: NOT CLOSED, savings quantified

Date: 2026-10-22. Owner: algo-bpv. Scope: `impl/PCATM/**`, `impl/common/strict/**` (all new
behaviour env-gated; default paths byte-identical).

## Bottom line

- Target ≤27.48 (hard cap) NOT reached this pass; current faithful row stays
  **71,550 B / 28.834819 bpv** (r40 q12, decode six booleans true).
- **Measured upper bound**: if mode-2 recovery worked, `colorLZ` drops
  **22,272 -> 15,683 B (-6,589 B ≈ -2.65 bpv)** with all other streams unchanged
  => estimated **~26.2 bpv (hard cap PASS, ideal 25.37 not yet reached)**.
- Remaining blocker is **decoder-side recovery ambiguity**, not the bit budget:
  the paper §4 2-color scheme is only recoverable when (a) every patch is a
  Z-shape triangulation with the alternating invariant enforced during
  decimation and (b) the minimum-angle / oriented-Z matcher can uniquely pick the
  two external zero triangles. Our strict path satisfies neither for 100% of the
  patches.

## What was implemented (all env-gated, default bytes unchanged)

| piece | env | status |
|---|---|---|
| mixed mode-2/4 color stream (symbol codecs; decoder already derives width from per-round modes header) | automatic when mixed | works |
| per-round color streams (1-bit packing for mode-2 rounds) | `PMSF_PCATM_STRICT_FORCE_MODE2_LEVELS=N` (probe only) | works, used for the bound measurement |
| constructive 2-color assignment (each patch: 2 external zeros + connected middle run, no edge-adjacent blues across patches), validated by the decoder's own matcher | `PMSF_PCATM_STRICT_CONSTRUCTIVE_2COLOR=1` | implemented |
| combinatorial ("legacy") zero-pair uniqueness matcher on both sides | `PMSF_PCATM_STRICT_2COLOR_LEGACY_MATCH=1` | implemented; ambiguous in practice |
| phase-limited high-valence selection so 2-color rounds have no valence-3/4 patches (blue-less components) | `PMSF_PCATM_STRICT_2COLOR_PHASE=N` (+ `..._ENFORCE_MIN_VALENCE`) | works: phase levels have only valence-5/6 patches, total removals preserved |
| forced mode-2 upper-bound probe (encode-only, NOT decodable) | `PMSF_PCATM_STRICT_FORCE_MODE2_LEVELS=N` | measurement only |

## Measurements (horse19k, q12, rounds=40)

| config | bytes | bpv | colorLZ | mode2 rounds | decode |
|---|---:|---:|---:|---:|---|
| faithful strict baseline (T5) | 71,550 | 28.834819 | 22,272 | 0/40 | PASS (six true) |
| + phase min-valence 4 (no mode2 yet) | 73,716 | 29.708 | 22,704 | 0/40 | PASS |
| true-Z triangulation + phase + guard | 79,885 | 32.194 | 29,533 | 0/40 | — |
| **forced mode2 upper bound (4 big rounds mode-2, per-round color, residual accounting intentionally invalid)** | 54,268 | 21.870 | **15,683** | 4/40 | not decodable |

Upper-bound correction for the invalid residual accounting:
`71,550 - (22,272 - 15,683) = 64,961 B = 26.18 bpv`.

## Root cause detail

1. **Blue-less patches**: mode-2 recovery is seeded from BLUE middle triangles, so
   every patch needs valence ≥ 5. Natural rounds always contain valence-3/4
   removals (`deg3+deg4 > 0` in 40/40 rounds) => `strictMode2RecoverablePerRound_=0`.
   Phase-limited min-valence fixes this for a prefix of levels (the large,
   last-serialized rounds), at no removal-count loss.
2. **Patch triangulation is not always a Z-path**: for a fraction of patches the
   fan-fallback triangulation's dual graph cannot be split into
   "connected middle run + 2 external zeros" that the matcher accepts
   (constructive search fails at comp ~240/3014 on level 0, ~15/2249 on level 1).
3. **Geometric matcher vs fan triangulation**: `paperMode2Zeros()`'s
   `recoverDirectedPath` walks the paper's exact zigzag alternation; the
   fan-fallback (current strict path) fails it for most patches; the true-Z path
   accepts the first ~15 patches but blue-boundary conflicts persist
   (1305 warnings, 48-198 cross-patch blue adjacencies even with the
   `TWO_COLOR_ONLY` guard active) => rounds never become fully recoverable.
4. **Combinatorial uniqueness is ambiguous**: unrelated zero-coloured faces
   adjacent to a blue face produce additional Z-path completions, so the legacy
   "exactly one pair" rule rejects legitimate patches.

## Next step (recommended, ~0.5 day, deterministic, no geometric matcher)

Explicit **side-info disambiguation** inside the (new, env-gated) mode-2 colour
stream:
1. 1 bit/face colour plane (blue = middle run), as the paper scheme.
2. Per patch, 1–2 bits selecting the zero-pair index into a **deterministic
   candidate list** (candidate pairs sorted by quantized centroid; both encoder
   and decoder enumerate the same list from the same blue run). Removes the
   geometric/legacy matcher entirely.
3. Candidate list size > 4 => round falls back to mode-4 (rare).
4. Keep per-round color streams so mode-2 rounds really are 1 bit/face.
Expected: side info ≈ 2 bits × 19,694 patches ≈ 4.9 KB raw (compressible ~3 KB)
vs color saving 6.6 KB => net **-3 to -4 KB => 26.0-26.5 bpv (hard cap PASS)**.
Ideal 25.37 additionally needs ~1 bpv, likely from a residual tweak.

## Reproduce / verify

```bash
# default invariance (byte-identical, re-verified after every edit):
PURE_K3 canonical        -> 69,187 B / 27.882525
faithful strict r40      -> 71,550 B / 28.834819, decode true
# upper-bound probe (encode only):
PMSF_PCATM_USE_STRICT_PAPER=1 PMSF_PCATM_STRICT_ENABLE_MODE2=1 \
PMSF_PCATM_STRICT_CONSTRUCTIVE_2COLOR=1 PMSF_PCATM_STRICT_2COLOR_PHASE=4 \
PMSF_PCATM_STRICT_FORCE_MODE2_LEVELS=4  ./build/PMSFCompress <c.json>
# -> colorLZ 15,683 (vs 22,272)
```

---

# Side-info disambiguation — implementation log (Lead-approved §4)

Status: **implemented, syntax-clean, not yet run** (team build blocked by an unrelated
MMSE WIP TU at the time of writing; waiting for the Lead's build window).

## Design (env: `PMSF_PCATM_STRICT_2COLOR_SIDE=1`)

1. Per-round color streams (replaces the global color stream for this mode). Mode-2
   rounds are 1 bit/face; mode-4 rounds keep 2 bits/face. The decoder layout switches
   to per-round streams only when the same env is set (encoder/decoder agree).
2. Per patch, a **3-bit index** into a deterministic candidate zero-pair list
   (0..7; >7 candidates ⇒ round falls back to mode-4). Indices are appended after the
   self-describing per-round color stream (`"C2SI"` magic, count, packed bytes).
3. Candidate list (`enumerateMode2ZeroPairs(..., requirePath=false)` in
   `impl/PCATM/PCATMPaperMode2.h`): all zero-coloured faces adjacent to the blue run,
   pairs whose union with the run is a **connected triangulated disk** (n-1 shared
   edges), sorted lexicographically by quantized face-centroid keys; key-collision
   groups are dropped. Encoder and decoder enumerate from the same final color map, so
   the list is bit-identical on both sides (only live connectivity + quantized
   positions + colors participate).
4. Encoder assignment relaxes the paper path shape: any connected middle run is
   allowed (the patch dual graph of a triangulated polygon is always a tree), so each
   patch usually has several candidate zero pairs. A greedy assignment with
   no-edge-adjacent-blues runs under **multi-start deterministic orders** (natural,
   reverse, centroid-sorted, plus failing-patch-first rotations); rare conflicts are
   also repaired by reassigning the already-assigned neighbour to an alternative pair.
5. Decoder mode-2 branch consumes one side index per recovered blue component and
   takes the corresponding zero pair instead of running the geometric matcher;
   subsequent tree checks are unchanged.
6. `PCATMPaperEliminateOperator::canTriangulateStrictTwoColorWithoutBlueConflict()`
   bypasses elimination-time blue-conflict filtering for this mode (removals are
   validated later by the collector).

## Verified so far
- `g++ -fsyntax-only` (build flags from `build/CMakeFiles/PMSFCompress.dir/flags.make`):
  collector rc=0, reader rc=0, eliminate operator rc=0.
- Default invariance after the side-mode edits (built 07:50 binary):
  - PURE_K3 canonical: **69,187 B / 27.882525**
  - faithful strict r40: **71,550 B / 28.834819**, decode six booleans true
- First side-mode run (before the relaxed-candidate fix): decode OK, but
  `mode2Rounds=0` — assignment failed (1 structurally valid pair per patch, the rare
  conflict unrecoverable). Relaxed candidates remove that failure; next run must be
  re-measured after the build window.

## Next commands (after “build window clear”)
```bash
flock /tmp/pmsf_build.lock -c 'cmake --build build -j32 --target PMSFCompress PMSFDecompress'
# r40 side-info PHASE=4 sweep, expect mode2Rounds=4, colorLZ ~16-19 KB (3-bit indices),
# decode six booleans true
python3 /tmp/wsb/run_algo.py --algo PCATM --model horse19k --q 12 --rounds 40 \
  --tag t18_side_k4 --env PMSF_PCATM_USE_STRICT_PAPER=1 \
  --env PMSF_PCATM_STRICT_ENABLE_MODE2=1 --env PMSF_PCATM_STRICT_2COLOR_SIDE=1 \
  --env PMSF_PCATM_STRICT_2COLOR_PHASE=4 --env PMSF_PCATM_MODE2_DIAG=1 \
  --drop-extra PMSF_PCATM_USE_PURE_K3 \
  --quant-extra PMSF_DEFAULT_QUANT_BITS,PMSF_PCATM_QUANT_BITS
```
WIP backup patch: `/tmp/pcatm_side_wip_1792626777.patch`.

---

# FINAL STATUS (2026-10-22, Lead-approved stop-loss) — FAIL, not PASS

Per Lead's stop-loss after the allotted build iterations, PCATM §4 2-color Z-shape
recoverability is **NOT closed**. Do not report a PASS.

## Decodable evidence (real artifact, not projection)
- `v2_side_k4c`: r40 q12, PHASE=4, `PMSF_PCATM_STRICT_2COLOR_SIDE=1`
  - **bytes 75,650 / bpv 30.487129**, **decode_ok=true** (counts/connectivity/
    topology/nearest-geometry/test_valid all true), `mode2Rounds=4/40`
  - colorLZ 27,150 of which ~12.3 KB is the uncompressed side stream; color-only
    ≈ 14.85 KB (matches the forced-mode2 probe's 15.7 KB estimate)
- Default invariance re-verified with the same binaries (byte-identical):
  - PURE_K3 r50: **69,187 B / 27.882525** (md5 `e91bb4bc3cb3debdfddd223fe214456e`
    compact, `5d8a5d9827ba8224b3dad91f074f8a2e` residual)
  - faithful strict r40: **71,550 B / 28.834819, decode true** (md5
    `ac222c20299abd2ea2eb2fd2c418eda2` compact, `52b17d0c4e1e1ecdd7b0a7934291cca0`
    residual)

## Projection (NOT a PASS; side-stream coding only)
If the side stream were entropy-packed to ~2–3 KB (2–3 bits/patch + LZ on the
bit-packed records), total color ≈ 17–18 KB → bytes ≈ 66.5–68.2 K →
**bpv ≈ 26.8–27.5** (hard cap 27.48 borderline; ideal 25.37 out of reach without
residual work). This projection is *not* backed by a decodable artifact.

## Root cause of the remaining gap (candidate-table sync)
- Encoder and decoder candidate lists (relaxed `enumerateMode2ZeroPairs`) can differ
  in **count** for individual patches: observed `idx=8 candidates=6` at r39.
- Evidence: candidate keys are identical, but the ordering/count diverges; duplicate
  key-group dropping + per-round face ordering/quantization make the list ordering
  encoder-vs-decoder unstable for a small fraction of patches.
- Fixed-width indices + deterministic count checksum were the last attempted fix; the
  v3 run still desyncs on level 0 (largest round).
- Recommended next step (out of the approved budget): make the candidate list
  **topology-only** (drop position keys entirely; canonical seed = lowest-quantized
  centroid of the blue run with a topology tiebreak) and transport a per-round
  candidate-count checksum with whole-round mode-4 fallback on mismatch.

## Current code state
- All side-info logic is env-gated (`PMSF_PCATM_STRICT_2COLOR_SIDE=1`); with the env
  unset the default artifacts above are byte-identical.
- WIP backup: `/tmp/pcatm_side_wip_final_*.patch` (6 files).

---

# v4 (task-23) — wiring ready, awaiting build slot (build serialized with aptm-rd H3)

Changes (all env-gated behind `PMSF_PCATM_STRICT_2COLOR_SIDE=1`; default paths untouched):
1. **Candidate-table count is now position-independent**: `enumerateMode2ZeroPairs` no
   longer drops key-collision groups (count = number of tree-valid completions, which
   depends only on decoded colors + topology). Order still uses quantized centroids
   (bit-identical across encoder/decoder for a lossless codec) with a deterministic
   sort.
2. **Per-round checksums in the side payload**: after the merged-component list the
   header carries `patchCount` (16 bit) and `candSum` (32 bit). The decoder validates
   `patchCount` against `residualsPerRound_` *before* consuming indices and validates
   `candSum` at the end of the round, throwing a precise "candidate-table checksum
   mismatch" instead of an OOB deep inside the stream.
3. **Whole-round fallback**: if any patch has >16 candidates (4-bit index cap) the
   encoder aborts the side payload and the round is serialized as mode-4 (never emits
   a corrupt stream).
4. **Index width**: adaptive ceil(log2(candidateCount)) bits per patch (≈2–3 bit/patch
   for the observed lists), payload LZ-compressed (`C2S3` container as before).
5. **Diagnostics**: `PMSF_PCATM_SIDE_DUMP=1` dumps per-patch `(i, ones, cands, idx)`
   on both encoder and decoder, so a remaining desync can be pinned to the first
   divergent patch in one extra run.

If the checksum still mismatches after this, the dump diff gives the exact patch and
candidate list to fix in the second and final v4 iteration; otherwise this should
produce a decodable artifact with color ≈17–19 KB and bpv ≈26.9–27.6 (hard-cap
borderline — the honest target of this round).

---

# v4 FINAL (task-23) — FAIL, root cause pinned (side trailer locator off by 23 B)

Build/run done with the last allowed build. Result: **FAIL** (no v5 in this round).

## Numbers
- side v4 (r40 q12, PHASE=4, SIDE=1, EB+z22): **74,987 B / 30.219939 bpv, decode_ok=false**
  (error `mode-2 side patchCount mismatch round=39 header=34404 expected=3014`)
- side v4 without EB: 76,546 B / 30.848219 bpv, decode_ok=false (same error)
- default invariance re-verified with the same binaries (byte-identical):
  - PURE_K3 r50 **69,187 B / 27.882525**; md5 compact `e91bb4bc3cb3debdfddd223fe214456e`, residual `5d8a5d9827ba8224b3dad91f074f8a2e`
  - faithful strict r40 **71,550 B / 28.834819, decode true**; md5 compact `ac222c20299abd2ea2eb2fd2c418eda2`, residual `52b17d0c4e1e1ecdd7b0a7934291cca0`

## Root cause (hex evidence)
- Encoder: `[2c-side-bytes] level=0 bits=9248 payload=1156 side=452 hex=80c0c7210c91d086`
  (side container = 12 B `C2S3` header + 440 B LZ payload).
- Decoder: `[2c-side-parse] stream=3235 lzBytes=2740 tailOff=2760 rest=475 rawBytes=452 plz=463 payload=452 hex=4332533384040000`
- Interpretation: the decoder locates the side trailer by `colorHeader.lzBytes` (field 5) but the
  actual appended side container starts **23 bytes later** than that arithmetic implies
  (`rest=475 = 452 (side) + 23`). Thus the "payload" it inflates is the **side container
  header itself** (`43 32 53 33` = "C2S3", `84 04 00 00` = 1156) → garbage patchCount/candSum.
- The 23 unaccounted bytes are per-round color-stream trailing bytes not described by the
  color header's last field, so **side trailers must not be located from the color header**.

## Fix plan (next round, not executed here)
1. Terminate each round stream with an explicit footer: `[sideLen u32][magic u32]`; decoder
   reads the **last 8 bytes** of the round stream, then side start = `stream.size() - 8 - sideLen`.
   Independent of color candidate layout/length fields.
2. Keep the existing `patchCount`/`candSum` checksums + whole-round mode-4 fallback.
3. Then re-measure: expected color ≈17–19 KB (side ≈2–3 bit/patch + LZ), bytes ≈66.5–68.5 K,
   bpv ≈26.8–27.6 (hard-cap borderline), with decode six-true.

## v4 evidence file
`impl/PCATM/PCATM_V4_EVIDENCE.csv` — v4 FAIL rows（EB+z22 与 no-EB）、默认不变性 guard 行
（PURE_K3 69,187/27.882525、strict 71,550/28.834819/decode true + md5）、以及 v2 可解码/v3 失败
的 reference 行（便于对照）。

---

# v5 (task-24) — first build/run FAIL, TWO concrete bugs found and fixed in-tree (needs one extra build to verify)

## v5 build/run #1 result (the one allowed by the original budget)
- side v5 (r40 q12, PHASE=4, SIDE=1, EB+z22): **74,995 B / 30.223163 bpv, decode_ok=false**
- defaults byte-identical: PURE_K3 69,187/27.882525 (md5 e91bb4bc… / 5d8a5d98…), strict
  71,550/28.834819/decode true (md5 ac222c20… / 52b17d0c…)
- footer locator worked mechanically (`[2c-side-parse] stream=3243 sideLen=475 sideStart=2760`),
  but the decoded payload was still garbage (`hex=4332533384040000`) → revealed bug A.

## Bug A — side container double-wrapped at export
`buildPaper4Round` already stores `mode2Side = encodeMode2SideInfo(payload)` (12 B "C2S3"
container), but the export path called `encodeMode2SideInfo(strictMode2SidePerRound_[er])`
again → outer container whose LZ payload is the *inner container*. Decoder inflates the
outer wrapper and receives the inner "C2S3"+len bytes as "payload" — exactly the observed
`hex=4332533384040000`, hence `patchCount=34404` garbage. This also explains the v4 23-byte
mystery (475 = re-wrapped container size vs 452 inner). **Fix:** export appends the stored
container directly (no second wrap).

## Bug B — 4-bit candidate index cap too small
Levels 1–3 fell back to mode-4 because individual patches had 20–27 candidates (>16 cap)
→ only **1/4 phase rounds** became mode-2 (`colorLZ=25,474`), which cannot reach the hard
cap even with a perfect decoder. **Fix:** cap raised to 32 (5-bit adaptive width).

## Bug C (cost) — candidate lists inflated by foreign-blue zeros
Side-mode enumeration now filters zero candidates that touch a blue face outside the run
(they belong to an adjacent patch). Pure connectivity+colors filter → encoder/decoder agree;
expected to cut per-patch candidate counts to ~1–3 → 1–2 bits/patch, side stream ≈1–2 KB
for all four phase rounds.

## Expected after one verification build
4/4 phase rounds mode-2; color ≈15.7–16.5 KB; side ≈1–2 KB; total color ≈17–18 KB →
bytes ≈66.5–68 K → **bpv ≈26.8–27.4 (hard-cap PASS plausible)**. If the verified run is not
≤27.48 it will be reported as FAIL/borderline honestly (no v6).

---

# v5 (task-24) FINAL — emergency verification run result: FAIL (no v6)

Approved emergency build+run executed (v5 fixes A/B/C in tree).

## Result
- v5 side verify (r40 q12, PHASE=4, SIDE=1, EB+z22):
  **75,778 B / 30.538713 bpv**, decode six booleans **true**, but **mode2Rounds=0/40**
  (`colorLZ=26,245`, `residual=47,621`, `container=340`).
- All 4 phase levels report `assigned=0` in `[2c-side]` (e.g. level0 failComp=3 k=4,
  level1 failComp=3 k=4, level2 failComp=7 k=4, level3 failComp=8 k=4).
- Decode six-true is therefore **mode-4-only** (per-round color streams); the side path was
  never exercised in this run.
- Defaults byte-identical: PURE_K3 69,187/27.882525 (md5 e91bb4bc… / 5d8a5d98…), strict
  71,550/28.834819/decode true (md5 ac222c20… / 52b17d0c…).

## Root cause of this run's failure
- Fix A (no double-wrap) and Fix B (candidate cap 32) are in the tree but **not observable**
  because the side path never gets selected.
- Fix C (foreign-blue zero filter) is **incompatible with the independent blue assignment**:
  the assignment may place a patch's blues next to another patch's zero faces; the filter then
  removes the chosen zero pair from the candidate list on the final color map, so
  `mode2FindPairIndex` returns -1 for all 4 phase levels → whole round falls back to mode-4.

## What is needed next (out of the approved budget; do NOT run v6 this round)
1. Assignment repair pass: after the initial assignment, for each patch whose chosen pair is
   excluded by the filter, search alternative structural candidates (zeros away from foreign
   blues) and update the color map; iterate 2–3 passes, fallback mode-4 only if a patch cannot
   be repaired. Then the filter keeps candidate lists at ~1–3 (1–2 bit/patch side cost).
2. Alternative cheaper fallback: drop the filter (side cost ~5 bit/patch, expected color
   ≈19.5 KB → bytes ≈69.5 K → bpv ≈28.0) — still above the hard cap, so the repair pass is
   the only path to ≤27.48.

## Honest status
- PCATM remains **FAIL (28.834819 faithful)** for this phase; no PASS is claimed.
- v5 WIP patch: `/tmp/pcatm_v5_wip_1792629807.patch`; evidence rows:
  `task24_v5_side_verify` in both CSVs; defaults md5 guards recorded.

---

# v6 (task-25) FINAL — assignment repair did not converge: FAIL (no v7)

Two allowed builds were used. Both runs: **75,778 B / 30.538713 bpv**, decode six-true but
**mode2Rounds=0/40** (mode-4-only; `colorLZ=26,245`, `residual=47,621`, `container=340`).
Defaults byte-identical (PURE_K3 69,187/27.882525 md5 e91bb4bc…/5d8a5d98…; strict 71,550/
28.834819/decode true md5 ac222c20…/52b17d0c…). Hard cap 27.48 **not met**.

## Build 1 — patch-local repair only
`[2c-repair] level=0..3 repairs=0 valid=0`; `[2c-fix] level=0 comp=3 idx=-1 cands=1`.
Failure analysis: at level 0, comp 3 (k=4) has exactly one structural candidate; the
foreign-blue filter removes that pair from its final-map candidate list, and the single
remaining filtered candidate is a **spurious non-patch pair** (one zero is an external face,
so `ones + pair != comp`). With no alternative the round falls back to mode 4.

## Build 2 — repair extended with blocking-neighbour reassignment
For each blocked patch, the foreign blue faces touching its chosen zeros are traced to their
owner patches, whose structural candidates are then searched for one that avoids those zeros.
Result: `repairs=0` again — the blocking owners are themselves single-candidate patches (their
only structural pair is already used), so the search is empty; the 4 phase rounds all fall
back to mode 4 (`mode2Rounds=0`).

## Root cause (final attribution)
The v5 "foreign-blue zero" filter does not make the candidate list equal to the
patch-complement sets: `enumerateMode2ZeroPairs(ones, colors, mesh, requirePath=false)`
enumerates every pair of adjacent zero faces, so it can contain pairs whose union with `ones`
is NOT the removed vertex's patch (external-face zeros). The encoder's genuine pair can be
filtered out while a spurious pair survives, and repair cannot fix a single-candidate patch.
The side format therefore needs the candidate list / zero selection to be **patch-exact by
construction**, not merely filtered, e.g.:
1. Encode the two zeros as indices into the zero-candidate list (2–3 bits each) and validate
   the resulting patch by the decoder's tree check (no pair-index list needed at all), or
2. Restrict candidates to "zeros each adjacent to the run AND the pair's union with the run
   forms a tree whose face count equals valence−2 for the *recovered* residual order", which
   needs the valence to be decoded first (it is derivable from the component tree size).
Option (1) is the cheapest next step: ~4–6 bits/patch before LZ, but with no candidate
ordering dependency; if the budget is 2–3 bits/patch, a range-coded zero index is needed.

## Status
PCATM remains **FAIL (28.834819 faithful)** for Phase 1; no PASS is claimed and no v7 was
executed. Evidence: `PCATM_V5_EVIDENCE.csv` (task25_v6_repair_build1/2 + guards),
`PCATM_STRICT_EVIDENCE.csv` (task25_v6_repair), this section, WIP
`/tmp/pcatm_v6_wip_*.patch`.

---

# v7 (task-36) FINAL — direct zero-index side stream: mode-2 DECODES (4/40) but FAIL on size

Implemented the v6 recommendation option 1: per patch write the two zero faces as indices into a
topology/geometry-ordered candidate list (no pair-index table, no assignment repair), retain the
v5 footer + patchCount/candSum checksums, validate at the decoder with the existing tree check.
Two integration bugs were found and fixed across the 3 approved builds (duplicate mode-2 residual
append -> `mode2Rec=0`; encoder allowed-set was the round's patch-face union while the decoder sees
the whole mesh -> candidate OOB). Final run:

- **74,293 B / 29.940255 bpv** (paper 21.14, 1.4163x) -> **hard cap 27.48 FAIL (+6,108 B)**;
  worse than strict baseline 28.834819 (71,550 B). No PASS claimed.
- **mode2Rounds = 4/40**, **decode six flags ALL TRUE** — first time the 2-color Z-shape mode
  actually round-trips (v2-v6 all had mode2Rounds=0).
- Ledger: color+side 24,799 B (baseline colorLZ 22,272; +2,527), side raw 30,282 bits = 3,785 B
  (4.2 bits/patch), residual 47,621 (+323), container 340, compact 908, parquet 73,385.
- Attribution: 4/40 mode-2 coverage + side-mode per-round color-stream fragmentation (global
  colorLZ -> 40 streams, ~6 KB lost LZ context) + side index cost => net +2,527 B on color+side;
  residual/geometry stream unchanged.
- Defaults unchanged: PURE_K3 69,187/27.882525/md5 e91bb4bc…/5d8a5d98…; strict 71,550/28.834819/
  md5 ac222c20…/52b17d0c… + six true.
- Evidence: `impl/PCATM/PCATM_V7_EVIDENCE.csv`, `impl/PCATM/PCATM_V7_FINAL.md`, run dirs
  `/tmp/pcatm_v7_final_r40/`, `/tmp/pcatm_v7_dbg8/`, guard dirs; WIP `/tmp/pcatm_v7_final_wip_*.patch`.
- Builds used: 3 (2 budgeted + 1 Lead-approved extra); no v8, no PHASE expansion.
