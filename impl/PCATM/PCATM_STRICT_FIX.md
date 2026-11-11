# PCATM strict decoder fix + faithful bpv reality (WS-B, task-5)

Date: 2026-10-22. Owner: algo-bpv. Scope: `impl/common/strict/**`, `impl/PCATM/**`.

## Verdict

- The strict paper decoder **now round-trips correctly** (counts / connectivity /
  topology / nearest_geometry / coordinate_remap / manifold **all true**, quality
  unchanged: surface_rms_max = 9.23242365323e-06).
- But the faithful bpv is **28.8348 bpv** (r40, q12, horse19k, 71,550 B) =
  **1.365x paper 21.14** -> **hard cap 27.48 FAIL** (needs -3,357 B / -4.7%).
- The previously reported **25.558 bpv (1.209x) was an artifact of a broken
  encoder/decoder pair** (see root cause #1): the color stream only carried the
  root triangle of each patch, so it was ~half the correct size and the decoder
  silently rebuilt valence-3 patches. It must not be used as a PCATM row.

## Root cause #1 (the real decoder failure): patches under-grouped by the strict collector

The strict configuration runs `PCATMPaperEliminateOperator` with
`PMSF_PCATM_STRICT_FAN_FALLBACK=1` (the fan triangulation branch). That branch
created the N-2 patch triangles but never stamped the fan diagonals with the
current `paperLevel`, so `isInternalPatchEdge()` in
`PCATMStrictPMSFSymbolCollectOperator::buildPaper4Round` rejected them. Result:
every patch component collapsed to its single root triangle:

```text
before fix (r20, er=19): comp size=1 for 221/221 patches,
  while true polyValence = 3..9 (160x v3, 22x v4, 18x v5, 13x v6, 6x v7, 1x v8, 1x v9)
  -> 61 valence>3 patches == exactly the 61 wrongly inserted vertices observed
     in round-0 reconstruction (face set 2215/2530 common afterwards);
     decoder then rebuilt valence-3 patches -> round-1 color grouping 55 vs 60.
```

Fix (impl/PCATM/elimination/PCATMPaperEliminateOperator.cpp, fan-fallback branch):
mark every internal fan diagonal `setAdded()` + `setPaperLevel(current level)`
(the parent already sets `isAdded()` but never `paperLevel`). After the fix:

```text
after fix (r20, er=19): comp size = polyValence-2 for 221/221 patches
  (160x1, 22x2, 18x3, 13x4, 6x5, 1x6, 1x7)
```

## Root cause #2 (latent codec bug exposed by #1): adaptive-context color codec desync

With full patch-face colors, the encoder's automatic color-codec choice became
codec 8 ("adaptive ctx range"). Its decoder used the *current* symbol to update the
context immediately, while the encoder's `deriveCtx` returns the old context and
folds in `prev` only after coding the step — i.e. the context sequence was shifted
by one, producing garbage colors (decoder saw 1581 nonzero vs encoder 353).

Fix (impl/common/strict/PCATMStrictPMSFSymbolReadOperator.cpp,
`decodeBitsContextRange`): lag-1 context update to match the encoder exactly.
After the fix, codec 7 is auto-selected and decode is exact.

## Measured (q12 horse19k, decode all true)

| rounds | bytes | bpv | ratio vs 21.14 | colorLZ | residual | note |
|---:|---:|---:|---:|---:|---:|---|
| 40 | 71,550 | **28.834819** | 1.365 | 22,272 | 47,298 | **best** |
| 45 | 71,614 | 28.860612 | 1.365 | | | |
| 50 | 71,666 | 28.881568 | 1.366 | 22,435 | 47,664 | |
| 55 | 71,696 | 28.893658 | 1.367 | | | |

Best row config: `rounds=40`, `PMSF_PCATM_USE_STRICT_PAPER=1`,
`PMSF_BASE_EDGEBREAKER_NATIVE=1`, `PMSF_PARQUET_ZSTD_LEVEL=22`; color codec 7
auto; component split at r40: `colorLZ=22,272 (8.97 bpv) + residual=47,298
(19.06 bpv) + container 340`; blob 69,910 + compact 859 + Parquet = 71,550.

Paper PCATM Table 2 horse: connectivity 5.72 bpv + geometry (total 52,446 B =
21.14 bpv). Our color stream alone is 8.97 bpv vs paper connectivity ~5.7-6; the
paper's cheaper stream is the **2-color Z-shape (1 bit/triangle)** scheme, which
our encoder does not produce: `mode2Rounds=0` for all 40 rounds
(`strictMode2RecoverablePerRound_` = 0, blue-boundary conflicts on the fan/Z
triangulation), so every round pays 2-bit-per-triangle 4-color coding.

## Next steps to reach the hard cap (-3,357 B)

1. **Make the 2-color Z-shape mode recoverable** (paper §4): halve the raw color
   bits (462,644 -> ~231k) and roughly halve colorLZ (~22.3KB -> ~11-13KB) =>
   estimated total ~19-20 bpv, i.e. **within both 1.3x and 1.2x**. This needs the
   alternating-invariant/blue-conflict constraint to be enforced during
   elimination (currently `strictMode2RecoverablePerRound_=0`, and the true-Z path
   produces blue boundary conflicts) — dedicated codec task, ~0.5-1 day.
2. Residual stream: 19.06 bpv vs paper geometry ~15.4 (52,446 B total - 11,090 B
   connectivity = 41,356 B = 16.7 bpv excluding base). A PCATM-specific residual
   context/prediction pass could give another ~1-2 bpv.
3. Container: compact 859 B + Parquet overhead ~700 B (~0.16 bpv) — minor.

## Reproduce

```bash
# best faithful row (decode all true, bpv 28.834819):
PMSF_PCATM_USE_STRICT_PAPER=1 PMSF_BASE_EDGEBREAKER_NATIVE=1 \
PMSF_PARQUET_ZSTD_LEVEL=22 PMSF_PCATM_STRICT_QUANT_BITS=12 \
  ./build/PMSFCompress <horse19k q12 rounds=40 config>
  ./build/PMSFDecompress ...
# script: impl/PCATM/pcatm_strict_roundtrip.sh
```

Default-path invariance: `PURE_K3` canonical unchanged (**69,187 B / 27.882525**,
re-verified after every rebuild); the fan-diagonal stamping only executes in the
`PMSF_PCATM_STRICT_FAN_FALLBACK=1` strict branch.

## Evidence

- `/tmp/wsb/runs/pcatm_final_auto8_r40/` (best), `pcatm_final_auto8_r50/`
- `/tmp/wsb/runs/pcatm_fix20*`, `pcatm_truez_r50_*` (mode/codec ablations)
- `impl/PCATM/PCATM_STRICT_EVIDENCE.csv`
