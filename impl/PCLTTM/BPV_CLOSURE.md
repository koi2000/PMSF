# PCLTTM faithful horse19k q12 bpv closure (M1, WS-B)

Status: **hard cap PASS**, ideal (1.2x) not reached.

| item | value |
|---|---|
| faithful path | PCLTTM tag (`BlobLayout=CODEC_016` sv=3), no variant, no q change |
| model / q | `datasets/PaperModels/horse19k.off`, 12-bit |
| rounds | 44 |
| artifact bytes | **66,755** (`art_compact.bin` 832 B + `art_residual.parquet` 65,923 B) |
| bpv | **26.902423** |
| paper reference | 20.87 bpv (A&D horse19k, RD paper Table 2/3) |
| ratio | **1.289x** (hard cap 1.3x = 27.131 -> PASS; ideal 1.2x = 25.044 -> FAIL) |
| decode | counts_ok / connectivity_ok / topology_ok / nearest_geometry_ok / coordinate_remap_complete / manifold_profile_ok **all true** |
| quality | surface_rms_max = 9.23419108926e-06; surface_mse_sym = 8.52539657928e-11 (unchanged vs old canonical: transport lossless) |

## What was changed

No algorithm change and **no default-path byte change**: the wins are two
env-gated, lossless transport improvements plus rounds tuning.

1. `rounds=44` (sweep 35..100, flat minimum 44-46; round 50 = 27.469, 44 = 27.472
   with old transport).
2. `PMSF_BASE_EDGEBREAKER_NATIVE=1` — in-process EdgeBreaker encode of the quantized
   base mesh (symmetric decoder support exists; decode verified exact). Base blob:
   `art_compact.bin` 1600 -> 832 B at rounds=44.
3. `PMSF_PARQUET_ZSTD_LEVEL=22` — ZSTD-22 for the residual Parquet payload
   (65,923 vs 66,569 B, -646 B). Same schema/container.
4. Residual blob components at best point (diagnostics): `valence=4,070`,
   `fusedFaceSym=15,497`, `residual=45,162`, `containerOverhead=548`, total blob 65,277 B;
   symbols: fusedSyms=326,632, fusedActive=19,700, valenceSyms=19,700, residualCoords=59,100.

## Reproduce

```bash
impl/PCLTTM/pclttm_best_bpv_roundtrip.sh
# -> artifact_bytes=66755 bpv=26.902423 ratio=1.289048
#    [PCLTTMTag bpv sv=3] valence=4070 fusedFaceSym=15497 residual=45162 containerOverhead=548
#    counts_ok=true connectivity_ok=true topology_ok=true nearest_geometry_ok=true
```

Raw measurements/logs: `/tmp/wsb/runs/pclttm_q12_r44_eb_z22/` and
`/tmp/wsb/runs/results.csv`; this directory's `PCLTTM_BPV_EVIDENCE.csv`.

Default-path invariance: with the two envs unset the artifact is byte-identical to
the previous canonical PCLTTM run (rounds 50: 68,160 B / 27.468641).

## Remaining gap to ideal (25.044 bpv = 62,124 B, -4,631 B)

Not attempted in this pass. Highest-leverage in-scope options (impl/PCLTTM only):

- global (cross-round) range-coder models for `fusedFaceSym` / residual bitplanes:
  the current format resets RC models and repeats 4-byte count headers every round
  (44 rounds -> ~548 B stream table + ~528 B count headers + per-round adaptation
  loss). Expected 1-3 KB.
- `fusedFaceSym` context model: 326,632 symbols, hard-coded 4-symbol adaptive model
  with no context; static entropy lower bound is ~13.3 KB vs 15.5 KB actual
  (~1-2 KB headroom).
- residual bitplane model persistence across rounds (largest single stream: 45.2 KB;
  it is currently the shared RD `encodeResidualBitplane` called once per round, so a
  PCLTTM-local persistent-model variant is needed for symmetry).

Each of these is a codec change inside `impl/PCLTTM/{compress,decompress}` and must
stay bit-symmetric; verify with MeshCompare + SurfaceMetrics.
