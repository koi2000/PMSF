# HelloHook — new algorithm in 45 LOC (0 custom operators)

This directory contains the concrete realisation of the PMSF paper's headline claim:

> **A new mesh-compression algorithm can be reproduced on PMSF in <50 lines of code, with zero algorithm-specific operator subclasses.**

## What's here

- `HelloHookCompress.cpp` — 33 effective LOC
- `HelloHookDecompress.cpp` — 12 effective LOC
- `HelloHookColumnar{Compress,Decompress}.cpp` — one-line `#include` redirects for the Parquet backend target

**Total headline LOC**: 45 (= 33 + 12)
**Custom operators**: 0
**Total `impl/HelloHook/` LOC**: 49

Compare this to the pre-framework landscape: even the simplest existing algorithm has 3-5 supporting header files defining algorithm-specific IsRemovable / Error / Prediction classes.

## How it works

`HelloHookCompress.cpp` declares a pipeline by picking from the framework's
curated hook library:

| Choice | What we picked | Alternatives |
|--------|----------------|--------------|
| Cost functor | `pmsf::EdgeLengthError` | `QuadricError`, `OptimalPlacementQuadricError`, `QuadricWithNormalWeight` |
| Prediction | *(none — identity)* | `ParallelogramPredictor`, `ButterflyPredictor`, `DualParallelogramPredictor`, `DualRingPredictor` |
| Entropy | *(Parquet V2 default zstd)* | `RangeCoderEntropyCoder`, `StaticHuffmanCoder`, `ContextAdaptiveRangeCoder` |
| Codebook | *(none)* | `KMeansCodebook`, `StochasticRelaxationVQCodebook` |
| Features | *(none)* | `CurvatureHysteresisDetector`, `ClusterBoundaryDetector`, `SeamDetector` |

The `pmsf::makeHalfedgeErrorAccessor` and `pmsf::makeOnAfterEliminate` bridges in `src/core/HookBridge.h` convert the hook objects into the `std::function` callbacks SelectOptions expects — no hand-written lambdas.

## Build and run

```bash
MAKE_J=4 scripts/tests/build.sh  # builds all targets incl. HelloHook
./build/HelloHookColumnarCompress datasets/Stanford/bunny.off /tmp/hello
./build/HelloHookColumnarDecompress /tmp/hello /tmp/hello.off
./build/HausdorffDistance datasets/Stanford/bunny.off /tmp/hello.off
```

Expected Hausdorff ≈ 3.15e-05 (roughly equivalent to the APTM baseline on bunny).

## Adapting this as a template for your own paper

1. Copy `HelloHookCompress.cpp` to `impl/<YourAlgo>/<YourAlgo>Compress.cpp`.
2. Swap `pmsf::EdgeLengthError` for whatever cost functor your paper uses
   (subclass `pmsf::ErrorMetric` if it's novel; ~15-30 LOC).
3. Optionally add `.withPredictor(your_predictor)` /
   `.withFeatureDetector(your_detector)` for paper-specific residual or
   anchor logic.
4. Register with CMake: `pmsf_add_algo_pair(<YourAlgo>)`.
5. Run `scripts/loc_report.sh --markdown` to confirm your headline LOC is
   below 50, and `scripts/tests/validate_paper_fidelity.sh --algo <YourAlgo>`
   against the bpv/Hausdorff targets declared in
   `docs/refactor/paper_targets.yaml`.

See `docs/AUTHOR_GUIDE_V3.md` for the full flow with HookBridge usage patterns.
