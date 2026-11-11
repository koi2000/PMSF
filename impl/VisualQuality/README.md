# VisualQuality branch

This is a standalone visualization-first compression path. It intentionally does
not use the PMSF operator pipeline or decoder contracts.

Goals:

- export clean progressive OFF meshes for visual inspection;
- avoid overlapping, duplicated, zero-area, and non-manifold display faces;
- keep the algorithms distinct while sharing only display-safety checks.

Algorithm paths:

- `default`: basic patch vertex-removal on a triangle mesh. It uses no QEM,
  no curvature score, and no best-placement optimization; it only keeps the
  topology/convexity guards needed to avoid invalid viewer output.
- `qem`: Garland-Heckbert style quadric edge contraction. It computes summed
  plane quadrics, tries the optimal quadric point first, falls back to
  segment/end-point choices, and rejects normal flips.
- `rd`: Ahn et al. style triangle patch simplification. It removes low-valence
  vertex patches, prefers valence 4-6 before valence-3 cleaning candidates, and
  scores candidates with a curvature-predicted residual instead of QEM.
- `ppmc`: polygon patch decimation following PPMC. It preserves polygon faces:
  removing a center vertex creates a marked n-gon face and does not force a
  global triangulation. If an incident face is already non-triangular, the
  implementation first performs the PPMC corner cut: only the center corner is
  consumed, while the non-center vertex chain is kept as a remainder polygon.
  For visualization, generated n-gons are kept simple and convex; when no such
  patch remains, the PPMC run stops.

The executable is `VisualQualityCompress`.

Example:

```bash
cmake --build build --target VisualQualityCompress
build/VisualQualityCompress \
  --algo=qem \
  --input=datasets/Stanford/bunny.off \
  --output-dir=build/visual_quality_branch/bunny/round_meshes/QEM \
  --rounds=30
```

This branch writes `round_00.off`, `round_01.off`, ... directly. It is meant for
visualization experiments only, not for bitstream or decoder evaluation.

For paper screenshots, run:

```bash
MAKE_J=4 VISUAL_QUALITY_ROUNDS=60 scripts/new/bunny_visual_quality_four_algos.sh
scripts/new/select_visual_quality_paper_levels.sh
```

The selected high/mid/low meshes are written under
`build/visual_quality_branch/bunny/paper_levels`.
