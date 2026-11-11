# PCATM task-58 full paper-style self-describing elimination — design handoff + session budget blocker
# (NO build run; no live/M4/M5 changes)

Date: 2026-10-22. Owner: algo-bpv. **This session cannot execute the ≥3-build elimination+decoder
rewrite: its context budget is exhausted** after tasks 42–57 (all evidence below). Per delegation
rules I state the limitation immediately instead of starting a 600–1250 LOC rewrite that cannot be
completed/validated here. This document is the handoff design for a fresh session/agent.

## 0. Status
PCATM remains **FAIL 28.834819 / 1.3644x**; matrix unchanged; phase-1 8/1/1; phase-2 double PASS.
No artifact/format/default changes were made by this task; no build was run.

## 1. Paper-faithfulness clarification (important for the fidelity gate)

Earlier tasks worried that "paper Z-shape/alternating" means retriangulating patches (changing the
decoded triangle set → MeshCompare connectivity/topology fail). **That is not necessary.** In PCATM,
a removed interior vertex `v` produces a **fan patch** around `v`:
`t_i = (v, r_i, r_{i+1})` over the ring `r_0..r_{k-1}`. A fan's dual graph is a **path**, and the
paper's 2-colouring/Z-shape is simply a colouring of that path (the two fan-end triangles are the
"zero" pair adjacent to the polygon boundary, the middle run is "ones"). The **original triangles are
preserved**; no retriangulation is required. Therefore the fidelity gate stays exactly as today:
decoded mesh must match the original (`MeshCompare` 7/7, counts/connectivity/topology/geometry), and
`decode 7/7` is achievable in principle. Any implementation that changes the triangle set must
explicitly declare a **new fidelity gate** to Lead before building — do not silently weaken it.

## 2. What must be rewritten (concrete)

### 2.1 Encoder / elimination patch builder
The current strict collector builds `Paper4PatchComponent` sets via
`isInternalPatchEdge(h, level) = (h->isAdded() && h->paperLevel()==level)` and emits a **2-bit
per-live-face colour map** (compressed to 22,272 B; 231,322 symbols → 0.772 b/sym). Instead:
- Elimination must produce patches in the paper's canonical **fan/alternating form** and record only
  self-describing per-patch data: root polygon identity (which coarse face), valence `k`, and the
  patch order (round, sequence). The two zero faces are **the fan end triangles** (identifiable from
  the polygon ring at decode time); no per-face colour is needed.
- Per round, emit: `patchCount` + a compact **root selector** over the current polygon-candidate list
  (one bit per candidate, ≈19,694 bits total over 40 rounds ≈ 2.5 KB raw) + any valence delta if not
  already on the polygon face.
- Remove the colour stream entirely (`color = 0` in the evidence fields).
- Keep patch-exact/tree/checksum invariants from the current strict path as internal encoder
  round-trip validation (fail-closed).

### 2.2 Decoder replay
- New path: for each round, read the root mask over the deterministic candidate-polygon list; for
  each selected root build the patch from the coarse polygon ring + valence + residual (removed-vertex
  position), then reuse the existing per-round `BasicReconstructOperator` merge/insert replay
  (`src/reconstruct/BasicReconstructOperator.cpp` strict-paper path L840–935) driven by the derived
  fan structure instead of colour-derived `patchFaceSet`.
- Deterministic candidate list: faces with `polyValence>0` in decoder mesh/creation order (encoder
  builds the identical list); deterministic patch order = mask order (identical to current residual
  order, so residual parsing is unchanged).
- Keep fail-closed checks (tree/overlap/checksum) and, if needed in B2, a temporary colour safety
  channel for cross-checking.
- Do **not** change the residual stream, quantization, prediction, base mesh handling.

### 2.3 Fidelity / determinism
- Encoder must preserve the original triangle set (fan patches are the original triangles).
- Decoder replay must be bit-deterministic (same candidate list, same mask order, same residual
  order) and fail-closed on any inconsistency.
- Golden expectation: decoded mesh == canonical strict decoded mesh (sha `c3382783…` on PHASE=0
  canonical schedule) if the elimination order/removals stay identical (removed=19,694). If the
  elimination rewrite changes removal order/rounds, removed≈19,683 (PHASE=4-style) is acceptable per
  Lead but must be reported, not silently.

## 3. Byte estimate and why this route can pass
| component | current strict | paper-style self-describing |
|---|---:|---:|
| colour/membership | 22,272 | **0** |
| root mask (+valence if needed) | 0 | ≈2.5–4 KB raw → ≈1.5–3 KB coded |
| residual | 47,298 | 47,298 (unchanged) |
| container/overhead | 1,980 | 1,980 |
| **total** | **71,550 / 28.834819** | **≈50.8–52.3 KB / 20.5–21.1 bpv** |
=> comfortably below 68,185 (hard cap) and below ideal 25.37; mode2Rounds>0 can be satisfied by
keeping the PHASE=4 round schedule (4 mode-2 rounds) or by reporting replay rounds per new gate.

## 4. Staged builds/gates (for the fresh session)
- **B1 (1 build, encoder only):** elimination/patch-builder rewrite + self-describing artifact format
  (no colour stream); internal encoder round-trip check (every patch replayable from root mask + ring
  + valence + residual, 100%); artifact ≤75 KB; full 40 rounds, removed≈19,683/19,694; no PASS.
- **B2 (1 build, decoder replay):** decoder replay path; decode 7/7 vs original mesh + quality/coord;
  patch-exact/fail-closed/determinism; artifact ≤75 KB.
- **B3 (1 build, final):** target ≤68,185 / ≤27.48 + decode 7/7 + mode2Rounds>0 → PASS candidate
  (Lead decides matrix update). Failure ⇒ ledger, no further build.
- Worktree: derive from `.agents/worktrees/task49-pcatm-phaseB2b` (or task57/task56 trees); do not
  touch live/M4/M5; default guards (PURE_K3 69,187/27.882525, strict 71,550/28.834819/md5, PCLTTM
  68,160, matrix 8/8, fusion 90,490, v1 four) must stay unchanged.

## 5. Key code anchors (for fast resume)
- `impl/common/strict/PCATMStrictPMSFSymbolCollectOperator.cpp`
  - `Paper4PatchComponent`, `isInternalPatchEdge`, comp BFS (~L1007–1750), `buildPaper4Round` colour
    assignment (~L1980–2400), per-round colour emission (~L3830–3990).
- `PCATMStrictPMSFSymbolReadOperator.cpp`
  - `recoverPatch` (~L2567): colour `ones`+zeros → `patchFaceSet` → `collectPatchTreeDiagonals`;
    marks root splittable/polyValence/removedPos; pushes diagonals.
  - per-round colour decode (~L1700–1775); `collectFromDecoded` (~L1867+).
- `src/reconstruct/BasicReconstructOperator.cpp` L840–935: per-round strict-paper
  `mergeOneStrictPaperDiagonal` + `insert_point` replay (invoked from
  `src/decompress/OptimizedBasicDeserializeOperator.cpp:1116-1119`).
- Diagnostics: `PMSF_BPV_DIAG=1` decoder log `/tmp/t49_dec_diag/d.log` (colors==faces per round
  310→350→360…); artifact analysis in `impl/PCATM/PCATM_TASK4*_EVIDENCE.csv`.

## 6. Open risks (must be resolved in B1 design review)
- Deriving the fan face set at decode time: current decode merges existing faces before `insert_point`;
  the new path must either (a) derive merge membership from the polygon ring/fan order (paper
  alternating semantics — the intended route), or (b) restructure replay to build fan triangles
  directly from ring+residual. Choose (a) for fidelity (original triangles preserved); if (a) cannot
  be made deterministic, the paper scheme itself is not implemented faithfully and must not claim 7/7.
- Ordering sensitivity: candidate-polygon list / mask order / residual order must be identical on both
  sides; validate with encoder round-trip in B1 before any decode work.
- Budget: B1 alone is a substantial elimination rewrite; do not start B2/B3 until B1 round-trip passes.

## 7. Evidence / state
- Prior evidence chain: `PCATM_TASK42/44/46/47/48/49/50/51/56/57_*` (all measured or audited; summary:
  colour stream is the only membership carrier; explicit identity costs ≥ colour; forced retriangulation
  breaks MeshCompare).
- No build in this task; no artifact/format/default changes; PCATM matrix row unchanged (FAIL
  28.834819 / 1.3644x).
