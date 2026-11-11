# PCATM task-56 Build1 (root-mask fan-replay kernel) — implementation audit result: **FINAL BLOCKER
# (2-build scope insufficient); no build spent**

Date: 2026-10-22. Owner: algo-bpv. **No build run** (Build1 budget preserved); no live/M4/M5 changes.
Task: implement decoder kernel reading per-round root mask + coarse root polygon ring/valence +
residual and **creating fan child faces**, with colour cross-check/fallback; gate replay-match 100 %.

## 1. What the decoder really does (verified by running the existing decoder diagnostics)

Ran the frozen decoder on the canonical strict artifact with `PMSF_BPV_DIAG=1`
(`/tmp/t49_dec_diag/d.log`, rc=0), revealing the actual per-round mechanics:

```
[color-only-start round=0 mode=4 colors=310 faces=310 coloredFaces=24 expected=20]
[color-only round=0] faces=310 patches=20 diagonals=4 coloredFaces=24 recoveredColoredFaces=24
[color-only-start round=1 mode=4 colors=350 faces=350 coloredFaces=14 expected=5]
[color-only round=1] faces=350 patches=5 diagonals=9 coloredFaces=14 recoveredColoredFaces=14
... 40 rounds, rc=0
```

Key facts:
- `colorSymbolsPerRound_` has **one symbol per live face** (`colors==faces`), so the colour stream is
  what defines `facesInOrder`/membership.
- `recoverPatch` consumes `ones`(colour) + zeros → `patchFaceSet`, then computes
  `diagonals = collectPatchTreeDiagonals(root, patchFaceSet)`; it does **not** create faces. It marks
  the root splittable and pushes the root + its diagonal halfedges.
- The actual face creation/merge happens in `BasicReconstructOperator::reconstruct`, which is invoked
  **once per round** (after `collectFromDecoded`, see
  `src/decompress/OptimizedBasicDeserializeOperator.cpp:1116-1119`):
  strict-paper path (`BasicReconstructOperator.cpp` L840–935) does
  `while facet_degree()<polyValence: mergeOneStrictPaperDiagonal(root, diagonals…)` then
  `insert_point(root)` → this is where fan children appear and the face count grows (310→350→360…).
  The merge/insert replay needs the **diagonal halfedge list** that the symbol reader derived from
  `patchFaceSet`, i.e. from colour membership.

## 2. Why root mask + ring/valence + residual cannot supply that list

To create the fan, the decoder must know the patch's internal diagonals (`patchFaceSet`), not just
the root. The root mask selects which coarse face is split; the ring/valence gives k; the residual
gives the removed-vertex position. None of these identifies **which existing faces of the current
mesh are the patch's k−2 children / 2 zeros** — that set is defined by encoder-added halfedges
(`isInternalPatchEdge = isAdded() && paperLevel()==level`), which are not in the artifact and only
become known by consuming colour membership. Measured ambiguity (Phase A/B): 13.06 candidate zero
pairs per high-valence patch, first-candidate correct 12.2 %, geometric matcher reproduces the true
complement 13.4 %, retargeted matcher pairs decode-fail (patch-exactness violated). The B2b
deterministic fan-orderings changed nothing (fixes=0, artifact sha identical).

Therefore the Build1 kernel would need, in addition to mask/ring/residual, a **colour-independent
source for the internal diagonal set** — which is exactly what the colour stream encodes today.
Options and their costs:
1. transmit the diagonal list explicitly: ≈32k–52k diagonal face references ≈ 12 bits each =
   48–78 KB raw (compressed ≈20–30 KB) → replaces the 22,272 B colour stream at **no net saving**,
   plus mask ≈2 KB → total stays ≈71–73 KB > 68,185 (and > hard cap after adding mask).
2. keep colour for ambiguous patches: ambiguous patches exist in every round (B1 compliance
   5.63–34.4 %), so colour is retained nearly everywhere → no saving.
3. rewrite elimination so patches are self-describing (e.g., canonical fan order): this changes the
   decoded mesh ordering/content and requires eliminating the encoder-added diagonal state entirely
   (paper's Z-shape scheme) → the full multi-build mesh rewrite (B audit: ≈600–1250 LOC, ≥3 builds,
   ordering-sensitive), beyond this 2-build task.

## 3. Why Build1 would fail the gate as written

Build1 requires **replay-match = 100 % (mask roots vs colour roots + child sets)** and decode 7/7.
- mask↔root-set matching is testable cheaply (candidate splittable faces order vs recovered roots);
  expected mismatch rate is non-zero because the materialised decode roots are the colour-derived
  `recoveredZeros[0]` faces, while encoder comp roots are components of the pre-reconstruction mesh.
- child-set matching is **not implementable** without the diagonal set (§2): the decoder-new-path
  cannot create the fan children at all; so the gate is unreachable within Build1 by construction.
Per task stop rules (failure ledger, no further build), the 1-build Build1 budget is not spent on an
implementation that cannot pass; the concrete failure mode is already known.

## 4. Verdict / recommendation

**FINAL BLOCKER for the PCATM 2-build decoder-kernel scope.** The colour stream is not a redundant
"membership hint" — it carries the patch internal diagonal/child structure that the current decode
pipeline (symbol reader → per-round reconstruct merge/insert) consumes; removing it requires either
(a) an equivalent explicit diagonal stream whose size negates the saving, or (b) a full elimination/
decoder redesign (≥3 builds, ordering-sensitive, uncertain payoff). No Build1/Build2 build is
requested. **PCATM stays FAIL 28.834819 / 1.3644x**; matrix unchanged; phase-1 8/1/1; phase-2 double
PASS.

## 5. Evidence
- decoder diagnostic run: `/tmp/t49_dec_diag/{d.json,d.log}` (existing frozen binary, no build),
  showing per-round faces/patches/diagonals and 40 rounds rc=0.
- code anchors: `PCATMStrictPMSFSymbolReadOperator.cpp` L2567 `recoverPatch` (no face creation;
  uses colour `ones`+zeros), `src/decompress/OptimizedBasicDeserializeOperator.cpp:1116-1119`
  (reconstruct per round), `src/reconstruct/BasicReconstructOperator.cpp` L840–935
  (mergeOneStrictPaperDiagonal + insert_point).
- prior measured evidence: `PCATM_TASK49_*` (fan ordering fixes=0, sha identical),
  `PCATM_TASK48_*` (retarget 96.4 %, decode FAIL), `PCATM_TASK46_PHASEA_*` (13.4 %, 13 candidates),
  `PCATM_TASK50_B_AUDIT.md`, `PCATM_TASK51_BUILD1_AUDIT.md`.
- no build was run for this report; no artifact/format/default changes.
