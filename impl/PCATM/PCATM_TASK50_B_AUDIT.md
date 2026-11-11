# PCATM task-50 B — read-only audit: can the strict decoder build fan children
# color-independently from root mask + root polygon + residual? **Recommendation: FINAL BLOCKER**

Date: 2026-10-22. Owner: algo-bpv. **No build** (per Lead); no live/M4/M5 changes.
Scope: answer whether a color-independent replay kernel is small enough to justify the remaining
2-build budget, or whether it is a large mesh-reconstruction rewrite with uncertain payoff.

## 1. Audit answer (short)

**No.** The strict decoder does not create the fan's child triangles from the root polygon + residual;
it **operates on already-existing faces** and defers creation to a separate join/insert replay:
- `PCATMStrictPMSFSymbolReadOperator.cpp` `recoverPatch` (~L2567): for a patch it only
  1. reads `ones` (blue faces) and zeros from the colour stream / side / matcher,
  2. builds `patchFaceSet = ones + zeros`, `root = recoveredZeros[0]`, runs
     `collectPatchTreeDiagonals(root, patchFaceSet)` on **existing face objects**,
  3. marks `root->setSplittable(); setPolyValence(v); setRemovedVertexPosInt(residual);`, sets
     `hd->setAdded()` on the diagonals, and `facets.push_back(root)`.
  There is **no face creation** in this operator, and no path that enumerates patch children without
  the colour-derived `ones` set (`consumed`/overlap/tree checks all key off it).
- `src/reconstruct/BasicReconstructOperator.cpp` strict-paper path (~L840–935): for each root facet
  with `protocolHint()==20` it repeatedly `mergeOneStrictPaperDiagonal(...)` (`join_face` until
  `facet_degree()==polyValence`) and then `insert_point(root)` — this is where the fan geometry is
  finally materialised. It relies on the added diagonals and on the **neighbouring faces already
  present** at that point; ordering matters (`PMSF_RECON_PATCH_SORT` exists precisely to explore
  that sensitivity) and the code carries long invariant comments about historical silent-corruption
  failures (join-face same-face crash, D4/D18 families, folded joins, missing proxies).

Therefore a root bitmask can select *which* coarse face is split, but it cannot tell the decoder
**which existing face objects are that patch's k children / zero pair**, which is exactly what
`recoverPatch` needs before the join/insert replay can run. Removing colour membership requires
replacing that input with a deterministic derivation — i.e., rewriting the patch identity + replay
contract across the symbol reader **and** the reconstruct operator, not adding a mask stream.

## 2. Minimal viable kernel change (if it were attempted)

| piece | file / function | change | LOC (est.) |
|---|---|---|---:|
| new root-mask stream + candidate-root ordering | `PCATMStrictPMSFSymbolCollectOperator.cpp` (+ header), new blob stream id in storage constants | encode per-round candidate-root list (splittable faces in mesh order) + selected bits; decoder side candidate list | 150–250 |
| color-independent patch membership | `PCATMStrictPMSFSymbolReadOperator.cpp` `recoverPatch` + per-round loops | derive child/zero face ids deterministically (geometry/envelope rule) instead of `ones`; new tree/overlap/checksum with fail-closed fallback to colour path | 300–600 |
| replay contract | `src/reconstruct/BasicReconstructOperator.cpp` strict-paper path | make merge/insert replay consume mask-derived membership deterministically; re-validate the D4/D18 invariants; possibly rework `insert_point`/`join_face` ordering | 150–300 |
| validation/guards | tests/scripts | new mode stats, fallback accounting, canonical sha/md5 comparison, all default guards | 50–100 |

Total **≈600–1250 LOC across 3–4 files**, ≥2–3 builds, with the highest risk in the
`BasicReconstructOperator` path (silent bad-mesh failure modes are documented in-tree; MeshCompare
would catch them only after full decode).

## 3. Feasibility / risk / payoff

- **Determinability unknown and likely incomplete:** the colour-free rule would need to reconstruct
  the exact patch face set from root ring geometry; the current data (Phase A/B) shows 86.6 % of
  patches were not geometrically recoverable by the paper matcher, so any geometric rule needs its
  own cross-check and fallback. A fallback (keep colour for ambiguous patches) reduces bytes: the
  B3b estimate 51.3–52.0 KB assumes **100 %** coverage; realistic coverage with fallback is
  unknown and could land anywhere between ≈53 KB and >68 KB.
- **Ordering sensitivity:** patch replay already needed env levers (`PMSF_RECON_PATCH_SORT`) to
  investigate ordering-dependent output; a new membership rule multiplies the ordering surface.
- **Budget mismatch:** the approved B3 scope is 2 builds (B3a/B3b); the audited kernel change is
  ≥3 builds (audit-instrument → kernel replay with colour safety → colour removal). Attempting it in
  2 builds would very likely stop at the B3a gate with no usable artifact.
- **Downside risk:** touching the strict reconstruct path threatens the frozen defaults; all
  default guards (strict md5, PURE_K3, PCLTTM, matrix, fusion, v1) must stay byte-identical, so the
  new path must be fully env-gated and validated — extra build/time beyond the 2 budget.

## 4. Staged plan if Lead nevertheless sponsors a larger budget (not approved, no build)

1. **Build 1 (audit, no artifact change):** instrument the decoder to measure, for every patch,
   whether its child+zero set can be derived from (root id, ring, residual, current mesh) by a
   proposed geometric/envelope rule; report determinability %, mismatch classes, and a projected
   colour-removal byte saving. Gate: determinability ≥99.9 % else FINAL BLOCKER.
2. **Build 2 (kernel + colour safety):** implement membership derivation + replay; colour path kept
   as cross-check/fallback. Gate: replay-match 100 %, decode 7/7, bytes ≤75 KB, defaults unchanged.
3. **Build 3 (colour removal):** drop colour for verified rounds. Gate: ≤68,185 B / ≤27.48 bpv,
   decode 7/7, mode2Rounds>0, defaults unchanged → PASS candidate.

## 5. Recommendation — **FINAL BLOCKER for the B kernel rewrite**

The audited change is a large, ordering-sensitive mesh-reconstruction rewrite (≈600–1250 LOC,
≥3 builds, uncertain determinability/coverage) that exceeds the approved B3 budget (2 builds) and
whose byte payoff is not guaranteed to stay ≤68,185 B. **Recommendation: do not build; accept PCATM
as FAIL 28.834819 / 1.3644x** (matrix already FAIL, phase-1 8/1/1, phase-2 double PASS unchanged).
If the sponsor insists on a PCATM PASS, it needs a dedicated multi-build decoder-reconstruction
project with its own gate budget (Build 1 determinability audit first).

## 6. Evidence
- `impl/PCATM/PCATM_TASK50_B3_DESIGN.md` (B3′ proposal; literal B3 gate contradiction),
  `PCATM_TASK49_PHASEB2B_{EVIDENCE.csv,REPORT.md}` (fan-ordering falsified),
  `PCATM_TASK48_*`, `PCATM_TASK46_PHASEA_REPORT.md`, `PCATM_TASK47_*`
- code audit anchors: `impl/common/strict/PCATMStrictPMSFSymbolReadOperator.cpp` L2567
  (`recoverPatch`), `src/reconstruct/BasicReconstructOperator.cpp` L840–935 (strict-paper
  merge/insert replay), `impl/common/strict/PCATMStrictPMSFSymbolCollectOperator.cpp` L3830+
  (per-round colour/side emission), stream ids in `pmsf::kPaperBlobStreamPCATMStrictPaper*`
- no build was run for this audit; no artifact/format/default changes.
