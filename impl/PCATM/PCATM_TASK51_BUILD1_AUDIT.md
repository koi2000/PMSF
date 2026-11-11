# PCATM task-51 — Build1 color-independent determinability audit: **pre-build gate analysis →
# FINAL BLOCKER recommendation (no build spent)**

Date: 2026-10-22. Owner: algo-bpv. **No build run** (1-build budget preserved); no live/M4/M5
changes. Task-50 B audit established the kernel rewrite size (≈600–1250 LOC, ≥3 builds). Build1's
purpose is the determinability gate (≥99.9 %); this note shows — from measured data + the defining
predicate of the patch set — that the gate is **unreachable by construction**, so Build1 would only
confirm a foregone FINAL BLOCKER.

## 1. The patch child/zero set is defined by non-stored history state

Encoder definition (measured code): a patch component is the connected face set linked by
`isInternalPatchEdge(h, paperLevel)` = `(h->isAdded() && h->paperLevel()==paperLevel)`; the two zero
faces are the external faces adjacent to the run ends; `patchFaceSet = ones ∪ zeros`.
The **defining predicate is encoder-added diagonal state** (`isAdded()`/`paperLevel()`), which is
created during elimination and is **not stored in the artifact**; the decoder only learns it while
consuming the colour stream (it calls `setAdded()` on recovered diagonals). Before colour is
consumed, the decoder has exactly: root mask + root polygon ring (quantized positions) + valence +
residual + current mesh topology. Nothing in that tuple references the added-diagonal history.

## 2. Measured upper bound on geometric/topological determinability

| measurement (full 40 rounds, real artifacts) | value |
|---|---:|
| B1 paper-Z matcher reproduces the true zero pair (strict / PHASE=4) | **13.40 %** (892/6,655) |
| B2b deterministic fan→Z orderings (canonical/reversed/all rotations) adding a match | **0** (fixes=0, artifact sha identical) |
| candidate zero pairs per high-valence patch (geometry ambiguity) | **13.06** (strict) / 13.70 (PHASE=4) |
| first-candidate correctness | 12.2 % |
| B2 retargeting zero pair to matcher output → decode | **FAIL** (round 36 ambiguous/overlapping; matcher pair ≠ patch-exact complement) |

Interpretation: geometry/topology alone identifies the true patch complement in at most **13.4 %** of
patches (the only measured rate where any colour-free rule reproduced the true set); orderings add
nothing. A decoder rule limited to decoder-available signals can therefore be deterministic for
<13.4 % of patches — three orders of magnitude below the 99.9 % gate. (The matcher is the strongest
colour-free heuristic available in-tree; no other rule is documented in the code history.)

## 3. Why fallback cannot rescue the cap

Two fallback options for the non-determinable ≥86.6 % of patches:
1. **Keep the colour stream for affected rounds.** Ambiguous patches occur in every round
   (per-round compliance 5.63–34.4 %), so colour is retained everywhere → zero saving; total stays
   ≈71,550 B > 68,185.
2. **Explicitly transmit the child/zero face list.** Cheapest granularity is per-face identity:
   `Σ(v−2) = 52,088` child faces + 39,388 zero references; even at 12 bits/face (pool-id delta) this
   is ≈78 KB raw — far over the 16.9 KB fallback room under the cap (and over the 68,185 cap itself).
   A joint source coder below ~2.6 bits/face is not plausible for arbitrary patch sets.
Hybrid side-byte estimates therefore exceed 68,185 B even under optimistic assumptions; the gate's
second condition fails together with determinability.

## 4. Build1 instrumentation (if the sponsor still demands the number)

The proposed instrumentation is implementable (add env-gated stats in `buildPaper4Round`: per round /
per patch, enumerate candidate child sets under a declared decoder-available rule and compare with the
true comp set; count mismatches/ambiguity/side bits). Expected outcome by §2: determinability
≤ ≈13 %, ambiguity ≥ ≈86 %, fallback > cap → **FINAL BLOCKER**. Spending the single Build1 budget cannot
change the verdict; the audit is therefore reported pre-build to preserve the budget for other work.

## 5. Verdict

**FINAL BLOCKER for the PCATM decoder-kernel route.** Determinability gate (≥99.9 %) is unreachable
from decoder-available signals (measured ceiling ≈13.4 %), and any fallback encoding exceeds the
68,185 B cap. No Build1/B2/B3 build is requested; the B kernel rewrite
(≈600–1250 LOC, ≥3 builds, ordering-sensitive) has no credible path to a PASS within budget.
**PCATM stays FAIL 28.834819 / 1.3644x**; matrix unchanged; phase-1 8/1/1; phase-2 double PASS.

## 6. Evidence
- `impl/PCATM/PCATM_TASK50_B_AUDIT.md` (+_OPTIONS), `PCATM_TASK50_B3_DESIGN.md`
- `PCATM_TASK49_PHASEB2B_{EVIDENCE.csv,REPORT.md}` (fixes=0, sha identical)
- `PCATM_TASK48_PHASEB2_{EVIDENCE.csv,REPORT.md}` (retarget 96.4 %, decode FAIL)
- `PCATM_TASK47_PHASEB1_*`, `PCATM_TASK46_PHASEA_{STATS.csv,REPORT.md}` (13.4 %, 13 candidates/patch)
- code anchors: `PCATMStrictPMSFSymbolReadOperator.cpp` L2567 (`recoverPatch`);
  `src/reconstruct/BasicReconstructOperator.cpp` L840–935; `PCATMStrictPMSFSymbolCollectOperator.cpp`
  `isInternalPatchEdge` / comp BFS; stream ids `pmsf::kPaperBlobStreamPCATMStrictPaper*`.
- no build was run for this audit; no artifact/format/default changes.
