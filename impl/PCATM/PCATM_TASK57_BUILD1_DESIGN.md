# PCATM task-57 ≥3-build decoder-reconstruction — Build1 feasibility audit (options A/B/C):
# **gate-inconsistent as scoped → FINAL BLOCKER recommendation (no build spent)**

Date: 2026-10-22. Owner: algo-bpv. **No build**; no live/M4/M5 changes. PCATM baseline:
strict 71,550 B / 28.834819 bpv; hard cap 68,185 B / 27.48; color stream = 22,272 B
(231,322 live-face symbols = 0.772 bits/symbol); residual 47,298; overhead 1,980.

## 1. Build1 option A (explicit root mask + child/zero identity, colour retained as cross-check)

Build1 gate: full 40 rounds, replay-match 100 %, decode 7/7, **bytes ≤75 KB**, colour only cross-check.

**Byte-budget contradiction (provable, not an estimate):** if the colour stream is retained for
cross-checking (Build1 spec), the artifact already carries 71,550 B; the new explicit identity stream
must therefore be **≤75,000 − 71,550 = 3,450 B** for the gate to pass. But:
- root mask alone over 19,694 patches = 19,694 bits = **2,463 B** (incompressible-ish selector bits);
- child/zero identity for 19,694 patches = at absolute minimum ≥1 bit/patch for presence plus the actual
  id/range coding: even a *1 bit/patch* lower bound adds 2,463 B → total ≥4,926 B, and any real child/zero
  coding (face ids/ranges/deltas) costs several bits/patch (raw estimate: child range [root delta,
  first-child delta, valence≈4 b] ≈20 b/patch = 49 KB; zero ids 2×≈12 b = 59 KB; raw ≈108 KB, LZ ≈40 KB).
  **Total identity stream ≥4,926 B > 3,450 B budget** → option A cannot pass the Build1 byte gate with
  colour retained. (Even the root-mask-only minimum 2.46 KB plus one extra bit/patch already exceeds it.)
- If instead Build1 *replaces* colour with identity (colour no longer fully retained), the identity stream
  budget becomes ≈22,272 + 3,450 = **25.7 KB** and the raw ≈108 KB must compress ~4.2×; the existing
  colour stream already encodes the same partition at 0.772 bits/symbol (22.3 KB), so an id-based
  encoding cannot be expected to come in below it — the partition information is not redundant.

## 2. Build1 option B (encoder forces deterministic Z-shape/alternation, decoder reconstructs)

- Retriangulation changes the decoded triangle set (fan diagonals → Z diagonals), so
  `MeshCompare(original, decoded)` **connectivity/topology fail by construction** (proved in task-50
  B3 design; B2b showed ordering attempts change nothing: fixes=0, artifact sha identical).
- Keeping the original triangles means the patch is already fan-shaped, not Z; forcing Z requires
  changing the mesh → decoded mesh ≠ original → decode 7/7 gate impossible.
- Therefore option B is gate-invalid unless the fidelity gate itself is redefined (not allowed here).

## 3. Why this is the same wall as tasks 45–56

The colour stream is not a membership hint; it is the **only transport of the partition/diagonal
structure** the decoder consumes (`recoverPatch` → `patchFaceSet` → `collectPatchTreeDiagonals` →
per-round `BasicReconstructOperator` merge/insert; verified with frozen decoder diagnostics
`/tmp/t49_dec_diag/d.log`: colours==faces per round 310→350→360…). Removing or duplicating it costs
at least as many bytes as it saves at the current 0.772 b/symbol efficiency; explicit identity is
strictly worse (≥4.9 KB minimum vs 3.45 KB Build1 headroom).

## 4. What *could* reach ≤68,185 (different project, not this plan)

Only a **full paper-style self-describing elimination** could avoid transmitting membership at all:
elimination enforces Z/alternating patches so the decoder rebuilds every patch from coarse ring +
residual with no colour/identity stream (ideal estimate ≈51–52 KB / 20.7–21.0 bpv). That project:
1. rewrites the elimination operator to produce paper-Z patches (changing the simplified mesh ordering),
2. rewrites `recoverPatch`/reconstruct to the paper 2-colour replay from scratch,
3. re-validates all golden gates (decode vs original mesh, defaults byte-identity, quality).
It is a new ≥3-build algorithm project with uncertain payoff, **not** an incremental decoder-kernel
change; Build1 here (explicit identity) cannot be a stepping stone because it loses on bytes by
construction.

## 5. Recommendation

**FINAL BLOCKER for task-57 as scoped** (Build1 option A gate-inconsistent: explicit identity ≥4.9 KB
vs ≤3.45 KB headroom with colour retained; option B violates MeshCompare/decode by construction).
Do not spend the Build1 budget. If the sponsor still wants PCATM PASS, open the full paper-Z
elimination/replay project (new scope, new gates, ≥3 builds) instead of this decoder-kernel plan;
otherwise accept PCATM as **FAIL 28.834819 / 1.3644x** (matrix unchanged, phase-1 8/1/1, phase-2
double PASS).

## 6. Evidence
- colour/partition mechanics: `/tmp/t49_dec_diag/{d.json,d.log}` (frozen decoder, no build; per-round
  colours==faces, patches/diagonals counts), `PCATMStrictPMSFSymbolReadOperator.cpp` L2567,
  `BasicReconstructOperator.cpp` L840–935, `OptimizedBasicDeserializeOperator.cpp:1116-1119`.
- byte accounting: strict baseline 71,550 B (color 22,272 / 231,322 symbols = 0.772 b/sym; residual
  47,298; overhead 1,980), measured artifacts `PCATM_TASK42/44/48/49_EVIDENCE.csv`.
- prior audits: `PCATM_TASK45_DESIGN.md`, `PCATM_TASK50_B_AUDIT.md`, `PCATM_TASK51_BUILD1_AUDIT.md`,
  `PCATM_TASK56_BUILD1_RESULT.md`.
- no build was run for this audit; no artifact/format/default changes.
