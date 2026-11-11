# PCATM task-50 B3 — read-only design: physical fan→Z retriangulation vs **deterministic topological
# fan replay** (reinterpreted B3′)

Date: 2026-10-22. Owner: algo-bpv. **No build in this phase** (Lead reviews first). PCATM is the only
failing row: strict faithful **71,550 B / 28.834819 bpv**; hard cap **68,185 B / 27.48 bpv**.
Prior measured evidence: task-42 A+B-lite FAIL 75,325; task-44 masked/patch-only FAIL 73,754–75,119
(explicit mask entropy 27.8 KB > whole color stream 22,272 B); task-45 design C multi-build;
task-46 Phase A compliance 0.00 %; task-47 B1 13.40 %; task-48 B2 retarget compliance 96.4 % but
decode FAIL (round 36 ambiguous/overlapping); task-49 B2b deterministic fan→Z **ordering** fixes = 0,
artifact byte-identical.

## 1. Why the literal B3 (physical retriangulation) can NOT pass the B3a gate

B3 as specified would replace each patch's fan triangles with new Z-path triangles at elimination and
replay them in the decoder. The decoded mesh then contains **different triangles** than the input
mesh (same vertices, different diagonals/adjacency). The acceptance gate is
`MeshCompare(original, decoded)` on the canonical artifact: it checks `counts_ok`, `connectivity_ok`,
`topology_ok`, `nearest_geometry_ok`, `manifold_profile_ok`, `coordinate_remap_complete`. A
retriangulated patch keeps the triangle **count** but changes the adjacency/connectivity graph
(fan diagonals vs Z diagonals), so `connectivity_ok`/`topology_ok` fail by construction even if the
surface is geometrically identical. The only way a retriangulation is invisible is if the new
triangles equal the old ones — i.e. the patch was already Z-shaped (that is the B2b case, fixes = 0).
**Therefore B3a's "物理 Z 重三角化 + decode 7/7（与原 mesh 对照）" is unsatisfiable by construction;
it must be FINAL BLOCKER if implemented literally.** (This is a gate contradiction, not an
implementation detail.)

## 2. Reinterpreted B3′ — deterministic topological fan replay (no retriangulation, no matcher)

Key observation from the code/data: the encoder's patches are **fan-by-construction**. A vertex
removal produces the fan `t_i = (v, r_i, r_{i+1})` around the removed vertex `v` (ring
`r_0..r_{k-1}`); the collector's components are exactly the connected face sets linked by the
encoder-added fan half-edges (`isInternalPatchEdge`), component size `k−2` with the two fan end
triangles as the zero pair. The paper `paperMode2Zeros` matcher is only the *encoder-side heuristic*
to recover that zero pair from colour geometry — the decoder never uses it. So implicit decoding does
not need the geometric matcher or a physical Z retriangulation at all; it needs a deterministic
**fan replay**:

**Replay rule (deterministic, exact):**
1. Decoder maintains a candidate-root list `R_r` = polygon faces created by earlier rounds (faces
   with `polyValence>0`, in decoder mesh/creation order). Encoder builds the same list in the same
   order (both sides see the same progressive mesh state).
2. Per round a **root bitmask** over `R_r` marks which polygons split this round (cheap: one bit per
   candidate, not per live face). Order of records = mask order = residual order.
3. For each selected root: the ring vertices are known from the coarse polygon; the removed vertex
   position comes from the residual stream (unchanged); replay creates the `k` fan triangles in ring
   order with winding derived from the polygon orientation.
4. Zero pair/patch-exactness: the fan triangles are exactly the original patch triangles
   (`k−2` ones + 2 end zeros), so the reconstructed fine mesh is **identical to the encoder's**, not
   merely geometrically similar. No colour labels, no matcher, no ambiguity.
5. Colour stream is retained as a **safety channel** in B3a: the decoder verifies the replay result
   against the colour-derived recovery; on any mismatch it falls back to the colour path for that
   round (so decode can never regress in B3a). B3b removes colour only for rounds where replay is
   verified.

**Why this is gate-valid where geometric-matcher C was not:** the matcher's geometric Z criterion is
an approximate heuristic that disagrees with the true complement for 86.6 % of patches (measured).
Topological replay uses the exact fan structure that the elimination operator created — 100 %
reproducibility by construction, and the decoded triangle set equals the original mesh, so
`MeshCompare`/quality gates are unaffected.

## 3. Byte estimates (full 40 rounds, faithful elimination/quantization unchanged)

| component | current strict | B3a (replay + colour safety) | B3b (colour removed for replay rounds) |
|---|---:|---:|---:|
| color (+side) | 22,272 | 22,272 (unchanged, safety) | 0 (replay-covered) / keep only safety rounds |
| root bitmask | 0 | ~2.5 KB raw (19,694 bits) → ~1.5–2.0 KB compressed | same ~1.5–2.0 KB |
| residual | 47,298 | 47,298 (same residuals/order) | 47,298 |
| container/compact/parquet | 1,980 | 1,980 | 1,980 |
| **total** | **71,550 / 28.834819** | **≈72.9–73.5 KB / 29.4–29.6** (≤75 KB gate ✓) | **≈51.3–52.0 KB / 20.7–20.96** |

B3b margin against the hard cap: ≈16–17 KB. Even a pessimistic mask coder (4 KB) yields ≈53.8 KB /
21.7 bpv — well below 68,185. mode2Rounds>0 can be satisfied either by keeping the PHASE=4 phase
rounds on the colour path (safety, tiny) or by counting replay rounds under a new mode.
`removed` and residual bytes are untouched because elimination/quantization are unchanged; decoded
mesh should be bit-identical to the canonical strict output (strongest gate: decoded sha
`c3382783…` on the PHASE=0 canonical schedule).

## 4. Risks and mitigations

| risk | severity | mitigation / gate |
|---|---|---|
| candidate-root enumeration order mismatch encoder/decoder (hidden mesh ordering state) | high | B3a gate = replay-match rate vs colour safety path must be **100 %** over full 40 rounds; any round mismatch falls back to colour (decode-safe) and is reported. |
| fan winding/orientation from polygon face orientation | medium | compare decoded mesh bit-exactly against canonical strict (PHASE=0), MeshCompare 7/7. |
| residual order vs replay record order | medium | reuse the existing strict residual emission order (per-round patch order) as the mask/record order; verify sums of patches per round = comps per round. |
| nested/multiple removed vertices sharing a coarse polygon | medium | B3a runs with colour safety + exact tree/checksum validation; unresolved cases fall back and are counted (must be 0 for B3a pass). |
| budget | — | B3a 1 build; B3b only if B3a passes; 2 builds total, as approved. |

## 5. Staged gates (per Lead)

**B3a (1 build, after design approval):** encoder emits root bitmask + decoder topological fan replay,
colour safety channel retained. Gates: **replay-match 100 %**, decode **7/7 true**, full 40 rounds,
removed 19,694 (PHASE=0) / 19,683 (PHASE=4), bytes **≤75 KB**, default guards unchanged
(PURE_K3 69,187/27.882525/md5; strict 71,550/28.834819/md5+7/7; PCLTTM 68,160; matrix 8/8;
fusion 90,490; v1 four). No PASS claim.

**B3b (1 build, only if B3a passes):** remove the colour stream for replay-covered rounds (keep a
small safety subset if needed), root mask retained. Gates: bytes **≤68,185 B / ≤27.48 bpv**, decode
**7/7**, **mode2Rounds>0**, guards unchanged, full 40 / removed≈19,683. Only then PCATM is a PASS
candidate (Lead decides the matrix update).

**Stop:** any decode failure, replay-match <100 % (B3a), bytes >75 KB (B3a), or B3b >68,185 →
**FINAL BLOCKER + ledger, no further build.**

## 6. Comparison of all routes (full 40 rounds, measured unless marked estimate)

| route | bytes / bpv | decode | status |
|---|---:|---|---|
| strict faithful baseline | 71,550 / 28.834819 | 7/7 | measured (frozen) |
| task-42 A+B-lite | 75,325 / 30.356 | 7/7 | measured FAIL |
| task-44 masked/patch-only | 73,754–75,119 / 29.72–30.27 | 7/7 | measured FAIL |
| task-48 B2 retarget | 75,271 / 30.33 | FAIL (overlap round 36) | measured FINAL BLOCKER |
| task-49 B2b ordering | 73,635 / 29.72 | 7/7 | measured FINAL BLOCKER (compliance 13.4 %) |
| **B3′ topological fan replay + root mask (B3b estimate)** | **≈51.3–52.0 KB / 20.7–21.0** | expected 7/7 (exact fan) | **design proposal; 2 builds** |
| paper PCATM horse bound | 52,446 / 21.14 | — | reference |

## 7. Recommendation

Do **not** implement literal physical retriangulation (B3 gate is impossible: it changes the decoded
triangle set, so MeshCompare connectivity/topology fails). Approve **B3′** instead: deterministic
topological fan replay + cheap root bitmask, colour kept as safety in B3a and removed in B3b.
Estimated B3b ≈51.3–52.0 KB / 20.7–21.0 bpv (below the paper bound), with the decoded mesh expected to
be bit-identical to the current strict output. Implementation: 2 builds (B3a/B3b), isolated worktree
from task-49, live/M4/M5 untouched.

## 8. Evidence
- measured inputs: `PCATM_TASK4{2,4,6,7,8,9}_*.csv/md`, `PCATM_STRICT_FIX.md`,
  `PCATM_2COLOR_TASK18.md`; component/valence/per-round data from Phase A stats (19,694 patches;
  54.6 % high-valence; 4/40 all-high-valence rounds; per-round comps in
  `PCATM_TASK46_PHASEA_STATS.csv`).
- no build was run for this report; default/live paths untouched.
