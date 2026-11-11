# QEM faithful E250 audit (WS-B, task-8 → task-16) — **PASS via framework-owned reference-style engine**

Date: 2026-10-22. Owner: algo-bpv (task-8 audit) / qem-repro (task-16 engine port).
Scope: `tools/QEMFaceBudget.cpp` (env-gated reference-style collapse policy) +
measurement; task-16 adds `impl/QEM/QEMRefStyleEngine.h`.

> **Update 2026-10-22 (task-16, qem-repro): the FAIL below is resolved.**
> The framework-owned union-find/face-list engine
> (`impl/QEM/QEMRefStyleEngine.h`, env `PMSF_QEM_REF_ENGINE=1`) produces bunny at
> exactly **250 faces / 127 vertices** with **E250 = 1.31833754428e-06**
> = **0.913 ×** the reference exact250; the gate target is
> **≤ 1.732377043668e-06** ⇒ **PASS**. Default deterministic (`seeded`)
> tie-break; a 5-seed sweep stays 1.318–1.505e-06 (0.913–1.043 ×).
> Port-fidelity mode (`PMSF_QEM_REF_TIEBREAK=random`, reference `rand()&1`
> ordering) reproduces the reference 250-face mesh **byte-for-byte**.
> Legacy default E250 (9.14274932589e-06) and MCGAL ref-style E250
> (2.52627263706e-05) are bit-identical regression guards.
> See "§ task-16 final status" at the end; the original task-8 audit is kept
> below for history.


## Verdict

- Target: `E250 <= 1.2x reference = 1.732e-06` at exactly 250 faces, bunny, framework path.
- Best in-framework path (legacy six-pass `500->300->287->260->258->250`):
  **E250 = 9.14274932589e-06** (reproduced to all digits) = **6.33x reference** -> **FAIL**.
- New reference-style path (this task): single pass `bunny -> 250`:
  **E250 = 2.526e-05**; it is 5.3x better than the legacy single pass (1.34e-04,
  documented) but still 17.5x above reference and worse than the legacy six-pass.
  Hybrid `legacy->500 + ref-style->250`: 1.401e-05. All FAIL.
- Reference `QEMFaithfulPort` (in-tree reference source) stays **1.44364753639e-06**
  and is **not** used as Our.

## What was implemented (tools/QEMFaceBudget.cpp, env `PMSF_QEM_REF_STYLE=1`)

Reference `mesh_simp.cpp`-style collapse policy, add-only behind env (legacy default
path verified byte-identical: six-pass E250 9.14274932589e-06 exactly):

1. **Per-round independent-set protection**: `cannot_delete` is rebuilt every round;
   protected candidates are deferred to the next round instead of dropped
   (the legacy path blocks the one-ring persistently).
2. **Exact per-vertex quadric recomputation** from surviving incident faces for the
   survivor + its one-ring after every collapse (legacy accumulates Q(u)+=Q(v),
   which keeps quadrics of faces that died in the collapse).
3. **Local candidate re-push** for survivor + ring with edge timers
   (stale entries dropped by version).
4. **Face-normal flip penalty**: for every surviving face incident to the collapsed
   edge, old vs new normalized normal dot <= 0.25 => cost 1e5 (reference §flip check).
5. Stall fallback: full candidate rebuild when the link-condition deferral drains
   the heap; give up only if a rebuilt heap also stalls.

## Why it still fails (attribution)

The reference reaches 1.44e-06 with a **standalone union-find/face-list QEM** that
performs plain edge collapses in global priority order and has no topology link
condition. The framework path runs on MCGAL and must use
`Mesh::new_is_collapse_ok`/`new_collapse`; at ~3k faces the link-condition +
half-edge bookkeeping rejects/deferrals change the collapse order (our ref-style
run stalls at 3,098 faces until a forced rebuild, and many low-cost candidates are
unreachable). Residual quadric/flip formulations are equivalent, so the gap is
**topology-engine driven**, not a formula bug:
- legacy single pass (`E250 1.34e-04`) -> ref-style single pass (`2.53e-05`) shows the
  reference policy helps, but
- reference's own union-find engine is another **~17x** better than that.

## Reproduce

```bash
# reference calibration (NOT Our):
scripts/tests/qem_faithful_port_e250.sh        # -> E250 1.44364753639e-06, 250 faces

# legacy six-pass (current best framework/source path):
./build/QEMFaceBudget datasets/Stanford/bunny.off /tmp/qem500.off 500
./build/QEMFaceBudget /tmp/qem500.off /tmp/qem300.off 300
./build/QEMFaceBudget /tmp/qem300.off /tmp/qem287.off 287
./build/QEMFaceBudget /tmp/qem287.off /tmp/qem260.off 260
./build/QEMFaceBudget /tmp/qem260.off /tmp/qem258.off 258
./build/QEMFaceBudget /tmp/qem258.off /tmp/qem250.off 250
python3 scripts/tests/quality_metrics.py --measure datasets/Stanford/bunny.off \
  /tmp/qem250.off --algorithm qem --dataset bunny --quant-bits 12 \
  --surface-samples 5000 --reference --output /tmp/qem250.csv   # E250 9.14274932589e-06

# reference-style (new, env-gated; single pass):
PMSF_QEM_REF_STYLE=1 ./build/QEMFaceBudget datasets/Stanford/bunny.off /tmp/qem_ref250.off 250
# -> 250 faces, E250 2.52627263706e-05
```

## Next step (out of this session's budget)

Port the reference collapse engine itself into the framework as a dedicated
**framework-owned** QEM module (not the in-tree `reference/` source build): own
vertex/face/edge storage with union-find, the reference cost (quadric + flip
penalty), exact quadrics, no MCGAL link condition. That is the only path that can
reproduce 1.44e-06 while remaining "Our" code; estimated 1-2 days, and it also
needs the 250-face snapshot -> progressive artifact wiring the task asks for.

## Evidence

- `/tmp/wsb/qem/{leg250,ref250,ref_single250,hybrid_ref250,hybrid2_ref250}.{off,csv}`
- `impl/QEM/QEM_E250_EVIDENCE.csv`
- tool: `tools/QEMFaceBudget.cpp` (legacy default + `PMSF_QEM_REF_STYLE=1` path)

---

## task-16 final status (2026-10-22, qem-repro) — PASS

### Verdict

| row | faces / vertices | E250 | ratio vs reference | verdict |
|---|---|---|---|---|
| **Our** framework-owned engine, `seeded` default (deterministic) | 250 / 127 | **1.31833754428e-06** | **0.913** | **PASS** |
| Our, `fifo` tie-break | 250 / 127 | 1.31088350229e-06 | 0.908 | PASS |
| Our, `lifo` tie-break | 250 / 127 | 1.3818179277e-06 | 0.957 | PASS |
| Our, reference `rand()` tie-break, full-precision output | 250 / 127 | 1.44927852145e-06 | 1.004 | PASS |
| Our, reference `rand()` tie-break, reference 4-decimal output grid (port-equivalence) | 250 / 127 | 1.44364753639e-06 | 1.000 | PASS |
| reference `QEMFaithfulPort` (in-tree reference source; **NOT Our**) | 250 / 127 | 1.44364753639e-06 | 1.000 | reference only |
| legacy guard: default MCGAL six-pass | 250 / 127 | 9.14274932589e-06 | 6.333 | bit-identical guard |
| legacy guard: `PMSF_QEM_REF_STYLE=1` MCGAL single pass | 250 / 127 | 2.52627263706e-05 | 17.499 | bit-identical guard |

- Gate target: `E250 <= 1.2 × 1.44364753639e-06 = 1.732377043668e-06`.
- Seed sweep (deterministic `seeded`, seeds 1–5):
  1.3183 / 1.3844 / 1.3992 / 1.3213 / 1.5052 e-06 (0.913 / 0.959 / 0.969 /
  0.915 / 1.043 ×) — every sample passes; default seed 1 is not cherry-picked.
- Reproduce: `scripts/tests/qem_ref_engine_e250.sh` (28–30 s, regenerates
  `impl/QEM/QEM_E250_EVIDENCE.csv`, exits non-zero on FAIL).

### What was implemented (framework-owned, no reference dependency)

New module `impl/QEM/QEMRefStyleEngine.h` (header-only, our own source; compiled
into `build/QEMFaceBudget`; it never links, reads or shells out to
`reference/OpenMesh_QEM/`). It reimplements the reference **topology policy** on
its own vertex/face/edge storage:

1. union-find vertex representatives + per-vertex live incident-face lists;
2. **independent-set deferral**: `cannot_delete` rebuilt every round; protected
   candidates are deferred to the next round instead of dropped;
3. **exact quadric recomputation** from surviving incident face planes for the
   survivor + its live one-ring after every collapse (paper Eq. 2/3);
4. **local candidate re-push** for survivor + one-ring with per-pair timers
   (stale heap entries versioned out);
5. **face-flip penalty**: every surviving face incident to the pair compares old
   vs new normalized normal; dot ≤ 0.25 ⇒ cost 1e5 (paper §6);
6. reference cost = unit-plane quadric sum + paper Eq. (1) optimal placement
   (4×4 partial-pivot solve, midpoint fallback when singular);
7. **no MCGAL link condition** — this is the decisive difference from the MCGAL
   path, which stalls at ~3,098 faces (see task-8 audit above).

Wiring is env-gated in `tools/QEMFaceBudget.cpp`:
`PMSF_QEM_REF_ENGINE=1` dispatches to the engine (reads/writes OFF directly);
the default MCGAL legacy path is untouched and its E250 stays bit-identical.
Optional knobs: `PMSF_QEM_REF_TIEBREAK={seeded|fifo|lifo|random}`,
`PMSF_QEM_REF_SEED`, `PMSF_QEM_REF_OUT_DECIMALS`, `PMSF_QEM_REF_T`
(pair threshold t; only needed for paper Fig. 7 style non-edge pairs).

### Decode topology validity

- Snapshot: `build/MeshAudit our_seeded_250.off` ⇒ `valid_triangle_mesh=true`,
  `two_manifold_with_boundary_ok=true`, `closed_two_manifold_ok=true`,
  `non_manifold_edges=0`, `boundary_edges=0`, `zero_area_faces=0`,
  `duplicate_faces=0` (127 v / 250 f).
- Port fidelity: `build/MeshCompare reference_250.off our_random_4dp_250.off
  --tol 1e-4` ⇒ `counts_ok / connectivity_ok / topology_ok /
  nearest_geometry_ok` all `true`; the 250-face OFF is byte-identical to the
  reference harness output. So the engine, not a shortcut, is what reaches the
  reference operating point.
- Decode-topology proxy for the snapshot: default MCGAL path
  `build/QEMFaceBudget our_seeded_250.off framework_roundtrip_250.off 250`
  ingests and re-emits the snapshot (`collapses=0`), and
  `MeshCompare(our_seeded_250, framework_roundtrip_250) --tol 1e-6` reports
  `counts_ok / connectivity_ok / topology_ok / nearest_geometry_ok` all true,
  `MeshAudit` still closed 2-manifold. (The gate script automates this check.)
- Progressive QEM path regression (existing framework path, unchanged):
  `PMSFCompress`/`PMSFDecompress` QEM columnar round-trip on bunny →
  `MeshCompare datasets/Stanford/bunny.off qem_dec.off --tol 1e-2` ⇒
  `counts_ok=true counts=34834/69664`, `connectivity_ok=true`,
  `topology_ok=true`, `nearest_geometry_ok=true`,
  `manifold_profile_ok=true`.

### Regression-guard strength

- Legacy six-pass r250 OFF is **byte-identical** to the pre-task-16 artifact
  (`diff /tmp/wsb/qem/leg250.off build/.../legacy/r250.off` = empty) in addition
  to the exact E250 string match; the MCGAL ref-style row is also exactly
  2.52627263706e-05.
- The only source changes on the legacy path side are: new env-gated branch +
  additive header include in `tools/QEMFaceBudget.cpp`, and the 1-spot
  `HausdorffDistance` BVH fix described below (measurement tool only).

### Tooling fix required by the canonical measurement path

- Fixed a pre-existing latent defect in `tools/HausdorffDistance.cpp`:
  `TriangleBvh::build`'s `std::nth_element` comparator double-indexed
  `tris_[order_[i]]` although the comparator receives triangle ids, which made
  the ordering inconsistent and could overrun the range
  (`__unguarded_partition` segfault on some meshes — including our 250-face
  snapshots). One-line fix (centroid takes a triangle id); verified
  `bunny vs bunny = 0`, byte-identical JSON on a previously non-crashing pair
  (bunny vs reference250), and our 250-face meshes no longer crash.

### Remaining (not blocking the E250 gate)

- **Progressive 250-face artifact wiring (T8 完整项) is NOT done.** The QEM paper
  has no compressed bitstream (no bpv), and the accepted phase-1 row is the
  E250 snapshot, so the numeric gate is closed. A bit-exact progressive
  encode/decode that replays this collapse sequence cannot ride the MCGAL
  pipeline (same link-condition incompatibility that motivated this port); it
  needs a paired progressive encoder/decoder over the independent engine
  (~1–1.5k LOC, 2–3 days) — recommended as a separate follow-up task, and it
  must not be claimed before implemented.
- `PMSF_QEM_REF_T > 0` (paper Fig. 7 pair threshold) uses brute-force non-edge
  pair selection (≤ 8,000 vertices) instead of the reference k-d split; only the
  t = 0 operating point is bit-comparable and it is the reference calibration
  setting.
