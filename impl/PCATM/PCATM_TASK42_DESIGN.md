# PCATM structure R&D (task-42) — read-only design & lower-bound report

Date: 2026-10-22. Owner: algo-bpv. Mode: **read-only design, no build yet** (lead reviews first).
Goal: faithful PCATM ≤ **68,185 B / 27.48 bpv** + decode six flags true. Constraints: no
conquest/alias, no q/codebook reduction; default PURE_K3 69,187/27.882525 and strict
71,550/28.834819/md5 unchanged; spike in an isolated worktree, live/M4/M5 untouched.

## 1. Measured inputs (real artifacts, no projections)

| item | value |
|---|---|
| baseline strict faithful (r40 q12) | 71,550 B / 28.834819 bpv; colorLZ 22,272; rawColorBits 462,644; residual 47,298; CONN hdr 340; compact 859; parquet 700 |
| removed patches / valence mix | 19,694 total: deg3 6,203 / deg4 2,737 / deg5 3,610 / deg6 6,474 / degOther 670 → **high-valence (deg≥5) = 10,754 (55%)** |
| v7 side-info (measured, 3 builds) | 74,293 B / 29.940255 bpv; color+side 24,799; residual 47,621; CONN hdr 340; compact 908; parquet 73,385-72,760=625; **mode2Rounds 4/40, decode six true** |
| v7 side bits | 30,282 bits = 3,785 B raw for 6,655 patches = **4.55 bits/patch** (current 2·ceil(log2 n) scheme) |
| measured candidate count n (v7 r40, 6,655 patches) | n=3:1,094; **n=4:4,953**; n=5:397; n=6:114; n=7:56; n=8:26; n=9:11; n=10:3; n=12:1 |
| chosen pair distribution | **(0,1) = 4,141/6,655; (0,2)=653; idx0==0 in 96.9%**; static entropy of the pair symbol = **0.828 bits/patch** |
| global color LZ efficiency | 22,272 B for 462,644 raw bits = **0.3851 bits/rawbit** |
| color per-round face counts (v7 PHASE=4 run) | sum all rounds 271,338; first 4 rounds 117,216 (6,655 patches); first 8 rounds 187,098 (13,243 patches, upper-bound since rounds 4-7 would change under PHASE=8) |

## 2. Route analysis (details in `PCATM_TASK42_OPTIONS.csv`)

### A) Single global mixed-width color stream + global side blob  ← **recommended with B-lite**
v7's failure was not mode-2 logic (that now decodes) but **per-round color streams**: 40
PaperStream streams lost the cross-round LZ context (~+6 KB), so color+side was 24,799 vs baseline
colorLZ 22,272. Fix: one color stream for the whole artifact, self-describing per round:
`[magic/version][numRounds][per-round: mode(1=1-bit mode2 / 2=2-bit mode4), faceCount][concatenated symbols][sideLen][side payload]`,
LZ once. Per-round mode2 rounds write 1 bit/face; mode4 rounds 2 bits/face (exact baseline
semantics for those rounds). Side payload becomes one global blob (per round: patchCount +
pair codes + checksum) instead of per-round appendices.
- Color saving (model = faces_in_1bit_rounds × 1 bit × 0.3851 bits/rawbit):
  PHASE=4 → 5,643 B; PHASE=8 → 9,007 B; PHASE=20 → 12,169 B; all → 13,062 B.
- Side cost, current 4.55 bits/patch (LZ ~0.5 assumption): PHASE=4 1,893 B; PHASE=8 3,766 B.
- **A alone**: PHASE=4 ≈ 68,123 B (27.45 bpv, margin **62 B — too thin**); PHASE=8 ≈ 66,632
  (26.86, margin 1,553) but PHASE>4 schedule is unverified.
- Replayability: fully deterministic (self-describing header + per-round counts/checksums); all
  existing fail-closed checks retained.
- Risk: new color format bugs; PHASE>4 may exhaust high-valence removals; model assumes the
  measured global LZ efficiency holds for mixed-width symbols (conservative: 1-bit rounds are
  sparser, likely *better* efficiency).

### B) < 2 bits/patch side coding  ← **cheap add-on, measured feasible**
Measured pair distribution is extremely skewed ((0,1) 62%, idx0=0 96.9%), so the static entropy
is **0.828 bits/patch** vs current 4.55. Implement rank0-first unary:
rank list = [(0,1), (0,2), (1,2), (0,3), …]; encode rank 0 as `0`, else `1` + gamma/rank bits.
Expected **~1.05-1.1 bits/patch** (measured distribution), decodable without any table.
- Side raw: PHASE=4 915 B (vs 3,785 B); PHASE=8 1,821 B; all 2,707 B.
- **B alone is insufficient** (fragmentation unchanged → 75,208 B); it is a ~2.9-4.5 KB add-on to A.
- Proof of the "≤1.2 bits/patch" number: with A + all-phase coverage the color saving is
  13,062 B; side at 4.55 b/patch costs 5,597 B (LZ) → net 7,465 B; at 1.2 b/patch side costs
  1,476 B → net 11,586 B. So cheap side is *not required* if coverage can reach PHASE≈20, but it
  is required to make the **1-build PHASE=4/8 spike** robust (margin 1.0-3.5 KB vs 62 B/1.5 KB).
- Replayability: trivial; no tables transmitted; decoder derives the same candidate list as v7.
- Risk: low (pure entropy coding of an already-validated integer stream); correctness mirroring
  is easy (symmetric bit writer/reader in the same TU pair).

### C) Deterministic zero rule (no side)  ← paper-faithful ceiling, **not a 1-build spike**
A rule that derives the two zero faces purely from topology/traversal would remove the side
stream entirely (lowest bound: color 11,136 B + residual 47,298 + overhead ≈ **60,737 B / 24.47
bpv**; with paper-level residual 41,356 → 54,472 B / 21.95). But v7 data shows the ambiguity is
structural: 5,257/6,655 phase patches have ≥2 valid candidate pairs; the geometric matcher was
never unique (v2-v6 failures). Making it deterministic needs elimination-time Z-shape/alternating
invariant enforcement (paper §4) = multi-day, multi-build work → keep as the follow-up route if
the A+B spike passes, or as the route to 21-22 bpv.
- Decode replay: only possible if the rule is proven injective; currently unproven → not spike-safe.

### D) Other new formats (lower bounds)
- **D-color-entropy only**: need colorLZ ≤ 18,907 (-15.1%); total would sit exactly at the cap
  (margin 0 B) → not credible alone, needs overhead/residual help.
- **D-residual only**: residual 47,298 → need ≤ 43,933 (-7.1%); residual is 19.06 bpv vs paper
  geometry ~16.7, so the gap exists, but a predictor change risks bit-exact decode; multi-iteration.
- **D-overhead only**: parquet+compact+CONN ≈ 1,980 B; max realistic saving ~780 B → FAIL alone.
- Absolute paper bound: 52,446 B / 21.14 bpv (paper Table 2 horse).

## 3. Recommendation: **spike = A + B-lite** (one build, env-gated, isolated worktree)

Why: A removes the measured v7 fragmentation loss (~6 KB) with a self-describing format change;
B-lite removes the side-stream cost (4.55 → ~1.1 bits/patch) with ~60 lines of symmetric code and a
measured entropy basis. Modeled result **PHASE=4 ≈ 67,145 B / 27.056 bpv (margin ~1,040 B)**,
**PHASE=8 ≈ 64,687 B / 26.07 bpv (margin ~3,500 B)**; both decode six true by construction
(same v7 validation + generation logic, only container/entropy changed).

Spike plan (only after Lead approval):
1. Isolated worktree `git worktree add .agents/worktrees/task42-pcatm-global-color -b task42-pcatm-global-color 3de2e54`,
   apply the v7 base `/tmp/pcatm_v7_final_wip_1792636615.patch` (v7 code + docs, no live change).
2. New env `PMSF_PCATM_STRICT_GLOBAL_MIXED_COLOR=1` (+ existing PHASE env). Files:
   - `impl/common/strict/PCATMStrictPMSFSymbolCollectOperator.{h,cpp}`: build one global
     mixed-width color payload + one global side payload (self-describing header; keep per-round
     patchCount/checksum; mode4 rounds byte-identical semantics).
   - `impl/common/strict/PCATMStrictPMSFSymbolReadOperator.{h,cpp}`: parse the new payload, split
     per-round colors, derive side candidates exactly as v7 (all live faces, geometry-key order,
     tree check), decode rank0-first unary.
   - Default path (env off) untouched.
3. `build_task42/` cmake configure + build only `PMSFCompress PMSFDecompress` (separate from live
   `build/`), run r40 q12 PHASE=4 and PHASE=8 (same binary, env only), plus optionally PHASE=12;
   pick best ≤68,185 B with six flags true.
4. Guards in the same build: PURE_K3 69,187/27.882525/md5, strict 71,550/28.834819/md5, decode six.
5. Deliver: spike patch (`/tmp/pcatm_task42_spike.patch`), evidence CSV row, MD update; live tree
   only gets docs/CSV (no code/default change).

## 4. What would make the spike FAIL (honest stop conditions)
- Mixed global LZ efficiency degrades >10% vs model → PHASE=4/8 margins invert; then try PHASE=12/20
  (same build, env only) before declaring FAIL.
- PHASE>4 changes removal schedule such that 1-bit rounds cannot be formed (high-valence exhausted)
  → fall back to PHASE=4 numbers (marginal 62 B with A-only / 1,040 B with A+B).
- Any decode desync (checksums/tree check) → FAIL; no silent acceptance.
If all variants >68,185 B or decode fails → FAIL + per-item ledger; no PASS claim.

## 5. Evidence
- `impl/PCATM/PCATM_TASK42_OPTIONS.csv` (12 routes: measured baselines, A/B/C/D estimates,
  lower bounds, predicted totals/bpv/decode replay/faithful/risk/verdict).
- v7 measured inputs: `impl/PCATM/PCATM_V7_FINAL.md`, `impl/PCATM/PCATM_V7_EVIDENCE.csv`,
  run dirs `/tmp/pcatm_v7_final_r40/`, `/tmp/pcatm_v7_side_dump/` (pair distribution dump),
  WIP `/tmp/pcatm_v7_final_wip_1792636615.patch`.
- No build was run for this report; live defaults untouched.
