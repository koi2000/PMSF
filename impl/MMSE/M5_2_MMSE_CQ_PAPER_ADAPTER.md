# M5-2 MMSE half: `mmse_cq` Paper reader pair — interface & storage asks

Status: **code prepared, NOT built** (Lead read-only/build-window discipline, 2026-10-22).
Owner: mmse-repro (task-32 MMSE half; task-32 owner = storage-architect).
Scope of this file: `impl/MMSE/**`, `src/predict/**` — no `src/` change needed for the
MMSE half; the registration seam is the existing `NamedDecompressOperatorRegistrar`
(algorithm-side, same as the columnar triple).

---

## 0. TL;DR

* Decode SIGSEGV root cause is **one missing registration**, not a protocol gap:
  `algorithm=mmse_cq` had Binary/Columnar/Unified decompress triples but **no
  `Backend::Paper` triple**. Paper decode therefore fell through to the framework
  Default paper triple (`OptimizedSymbolReadOperator`) and parsed the layout-15
  CQST blob as an Optimized stream.
* Fixed by `impl/MMSE/decompress/MMSECQuestPaperTripleParquet.cpp`:
  `NamedDecompressOperatorRegistrar("mmse_cq", Backend::Paper, …)` →
  `PaperDeserializeOperator` + the same `MMSECQuestSymbolReadOperator` /
  `MMSECQuestReconstructOperator` / `MMSECQuestGeomState` pair as the columnar path.
  Env escape hatch `PMSF_MMSE_CQ_PAPER_PAIR=0` restores the pre-M5-2 baseline.
  `g++ -fsyntax-only` (exact build flags of the PMSFDecompress target): **rc=0**.
* Payload contract is already satisfied by the current Paper v2 artifact (probe
  `docs/memory/PHASE2_M5_PROBE/mmse/art_paper.parquet`): `algorithm=mmse_cq`,
  `blob_layout=15`, `codec_version=2`, `payload_mode=native`,
  `native_payload.frame=whole`, `payload_bytes=54,567` (raw export blob). The
  storage reader already hands that blob verbatim to the layout-15 dispatch branch,
  which calls `ConquestSymbolReadOperator::setBufferSize` +
  `importFromRangeCodedVector(blob, 0, 0)` — our reader overrides exactly that.
* Remaining gap for the task's **≤54,601 B** target is Paper v2 container overhead,
  not MMSE payload semantics: current artifact = 59,750 B =
  54,630 (payload page) + 2,519 (object RG: JSON header + base) + 1,305 (LOD-row
  columns) + 1,296 (parquet footer/metadata). See §3 ledger / §4 asks.

---

## 1. Why the encoder already works

Paper compress for `mmse_cq` runs the standard pipeline with the algorithm's own
collector (`MMSECQuestSymbolCollectOperator`). `PaperSerializeOperator` captures the
collector's authoritative native blob via `paperExportCollectorPayloadBuffer()`
(our `MMSECQuestLayoutPlugin` answers layout=15 + `codecVersionOverride=2`), and the
blob is our container:

```
[u32 trailerLen][ "MMT1" trailer (per-layer 2-bit modes) ][ CQST v2 layers ]
```

The probe artifact confirms the storage side kept it intact as a **whole-frame**
native payload (54,567 B, frame=whole, `payload_bytes=54567`; stored with a 30-byte
`PNPL` page wrapper → 54,597 B column value).

## 2. Why decode SIGSEGV'd

`ISimplificationAlgorithm::makeDecompressOptions(Backend::Paper, …)`:
1. `PMSF_PAPER_RETRI=1` → `makeRetriPaperDeserializer(protocol="")` → `nullptr`
   (mmse_cq declares no retriangulation protocol; retri seam correctly declines);
2. fallback `makeDeserializeOperatorForAlgorithm("mmse_cq", Backend::Paper)` →
   **no registration existed** → framework Default paper triple
   (`OptimizedSymbolReadOperator` + `BasicReconstructOperator`);
3. that reader interprets the first bytes of our blob (`trailerLen` u32) as an
   Optimized stream header → crash immediately (probe log: decode dies right after
   `start mmse_cq_paper_decompress`, rc=-11).

The columnar path never hit this because `MMSECQuestAlgorithm.cpp` registers three
backends; `Backend::Paper` was simply omitted.

## 3. Current artifact composition (probe, r16)

`docs/memory/PHASE2_M5_PROBE/mmse/art_paper.parquet` = **59,750 B**, 2 row groups,
11 columns, 18 rows (`record_type` object row + PPTH partition row + 16 empty LOD rows):

| component | bytes | note |
|---|---:|---|
| native payload column chunk | 54,630 | raw blob 54,567 + `PNPL` page wrapper 30 + page hdr |
| — raw collector blob | **54,567** | our `[trailer][MMT1][CQST]` bytes; `payload_bytes` in JSON |
| object row group (RG0) | 2,519 | JSON header 2,200 B compressed 595 + dictionary/pages of 10 tiny columns |
| RG1 non-payload columns | 1,305 | `PPTH` header 87 + base bundle 827 + 8 empty/tiny columns |
| parquet footer + metadata | 1,296 | 2 row groups × 11 columns |
| **total** | **59,750** | vs canonical `mmse_cq` artifact 54,601 B (compact 468 + symbols 54,133) |

JSON header (`header` column row 0, 2,200 B) currently contains large dead weight for
a native whole-frame row: `streams` (≈20 typed descriptor entries for Cf/CfQ/Cv/CvQ…),
`payloads: []`, `boundary`, `layout.partitions` etc. Decoder-critical fields are only:
`algorithm`, `format/version`, `blob_layout`, `codec_version`, `schema`,
`payload_mode`, `native_payload.frame`, `num_rounds`, `num_partitions`,
`quant {enabled,bits,step,bbox_min}`, `total_vertices/faces`, base-mesh locator.

## 4. Storage-side asks (for storage-architect / M5-2 integration)

Decode is **unblocked by the MMSE-side registration**; the following are needed to
reach the task's byte target and to hard-pin semantics. Numbered by priority:

1. **Keep frame = Whole for layout 15 `mmse_cq`.** Do not extend CONN-slicing to
   CQST (the blob starts with `u32 trailerLen`, not `CONN`). The reader needs the
   bytes verbatim from offset 0, including our 4-byte prefix; `PNPL` unwrap must
   yield exactly the collector export bytes.
2. **Payload compression**: the current v2 native path stores the raw blob
   (54,567 B). Canonical transport stores the same bytes SBC1-wrapped
   (zlib, `stored_bytes=53,447` for r16; zstd may be better). Since
   `ColumnarDeserializeOperator::initPaperV2Native()` already calls
   `unwrapSymbolBlobIfNeeded()`, storage may keep/re-create the **SBC1 wrapper** as
   the native payload (or add a native-payload codec field). This alone saves
   ≈1,120 B.
3. **Shrink the JSON header** for native rows to the decoder-critical fields
   (§3): estimate 2,200 → ~300–500 B. Omit `streams`/`payloads`/`boundary`/
   per-partition locator arrays when `payload_mode=native`.
4. **Drop the empty LOD rows** for whole-frame native rows (17 rows × tiny columns
   currently cost RG1 1,305 B + row overhead); one object/partition row suffices
   (`num_lods` can stay in JSON).
5. **Base bundle**: RG1 `base_mesh` is an 827 B envelope around the 598 B canonical
   compact payload (V2CP). Storing the raw V2CP payload (598 B) is enough for
   `readV2CompactBin` if the envelope header is reconstructed — ask: can the base
   bundle reuse the canonical `art_compact.bin` bytes (598 B)?
6. **Footer/row-group overhead**: 1,296 B for 2 RG × 11 columns; with 1 RG × 3
   columns (header/payload/base) and dictionary disabled this should drop
   substantially.
7. **Metadata pin** (no size cost): keep `blob_layout=15`, `codec_version=2`,
   `algorithm=mmse_cq`, `num_rounds`, `quant` block. The layout-15 dispatch in
   `ColumnarDeserializeOperator` is what routes to the conquest import branch; a
   missing/incorrect layout would silently pick a wrong reader.

Realistic envelope if 2+3+4+5+6 land: 53,447 (zlib) + ~400 (JSON) + ~600 (base V2CP)
+ ~700 (footer/RG) ≈ **55,1k** — still above 54,601 unless zstd gives another
~0.5–1.0 kB or overhead goes lower. **The ≤54,601 target is a container-overhead
problem; MMSE payload semantics are fixed and lossless.** If the storage-architect's
M5 zero-overhead framing cannot reach it, the honest outcome is `paper_v2 > canonical`
with the exact ledger above (same situation as PPMC's +3,207 B), not a protocol hack.

## 5. MMSE-side deliverable in this task

`impl/MMSE/decompress/MMSECQuestPaperTripleParquet.cpp` (new, 97 lines):

* `makeMMSECQuestPaperTriple()` — `PaperDeserializeOperator(reader, reconstructor)`
  where reader = `MMSECQuestSymbolReadOperator`, reconstructor =
  `MMSECQuestReconstructOperator(reader, state)`, state =
  `MMSECQuestGeomState` (shared decoder H3 replay state; **must be one instance**
  for reader↔reconstructor, same as the columnar triple).
* `NamedDecompressOperatorRegistrar("mmse_cq", Backend::Paper, …)` — inert for all
  other algorithm/backend rows (before this file, that lookup had no entry).
* `PMSF_MMSE_CQ_PAPER_PAIR=0` — A/B escape hatch reproducing the pre-M5-2 SIGSEGV.
* Wrapped in `#if defined(COLUMNAR_SUPPORT)` and named `*Parquet*` per repo
  convention (same pattern as `src/decompress/PaperDecompressFactoryRegistrationParquet.cpp`).

No change to: `algorithm=MMSE` canonical path (62,633 B / md5
`9744d99c…`/`06a63531…`), columnar `mmse_cq` artifacts (54,601 B / payload md5
`3ebb7197…`), conquest fusion, or any default env path.

## 6. Verification plan (after Lead clears the build window)

```bash
# 1) build (flock)
flock /tmp/pmsf_build.lock -c 'cmake -S . -B build && cmake --build build -j32 \
    --target PMSFCompress PMSFDecompress'

# 2) canonical reference (must stay byte-identical)
BUILD_DIR=/tmp/mmsep1/bin bash impl/MMSE/mmse_cq_probe.sh /tmp/mmsep1/m52 16

# 3) paper-v2 encode (same c.json as probe, backend=paper) + retri env
PMSF_PAPERSTREAM_V2=1 PMSF_PAPERSTREAM_V2_PAYLOAD=native PMSF_PAPER_RETRI=1 \
  ./build/PMSFCompress docs/memory/PHASE2_M5_PROBE/mmse/c_paper_v2.json
# 4) paper-v2 decode (must now be rc=0)
PMSF_PAPERSTREAM_V2=1 PMSF_PAPER_RETRI=1 \
  ./build/PMSFDecompress docs/memory/PHASE2_M5_PROBE/mmse/d_paper_v2.json
# 5) bit-identical check
cmp dec.off canonical_mmse_cq_r16.dec.off   # canonical dec.off md5 d856290b614ffa3f2276aaf71d1f5dae
# 6) default guards
#    canonical MMSE 62,633 B / md5 9744d99c…/06a63531…; default conquest r16 payload md5 a3054310…
# 7) A/B: PMSF_MMSE_CQ_PAPER_PAIR=0 must reproduce the old SIGSEGV (rc=-11)
```

Acceptance: paper decode rc=0, `dec.off` **bit-identical** to columnar `mmse_cq`
decode (`d856290b…`), paper bytes **≤54,601** (or FAIL + the §3 ledger), env OFF
paper matrix 8/8 unchanged, default MMSE/conquest md5 unchanged.

## 7. Open questions for storage-architect

1. Which of the §4 items are in M5-2 storage scope vs deferred to the M5 v2
   overhead task? (PPMC needs the same measures.)
2. Does v2 already have a payload codec field that could carry SBC1/zstd, or should
   we reuse the `PNPL` wrapper page for it?
3. When the base bundle shrinks, can `readV2CompactBin` take the raw V2CP bytes
   directly (no envelope file)?
4. Should the `mmse_cq` paper artifact record `PMSF_PAPER_RETRI` in the manifest
   (matrix notes) even though no retri protocol is used? Current probe does not.

---

## 8. First measurement after build green (2026-10-22 09:43–09:46)

Build: Lead added the file to the paper source list in CMakeLists.txt and built
`PMSFCompress`/`PMSFDecompress` → BUILD_RC=0. Binaries used: snapshot
`/tmp/mmsep1/bin2` (build 09:43, md5 recorded). Run dirs: `/tmp/mmsep1/m52/`.

### 8.1 Canonical regressions (unchanged)

| item | value | expected |
|---|---|---|
| `mmse_cq` r16 artifact | 54,601 B / 22.004332 bpv, decode six true | ✅ |
| `mmse_cq` r16 payload md5 | `3ebb7197…` (≠ conquest) | ✅ |
| default `algorithm=MMSE` r50 | 62,633 B, compact md5 `9744d99c…`, residual `06a63531…` | ✅ |

### 8.2 Paper-v2 decode — **reader pair works** (SIGSEGV gone)

Encode: `PMSF_PAPERSTREAM_V2=1 PMSF_PAPERSTREAM_V2_PAYLOAD=native PMSF_PAPER_RETRI=1
PMSFCompress c.json` → rc=0, artifact **59,750 B**.
Decode (same two env vars) → **rc=0** (was rc=-11 SIGSEGV before the registration),
`art.dec.off` 1,636,908 B, all six MeshCompare flags **true**:
`counts_ok, connectivity_ok, topology_ok, nearest_geometry_ok,
manifold_profile_ok, coordinate_remap_complete = true` (vs the original mesh;
`remapped_common_edges=59,547` = all edges).

A/B control: `PMSF_MMSE_CQ_PAPER_PAIR=0` → **rc=139 (SIGSEGV)** exactly as the
pre-M5-2 baseline ⇒ the registration is the thing that fixes decode.

### 8.3 Remaining gaps are storage-side, with attribution proof

1. **Not bit-identical to canonical columnar decode**: paper dec.off md5
   `9e4db23f…` vs canonical `d856290b…`. But the difference is **vertex
   ordering only**: `MeshCompare canonical_dec.off paper_dec.off` →
   `remapped_common_edges=59,547` (= all), `remapped_same_unordered_faces=39,698`
   (= all), `nn_ref_to_test_max = 0` (every canonical position exists exactly),
   six acceptance flags true ⇒ same mesh, different base-vertex labeling.
   **Attribution (control experiment)**: the shipped **conquest** paper path shows
   the *same* md5 `9e4db23f…` (both paper-v1 bridge 59,142 B and paper-v2 native
   59,616 B) while its canonical columnar decode is `d856290b…`. So this is
   **generic paper-container base-bundle ordering**, not the MMSE reader:
   `PaperDeserializeOperator::initV2` rebuilds the temp compact bin from
   `partitions.front().baseMesh` (paper's own 827 B/raw-929 B builder) instead of
   the canonical compact payload (raw 538 B) whose vertex order the canonical
   decoder uses. Fix = §4 item 5: carry the canonical compact base payload
   (vertex order included) in the paper artifact.
2. **Bytes**: 59,750 > 54,601 (**FAIL**, Δ +5,149 B) — ledger in §3 unchanged;
   payload 54,630 (raw, uncompressed) + RG0 2,519 + RG1 non-payload 1,305 +
   footer 1,296. Storage asks §4 items 2–6.

### 8.4 Verdict (honest)

* MMSE half of task-32 (Paper reader/reconstructor pair + registration):
  **DONE and verified** (decode rc=0, six flags true, A/B proves causality).
* Task-32 acceptance "paper decode == canonical columnar bit-identical AND
  bytes ≤54,601": **FAIL on paper-container side**, with exact attribution above;
  both remaining items are in the storage/M4-4 scope (`task-32` storage half),
  no fake PASS. `mmse_cq` protocol/payload semantics are proven lossless.

---

## 9. Second measurement: storage M5-2 patch v1+v2 applied (2026-10-22 09:57)

Lead applied `/tmp/m5_2_delivery/m5_2_storage_1792633858.patch` +
`/tmp/m5_2_delivery_v2/m5_2_storage_1792634033.patch` and built
(`PMSFCompress` 09:55:22, `PMSFDecompress` 09:55:38). Rerun via
`/tmp/mmsep1/m52b/rerun_m5_2_mmse.sh`, run dir `/tmp/mmsep1/m52b/run2`.

Encode env (storage instruction): `PMSF_PAPERSTREAM_V2=1
PMSF_PAPERSTREAM_V2_PAYLOAD=native PMSF_PAPERSTREAM_V2_CONTAINER=blob
PMSF_PAPERSTREAM_V2_PAYLOAD_CODEC=sbc1 PMSF_PAPERSTREAM_V2_BASE_SOURCE=canonical
PMSF_PAPER_RETRI=1` (no `_BASE=raw-v2cp`: MMSE canonical compact is the 468 B
envelope with extended bit-ctx codec, so raw-v2cp cannot shrink it).
Decode: `PMSF_PAPERSTREAM_V2=1` only.

### 9.1 Result

| item | value | verdict |
|---|---|---|
| paper artifact (parquet single-cell `paper_blob`) | **54,573 B** = P2BL 54,363 + parquet framing 210 | **≤54,601 ✅ (margin 28 B)** |
| P2BL container | v2, payloadCodec=2 (SBC1-zstd), baseCodec=0 (envelope), codecVersion=2, blobLayout=15, payloadMode=native, nativeFrame=whole, raUnit=partition, numPartitions=1, numRounds=16, 0 trailing bytes | contract ✅ |
| payload segment | 53,447 B, sha256 `9080a33e…` | **== canonical `rc_blob` sha256 (byte-identical) ✅** |
| base segment | 827 B, sha256 `e99e9d1e…` (envelope raw=929/comp=819) | **≠ canonical `art_compact.bin` 468 B sha256 `f36e1ee1…` (raw=538/comp=460, bit-ctx) ❌** |
| decoded file sha256 | `5df29929…` vs canonical `3a4d07e7…` | **not bit-identical**; MeshCompare: six flags true, remapped all 59,547 edges, `nn_ref_to_test_max=0` ⇒ same mesh, base vertex order only |
| A/B `PMSF_MMSE_CQ_PAPER_PAIR=0` | rc=139 SIGSEGV (old baseline), ON rc=0 | registration causality ✅ |
| canonical `mmse_cq` r16 | 54,601 B / 22.004332 / payload md5 `3ebb7197…` | unchanged ✅ |
| default MMSE r50 | 62,633 B / compact `9744d99c…` / residual `06a63531…` | unchanged ✅ |

### 9.2 Remaining blocker (storage-side, single item)

`PMSF_PAPERSTREAM_V2_BASE_SOURCE=canonical` does **not** reproduce the canonical
compact blob: the paper path still emits the legacy bridge compact
(raw 929/comp 819/827 B) instead of the canonical (raw 538/comp 460/468 B).
Since the payload segment is byte-identical (S1 verified for CQST) and decode is
`same mesh up to relabeling`, the whole remaining bit-identity gap is the base
bundle builder. Suggested fix reported to storage-architect:
`src/compress/PaperSerializeOperatorParquet.cpp:295-306` (`setV2CompactMode(false)`
+ reused `ColumnarSerializeOperator::serialize`) must use exactly the canonical
columnar writer configuration — or simply take the canonical `*_compact.bin`
bytes. Expected after fix: paper total = 54,573 − 827 + 468 = **54,214 B
(margin 387)** and decoded sha256 == `3a4d07e7…`.

---

## 10. FINAL PASS: missing env was `PMSF_BASE_EDGEBREAKER_NATIVE=1` (2026-10-22 10:00)

Root cause of the §9 symptom was **an env gap on our side**, not storage code:
the canonical reference is produced by `impl/MMSE/mmse_cq_probe.sh`, which sets
`PMSF_BASE_EDGEBREAKER_NATIVE=1` before compress; the paper-v2 rerun did not, so
the paper bridge's temp compact used the legacy base builder (raw 929 / envelope
827) while the canonical uses the native-EdgeBreaker base (raw 538 / envelope 468).
Adding that env to the paper encode (decode unchanged) makes the base bytes
identical, and everything else follows.

Final encode env (only addition vs §9: `PMSF_BASE_EDGEBREAKER_NATIVE=1`):
```
PMSF_BASE_EDGEBREAKER_NATIVE=1
PMSF_PAPERSTREAM_V2=1 PMSF_PAPERSTREAM_V2_PAYLOAD=native
PMSF_PAPERSTREAM_V2_CONTAINER=blob PMSF_PAPERSTREAM_V2_PAYLOAD_CODEC=sbc1
PMSF_PAPERSTREAM_V2_BASE_SOURCE=canonical
PMSF_PAPER_RETRI=1   (+ the original c.json extra env)
# decode: PMSF_PAPERSTREAM_V2=1 only
```
Run dir: `/tmp/mmsep1/m52b/run3`, script `/tmp/mmsep1/m52b/rerun_m5_2_mmse.sh`.

### 10.1 Result — task-32 MMSE acceptance **PASS**

| item | value | verdict |
|---|---|---|
| paper artifact `_paper.parquet` | **54,214 B** (P2BL 54,004 + parquet framing 210), sha256 `4df0f5d7…` | **≤54,601 ✅ (margin 387 B; −597 B vs canonical)** |
| base segment | **468 B**, sha256 `f36e1ee1c06f5bf8239102b3737272d7377d2ecbebbedaf4f6e36bb2f6444db7` | **== canonical `art_compact.bin` byte-identical ✅** |
| payload segment | **53,447 B**, sha256 `9080a33ed4454cad4bd0900207de1b114441fea90d740cd2f07917dec4b158cd` | **== canonical rc_blob byte-identical ✅** |
| P2BL container | v2, payloadCodec=2 (SBC1-zstd), baseCodec=0, layout=15, codecVersion=2, nativeFrame=whole, payloadMode=native, numRounds=16, trailing=0 | contract ✅ |
| decoded file | sha256 `3a4d07e7d95530cfdd434828459aadf0eb59113720f12a31a046c20564ec2b33` | **== canonical columnar decode (bit-identical) ✅** |
| MeshCompare vs canonical | six flags true **including `index_connectivity_ok=true`**, remapped all 59,547 edges, `nn_ref_to_test_max=0` | fully identical ✅ |
| A/B `PMSF_MMSE_CQ_PAPER_PAIR=0` | rc=139 SIGSEGV (pre-M5-2 baseline), ON rc=0 | registration causality ✅ |
| canonical `mmse_cq` r16 | 54,601 B / 22.004332 / six true / payload md5 `3ebb7197…` | unchanged ✅ |
| default MMSE r50 | 62,633 B / compact md5 `9744d99c…` / residual `06a63531…` | unchanged ✅ |

No storage code change was needed for the MMSE row (the §9 `BASE_SOURCE`
diagnosis narrowed it to the base builder; storage-architect identified the
missing env). MMSE side remains: one registration file
(`MMSECQuestPaperTripleParquet.cpp`) + this documentation, zero `src/` changes.
