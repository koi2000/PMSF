# APTM PaperStream M5-2 接口设计（`aptm_polar_l48` paper native payload + reader pair）

日期：2026-10-22 / 负责人：aptm-repro（WS-B3）/ 任务：task-32 的 APTM 侧
关联：`docs/memory/PHASE2_M5_PROBE/FINDINGS.md`（M5-0 探针：APTM paper-v2 74,005 B，
decode SIGSEGV）、`/tmp/m5_adapter_interface_draft.md`（M4-P4 seam 清单）、
`impl/APTM/P3_POLAR_REPORT_2026-10-22.md`（canonical polar l48 22.31 bpv 证据）。

状态：**接口设计 + impl/APTM 内 env-gated 实现已完成（syntax-only 校验通过），
未 build、未改默认路径**。等 Lead build window 后跑 `scripts/tests/test_aptm_paper_v2.sh`。

---

## 0. 结论（给 Lead / storage-architect）

1. **decode SIGSEGV 根因不在 payload，而在 paper 三件套选错**。src 的 paper 工厂给
   `"aptm"` 注册的是通用 `Optimized + BasicReconstruct` 三件套
   （`src/decompress/PaperDecompressFactoryRegistrationParquet.cpp`）；APTM polar 产物是
   **layout=15（ConquestRC v2）**，通用 reader 读不了 → 崩溃。M4-4 的 paper v2 native
   解码本身是通的：它把 compact base + native blob 直接喂给
   `ColumnarDeserializeOperator::initPaperV2Native()`。
2. **APTM 侧修复 = 在 `APTMAlgorithm::makeDecompressOptions` 里加一个 env-gated
   `Backend::Paper && aptmConquestP3On()` 分支**（~40 行，本文件 §3）：
   复用已注册的 `"conquest"` paper 三件套（`PaperDeserializeOperator` +
   `ConquestSymbolReadOperator` + `ConquestReconstructOperator(reader)`），
   再装上与 canonical columnar APTM 解码**同一份** polar hooks（levels 由 env 决定）。
   **不改 src/、不改 CMake、不新增注册**（避免 static-init 顺序覆盖 `"aptm"` 注册）；
   默认 paper APTM（env 未设）路径零变化。
3. **payload 语义已逐字节确认**（§1）：paper `base_mesh` == canonical
   `art_compact.bin` 逐字节相同；`native_payload` == PNPL 信封 + **裸 CQST v2 blob**
   （66,565 B，即 collector `exportToBufferWithRangeCoder` 原样输出，**未做 SBC1 包装**）；
   `header` 是 2,199 B JSON 字典（含大量 native 模式用不到的 typed-stream 描述）。
4. **storage S1–S3 已落地并实测通过（2026-10-22，M5-2 blob 容器）**：
   APTM paper artifact = **52,492 B ≤ 52,882 ✓**（container header 86 + base 2,780 +
   payload 49,416 = P2BL 52,282 + parquet framing 210）；payload 与 canonical
   `rc_blob` **逐字节相等**、base 与 canonical `art_compact.bin` **逐字节相等**，
   decode 保持 bit-identical（sha `10be7834…`）。env：
   `PMSF_PAPERSTREAM_V2=1 PMSF_PAPERSTREAM_V2_PAYLOAD=native
   PMSF_PAPERSTREAM_V2_CONTAINER=blob PMSF_PAPERSTREAM_V2_PAYLOAD_CODEC=sbc1`。
   以下 §4 是落地前的原始预算推导，保留作依据。
5. **原先的预算推导（落地前）：要达到 ≤52,882 B，storage 侧需要 3 个改动**（§4，按收益排序）：
   (a) native_payload 保留 canonical 的 **SBC1 包装**（49,416 B，省 17,179 B；解码端
   `unwrapSymbolBlobIfNeeded` 已经支持 codec 1/2，**无需解码改动**）；
   (b) 字典从 2,199 B JSON 改成 typed/binary（native 模式只需 ~200-300 B）；
   (c) 容器 framing/列裁剪：整帧 native 不需要 150 条空 LOD 行/11 列 schema
   （当前 overhead 2,431 B）。目标 framing ≤686 B（= canonical symbols parquet 的
   overhead）——若 (b)+(c) 达不到，再用更强 zstd 级别给 (a) 的 49,416 B 挤余量。
5. **decoder 语义**：不需要 anchors（只在编码端 veto，base mesh 已含锚点）、不需要
   Arch-2 状态（encoder 端已把 decoder-consistent 的 deltas 烘进 blob）；只需要
   polar 的 `transform/inverseTransform`（mode=polar、levels=48），与 canonical
   columnar 解码完全同一份 hook。

---

## 1. `aptm_polar_l48` paper-v2 native payload 字段清单（实测自 M5-0 探针产物）

探针产物：`docs/memory/PHASE2_M5_PROBE/aptm/art.parquet`（74,005 B，v2 单文件）。
Parquet 行结构：RG0 = 1 行 object(row 0)；RG1 = 151 行（1 行 partition + 150 行空 LOD）。

| 字段（列） | 行 | 实测大小 | 内容 / 语义 |
|---|---|---:|---|
| `header` | object | **2,199 B** | JSON 字典：`algorithm:"aptm"`, `blob_layout:15`, `codec_version:2`, `format:"pmsf-paperstream"`, `payload_mode:"native"`, `native_payload:{frame:"whole",stream_names:["symbols"]}`, `num_rounds:150`, `layout.partitions[0].payload_bytes:66565`, `quant:{enabled:true,bits:12,bbox_min:[…],step:4.475878813536838e-05}`, `ra.{prefix_unit:"partition",bytes_read_includes_footer:true}`, `total_vertices/faces`, `boundary.codec:"raw-int32"`, `streams:[…11 个 typed 描述, native 模式不用…]` |
| `base_mesh` | partition | **2,780 B** | canonical `art_compact.bin` **逐字节相同**（V2CP compact base：header+base mesh+quant 元数据）。解码端就是把它写成临时 compact bin 再 `readV2CompactBin()`。 |
| `native_payload` | partition | **66,595 B** | `PNPL` 信封（30 B，`PaperStreamCodecV2.h`：magic+u32 ver+u8 scope+u8 frame+u64 off+u64 len）+ **裸 CQST v2 blob 66,565 B**（= collector export 原样；`captureNativeBlob` 经 `unwrapSymbolBlobForNative` 把 SBC1 去掉了）。 |
| `connectivity` / `geometry` / `extensions` / `boundary` / `base_mesh`(LOD 行)/`native_payload`(LOD 行) | LOD | 全空 | whole-frame native：150 条 LOD 行仅占位（RA=partition 级）。 |
| 容器 | — | **2,431 B** | Parquet schema/footer/RG 开销（总 74,005 − 2,199 − 2,780 − 66,595）。 |

**native blob 内部（CQST v2，`src/conquest/ConquestBlobFormat.h`）**：

```
magic "CQST"(4) | u16 version=2 | u16 flags | f64 step | f64 bboxMin[3] | u32 numLayers
per layer: f64 seedPos[2][3] | u32 numSymbols | u32 symBytes | symBytes
           | u32 numDeltas | 3 × (i32 lo | u32 span | u32 bytes | bytes)
```

`aptm_polar_l48` 的实测总量（r150，与 canonical 同一 blob）：
`layers=150, symbols=128,702, null_patch=109,103, C=13,013 B, G=39,108 B,
container=14,444 B, totalBlob=66,565 B, verts=19,851, faces=39,698, anchors=139`。
- **C**（connectivity 流）= A&D valence/null_patch 符号（anchors 以 null_patch 形式保留）；
- **G**（geometry 流）= polar 三分量 `(R, θq, φq)`，`levels=48` ⇒ θq/φq ∈ [0,47]；
  Arch-2 已经把 deltas 算成「decoder 将看到的位置」的残差，**解码端无需再跑 Arch-2**；
- **container** = 每层 seed/bbox/stream 头（150 层）；**blob step/bboxMin** = q12 网格。

**解码所需 vs 不需要（论文/实现语义）**：
| 项 | 解码需要? | 说明 |
|---|---|---|
| blob bytes（C+G+container）+ step/bboxMin/base mesh | **需要** | `initPaperV2Native(compact, blob)` 直接喂 |
| layout=15 / codec_version=2 / scheme_version | **需要** | 来自 compact base（V2CP） |
| anchors=139 | **不需要** | 编码端 veto；base mesh 已含锚点，解码无 anchor 逻辑 |
| polar mode + levels=48 | **需要** | 解码 `inverseTransform` 的 frame/量化精度；当前由 env 提供（与 canonical 一致） |
| Arch-2 iters/state | **不需要** | 仅编码端；deltas 已烘进 blob |
| rounds=150（LOD 数） | **需要** | 来自 blob/header（whole-frame 时 paper LOD 行可空） |

---

## 2. reader/reconstructor 入口（canonical 对照）

canonical columnar APTM polar 解码（`APTMAlgorithm::makeDecompressOptions`）：
```
ConquestSymbolReadOperator            // 逐层符号游标（reader 与 recon 必须同一实例）
ConquestReconstructOperator(reader)   // layout-15 逐层 decodeLayer
  └─ setHooks(makeAPTMConquestHooks(nullptr, polar))  // RQ: transform/inverseTransform
ColumnarDeserializeOperator(reader, reconstructor)
```
paper 侧对应：把 `ColumnarDeserializeOperator` 换成 `PaperDeserializeOperator`
（单文件 `_paper.parquet`），内部仍是同一个 `ColumnarDeserializeOperator` +
同一对 reader/recon；M4-4 的 `initV2()` 会：
1. 读 paper `base_mesh` → 临时 `legacy.bin`；
2. `inner_->initPaperV2Native(compactPath, nativeBlob)` 种下 header/layout/quant/rcBlob；
3. `deserializeBaseMesh()/deserializeOneRound()/exportMesh()` 全部委托 `inner_`。

`"conquest"` 的 paper 三件套**已经注册**：
`src/decompress/PaperDecompressFactoryRegistrationParquet.cpp::makeConquestPaperTriple()`
= `PaperDeserializeOperator + ConquestSymbolReadOperator + ConquestReconstructOperator(reader)`。
APTM 直接 `makeDecompressOperatorsForAlgorithm("conquest", Backend::Paper)` 取用即可。

---

## 3. APTM 侧实现（已在 `impl/APTM` 完成，未 build）

`impl/APTM/APTMAlgorithm.cpp::makeDecompressOptions` 新增（在 tag 分支之前）：
```cpp
if (backend == Backend::Paper && aptmConquestP3On()) {
    const auto triple = pmsf::makeDecompressOperatorsForAlgorithm("conquest", backend);
    if (!triple.deserializer) throw std::runtime_error("[APTM][paper] ...");
    auto recon = std::dynamic_pointer_cast<ConquestReconstructOperator>(triple.reconstructor);
    if (!recon) throw std::runtime_error("[APTM][paper] ... wrong triple");
    const auto geomMode = aptm_conquest::resolveGeometryMode();
    if (geomMode != GeometryMode::Native) recon->setHooks(makeAPTMConquestHooks(nullptr, geomMode));
    return DecompressOptions::Builder()
        .setDeserializeOperator(triple.deserializer).setPath(input).setOutputPath(output).build();
}
```
- gate = `Backend::Paper && aptmConquestP3On()`（= env `PMSF_APTM_USE_CONQUEST_P3=1`）：
  与 canonical columnar 分支**同一 gate**，因此默认 paper APTM（matrix 行）完全不变，
  env 未设时不会走进来。
- 若想用 matrix 行名 `aptm_polar_l48`：**注册表键仍是 metadata 算法名 "APTM"/"aptm"**
  （compress 写的是 `info().name`）；`aptm_polar_l48` 应作为 **matrix/manifest row id**，
  不能让 code registry 换名（否则 metadata/decode 分发都要改）。若 storage 需要
  自描述，可在 paper dictionary 加 `aptm_geometry:"polar"`, `aptm_polar_levels:48`
  作为**provenance 字段**（解码当前仍读 env，与 canonical 一致）。
- 顺带修了一个 paper 路径的性能/幂等问题：`APTMConquestCollector::applyArch2()`
  现在 `arch2Computed_` 缓存，paper 的 `captureNativeBlob` 二次 export 不再重跑
  decoder 模拟（字节不变；paper 压缩时间约减半）。

**默认路径不变性**：`impl/APTM` 改动只增加 env-gated 分支 + 缓存标志；
默认 APTM 84,496 B/4-md5、paper 8/8 矩阵、conquest fusion 90,490 不受影响
（build 后脚本复核）。

---

## 4. storage 侧需要改的点（task-32 / M5-2，按收益排序）

现状 74,005 B = 2,199(header) + 2,780(base) + 66,595(payload) + 2,431(overhead)。
canonical = 52,882 B = 2,780(compact) + 50,102(symbols parquet：
`rc_blob` **SBC1 包装 49,416** + parquet overhead 686)。

**S1（最大，省 17,179 B）native_payload 保留 SBC1 包装。**
`captureNativeBlob` 抓到的 `lastSymbolsBlob_` 本来就是 canonical 的 SBC1 包装字节；
`unwrapSymbolBlobForNative` 在 `buildNativeV2Table` 前把它拆了。建议：
- env/选项如 `PMSF_PAPERSTREAM_V2_PAYLOAD=native-wrapped`（或 native 模式默认保留包装）；
- 存 49,416 B；解码**无需改**：`ColumnarDeserializeOperator::initPaperV2Native()`
  的 `unwrapSymbolBlobIfNeeded()` 已支持 codec 1/2，`PaperDeserializeOperator::initV2()`
  注释也明确 "tolerate wrapped blobs"。
- 若还想挤余量：包装压缩级别调高（如 zstd 19/22）或直接复用 canonical `rc_blob` 字节。

**S2（省 ~1,900-2,000 B）字典 typed/binary。**
native 模式可删 `streams[]`（11 个 typed 描述 ≈ 1.2 KB）等；只需
`{algorithm enum/name, formatVersion, schemaMode, payloadMode:native, nativeFrame:whole,
 blobLayout:15, codecVersion:2, quant{bits,step,bboxMin}, numRounds:150,
 totalV/F, partition payload offset/len, baseMeshCodec}` ≈ 200-300 B。
（也可先 zstd 压 JSON 作为低风险过渡。）

**S3（省 ~1,800 B）容器 framing / 列裁剪。**
whole-frame native 不需要 150 行空 LOD 行；native 表的 11 列里 typed 列全空。
建议 native 专用 schema（object+partition 2-3 行、仅 header/base_mesh/native_payload
+ 必要列），目标 overhead ≤686 B（对齐 canonical symbols parquet 的 overhead）。

**S1+S2+S3 后的预算**：2,780 + 49,416 + ~250 + ~500 ≈ **52,946 B**，
比目标 52,882 还多 ~64 B ⇒ 任一即可补齐：S1 用更强 zstd、或 S2 再省 ~100 B、
或 S3 overhead 压到 ~440 B。**结论：≤52,882 可达，但 S1/S2/S3 必须同时做。**

**不需要 storage 侧做的事项**：
- 不需要为 APTM 新注册 paper 三件套（复用 `"conquest"`，见 §3）；
- 不需要 anchor/Arch-2 元数据（解码不需要，见 §1）；
- 不需要 LOD 级 RA（whole-frame ⇒ `ra.prefix_unit="partition"`；page 级 RA 属 M6）。

---

## 4.1 实测结果（2026-10-22 build 后，APTM 侧已通过）

build：`flock … cmake --build build -j32 --target PMSFCompress PMSFDecompress` rc=0；
`strings build/PMSFDecompress` 含 `[APTM][paper]` 分支与 polar-levels warning。
脚本：`scripts/tests/test_aptm_paper_v2.sh /tmp/wsb3/aptm_paper_v2`（全绿）：

| 项 | 结果 |
|---|---|
| 默认 APTM（columnar） | **84,496 B / 34.052088 bpv / 4-md5 = 冻结审计逐字节一致** ✓ |
| canonical columnar polar l48 r150 | 52,882 B；decode rc=0；sha256 `10be78340dacc643f827d8387da0e39ea23d00aeae5b2bfd32d42425a2ed3a8f` |
| paper v2 native encode | rc=0；`_paper.parquet` **74,005 B**（预期，storage S1-S3 前） |
| paper v2 native decode | **rc=0（M5-0 SIGSEGV 已修复）**；decoded sha256 = canonical sha256 ⇒ **bit-identical=true**（无 tolerance） |
| decode 六项 | `counts/connectivity/topology/nearest/coordinate_remap/manifold(+valid)` **全 true** |
| paper 分量 | header 2,199 / base 2,780 / payload 66,595 / overhead 2,431 = 74,005；canonical 52,882 ⇒ **delta +21,123 B**（待 S1-S3） |
| 确定性 | 同 env 连跑两次 paper encode，`_paper.parquet` sha256 相同 ✓ |
| 默认 paper APTM（env 未设） | 分支不进入 ⇒ 与改动前同路径（matrix 8/8 由 Lead gate 复核） |
| **M5-2 blob 容器（S1–S3，2026-10-22 复跑）** | **52,492 B ≤ 52,882 ✓（delta −390）**；P2BL header 86 + base 2,780 + payload 49,416 = container 52,282 + parquet framing 210；**payload bytes == canonical rc_blob ✓**、**base bytes == canonical art_compact.bin ✓**、decode sha 仍 = canonical（bit-identical），六项全 true；两次 encode 字节一致（sha256 `3048057e…`） |

CSV：`impl/APTM/APTM_PAPER_V2_EVIDENCE.csv`（19 行，含上表全部 item）。
日志/产物：`/tmp/wsb3/aptm_paper_v2/{logs,canonical,paper,paper2}/`。
**结论：APTM 侧 payload/reader pair 已正确；唯一未达标项是 storage 容器字节
（74,005 > 52,882），归 S1-S3。**
另：若 paper decode 设了 `PMSF_APTM_USE_CONQUEST_P3=1` 但**漏设**
`PMSF_APTM_CONQUEST_POLAR_LEVELS=48`，会打 stderr WARN（避免静默按 levels=8 解错）。

---

## 5. 验证计划（build window 后执行）

脚本：`scripts/tests/test_aptm_paper_v2.sh [outdir] [build_dir]`
（产物 CSV：`impl/APTM/APTM_PAPER_V2_EVIDENCE.csv`）

1. **default APTM（columnar）**：84,496 B / 34.052088 / 4-md5 与
   `/tmp/wsb2/aptm_audit/base` 逐字节一致（env 未设）。
2. **canonical columnar polar l48 r150**：52,882 B；decode → `dec.off` 参考 sha。
3. **paper v2 native**（env：`PMSF_PAPERSTREAM_V2=1 PMSF_PAPERSTREAM_V2_PAYLOAD=native`
   + `PMSF_APTM_USE_CONQUEST_P3=1 PMSF_APTM_CONQUEST_GEOM=polar
   PMSF_APTM_CONQUEST_POLAR_LEVELS=48 PMSF_APTM_CONQUEST_ARCH2_ITERS=2`）：
   - encode rc=0、decode rc=0（修复前 SIGSEGV）；
   - **decoded OFF sha256 == canonical decode sha256**（bit-identical，无 tolerance）；
   - `MeshCompare tol=1e-3` 六项 true；
   - artifact bytes 报告 + 分量 breakdown（header/base/payload/overhead）；
   - bytes ≤52,882 作为 storage 优化验收（当前预计 74,005 → 修 S1-S3 后达标）。
4. **默认路径不回归**：paper matrix 8/8、conquest fusion 90,490、validate_regression rc=0
   （Lead 侧 gate 负责；本脚本只跑 APTM 行）。
5. **确定性**：同输入同 env 连跑两次 paper encode → 字节一致。
6. **decode 需要同 polar env**（与 canonical 一致）：不设 env 解 l48 产物会按 levels=8
   解错几何（静默错误）。建议 M6 让 paper 字典带 `aptm_polar_levels` 自描述；M5 先按
   env 口径记录在 matrix notes。

---

## 6. 待 storage-architect 确认（回信请答）

1. S1 包装保留的 env/选项名，以及是否会写进 matrix manifest？
2. S2 字典 schema 最终字段（我按 §1 的 native 必需字段给了清单，可增减；
   `aptm_geometry`/`aptm_polar_levels` provenance 是否加？）。
3. S3 native 专用 parquet schema 是否在 M5 范围（还是 M4-1 typed-compact 顺带做）？
4. bytes ≤52,882 的验收口径：是 `_paper.parquet` 文件大小还是 payload-only？
   （本脚本按**文件大小**对比 canonical artifact 总和。）
5. 是否需要我把 APTM 的 paper encode 也改成「直接复用 canonical artifacts 字节」
   作为 fallback（若 S1-S3 来不及，则 paper 容器退化为 canonical 两文件的薄包装；
   解码仍走同一 reader pair）？—— 这是 M5 保底选项，需你确认接口。
