# M5-5 VQMMC Paper v2 opaque-payload 实现草案（只读，2026-10-22）

Owner: algo-bpv（task-35）。状态：**接口草案**；等 storage-architect `PaperStreamOpaquePayloadV2`
helper patch + Lead build window 后落 code。未改 live 默认、未 build、未跑 phase-1 gate。

## 0. 一句话方案
VQMMC 的 paper 行不重写协议：**把 canonical binary artifact（64,510 B）整段/去前缀后作为
opaque native payload 放进 P2BL v2 blob 容器（zlib-raw），decode 走现有
`VQMMCDeserializeOperator` strict 三元组**（`VQMMCStrictSymbolReadOperator` +
`VQMMCReconstructOperator` + `VQMMCPostProcessor`，strictMode=true）。
预计 coded fused ≈ **62,959 B ≤ 64,510**（margin ~1,551）；raw ≈64,826 超支（披露）。

## 1. 回答 storage-architect 的 ①–⑤
### ① 64,510 B 能否 paper branch 逐字节捕获（不重编码）？
能。canonical binary 路径 = `runVQMMCCompress(Backend::Binary)` 的
`BasicSerializeOperator` 输出：`serializeHeader + serializeBaseMeshWithSeed + serialize(path)`
（strip collector；文件 = `[u32 contentLen][buffer]`）。paper branch 用**同一份代码**（把该
binary 序列化块抽成 `serializeCanonicalVQMMC(cfg, tmpPath)`，paper 分支传临时路径）→ 读回字节，
即与 canonical **逐字节相同**（确定性：同输入同 env 同 sha `3d34efa9…`）。推荐临时文件捕获
（零 src/compress 改动）；若接受 `src/compress` 小改，可加 `BasicSerializeOperator::serializeToVector()`
避免临时文件（需 storage-architect 确认写作用域）。

### ② strict decode 是否需要单独解析 777 header？
不需要。整个 stream 喂给现有 strict 解码路径即可：
- 文件布局：`u32 contentLen=64,506` + `buffer(64,506)`；buffer = 773 B binary header/base
  （含 quant metadata + base mesh，VQMMC 自定义 `readBaseMeshWithQuantization`） + 63,733 B
  CONN layout30/sv6。
- `VQMMCDeserializeOperator::deserializeHeader()` → `deserializeBaseMesh()`（strictMode 下把剩余
  buffer 当单个 CONN blob 交给 `VQMMCStrictSymbolReadOperator`）→ 每轮 `deserializeOneRound()`。
- 因此 opaque payload 建议存 **完整 canonical 文件 64,510 B verbatim**（decode 端剥 u32 前缀后
  `buffer_`/`totalSize=64,506`）；或存 file[4:] 64,506 B（zlib9 62,634，省 1 B）。二选一，倾向
  完整文件（可追溯 "verbatim canonical file"；只多 1 B）。
- 实现落点：`VQMMCPaperDeserializeOperator : public VQMMCDeserializeOperator`，重写
  `init(std::string path)`：用 storage helper 读 opaque payload（含 codec 解包）→ 剥 u32 前缀 →
  设置基类 public 字段 `buffer_`/`totalSize`/`dataOffset_=0` → 设置 strictMode(true)；
  `deserializeHeader/BaseMesh/OneRound` 复用父类。

### ③ env 命名
- encode：**`PMSF_VQMMC_PAPER_ARTIFACT=1`**（必需；缺省时 `Backend::Paper` 保持现有 throw，
  paper matrix/env-OFF 行为不变）。
- decode：**无需额外 env**；`backend=paper` + 注册的 named triple 自动走 opaque reader
  （reader 能识别 P2BL opaque 容器）。不引入 `_DECODE` 开关，避免双开关漂移。

### ④ coded ≤64,510 / raw 披露口径
接受。zlib-9 实测：full file 64,510 → **62,635**；file[4:] 64,506 → 62,634。
coded paper ≈ 62,635 + 8B zlib framing + ~90B P2BL + 16 + ~210B parquet = **~62,959**
（margin ~1,551）。raw 无 codec ≈ **64,826 > 64,510**（+316）→ 按 QEM 同口径在
evidence 里披露 "PERF 以 coded 为准；raw 超支不用于 zero-overhead 判定"。
zero-overhead 不计 tolerance：decode 必须与 canonical **bit-identical**（canonical decoded sha
`bd47e456f19d7648d2558bb35f7d352fc65f59019eafdef9d64ce3fdda6736d0`），cb=256/q12/Reading-B 不变。

### ⑤ paper triple 注册 TU/文件名
- `impl/VQMMC/VQMMCPaperDeserializeOperator.h/.cpp`（新，算法 scope）
- `impl/VQMMC/VQMMCPaperRegistration.h/.cpp`（新）：导出
  `pmsf::DecompressOperatorTriple makeVQMMCPaperTriple()`，并在 `#ifdef COLUMNAR_SUPPORT` 下
  自注册 `NamedDecompressOperatorRegistrar("vqmmc", Backend::Paper, makeVQMMCPaperTriple)`
  （storage 的 `src/decompress/PaperDecompressFactoryRegistrationParquet.cpp` 无需改动；
  若要集中注册，也可由它 include 我们的 header 加一行——二选一，推荐自注册）。
- `impl/VQMMC/VQMMCAlgorithm.cpp`：`runVQMMCCompress(Backend::Paper)` 由 throw 改为
  env-gated 分支（调 `serializeCanonicalVQMMC` + storage helper
  `writePaperStreamV2OpaquePayloadArtifact`）。
- 测试脚本：`impl/VQMMC/vqmmc_paper_roundtrip.sh`：canonical binary（cb256/q12/r55）→ paper encode
  → paper decode → sha vs canonical decoded sha、bytes ≤64,510、MeshCompare 六项、determinism、
  raw/coded 双数披露；默认 VQMMC 64,510/cb256/q12 守卫、paper matrix env-OFF 8/8、v1 四 sha。
- 证据 CSV：`impl/VQMMC/VQMMC_PAPER_V2_EVIDENCE.csv`。

## 2. 需要的 storage helper 接口（按 storage-architect 草案确认）
```cpp
writePaperStreamV2OpaquePayloadArtifact(path, payloadBytes,
    OpaquePaperArtifactSpec{algorithm="vqmmc", blobLayout=30, codecVersion=?, rounds=55,
                            totalV=19851, totalF=39698, boundaryCodec=""},
    PayloadCodec::ZlibRaw, error);
readPaperStreamV2OpaquePayloadArtifact(path, &payload, &codec, error);
```
- payload = canonical binary file bytes（64,510，含 u32 前缀）verbatim；helper 只做 zlib-raw 封装 +
  P2BL v2 header，不解析内容。（若 helper 侧希望去前缀，也接受 64,506 版本，decode 端相应调整。）
- baseCodec=none：base/quant/header 已在 opaque payload 内，不再单独放 base partition；
  P2BL 需要的 `totalV/totalF/rounds/blobLayout` 由 spec 填。

## 3. 风险/注意
1. `VQMMCDeserializeOperator` 的 buffer 语义：`init(char*)` 不设 totalSize；我们直接写
   public 字段 `buffer_`/`totalSize`（或加 additive `initFromOwnedBuffer(std::vector<char>)`）。
2. decode 必须有 `VQMMCPostProcessor`（geometry/postprocess 与 canonical decode 完全一致）；
   env（`VQMMC_PAPER_CONN=1`、`VQMMC_USE_PREDICTIVE_RESIDUAL=1`、`VQMMC_CODEBOOK_QUANT_BITS=8`、
   `PMSF_DEFAULT_QUANT_BITS=12`、pools）= 与 canonical 相同，由 config extras 带入。
3. paper encode 的临时文件放在输出目录/`/tmp`，用唯一名并清理；失败路径不得留坏产物。
4. 默认 VQMMC（backend=binary，cb=256/q12/r55=64,510）与 config 默认（cb=32）路径零改动；
   `Backend::Paper` 未开 env 时仍 throw（fail-fast 语义保留）。
5. 若 Lead 后续改 zero-overhead 口径为 raw，则本行 FAIL（64,826>64,510）——按现口径 coded PASS。

## 4. 下一步
等 storage-architect helper patch 合入 worktree → 我在 impl/VQMMC 落上述 5 个文件 +
sh 脚本（env-gated，默认字节不变）→ 请 Lead build window → 跑正式证据 → 报 5/6/6 行状态。
