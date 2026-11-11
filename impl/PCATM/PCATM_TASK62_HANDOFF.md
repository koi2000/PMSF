# PCATM task-62 handoff — 2-pass static/quantized-context residual coder (sv14)
（本会话上下文预算已接近耗尽，交 fresh agent 执行；**未 build、未改 live**。设计+实测已完成。）

## 0. 关键实测（offline，决定能否过 CIKM）
用 sv13 encoder dump 的 19,694 条残差（`/tmp/task59/standalone/residuals.bin`；59,082 值）做 2-pass 静态模拟（empirical cross-entropy + zlib 表头）：
最优：**round-group=8（每 5 轮一组）、axis=3、prev-magnitude bucket cap=2（2 桶）、fallback 阈值 n>=16** →
`payload=37,786 B + table=579 B = 38,365 B`（+ varint header ~40B ≈ 38,405）。
同族：rgrp=16/cap2/fb16 = 38,367；rgrp=8/cap2/fb8 = 38,408。均 **≤40,000 目标**。
预测 canonical = 68,027 − 43,711 + 38,405 = **≈62,721 B = 25.28 bpv**（CIKM 26.153 target 64,895 → 余量 ~2.2KB）；fused（≈canonical−1,036）≈61.7KB ≤64,895 ✓。decode 语义不变 ⇒ sha 应仍 `c3382783…`。

## 1. 编码设计（必须 encoder/decoder 完全对称）
- 上下文（binary 决策级）：
  - role=0: length tree node 0..14；role=1: mantissa bit k=0..13。
  - fine ctx: `(roundGroup = r*8/numRounds, axis, prevBucket=min(bitlen(prevSameAxis),2), role, idx)`；prev 在该 round 内按同 axis 前值（x,y,z 顺序，round 起始置 0）。
  - coarse ctx: `(axis, role, idx)`（fallback）。
- pass0（encoder）统计每个 fine/coarse ctx 的 n0,n1；决定 fine 概率 `p=(n1+0.5)/(n+1)` 8-bit 量化（1..254），若 fine n<16 → 表项写 0（flag=fallback，用 coarse 概率）。
- 表头（写入 geometry stream 的 payload 前）：varint 表项数（=fine 模型总数：8*3*2*29=1,392）+ 每项 1B（0=fallback）+ zlib 压缩（`pmsf::pcatm::lzDeflate/lzInflate`，长度 i32 前缀）+ coarse 概率表（3*29=87 × 1B，可 zlib 或不压缩）。
- pass1：range coder 静态编码（**不要 qsmodel 自适应**）：对每个决策，先按 ctx 取 p1（8-bit→`f1=(p1*16)`，clamp 1..4095；TOT=4096，f0=TOT−f1），`encode_shift(rc, bit? f1 : f0, bit? f0 : 0, 12)`；解码 `cf=decode_culshift(rc,12); bit = cf>=f0; decode_update(rc, sy, lt, 4096)`。encoder/decoder 用同一函数计算 f0/f1 ⇒ 对称。
- 实现位置：`impl/common/strict/PCATMStrictAdaptiveResidual.h` 新增 `encodeStaticContextRounds()/decodeStaticContextRounds()`（保留 sv13 函数不动）；sv14 注册：`impl/common/PaperBlobAlgorithmSpecs.h` 加 `kPCATMStrictPaperStaticContextResidualVersion = 14` + 谓词 case（IsStrictPaper/IsColorOnly/UsesGlobalColor/UsesAdaptiveResidual 的不含 sv14 版本 + 新 UsesStaticContextResidual）；collector export 在 sv13 分支旁加 `PMSF_PCATM_STRICT_STATIC_CTX_RESIDUAL=1` 分支；reader import 加 sv14 解码分支。sv13 路径必须逐字节不变（env-gated）。

## 2. 复现/验证
- offline 复现：脚本见本文件 §0 命令记录（Python 三分量：payload 用 empirical cross-entropy，table 用 zlib(8-bit table)，未含实际 range-coder 尾部 ~几字节）。
- live 验收（1 build）：`c.json = /tmp/task49_runs/guard_strict/c.json` + extra `"PMSF_PCATM_STRICT_STATIC_CTX_RESIDUAL":"1"`；artifact 目标 ≤64,895（理想 ≤62.8KB）、bpv ≤26.1；decompress sha `c33827838902cd42fe3d185b68738fd31495e0427dbfad4d49bf1eab94f46a91`；MeshCompare --tol 0.001 全 true；guards：默认 strict 71,550/md5 `ac222c20…/52b17d0c…`、PURE_K3 69,187/md5 `e91bb4bc…/5d8a5d98…`、PCLTTM/QEM/RD page-RA、phase1 9/1/1、phase2 8/8 不变。
- fused：M5-9 重融合（PMSF_PAPERSTREAM_V2=1 + SBC1-zstd）验证 fused ≤64,895；若需另跑，可由 storage 侧完成。

## 3. 风险
- 静态表/量化概率的 cross-entropy 是估计值；实际 range coder 可能 +0~200B（仍远优于 43,711）。
- 表项数固定 = rgrp*3*2*29（8*3*2*29=1,392）；fb 阈值写进流头（固定 16 即可）。
- 不得降 q/codebook/Conquest；不改 displacement 值/color/elimination/decoder 语义。
