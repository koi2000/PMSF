// CompactBinInfo.cpp —— 读 `<base>_compact.bin` 的**自描述字段**并打印。
//
// P30 物品4：`scripts/tests/verify_artifact_layout.sh` 原先在 Python 里自己解析
// 容器。容器换了**扩展后端**（`rawLen` 最高位置 1 ⇒ stream 首字节是 codec id）
// 之后，Python 只能解 codec 1(zlib)，对 2(brotli)/3(bitctx) 只能做结构检查，
// `codecVersion` / `blobLayout` 两个**自描述字段查不了**。
//
// 本工具把这件事交回 **C++ 解码器**——它本来就支持全部 codec，
// 于是门禁可以对**所有** codec 做**全字段**校验，而不是分级。
//
// 用法：
//   CompactBinInfo <file_compact.bin>
// 输出（成功，stdout，单行）：
//   OK codec=<n> version=<v> codecVersion=<cv> blobLayout=<bl> \
//      totalVertices=<nv> totalFaces=<nf> round=<r> groupNumber=<g> \
//      hasQuant=<0|1> quantBits=<q> bboxMinX=<x> bboxMinY=<y> bboxMinZ=<z> quantStep=<s>
// 失败：stderr 说明原因，退出码非 0；此工具不是 payload 完整性/校验和验证器。
#include "storage/PMSFCompactBinEnvelope.h"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>

namespace {

bool hasPayloadBytes(const std::vector<std::uint8_t>& payload,
                     std::size_t offset,
                     std::size_t count) {
    return offset <= payload.size() && count <= payload.size() - offset;
}

// readPaperCompactPayloadHeader() is intentionally a low-level parser and
// assumes that its caller already checked the complete variable-length header.
// CompactBinInfo is an input-validation boundary, so do that check here before
// handing the bytes to the shared parser.
bool payloadHeaderFits(const std::vector<std::uint8_t>& payload,
                       bool hasSecondFlagByte) {
    if (!hasPayloadBytes(payload, 0, 8) ||
        (std::memcmp(payload.data(), "V2CP", 4) != 0 &&
         std::memcmp(payload.data(), "V3CP", 4) != 0)) {
        return false;
    }

    const std::uint16_t flags =
        static_cast<std::uint16_t>(payload[6]) |
        (static_cast<std::uint16_t>(payload[7]) << 8);
    std::size_t off = 8;
    const auto consume = [&](std::size_t count) {
        if (!hasPayloadBytes(payload, off, count)) {
            return false;
        }
        off += count;
        return true;
    };

    // round, groupNumber, flagBits, flagBits2, codecVersion, blobLayout.
    if (!consume(4 + 4 + 1 + (hasSecondFlagByte ? 1 : 0) + 4 + 4)) {
        return false;
    }
    if ((flags & 0x80u) != 0 && !consume(4)) {
        return false;
    }
    // origTotalVertices, origTotalFaces.
    if (!consume(4 + 4)) {
        return false;
    }
    // quantBits, bboxMinXYZ, quantStep.
    if ((flags & 0x1u) != 0 && !consume(4 + 4 + 4 + 4 + 4)) {
        return false;
    }
    if ((flags & 0x100u) != 0) {
        if (!hasPayloadBytes(payload, off, 4)) {
            return false;
        }
        const std::uint32_t nameLen =
            static_cast<std::uint32_t>(payload[off]) |
            (static_cast<std::uint32_t>(payload[off + 1]) << 8) |
            (static_cast<std::uint32_t>(payload[off + 2]) << 16) |
            (static_cast<std::uint32_t>(payload[off + 3]) << 24);
        if (!consume(4) || !consume(static_cast<std::size_t>(nameLen)) ||
            !consume(4)) {
            return false;
        }
    }
    return true;
}

}  // namespace

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::fprintf(stderr, "用法: %s <file_compact.bin>\n", argv[0]);
        return 2;
    }
    const std::string path = argv[1];
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        std::fprintf(stderr, "无法打开: %s\n", path.c_str());
        return 2;
    }
    std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(in)),
                                    std::istreambuf_iterator<char>());
    if (bytes.empty()) {
        std::fprintf(stderr, "空文件: %s\n", path.c_str());
        return 1;
    }
    if (bytes.size() < 8) {
        std::fprintf(stderr, "外层头不足 8 B: %s\n", path.c_str());
        return 1;
    }

    // codec id 在**外层头**里：rawLen 最高位置 1 ⇒ stream 首字节是 codec id。
    const std::uint32_t rawLen = pmsf::readPaperCompactU32LE(bytes, 0);
    const bool extended = (rawLen & pmsf::kCompactBinExtendedCodecFlag) != 0;
    int codec = static_cast<int>(pmsf::kCompactBinCodecZlib);
    if (extended) {
        if (bytes.size() < 9) {
            std::fprintf(stderr, "扩展后端但无 codec 字节\n");
            return 1;
        }
        codec = static_cast<int>(bytes[8]);
    }

    // 边界不变式：`8 + compLen == fileSize`。这是**线格式注释自己写的**约束
    //（`PMSFCompactBinEnvelope.h`：「compLen 仍等于第 8 字节之后的总字节数，
    //  因此既有 8+compLen==fileSize 的边界校验全部保持成立」）。
    // ⚠️ 阴性对照逼出来的：**bitctx 路径并不强制 compLen** —— 把 byte[4] 翻掉后
    //    解码仍成功（码流只被读少了几个字节）。故这里显式补上这条不变式。
    const std::uint32_t compLen = pmsf::readPaperCompactU32LE(bytes, 4);
    if (bytes.size() != 8u + static_cast<std::size_t>(compLen)) {
        std::fprintf(stderr, "边界不变式不成立: 8+compLen=%zu 但文件 %zu B\n",
                     8u + static_cast<std::size_t>(compLen), bytes.size());
        return 1;
    }
    // Reject missing bitctx parameters/range-code initialization bytes before
    // entering the shared decoder (whose range coder has no input bound).
    if (codec == pmsf::kCompactBinCodecBitCtx && compLen < 1 + 5 + 5) {
        std::fprintf(stderr, "bitctx 参数或码流头不完整\n");
        return 1;
    }

    pmsf::PMSFCompactBinEnvelope envelope;
    if (!pmsf::readPMSFCompactBinEnvelopeBytes(bytes, path, "[CompactBinInfo]", envelope)) {
        std::fprintf(stderr, "解包失败（codec=%d）\n", codec);
        return 1;
    }
    if (envelope.payload.size() < 8 ||
        (std::memcmp(envelope.payload.data(), "V2CP", 4) != 0 &&
         std::memcmp(envelope.payload.data(), "V3CP", 4) != 0)) {
        std::fprintf(stderr, "payload magic 不是 V2CP/V3CP 或头不足\n");
        return 1;
    }
    const bool hasSecondFlagByte = envelope.payload[1] == '2';
    if (!payloadHeaderFits(envelope.payload, hasSecondFlagByte)) {
        std::fprintf(stderr, "payload header 不完整或越界\n");
        return 1;
    }

    const std::uint8_t* p = envelope.payload.data() + 4;
    const pmsf::PaperCompactPayloadHeader h =
        pmsf::readPaperCompactPayloadHeader(p, hasSecondFlagByte);

    // ── 自洽性校验（P30 物品4 的**阴性对照**逼出来的） ──────────────────────
    // 只查「>0」是不够的：把压缩流里一个字节翻掉之后，解码**不报错**，
    // 而是解出 `codecVersion=889737268` 这种垃圾，且仍 >0 ⇒ 会误判为 OK。
    // 容器本身**没有校验和**（这是格式性质，不是门禁的错），所以门禁能做的是
    // **字段自洽性**：取值域。下列 codec/layout 下界/上界取自
    // 当前全部 11 个算法产物的实测值域，留了充分余量。
    auto bad = [&](const char* why, long long v) {
        std::fprintf(stderr, "自描述字段不自洽(%s): codecVersion=%d blobLayout=%d "
                             "totalVertices=%d totalFaces=%d round=%d [v=%lld]\n",
                     why, h.codecVersion, h.blobLayout, h.origTotalVertices,
                     h.origTotalFaces, h.round, v);
        return 1;
    };
    if (h.codecVersion <= 0 || h.codecVersion > 64)
        return bad("codecVersion 越界", h.codecVersion);
    if (h.blobLayout < 0 || h.blobLayout > 64)
        return bad("blobLayout 越界", h.blobLayout);
    if (h.origTotalVertices <= 0 || h.origTotalFaces <= 0)
        return bad("顶点/面数非正", h.origTotalVertices);
    if (h.round < 0)
        return bad("round 为负", h.round);
    if (h.hasQuant && h.quantBits <= 0)
        return bad("quantBits 越界", h.quantBits);
    // origTotalVertices counts the original mesh, not bytes in the base mesh
    // or internally compressed segments. No vertex/byte ratio is invariant.
    if (envelope.payload.size() != envelope.rawLen)
        return bad("payload 长度与 rawLen 不符", static_cast<long long>(envelope.payload.size()));

    std::printf("OK codec=%d version=%u codecVersion=%d blobLayout=%d "
                "totalVertices=%d totalFaces=%d round=%d groupNumber=%d "
                "hasQuant=%d quantBits=%d "
                "bboxMinX=%.9g bboxMinY=%.9g bboxMinZ=%.9g quantStep=%.9g\n",
                codec, static_cast<unsigned>(h.version), h.codecVersion, h.blobLayout,
                h.origTotalVertices, h.origTotalFaces, h.round, h.groupNumber,
                h.hasQuant ? 1 : 0, h.hasQuant ? h.quantBits : 0,
                h.bboxMinX, h.bboxMinY, h.bboxMinZ, h.quantStep);
    return 0;
}
