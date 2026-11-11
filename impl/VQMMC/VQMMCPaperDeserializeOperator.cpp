// SPDX-License-Identifier: Apache-2.0
#include "VQMMC/VQMMCPaperDeserializeOperator.h"

#include "VQMMC/VQMMCPaperOpaqueIO.h"

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

#if PMSF_VQMMC_HAS_OPAQUE_PAPER
#include "storage/PaperStreamOpaquePayloadV2.h"
#include "storage/PaperStreamReaderV2.h"
#include "compress/ConnectivityBlobContainer.h"
#endif

void VQMMCPaperDeserializeOperator::init(std::string path) {
    clearPMSFSymbolContext();
    dataOffset_ = 0;
    pointOffset_ = 0;
    totalSize = 0;
    buffer_ = nullptr;

#if PMSF_VQMMC_HAS_OPAQUE_PAPER
    // Paper artifacts are named `<stem>_paper.parquet`; accept either the stem
    // or the already-suffixed artifact path (same convention as other paper rows).
    std::string paperPath = path;
    const std::string suffix = "_paper.parquet";
    if (paperPath.size() < suffix.size() ||
        paperPath.compare(paperPath.size() - suffix.size(), suffix.size(),
                          suffix) != 0) {
        std::string stem = paperPath;
        const auto slash = stem.find_last_of("/\\");
        const auto dot = stem.find_last_of('.');
        if (dot != std::string::npos &&
            (slash == std::string::npos || dot > slash)) {
            stem = stem.substr(0, dot);
        }
        paperPath = stem + suffix;
    }
    std::vector<std::uint8_t> payload;
    pmsf::paper_multistream::PayloadCodec codec = pmsf::paper_multistream::PayloadCodec::None;
    std::string error;
    pmsf::PaperStreamReaderV2 probe(paperPath);
    if (!probe.isBlobContainer()) {
        const auto table = queryMaxLod_ >= 0 ? probe.readPrefix(0, queryMaxLod_) : probe.readAll();
        if (probe.usedV1Fallback() || table.object.algorithm != "vqmmc" || table.partitions.size() != 1 || (table.lods.empty() && queryMaxLod_ != 0)) {
            throw std::runtime_error("VQMMC relational paper reader: wrong algorithm/version or missing rows");
        }
        pmsf::ConnectivityBlobBuilder builder(table.object.blobLayout, table.object.schemeVersion,
            static_cast<int>(table.lods.size()), 4);
        std::uint64_t offset = 0;
        for (std::size_t index = 0; index < table.lods.size(); ++index) {
            const auto& lod = table.lods[index];
            if (lod.lodId != static_cast<int>(index) || !lod.extension.count("vqmmc_indices") || !lod.extension.count("vqmmc_codebook")) {
                throw std::runtime_error("VQMMC relational paper reader: incomplete per-LOD stream mapping");
            }
            const auto& indices = lod.extension.at("vqmmc_indices");
            const auto& codebook = lod.extension.at("vqmmc_codebook");
            const std::size_t length = lod.F.size() + lod.Et.size() + indices.size() + codebook.size();
            if (lod.payloadByteOffset != offset || lod.payloadByteLength != length) {
                throw std::runtime_error("VQMMC relational paper reader: inconsistent payload directory");
            }
            offset += length;
            builder.appendStreamBytes(lod.lodId, 0, lod.F);
            builder.appendStreamBytes(lod.lodId, 1, lod.Et);
            builder.appendStreamBytes(lod.lodId, 2, indices);
            builder.appendStreamBytes(lod.lodId, 3, codebook);
        }
        payload = table.partitions.front().baseMesh;
        if (payload.empty()) throw std::runtime_error("VQMMC relational paper reader: missing base prefix");
        if (queryMaxLod_ >= 0) {
            PMSFHeader header;
            std::size_t offset = 0;
            header.deserialize(reinterpret_cast<char*>(payload.data()), offset);
            header.setRound(static_cast<int>(table.lods.size()));
            offset = 0;
            header.serialize(reinterpret_cast<char*>(payload.data()), offset);
        }
        const auto blob = builder.finalize();
        payload.insert(payload.end(), blob.begin(), blob.end());
    } else {
    if (!pmsf::paper_opaque::readPaperStreamV2OpaquePayloadArtifact(paperPath, &payload, &codec,
                                                      &error)) {
        throw std::runtime_error(
            "[VQMMC][paper] failed to read opaque payload artifact '" + paperPath +
            "': " + error);
    }
    }
    // The canonical binary artifact is [u32 contentLen][buffer(contentLen)].
    // The container already carries the payload length, so accept both the
    // "full file verbatim" (strip the legacy prefix) and the "body only" form.
    const char* body = reinterpret_cast<const char*>(payload.data());
    std::size_t bodyLen = payload.size();
    if (payload.size() >= 4) {
        std::uint32_t prefix = 0;
        std::memcpy(&prefix, payload.data(), sizeof(prefix));
        if (static_cast<std::size_t>(prefix) + 4u == payload.size()) {
            body += 4;
            bodyLen = static_cast<std::size_t>(prefix);
        }
    }
    buffer_ = new char[bodyLen];
    std::memcpy(buffer_, body, bodyLen);
    totalSize = bodyLen;
    setStrictMode(true);
#else
    throw std::runtime_error(
        "[VQMMC] Paper artifact decode requires the "
        "PaperStreamOpaquePayloadV2 storage helper (not built in this "
        "configuration); this is an M5-5 pending build.");
#endif
}
