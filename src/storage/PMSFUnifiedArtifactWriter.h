#ifndef PMSF_UNIFIED_PAPER_ARTIFACT_WRITER_H
#define PMSF_UNIFIED_PAPER_ARTIFACT_WRITER_H

#include "compress/SymbolCollectOperator.h"
#include "core/PMSFSideChannel.h"
#include "storage/PMSFStorageContextSideChannel.h"
#include "storage/PMSFSymbolPayloadBlobWriter.h"
#include "storage/PMSFSymbolSidecarWriter.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include <cstdint>
#include <cstdio>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace pmsf {

inline bool collectorHasPMSFSymbolTable(
    const std::shared_ptr<SymbolCollectOperator>& collector) {
    if (!collector) {
        return false;
    }
    const auto* table = collector->pmsfSymbolTable();
    return table != nullptr && !table->empty();
}

inline bool collectorHasPMSFSymbolMapping(
    const std::shared_ptr<SymbolCollectOperator>& collector) {
    if (!collector) {
        return false;
    }
    const auto* table = collector->pmsfSymbolTable();
    return table != nullptr && pmsfSymbolTableHasSymbolMapping(*table);
}

inline std::string stripOutputExtensionForPMSFArtifacts(const std::string& path) {
    return paperStorageStripExtension(path);
}

#ifdef COLUMNAR_SUPPORT
inline std::string paperPayloadBlobOutputPathForBase(const std::string& baseName) {
    const auto candidates = paperPayloadBlobCandidates(baseName);
    return candidates.empty() ? std::string() : candidates.front();
}

inline bool writePaperNamedBlobsForBase(const std::string& baseName,
                                        const std::vector<PaperNamedBlob>& blobs) {
    if (blobs.empty()) {
        return true;
    }
    const std::string path = paperPayloadBlobOutputPathForBase(baseName);
    return !path.empty() && writePaperNamedBlobTable(blobs, path);
}

inline bool writePMSFPayloadRowsForBase(const std::string& baseName,
                                         const PMSFSymbolPayloadTable& rows) {
    return writePaperNamedBlobsForBase(baseName, paperNamedBlobsFromPayloadRows(rows));
}

inline void removePaperSidecarAndPayloadBlobsForBase(const std::string& baseName) {
    std::remove(paperSymbolSidecarPath(baseName).c_str());
    for (const auto& candidate : paperPayloadBlobCandidates(baseName)) {
        std::remove(candidate.c_str());
    }
}
#endif

inline void writeUnifiedPMSFArtifactsForCollector(
    const std::string& outputPathOrBase,
    const std::shared_ptr<SymbolCollectOperator>& collector,
    bool stripExtension = true) {

#ifdef COLUMNAR_SUPPORT
    if (!unifiedStorageEnabled() || !collector) {
        return;
    }

    const std::string baseName = stripExtension
                                     ? stripOutputExtensionForPMSFArtifacts(outputPathOrBase)
                                     : outputPathOrBase;

    if (const auto* table = collector->pmsfSymbolTable()) {
        writePMSFSymbolSidecar(*table, paperSymbolSidecarPath(baseName));
    }

    const auto* payloads = collector->paperSymbolPayloadTable();
    if (!payloads) {
        return;
    }

    std::vector<PaperNamedBlob> blobs;
    appendPMSFPayloadRows(blobs, *payloads);
    if (blobs.empty()) {
        return;
    }

    writePaperNamedBlobsForBase(baseName, blobs);
#else
    (void)outputPathOrBase;
    (void)collector;
    (void)stripExtension;
#endif
}

#ifdef COLUMNAR_SUPPORT
inline void writePMSFPayloadOnlyFallbackSidecar(
    const std::string& baseName,
    const std::string& sideKeyPrefix,
    const std::vector<std::pair<std::string, std::int32_t>>& extraSideValues = {}) {
    if (!unifiedStorageEnabled()) {
        return;
    }

    PMSFSymbolTable table;
    PMSFSymbolRecord& row = appendPMSFSymbolRecord(table, 0, 0, 0);
    appendPMSFStorageContextMappingAndEntries(
        row.delta, sideKeyPrefix, 1, 0, extraSideValues);
    writePMSFSymbolSidecar(table, paperSymbolSidecarPath(baseName));
}

inline void writePMSFPayloadOnlyFallbackSidecarForCollector(
    const std::string& baseName,
    const std::shared_ptr<SymbolCollectOperator>& collector,
    const std::string& sideKeyPrefix,
    const std::vector<std::pair<std::string, std::int32_t>>& extraSideValues = {}) {
    if (!unifiedStorageEnabled()) {
        return;
    }

    PMSFSymbolTable table;
    if (collector) {
        if (const auto* source = collector->pmsfSymbolTable()) {
            table = *source;
        }
    }
    if (table.empty()) {
        appendPMSFSymbolRecord(table, 0, 0, 0);
    }

    appendPMSFStorageContextMappingAndEntries(
        table.front().delta, sideKeyPrefix, 1, 0, extraSideValues);
    writePMSFSymbolSidecar(table, paperSymbolSidecarPath(baseName));
}

inline void writeUnifiedPMSFArtifactsForTable(
    const std::string& outputPathOrBase,
    const PMSFSymbolTable& table,
    bool stripExtension = true) {
    if (!unifiedStorageEnabled()) {
        return;
    }
    const std::string baseName = stripExtension
                                     ? stripOutputExtensionForPMSFArtifacts(outputPathOrBase)
                                     : outputPathOrBase;
    writePMSFSymbolSidecar(table, paperSymbolSidecarPath(baseName));
}

inline void writeUnifiedPMSFArtifactsForTableAndNamedBlobs(
    const std::string& outputPathOrBase,
    const PMSFSymbolTable& table,
    std::vector<PaperNamedBlob> blobs,
    bool stripExtension = true) {
    if (!unifiedStorageEnabled()) {
        return;
    }
    const std::string baseName = stripExtension
                                     ? stripOutputExtensionForPMSFArtifacts(outputPathOrBase)
                                     : outputPathOrBase;
    writePMSFSymbolSidecar(table, paperSymbolSidecarPath(baseName));
    if (!blobs.empty()) {
        writePaperNamedBlobsForBase(baseName, blobs);
    }
}

inline void writeUnifiedPMSFArtifactsForCollectorAndNamedBlobs(
    const std::string& outputPathOrBase,
    const std::shared_ptr<SymbolCollectOperator>& collector,
    std::vector<PaperNamedBlob> blobs,
    bool stripExtension = true) {

    if (!unifiedStorageEnabled()) {
        return;
    }

    const std::string baseName = stripExtension
                                     ? stripOutputExtensionForPMSFArtifacts(outputPathOrBase)
                                     : outputPathOrBase;

    if (collector) {
        if (const auto* payloads = collector->paperSymbolPayloadTable()) {
            appendPMSFPayloadRows(blobs, *payloads);
        }
        if (const auto* table = collector->pmsfSymbolTable()) {
            writePMSFSymbolSidecar(*table, paperSymbolSidecarPath(baseName));
        }
    }

    if (!blobs.empty()) {
        writePaperNamedBlobsForBase(baseName, blobs);
    }
}
#else
inline std::string paperPayloadBlobOutputPathForBase(const std::string& baseName) {
    (void)baseName;
    return {};
}

inline bool writePaperNamedBlobsForBase(const std::string& baseName,
                                        const std::vector<PaperNamedBlob>& blobs) {
    (void)baseName;
    (void)blobs;
    return true;
}

inline bool writePMSFPayloadRowsForBase(const std::string& baseName,
                                         const PMSFSymbolPayloadTable& rows) {
    (void)baseName;
    (void)rows;
    return true;
}

inline void writePMSFPayloadOnlyFallbackSidecar(
    const std::string& baseName,
    const std::string& sideKeyPrefix,
    const std::vector<std::pair<std::string, std::int32_t>>& extraSideValues = {}) {
    (void)baseName;
    (void)sideKeyPrefix;
    (void)extraSideValues;
}

inline void writePMSFPayloadOnlyFallbackSidecarForCollector(
    const std::string& baseName,
    const std::shared_ptr<SymbolCollectOperator>& collector,
    const std::string& sideKeyPrefix,
    const std::vector<std::pair<std::string, std::int32_t>>& extraSideValues = {}) {
    (void)baseName;
    (void)collector;
    (void)sideKeyPrefix;
    (void)extraSideValues;
}

inline void writeUnifiedPMSFArtifactsForTable(
    const std::string& outputPathOrBase,
    const PMSFSymbolTable& table,
    bool stripExtension = true) {
    (void)outputPathOrBase;
    (void)table;
    (void)stripExtension;
}

template <typename BlobVector>
inline void writeUnifiedPMSFArtifactsForTableAndNamedBlobs(
    const std::string& outputPathOrBase,
    const PMSFSymbolTable& table,
    BlobVector blobs,
    bool stripExtension = true) {
    (void)outputPathOrBase;
    (void)table;
    (void)blobs;
    (void)stripExtension;
}

template <typename BlobVector>
inline void writeUnifiedPMSFArtifactsForCollectorAndNamedBlobs(
    const std::string& outputPathOrBase,
    const std::shared_ptr<SymbolCollectOperator>& collector,
    BlobVector blobs,
    bool stripExtension = true) {
    (void)outputPathOrBase;
    (void)collector;
    (void)blobs;
    (void)stripExtension;
}

inline void removePaperSidecarAndPayloadBlobsForBase(const std::string& baseName) {
    (void)baseName;
}
#endif

}  // namespace pmsf

#endif  // PMSF_UNIFIED_PAPER_ARTIFACT_WRITER_H
