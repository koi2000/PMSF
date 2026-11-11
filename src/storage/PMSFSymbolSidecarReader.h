/*
 * @file PMSFSymbolSidecarReader.h
 * @brief Read unified paper-symbol sidecars produced during storage migration.
 */
#ifndef PMSF_STORAGE_PAPER_SYMBOL_SIDECAR_READER_H
#define PMSF_STORAGE_PAPER_SYMBOL_SIDECAR_READER_H

#include "compress/UnifiedDeserializeOperator.h"
#include "storage/PMSFSymbolStorageAdapter.h"

#include <fstream>
#include <stdexcept>
#include <string>
#include <utility>

namespace pmsf {

struct PMSFSymbolSidecarReadResult {
    std::string path;
    PMSFSymbolTable table;
};

inline bool paperSymbolSidecarExists(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    return f.good();
}

inline PMSFSymbolTable readPMSFSymbolSidecar(const std::string& path) {
    if (!paperSymbolSidecarExists(path)) return {};
    UnifiedDeserializeOperator reader;
    SymbolStream stream = reader.deserialize(path);
    if (reader.hasPMSFSymbolTableMetadata() && !reader.isPMSFSymbolTable()) {
        return {};
    }
    PMSFSymbolTable table = pmsfSymbolTableFromStorageStream(stream);
    for (const auto& row : table) {
        visitPMSFSideChannelCellAt(
            row.delta, kPaperRelationModelVersionSideChannel, 0, [&](const auto& cell) {
            const std::int32_t version =
                paperSideCellAsInt32(cell, kPaperRelationModelVersion);
            if (version != kPaperRelationModelVersion) {
                throw std::runtime_error(
                    "unsupported paper relation model version in " + path +
                    ": " + std::to_string(version));
            }
        });
        visitPMSFSideChannelCellAt(
            row.delta, kPaperDeltaTupleAritySideChannel, 0, [&](const auto& cell) {
            const std::int32_t arity =
                paperSideCellAsInt32(cell, kPaperDeltaTupleArity);
            if (arity != kPaperDeltaTupleArity) {
                throw std::runtime_error(
                    "unsupported paper Delta tuple arity in " + path +
                    ": " + std::to_string(arity));
            }
        });
    }
    return table;
}

inline PMSFSymbolSidecarReadResult readFirstPMSFSymbolSidecarCandidate(
    const std::string& path) {
    for (const auto& candidate : paperSymbolSidecarCandidates(path)) {
        if (!paperSymbolSidecarExists(candidate)) {
            continue;
        }
        auto symbols = readPMSFSymbolSidecar(candidate);
        if (symbols.empty()) {
            continue;
        }
        return {candidate, std::move(symbols)};
    }
    return {};
}

}  // namespace pmsf

#endif  // PMSF_STORAGE_PAPER_SYMBOL_SIDECAR_READER_H
