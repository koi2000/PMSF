/*
 * @Author: koi
 * @Date: 2025-09-02 10:35:42
 * @Description:
 */
#include "DeserializeOperator.h"
#include "BufferUtils.h"
#include "EntropyCodec.h"
#include "storage/PMSFPMSFLegacySingleFilePayloadAdapter.h"
#include "storage/PMSFSymbolStorageAdapter.h"
#ifdef COLUMNAR_SUPPORT
#include "storage/PMSFSymbolPayloadBlobReader.h"
#include "storage/PMSFSymbolSidecarReader.h"
#endif
#include <cstddef>
#include <cstring>
#include <fstream>
#include <memory>
#include <string>
#include <utility>
#include <vector>

void DeserializeOperator::init(std::string path) {
    clearPMSFSymbolContext();
    dataOffset_ = 0;
    pointOffset_ = 0;
    totalSize = 0;
    if (!readBuffer(path)) {
        restoreLegacySingleFileBufferFromPMSFPayload(path);
        return;
    }
    loadPMSFSymbolSidecarIfPresent(path);
    // deserializeHeader(header_);
}

void DeserializeOperator::init(char* buffer) {
    clearPMSFSymbolContext();
    dataOffset_ = 0;
    pointOffset_ = 0;
    totalSize = 0;
    buffer_ = buffer;
}

void DeserializeOperator::clearPMSFSymbolContext() {
    pmsfSymbolTable_.clear();
    paperSymbolPayloadTable_.clear();
    paperSymbolSidecarPath_.clear();
    paperSymbolPayloadBlobPath_.clear();
    hasPMSFSymbolTable_ = false;
    hasPMSFSymbolPayloadTable_ = false;
    if (symbolReadOperator_) {
        symbolReadOperator_->setPMSFSymbolContext(nullptr, nullptr);
    }
}

void DeserializeOperator::setPMSFSymbolTable(pmsf::PMSFSymbolTable table) {
    pmsfSymbolTable_ = std::move(table);
    hasPMSFSymbolTable_ = true;
    if (symbolReadOperator_) {
        symbolReadOperator_->setPMSFSymbolContext(&pmsfSymbolTable_,
                                                   hasPMSFSymbolPayloadTable_ ? &paperSymbolPayloadTable_ : nullptr);
    }
}

void DeserializeOperator::setPMSFSymbolPayloadTable(pmsf::PMSFSymbolPayloadTable table) {
    paperSymbolPayloadTable_ = std::move(table);
    hasPMSFSymbolPayloadTable_ = true;
    if (symbolReadOperator_) {
        symbolReadOperator_->setPMSFSymbolContext(hasPMSFSymbolTable_ ? &pmsfSymbolTable_ : nullptr,
                                                   &paperSymbolPayloadTable_);
    }
}

const pmsf::PMSFSymbolPayloadRecord* DeserializeOperator::findPMSFSymbolPayload(int objectId,
                                                                                  int partitionId,
                                                                                  int lodId,
                                                                                  const std::string& payloadName) const {
    if (!hasPMSFSymbolPayloadTable_) {
        return nullptr;
    }
    return pmsf::findPMSFPayloadByNameOrSafeRole(
        paperSymbolPayloadTable_, objectId, partitionId, lodId, payloadName);
}

const std::vector<std::uint8_t>* DeserializeOperator::findPMSFSymbolPayloadBytes(
    int objectId,
    int partitionId,
    int lodId,
    const std::string& payloadName) const {
    const auto* row = findPMSFSymbolPayload(objectId, partitionId, lodId, payloadName);
    if (!row || row->payload.empty()) {
        return nullptr;
    }
    return &row->payload;
}

const pmsf::PMSFSymbolPayloadRecord*
DeserializeOperator::findPMSFSymbolPayloadByRole(
    pmsf::PMSFPayloadRole role,
    int objectId,
    int partitionId,
    int lodId) const {
    if (!hasPMSFSymbolPayloadTable_) {
        return nullptr;
    }
    return pmsf::findPMSFPayloadByRole(
        paperSymbolPayloadTable_, role, objectId, partitionId, lodId);
}

const std::vector<std::uint8_t>*
DeserializeOperator::findPMSFSymbolPayloadBytesByRole(
    pmsf::PMSFPayloadRole role,
    int objectId,
    int partitionId,
    int lodId) const {
    const auto* row = findPMSFSymbolPayloadByRole(
        role, objectId, partitionId, lodId);
    if (!row || row->payload.empty()) {
        return nullptr;
    }
    return &row->payload;
}

const pmsf::PMSFSymbolPayloadRecord*
DeserializeOperator::findFirstPMSFSymbolPayloadByCodecSuffix(
    const std::string& codecSuffix) const {
    if (!hasPMSFSymbolPayloadTable_ || codecSuffix.empty()) {
        return nullptr;
    }
    for (const auto& row : paperSymbolPayloadTable_) {
        if (row.codecName.size() < codecSuffix.size()) {
            continue;
        }
        if (row.codecName.compare(row.codecName.size() - codecSuffix.size(),
                                  codecSuffix.size(),
                                  codecSuffix) == 0) {
            return &row;
        }
    }
    return nullptr;
}

const pmsf::PMSFSymbolPayloadRecord*
DeserializeOperator::findFirstPMSFSymbolPayloadByRoleAndCodecSuffix(
    pmsf::PMSFPayloadRole role,
    const std::string& codecSuffix) const {
    if (!hasPMSFSymbolPayloadTable_ || codecSuffix.empty()) {
        return nullptr;
    }
    for (const auto& row : paperSymbolPayloadTable_) {
        if (!pmsf::paperPayloadRecordMatchesRole(row, role)) {
            continue;
        }
        if (row.codecName.size() < codecSuffix.size()) {
            continue;
        }
        if (row.codecName.compare(row.codecName.size() - codecSuffix.size(),
                                  codecSuffix.size(),
                                  codecSuffix) == 0) {
            return &row;
        }
    }
    return nullptr;
}

bool DeserializeOperator::restoreBufferFromPMSFPayload(const std::string& path,
                                                        const std::string& payloadName) {
    loadPMSFSymbolSidecarIfPresent(path);
    const auto* payload = findPMSFSymbolPayloadBytes(0, 0, 0, payloadName);
    if (!payload) {
        return false;
    }
    const auto len = static_cast<int>(payload->size());
    buffer_ = new char[len];
    std::memcpy(buffer_, payload->data(), payload->size());
    totalSize = len;
    dataOffset_ = 0;
    pointOffset_ = 0;
    return true;
}

bool DeserializeOperator::restoreLegacySingleFileBufferFromPMSFPayload(
    const std::string& path) {
    const auto spec = pmsf::legacySingleFileBinaryPayloadSpec();
    return restoreBufferFromPMSFPayload(path, spec.payloadName);
}

void DeserializeOperator::propagatePMSFSymbolContextTo(
    const std::shared_ptr<DeserializeOperator>& inner) const {
    if (!inner) {
        return;
    }
    if (hasPMSFSymbolTable_) {
        inner->setPMSFSymbolTable(pmsfSymbolTable_);
    }
    if (hasPMSFSymbolPayloadTable_) {
        inner->setPMSFSymbolPayloadTable(paperSymbolPayloadTable_);
    }
}

void DeserializeOperator::loadPMSFSymbolSidecarIfPresent(const std::string& path) {
#ifdef COLUMNAR_SUPPORT
    auto symbols = pmsf::readFirstPMSFSymbolSidecarCandidate(path);
    if (!symbols.table.empty()) {
        setPMSFSymbolTable(std::move(symbols.table));
        paperSymbolSidecarPath_ = std::move(symbols.path);
    }
    auto payloads = pmsf::readFirstPMSFPayloadBlobTableCandidate(path);
    if (!payloads.payloads.empty()) {
        setPMSFSymbolPayloadTable(std::move(payloads.payloads));
        paperSymbolPayloadBlobPath_ = std::move(payloads.path);
        return;
    }
#else
    (void)path;
#endif
}

void DeserializeOperator::deserializeInt(int& val) {
    val = readInt(buffer_, dataOffset_);
}

bool DeserializeOperator::readBuffer(std::string path) {
    std::ifstream fin(path, std::ios::binary);
    if (!fin.is_open()) {
        return false;
    }
    int len2 = 0;
    if (!fin.read((char*)&len2, sizeof(int)) || len2 < 0) {
        return false;
    }

    buffer_ = new char[len2];
    memset(buffer_, 0, len2);
    if (!fin.read(buffer_, len2)) {
        delete[] buffer_;
        buffer_ = nullptr;
        totalSize = 0;
        return false;
    }
    totalSize = len2;
    return true;
}
