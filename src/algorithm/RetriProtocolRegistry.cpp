/*
 * @file RetriProtocolRegistry.cpp
 * @brief 见 RetriProtocolRegistry.h。
 */
#include "algorithm/RetriProtocolRegistry.h"

#include <cstdlib>
#include <cstring>

namespace pmsf {
namespace {

RetriCompressOverrideFn& compressSlot() {
    static RetriCompressOverrideFn fn;
    return fn;
}

RetriDecompressOverrideFn& decompressSlot() {
    static RetriDecompressOverrideFn fn;
    return fn;
}

RetriDeserializerFactoryFn& deserializerSlot() {
    static RetriDeserializerFactoryFn fn;
    return fn;
}

RetriPaperDeserializerFactoryFn& paperDeserializerSlot() {
    static RetriPaperDeserializerFactoryFn fn;
    return fn;
}

RetriReaderPairFactoryFn& readerPairSlot() {
    static RetriReaderPairFactoryFn fn;
    return fn;
}

RetriDecodeSideEffectsFn& decodeSideEffectsSlot() {
    static RetriDecodeSideEffectsFn fn;
    return fn;
}

}  // namespace

bool paperRetriEnabled() {
    const char* value = std::getenv("PMSF_PAPER_RETRI");
    return value != nullptr && *value != '\0' && std::strcmp(value, "0") != 0;
}

void registerRetriCompressOverride(RetriCompressOverrideFn fn) {
    compressSlot() = std::move(fn);
}

void registerRetriDecompressOverride(RetriDecompressOverrideFn fn) {
    decompressSlot() = std::move(fn);
}

void registerRetriDeserializerFactory(RetriDeserializerFactoryFn fn) {
    deserializerSlot() = std::move(fn);
}

void registerRetriPaperDeserializerFactory(RetriPaperDeserializerFactoryFn fn) {
    paperDeserializerSlot() = std::move(fn);
}

void registerRetriReaderPairFactory(RetriReaderPairFactoryFn fn) {
    readerPairSlot() = std::move(fn);
}

void registerRetriDecodeSideEffects(RetriDecodeSideEffectsFn fn) {
    decodeSideEffectsSlot() = std::move(fn);
}

bool applyRetriDecodeSideEffects(const std::string& protocol) {
    auto& fn = decodeSideEffectsSlot();
    if (!fn) return false;
    return fn(protocol);
}

bool tryRunRetriCompressOverride(CompressOptions& options, const std::string& protocol,
                                 const RetriProtocolTuning& tuning, Backend backend) {
    if (protocol.empty() || protocol == "none") return false;
    const auto& fn = compressSlot();
    if (!fn) return false;
    return fn(options, protocol, tuning, backend);
}

std::shared_ptr<DeserializeOperator> makeRetriDeserializer(
    const std::string& protocol, std::shared_ptr<SymbolReadOperator> defaultReader,
    std::shared_ptr<ReconstructOperator> defaultReconstructor) {
    auto& fn = deserializerSlot();
    if (!fn) {
        return nullptr;
    }
    return fn(protocol, std::move(defaultReader), std::move(defaultReconstructor));
}

std::shared_ptr<DeserializeOperator> makeRetriPaperDeserializer(
    const std::string& protocol, std::shared_ptr<SymbolReadOperator> defaultReader,
    std::shared_ptr<ReconstructOperator> defaultReconstructor) {
    auto& fn = paperDeserializerSlot();
    if (!fn) return nullptr;
    return fn(protocol, std::move(defaultReader), std::move(defaultReconstructor));
}

bool makeRetriReaderPair(const std::string& protocol,
                         std::shared_ptr<SymbolReadOperator>& reader,
                         std::shared_ptr<ReconstructOperator>& reconstructor) {
    auto& fn = readerPairSlot();
    if (!fn) return false;
    return fn(protocol, reader, reconstructor);
}

bool tryRunRetriDecompressOverride(const std::string& protocol,
                                   const std::string& inputPath,
                                   const std::string& outputPath) {
    if (protocol.empty() || protocol == "none") return false;
    const auto& fn = decompressSlot();
    if (!fn) return false;
    return fn(protocol, inputPath, outputPath);
}

}  // namespace pmsf
