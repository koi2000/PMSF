/*
 * @file DefaultVariantRegistry.cpp
 * @brief DefaultVariantRegistry 实现。
 */
#include "DefaultVariantRegistry.h"

#include <stdexcept>

namespace pmsf {

DefaultVariantRegistry& DefaultVariantRegistry::instance() {
    static DefaultVariantRegistry registry;
    return registry;
}

void DefaultVariantRegistry::registerVariant(const std::string& name,
                                             DefaultVariantFactories factories) {
    if (!factories.compress || !factories.decompress) return;
    entries_[name] = std::move(factories);
}

bool DefaultVariantRegistry::has(const std::string& name) const {
    return entries_.find(name) != entries_.end();
}

DefaultCompressSetup DefaultVariantRegistry::makeCompressSetup(const PMSFConfig& cfg) const {
    auto it = entries_.find(cfg.variant);
    if (it == entries_.end()) {
        throw std::runtime_error("DefaultVariantRegistry: unknown variant '" + cfg.variant + "'");
    }
    return it->second.compress(cfg);
}

DecompressOperatorTriple DefaultVariantRegistry::makeDecompressTriple(const PMSFConfig& cfg) const {
    auto it = entries_.find(cfg.variant);
    if (it == entries_.end()) {
        throw std::runtime_error("DefaultVariantRegistry: unknown variant '" + cfg.variant + "'");
    }
    return it->second.decompress(cfg);
}

DefaultVariantRegistrar::DefaultVariantRegistrar(const std::string& name,
                                                 DefaultVariantFactories factories) {
    DefaultVariantRegistry::instance().registerVariant(name, std::move(factories));
}

}  // namespace pmsf
