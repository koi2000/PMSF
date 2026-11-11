/*
 * @file DecompressOperatorRegistry.cpp
 * @brief DecompressOperatorRegistry 实现。
 */
#include "core/DecompressOperatorRegistry.h"

#include <stdexcept>

namespace pmsf {

DecompressOperatorRegistry& DecompressOperatorRegistry::instance() {
    static DecompressOperatorRegistry registry;
    return registry;
}

namespace {
inline std::size_t backendIndex(Backend backend) {
    const auto i = static_cast<std::size_t>(backend);
    return i < 4 ? i : 0;
}
}  // namespace

void DecompressOperatorRegistry::registerFactoryByName(const std::string& name,
                                                       Backend backend,
                                                       DecompressOperatorFactory factory) {
    if (name.empty() || !factory) return;
    namedEntries_[backendIndex(backend)][name] = std::move(factory);
}

bool DecompressOperatorRegistry::hasName(const std::string& name, Backend backend) const {
    const auto& table = namedEntries_[backendIndex(backend)];
    return table.find(name) != table.end();
}

DecompressOperatorTriple DecompressOperatorRegistry::makeTripleByName(const std::string& name,
                                                                     Backend backend) const {
    const auto& table = namedEntries_[backendIndex(backend)];
    auto it = table.find(name);
    if (it == table.end()) {
        throw std::runtime_error(
            "DecompressOperatorRegistry: no factory registered for algorithm name='" + name +
            "' backend=" + std::to_string(static_cast<int>(backend)));
    }
    return it->second();
}

NamedDecompressOperatorRegistrar::NamedDecompressOperatorRegistrar(
    std::string name, Backend backend, DecompressOperatorFactory factory) {
    DecompressOperatorRegistry::instance().registerFactoryByName(std::move(name), backend,
                                                                 std::move(factory));
}

}  // namespace pmsf
