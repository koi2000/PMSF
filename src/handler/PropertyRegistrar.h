/*
 * @file PropertyRegistrar.h
 * @brief Static initializer macro so algorithms can register their own
 *        per-mesh properties without editing PMSF.cpp.
 *
 * Before Phase 12, `src/PMSF.cpp::RegisterProperties()` contained a
 * hand-maintained list of `REGISTER_PROPERTY_HANDLE(...)` calls — one per
 * algorithm-specific attribute (QEM quadrics, PPMC dirty flags, MMSE
 * clusters, …). Adding a new paper meant touching PMSF.cpp, which
 * violated the "zero core-file edits per new algorithm" contract.
 *
 * This header introduces an additive registrar: algorithms put a single
 * `PMSF_REGISTER_PROPERTY(name, lambda)` line at translation-unit scope
 * inside their `impl/<ALGO>/<Algo>Compress.cpp`, and the registrar queues
 * the lambda to be executed during pipeline setup. The legacy
 * PMSF::RegisterProperties() path is untouched — Phase 12 is purely
 * additive to preserve every existing algorithm's behaviour.
 *
 * A Phase-13 algorithm migration will move its REGISTER_PROPERTY_HANDLE
 * call out of PMSF.cpp into its own .cpp via this macro, one at a time.
 */
#ifndef PMSF_HANDLER_PROPERTY_REGISTRAR_H
#define PMSF_HANDLER_PROPERTY_REGISTRAR_H

#include <functional>
#include <string>
#include <vector>

namespace pmsf {

using PropertyRegistrarFn = std::function<void()>;

class PropertyRegistry {
public:
    static PropertyRegistry& instance() {
        static PropertyRegistry r;
        return r;
    }

    /// Queue `fn` to run once, the next time `runAll()` is called.
    void add(std::string name, PropertyRegistrarFn fn) {
        entries_.push_back({std::move(name), std::move(fn)});
    }

    /// Run every queued registrar and clear the queue. Idempotent: safe
    /// to call multiple times (subsequent calls just see an empty queue).
    void runAll() {
        for (auto& e : entries_) {
            if (e.fn) e.fn();
        }
        entries_.clear();
    }

    std::size_t pending() const { return entries_.size(); }

private:
    struct Entry { std::string name; PropertyRegistrarFn fn; };
    std::vector<Entry> entries_;
};

/// Helper that schedules a lambda via a static initializer.
class PropertyRegistrarGuard {
public:
    PropertyRegistrarGuard(const char* name, PropertyRegistrarFn fn) {
        PropertyRegistry::instance().add(name, std::move(fn));
    }
};

}  // namespace pmsf

/// Declare a translation-unit-scope registrar. Expands to a static object
/// whose constructor enqueues `fn` into the global PropertyRegistry. The
/// algorithm main calls `pmsf::PropertyRegistry::instance().runAll()` once
/// before compression — or inherits the default call from its pipeline
/// Builder (Phase 13 wiring).
///
/// Usage:
///   PMSF_REGISTER_PROPERTY("vpQuadrics", []() {
///       REGISTER_PROPERTY_HANDLE(vpQuadrics, Vertex, SymetricMatrix);
///   });
#define PMSF_PP_CAT_INNER(a, b) a##b
#define PMSF_PP_CAT(a, b) PMSF_PP_CAT_INNER(a, b)
#define PMSF_REGISTER_PROPERTY(NAME, LAMBDA)                         \
    namespace {                                                      \
    static ::pmsf::PropertyRegistrarGuard                            \
        PMSF_PP_CAT(_pmsf_reg_, __COUNTER__)(NAME, LAMBDA);          \
    }

#endif  // PMSF_HANDLER_PROPERTY_REGISTRAR_H
