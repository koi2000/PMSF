/*
 * @file PMSFConfigParser.cpp
 * @brief Phase A/B: JSON 配置解析实现。
 */
#include "PMSFConfigParser.h"

#include "algorithm/SimplificationAlgorithmRegistry.h"

#include "json.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace pmsf {

namespace {

using json = nlohmann::json;

std::string toLower(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return s;
}

Backend parseBackend(const std::string& s) {
    const std::string l = toLower(s);
    if (l == "binary" || l == "bin") return Backend::Binary;
    if (l == "columnar" || l == "parquetv2" || l == "v2") return Backend::Columnar;
    if (l == "unified" || l == "parquetv3" || l == "v3") return Backend::Unified;
    if (l == "paper" || l == "paperstream") return Backend::Paper;
    throw std::runtime_error("unknown backend: " + s);
}

template <class T>
T getOrDefault(const json& j, const char* key, const T& def) {
    if (j.contains(key) && !j.at(key).is_null()) {
        return j.at(key).get<T>();
    }
    return def;
}

}  // namespace

/// 每算法的默认轮数。键是 `algorithmName`（注册表的字符串键）。
///
/// 这张表在 Phase 4 之前是**死的**：它 switch 的是那个 preset 枚举字段，
/// 而那个字段自 Phase 1 起恒为 `Default`，所以无论配的是哪个算法都返回 30。
/// 改成按名字查之后语义才与实际一致（`PPMC` 15 轮 / `PCLTTMValence` 48 轮）。
///
/// 当前 15 份 config 都显式写了 `rounds`，因此本次改动不影响任何既有产物；
/// 表的值保持历史数值不变。
int PMSFConfig::defaultRoundsForAlgorithm() const {
    // 向**算法自己**要默认轮数，而不是在 src/ 里维护一张「算法名 -> 轮数」的表。
    //
    // 那张表有过两次静默错位：`pomar_v2` 随变体退役后成了死条目；
    // `pclttm_valence` 因为注册名实际是 `pclttmvalence`（toLower 不去下划线）
    // 而**从未命中**，一直静默回落到 30。让算法声明自己的默认值，
    // 这类"键与注册名不一致"的错位在结构上就不可能出现。
    auto& reg = SimplificationAlgorithmRegistry::instance();
    if (reg.has(algorithmName)) {
        if (auto algo = reg.create(algorithmName)) {
            return algo->defaultRounds();
        }
    }
    return 30;
}

PMSFConfig parseConfigFromString(const std::string& jsonText) {
    json j;
    try {
        j = json::parse(jsonText);
    } catch (const std::exception& e) {
        throw std::runtime_error(std::string("failed to parse JSON: ") + e.what());
    }

    PMSFConfig cfg;

    if (!j.contains("algorithm")) {
        throw std::runtime_error("config missing required field: algorithm");
    }
    // 算法名是注册表键，统一规范成小写（注册约定：小写、无空格）。配置里写
    // "QEM"/"qem"/"Qem" 都应命中同一个插件。
    cfg.algorithmName = toLower(j.at("algorithm").get<std::string>());

    cfg.backend = parseBackend(getOrDefault<std::string>(j, "backend", "columnar"));
    cfg.input = getOrDefault<std::string>(j, "input", "");
    cfg.output = getOrDefault<std::string>(j, "output", "");
    cfg.rounds = getOrDefault<int>(j, "rounds", -1);
    cfg.maxLod = getOrDefault<int>(j, "max_lod", -1);
    cfg.quantizationSet = j.contains("quantization");
    cfg.segmentsSet = j.contains("segments");
    cfg.segments = getOrDefault<int>(j, "segments", 0);
    cfg.quantization = getOrDefault<bool>(j, "quantization", false);
    cfg.prediction = getOrDefault<bool>(j, "prediction", false);
    cfg.segmentation = getOrDefault<bool>(j, "segmentation", false);
    cfg.adjustment = getOrDefault<bool>(j, "adjustment", false);
    cfg.compression = getOrDefault<bool>(j, "compression", false);
    if (j.contains("paper_storage")) {
        if (cfg.backend != Backend::Paper || !j.at("paper_storage").is_object()) {
            throw std::runtime_error("paper_storage requires backend=paper and an object");
        }
        const auto& storage = j.at("paper_storage");
        const std::string schema = storage.value("schema", std::string("typed"));
        const std::string payload = storage.value("payload", std::string("native"));
        if (schema != "typed" && schema != "compact") {
            throw std::runtime_error("paper_storage.schema must be typed or compact");
        }
        if (payload != "typed" && payload != "native") {
            throw std::runtime_error("paper_storage.payload must be typed or native");
        }
        // Explicit JSON options use existing transport switches; no new env is
        // introduced, and the resulting artifact records the chosen schema.
        cfg.extra["PMSF_PAPERSTREAM_V2"] = "1";
        cfg.extra["PMSF_PAPER_SCHEMA"] = schema;
        cfg.extra["PMSF_PAPERSTREAM_V2_PAYLOAD"] = payload;
    }
    if (j.contains("boundary_compression")) {
        bool enabled = true;
        const auto& bc = j.at("boundary_compression");
        if (bc.is_boolean()) {
            enabled = bc.get<bool>();
        } else if (bc.is_object()) {
            enabled = getOrDefault<bool>(bc, "enabled", true);
        }
        cfg.boundaryCompression = enabled;
        cfg.extra["PMSF_BOUNDARY_MODULE"] = enabled ? "1" : "0";
        // The module's pre-pass can expose bbox-max quantization vertices;
        // clamp the encoder to the representable nbQuantBits range so the
        // base-mesh writer and in-memory mesh agree.  Default path untouched.
        if (enabled) cfg.extra["PMSF_QUANT_MAX_CLAMP"] = "1";
    }
    // Optional compact boundary storage (does not touch the default format).
    // Example: "boundary_storage": { "mode": 30, "group_channel_varint": true }
    if (j.contains("boundary_storage")) {
        const auto& bs = j.at("boundary_storage");
        if (bs.is_boolean()) {
            if (bs.get<bool>()) cfg.extra["PMSF_BOUNDARY_STORAGE_MODE"] = "30";
        } else if (bs.is_object()) {
            const int mode = getOrDefault<int>(bs, "mode", 30);
            if (mode == 30 || mode == 31) {
                cfg.extra["PMSF_BOUNDARY_STORAGE_MODE"] = std::to_string(mode);
            }
            if (getOrDefault<bool>(bs, "group_channel_varint", false)) {
                cfg.extra["PMSF_SEGMENTED_GROUP_CHANNEL_VARINT"] = "1";
            }
        }
    }
    // Lossy boundary topology compression (two-stage): stage 1 simplifies the
    // boundary and dumps the simplified mesh; stage 2 compresses it normally.
    if (j.contains("boundary_topology_compression")) {
        const auto& btc = j.at("boundary_topology_compression");
        const bool enabled = btc.is_object() ? getOrDefault<bool>(btc, "enabled", true)
                                             : btc.get<bool>();
        if (enabled) {
            const int maxCollapse = btc.is_object()
                                        ? getOrDefault<int>(btc, "max_collapse", 1000)
                                        : 1000;
            cfg.extra["PMSF_BOUNDARY_COLLAPSE"] = "1";
            cfg.extra["PMSF_BOUNDARY_COLLAPSE_MAX"] = std::to_string(maxCollapse);
            cfg.extra["PMSF_GROUP_ANCHORS"] = "1";
        }
    }
    if (j.contains("round_stage_order")) {
        const auto& rso = j.at("round_stage_order");
        if (rso.is_array()) {
            std::string joined;
            for (const auto& stage : rso) {
                if (!stage.is_string()) continue;
                if (!joined.empty()) joined += ",";
                joined += stage.get<std::string>();
            }
            cfg.extra["PMSF_ROUND_STAGE_ORDER"] = joined;
        } else if (rso.is_string()) {
            cfg.extra["PMSF_ROUND_STAGE_ORDER"] = rso.get<std::string>();
        }
    }
    if (j.contains("anchor_one_ring_lock")) {
        const auto& lock = j.at("anchor_one_ring_lock");
        const bool enabled = lock.is_boolean() ? lock.get<bool>()
                                              : getOrDefault<bool>(lock, "enabled", true);
        if (enabled) cfg.extra["PMSF_ANCHOR_ONERING_LOCK"] = "1";
    }
    // First-class multi-round boundary staging configuration (see
    // docs/memory/BOUNDARY_COMPRESSION_DECISIONS.md, D485/D487).  The encoder
    // entry point turns cfg.extra entries into env vars, so these keys are a
    // self-describing user surface on top of the existing env plumbing:
    //
    //   "multi_round": {"rounds": [3, 7], "max_ops": 3,
    //                   "pre": true,              // all stage rounds PRE
    //                   "pre_rounds": [3]}        // mixed: listed rounds PRE
    //
    // Omitting "multi_round" or writing false keeps the legacy default path.
    if (j.contains("multi_round")) {
        const auto& mr = j.at("multi_round");
        bool enabled = false;
        if (mr.is_boolean()) {
            enabled = mr.get<bool>();
        } else if (mr.is_object()) {
            enabled = getOrDefault<bool>(mr, "enabled", true);
        }
        if (enabled) {
            cfg.extra["PMSF_SC_ALLOW_MULTIROUND"] = "1";
            if (mr.is_object()) {
                auto csvRounds = [](const json& v) -> std::string {
                    std::string out;
                    if (v.is_string()) return v.get<std::string>();
                    if (v.is_array()) {
                        for (const auto& item : v) {
                            if (item.is_number_integer()) {
                                if (!out.empty()) out += ",";
                                out += std::to_string(item.get<int>());
                            } else if (item.is_string()) {
                                if (!out.empty()) out += ",";
                                out += item.get<std::string>();
                            }
                        }
                    }
                    return out;
                };
                if (mr.contains("rounds")) {
                    const std::string rounds = csvRounds(mr.at("rounds"));
                    if (!rounds.empty()) cfg.extra["PMSF_SC_BOUNDARY_ROUNDS"] = rounds;
                }
                if (mr.contains("max_ops")) {
                    cfg.extra["PMSF_SC_BOUNDARY_MAXOPS"] = std::to_string(getOrDefault<int>(mr, "max_ops", 3));
                }
                if (getOrDefault<bool>(mr, "pre", false)) {
                    cfg.extra["PMSF_SC_BOUNDARY_PRE"] = "1";
                }
                if (mr.contains("pre_rounds")) {
                    const std::string preRounds = csvRounds(mr.at("pre_rounds"));
                    if (!preRounds.empty()) cfg.extra["PMSF_SC_BOUNDARY_PRE_ROUNDS"] = preRounds;
                }
            }
        }
    }
    cfg.alpha = getOrDefault<double>(j, "alpha", -1.0);
    cfg.codebookSize = getOrDefault<int>(j, "codebookSize", -1);
    cfg.variant = getOrDefault<std::string>(j, "variant", "");
    cfg.storageScheme = getOrDefault<std::string>(j, "storage_scheme", "");

    if (j.contains("algorithm_options") && j.at("algorithm_options").is_object()) {
        cfg.algorithmOptions = j.at("algorithm_options");
    }

    if (j.contains("hooks") && j.at("hooks").is_object()) {
        const json& h = j.at("hooks");
        cfg.hooks.builtin = getOrDefault<std::string>(h, "builtin", "");
        cfg.hooks.plugin = getOrDefault<std::string>(h, "plugin", "");
        cfg.hooks.factory = getOrDefault<std::string>(h, "factory", "pmsf_create_error_metric_v1");
    }

    if (j.contains("extra") && j.at("extra").is_object()) {
        for (auto it = j.at("extra").begin(); it != j.at("extra").end(); ++it) {
            const auto& v = it.value();
            if (v.is_string()) {
                cfg.extra[it.key()] = v.get<std::string>();
            } else if (v.is_boolean()) {
                cfg.extra[it.key()] = v.get<bool>() ? "1" : "0";
            } else if (v.is_number_integer()) {
                cfg.extra[it.key()] = std::to_string(v.get<int64_t>());
            } else if (v.is_number_float()) {
                cfg.extra[it.key()] = std::to_string(v.get<double>());
            } else {
                cfg.extra[it.key()] = v.dump();
            }
        }
    }

    return cfg;
}

PMSFConfig parseConfigFromFile(const std::string& path) {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
        throw std::runtime_error("cannot open config file: " + path);
    }
    std::ostringstream oss;
    oss << ifs.rdbuf();
    return parseConfigFromString(oss.str());
}

}  // namespace pmsf
