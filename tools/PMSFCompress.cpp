/*
 * @file PMSFCompress.cpp
 * @brief Phase A/B: 配置驱动的统一压缩入口。
 *
 * 用法: PMSFCompress <config.json>
 */
#include "config/PMSFConfigParser.h"
#include "algorithm/SimplificationAlgorithmRegistry.h"
#include "common/TimeRecorder.h"

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>
#include <algorithm>
#include <fstream>
#include <sstream>
#include <regex>
#include <set>
#include <filesystem>
#include <cstdio>
#include <sys/wait.h>
#include <unistd.h>


extern char** environ;

namespace {

std::string dirOf(const std::string& p) {
    const auto pos = p.find_last_of('/');
    return pos == std::string::npos ? std::string(".") : p.substr(0, pos);
}

bool runCmd(const std::string& cmd) {
    const int rc = std::system(cmd.c_str());
    if (rc == -1) return false;
    return WIFEXITED(rc) && WEXITSTATUS(rc) == 0;
}

// Verify one artifact by decoding it with the sibling PMSFDecompress and comparing
// against the original input with the sibling MeshCompare (strict counts/topology).
std::string envUnsetArgs();
double autoTolForOff(const std::string& path);

bool v3VerifyArtifact(const std::string& configPath, const std::string& artifact,
                      const std::string& origInput, const std::string& toolDir,
                      const std::string& tag, int* outFailRound = nullptr) {
    std::ifstream in(configPath);
    std::stringstream ss;
    ss << in.rdbuf();
    std::string text = ss.str();
    const std::regex rin("\"input\"\\s*:\\s*\"[^\"]*\"");
    const std::regex rout("\"output\"\\s*:\\s*\"[^\"]*\"");
    std::string off = artifact + "." + tag + ".verify.off";
    std::string decJson = std::regex_replace(text, rin, "\"input\": \"" + artifact + "\"");
    decJson = std::regex_replace(decJson, rout, "\"output\": \"" + off + "\"");
    std::string tmpJson = "/tmp/pmsf_v3_verify_" + std::to_string(::getpid()) + "_" + tag + ".json";
    {
        std::ofstream o(tmpJson);
        o << decJson;
    }
    const std::string decBin = toolDir + "/PMSFDecompress";
    const std::string decLog = artifact + "." + tag + ".dec.log";
    // Decode with a CLEAN environment (all harness PMSF_* stripped, then only
    // PMSF_BND_V3=1): verification must prove the artifact is decodable without the
    // encoder's auto-set env vars (the header must eventually self-describe the mode).
    int verifyTimeoutSec = 900;
    if (const char* vt = std::getenv("PMSF_BND_V3_VERIFY_TIMEOUT")) {
        const int v = std::atoi(vt);
        if (v > 0) verifyTimeoutSec = v;
    }
    const bool decOk = runCmd("env" + envUnsetArgs() + " timeout " +
                              std::to_string(verifyTimeoutSec) + " " + decBin + " " + tmpJson +
                              " > " + decLog + " 2>&1");
    const std::string cmpBin = toolDir + "/MeshCompare";
    const std::string cmpLog = artifact + "." + tag + ".cmp.log";
    char tolBuf[64];
    std::snprintf(tolBuf, sizeof(tolBuf), "%.6f", autoTolForOff(origInput));
    std::string tol = tolBuf;
    if (const char* t = std::getenv("PMSF_BND_V3_VERIFY_TOL")) tol = t;
    bool cmpOk = false;
    if (decOk && std::filesystem::exists(cmpBin)) {
        cmpOk = runCmd("timeout 600 " + cmpBin + " " + origInput + " " + off + " --tol " + tol +
                       " > " + cmpLog + " 2>&1");
    }
    std::fprintf(stderr, "[bnd-v3-verify] %s decode=%d compare=%d artifact=%s\n", tag.c_str(),
                 decOk ? 1 : 0, cmpOk ? 1 : 0, artifact.c_str());
    if (!cmpOk) {
        std::ifstream lf(cmpLog);
        std::string line;
        int shown = 0;
        while (std::getline(lf, line) && shown < 6) {
            std::fprintf(stderr, "[bnd-v3-verify]   %s\n", line.c_str());
            ++shown;
        }
    }
    std::filesystem::remove(tmpJson);
    if (outFailRound) {
        *outFailRound = -1;
        if (!decOk) {
            std::ifstream lf(decLog);
            std::string line;
            std::regex rr("round=([0-9]+)");
            while (std::getline(lf, line)) {
                if (line.find("failed") == std::string::npos &&
                    line.find("no id edge") == std::string::npos &&
                    line.find("bad payload") == std::string::npos)
                    continue;
                std::smatch m;
                if (std::regex_search(line, m, rr)) {
                    *outFailRound = std::stoi(m[1].str());
                    break;
                }
            }
        }
    }
    return decOk && cmpOk;
}

double autoTolForOff(const std::string& path) {
    std::ifstream in(path);
    if (!in) return 0.002;
    std::string magic;
    if (!(in >> magic)) return 0.002;
    long long nv = 0, nf = 0, ne = 0;
    if (!(in >> nv >> nf >> ne)) return 0.002;
    double mn[3] = {1e300, 1e300, 1e300};
    double mx[3] = {-1e300, -1e300, -1e300};
    for (long long i = 0; i < nv; ++i) {
        double x = 0, y = 0, z = 0;
        if (!(in >> x >> y >> z)) break;
        const double p[3] = {x, y, z};
        for (int k = 0; k < 3; ++k) {
            mn[k] = std::min(mn[k], p[k]);
            mx[k] = std::max(mx[k], p[k]);
        }
    }
    const double dx = mx[0] - mn[0], dy = mx[1] - mn[1], dz = mx[2] - mn[2];
    const double diag = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (!(diag > 0.0)) return 0.002;
    return std::max(1e-6, diag * 1e-3);
}

std::string envUnsetArgs() {
    // Strip every PMSF_* variable from the child environment: the child tool
    // re-applies the config `extra` envs itself, so only harness/stage envs that
    // were exported outside the config need removal for a clean plain baseline.
    std::string out;
    std::vector<std::string> keys;
    for (char** e = environ; e && *e; ++e) {
        const std::string kv(*e);
        if (kv.rfind("PMSF_", 0) != 0) continue;
        const std::size_t eq = kv.find('=');
        if (eq == std::string::npos) continue;
        keys.push_back(kv.substr(0, eq));
    }
    std::sort(keys.begin(), keys.end());
    keys.erase(std::unique(keys.begin(), keys.end()), keys.end());
    for (const auto& k : keys) out += " -u " + k;
    return out;
}

// Compress a clean plain baseline candidate (stage envs removed) with the sibling
// PMSFCompress binary, then verify it decodes+compares.  Used for bpv argmin emit.
bool makeCleanPlainCandidate(const std::string& configPath, const std::string& toolDir,
                             const std::string& plainOut, const std::string& origInput,
                             const std::string& tag, const std::string& extraEnv = std::string()) {
    std::ifstream in(configPath);
    std::stringstream ss;
    ss << in.rdbuf();
    const std::regex rout("\"output\"\\s*:\\s*\"[^\"]*\"");
    std::string childCfg = std::regex_replace(ss.str(), rout, "\"output\": \"" + plainOut + "\"");
    const std::string tmpCfg = "/tmp/pmsf_v3_plain_" + std::to_string(::getpid()) + ".json";
    {
        std::ofstream o(tmpCfg);
        o << childCfg;
    }
    const std::string bin = toolDir + "/PMSFCompress";
    const std::string log = plainOut + ".log";
    const std::string cmd = "env" + envUnsetArgs() + extraEnv + " timeout 1800 " + bin + " " + tmpCfg +
                            " > " + log + " 2>&1";
    const bool built = runCmd(cmd);
    std::filesystem::remove(tmpCfg);
    if (!built) {
        std::fprintf(stderr, "[bnd-v3-emit] %s: clean plain compress FAILED\n", tag.c_str());
        return false;
    }
    const bool verified = v3VerifyArtifact(configPath, plainOut, origInput, toolDir, tag);
    return verified;
}
void disableStageEnvs() {
    static const char* kKeys[] = {
        "PMSF_BND_V3", "PMSF_SC_BOUNDARY_V2", "PMSF_SC_ALLOW_MULTIROUND",
        "PMSF_SC_BOUNDARY_ROUNDS", "PMSF_SC_BOUNDARY_MAXOPS", "PMSF_SC_BOUNDARY_PRE_ROUNDS",
        "PMSF_SC_V2_PREFLIGHT", "PMSF_SC_V2_STAGE_ORDER", "PMSF_SC_V2_STAGE_ORDER_LIST",
        "PMSF_SC_V2_STAGE_ORDER_IFB_ROUNDS", "PMSF_SC_V2_STRICT_FAIL",
        "PMSF_SC_BOUNDARY_STAGE", "PMSF_SC_BOUNDARY_GROUP_STAGE",
        "PMSF_SC_BOUNDARY_ANCHOR_STAGE", "PMSF_SC_BOUNDARY_COMPACT_STAGE",
        "PMSF_SEG_REFRESH_SEEDS", "PMSF_BC_V2_AUTO", "PMSF_EDGEFLIP_ROUND_LIST",
        "PMSF_EDGEFLIP_LAST_ONLY",
        "PMSF_BND_V3_DEBUG"};
    for (const char* k : kKeys) unsetenv(k);
}


std::set<int> parseRoundSet(const char* envValue, int totalRounds) {
    std::set<int> out;
    if (!envValue || !envValue[0]) {
        for (int r = 0; r < totalRounds; ++r) out.insert(r);
        return out;
    }
    const std::string rs(envValue);
    std::size_t p = 0;
    while (p <= rs.size()) {
        const std::size_t comma = rs.find(',', p);
        const std::string tok =
            rs.substr(p, comma == std::string::npos ? std::string::npos : comma - p);
        if (!tok.empty()) out.insert(std::atoi(tok.c_str()));
        if (comma == std::string::npos) break;
        p = comma + 1;
    }
    return out;
}

std::string roundSetString(const std::set<int>& rs) {
    std::string out;
    for (int r : rs) {
        if (!out.empty()) out += ",";
        out += std::to_string(r);
    }
    return out;
}

std::string envValueOrEmpty(const char* k) {
    const char* v = std::getenv(k);
    return v ? std::string(v) : std::string();
}

std::string boundaryStageEnv(int maxOps) {
    std::string e = " PMSF_BND_V3=1";
    if (!envValueOrEmpty("PMSF_SC_BOUNDARY_V2").empty()) e += " PMSF_SC_BOUNDARY_V2=1";
    if (!envValueOrEmpty("PMSF_SC_ALLOW_MULTIROUND").empty()) e += " PMSF_SC_ALLOW_MULTIROUND=1";
    e += " PMSF_SC_BOUNDARY_MAXOPS=" + std::to_string(maxOps);
    const std::string maxOpsList = envValueOrEmpty("PMSF_SC_BOUNDARY_MAXOPS_LIST");
    if (!maxOpsList.empty()) e += " PMSF_SC_BOUNDARY_MAXOPS_LIST=" + maxOpsList;
    if (!envValueOrEmpty("PMSF_SC_V2_PREFLIGHT").empty()) e += " PMSF_SC_V2_PREFLIGHT=1";
    const std::string orderList = envValueOrEmpty("PMSF_SC_V2_STAGE_ORDER_LIST");
    if (!orderList.empty()) e += " PMSF_SC_V2_STAGE_ORDER_LIST=" + orderList;
    const std::string rounds = envValueOrEmpty("PMSF_SC_BOUNDARY_ROUNDS");
    if (!rounds.empty()) e += " PMSF_SC_BOUNDARY_ROUNDS=" + rounds;
    const std::string pre = envValueOrEmpty("PMSF_SC_BOUNDARY_PRE_ROUNDS");
    if (!pre.empty()) e += " PMSF_SC_BOUNDARY_PRE_ROUNDS=" + pre;
    e += " PMSF_BND_V3_VERIFY=0 PMSF_BND_V3_PRUNING=0";
    return e;
}

// Re-encode with a specific boundary maxOps (K) for writer-side K autotuning.
bool runKV3Reencode(const std::string& configPath, const std::string& toolDir,
                    const std::string& outPath, int maxOps,
                    const std::string& uniformMode = std::string()) {
    std::ifstream in(configPath);
    std::stringstream ss;
    ss << in.rdbuf();
    const std::string text = ss.str();
    const std::regex rout("\"output\"\\s*:\\s*\"[^\"]*\"");
    const std::string childCfgText =
        std::regex_replace(text, rout, "\"output\": \"" + outPath + "\"");
    const std::string tmpCfg = "/tmp/pmsf_v3_kauto_" + std::to_string(::getpid()) + ".json";
    {
        std::ofstream o(tmpCfg);
        o << childCfgText;
    }
    const std::string bin = toolDir + "/PMSFCompress";
    const std::string log = outPath + ".kauto.log";
    std::string modeEnv;
    // env option parsing: all -u options MUST come before assignments.
    if (uniformMode.empty() || uniformMode == "plain") {
        modeEnv = " -u PMSF_BC_V2_UNIFORM -u PMSF_BC_V2_UNIFORM_MAX";
    } else {
        // uniformMode: "u<maxRatio>", e.g. "u1.2"
        const std::string ratio = uniformMode.substr(1);
        modeEnv = " -u PMSF_BC_V2_UNIFORM -u PMSF_BC_V2_UNIFORM_MAX PMSF_BC_V2_UNIFORM=1";
        if (!ratio.empty()) modeEnv += " PMSF_BC_V2_UNIFORM_MAX=" + ratio;
    }
    const std::string cmd = "env" + modeEnv + boundaryStageEnv(maxOps) + " timeout 1800 " + bin +
                            " " + tmpCfg + " > " + log + " 2>&1";
    const bool ok = runCmd(cmd);
    std::filesystem::remove(tmpCfg);
    if (!ok) {
        std::fprintf(stderr, "[bnd-v3-kauto] K=%d re-encode FAILED\n", maxOps);
        return false;
    }
    return true;
}

// Re-encode with an explicit per-round maxOps list (writer-side K adaptation).
bool runListV3Reencode(const std::string& configPath, const std::string& toolDir,
                       const std::string& outPath, const std::string& roundList) {
    std::ifstream in(configPath);
    std::stringstream ss;
    ss << in.rdbuf();
    const std::string text = ss.str();
    const std::regex rout("\"output\"\\s*:\\s*\"[^\"]*\"");
    const std::string childCfgText =
        std::regex_replace(text, rout, "\"output\": \"" + outPath + "\"");
    const std::string tmpCfg = "/tmp/pmsf_v3_kadapt_" + std::to_string(::getpid()) + ".json";
    {
        std::ofstream o(tmpCfg);
        o << childCfgText;
    }
    const std::string bin = toolDir + "/PMSFCompress";
    const std::string log = outPath + ".kadapt.log";
    std::string e = "env PMSF_BND_V3=1 PMSF_SC_BOUNDARY_MAXOPS=1";
    if (!envValueOrEmpty("PMSF_SC_BOUNDARY_V2").empty()) e += " PMSF_SC_BOUNDARY_V2=1";
    if (!envValueOrEmpty("PMSF_SC_ALLOW_MULTIROUND").empty()) e += " PMSF_SC_ALLOW_MULTIROUND=1";
    const std::string rounds = envValueOrEmpty("PMSF_SC_BOUNDARY_ROUNDS");
    if (!rounds.empty()) e += " PMSF_SC_BOUNDARY_ROUNDS=" + rounds;
    const std::string pre = envValueOrEmpty("PMSF_SC_BOUNDARY_PRE_ROUNDS");
    if (!pre.empty()) e += " PMSF_SC_BOUNDARY_PRE_ROUNDS=" + pre;
    if (!envValueOrEmpty("PMSF_SC_V2_PREFLIGHT").empty()) e += " PMSF_SC_V2_PREFLIGHT=1";
    const std::string orderList = envValueOrEmpty("PMSF_SC_V2_STAGE_ORDER_LIST");
    if (!orderList.empty()) e += " PMSF_SC_V2_STAGE_ORDER_LIST=" + orderList;
    e += " PMSF_SC_BOUNDARY_MAXOPS_LIST=" + roundList;
    e += " PMSF_BND_V3_VERIFY=0 PMSF_BND_V3_PRUNING=0";
    if (std::getenv("PMSF_BND_V3_KADAPT_STATS")) e += " PMSF_SC_DEBUG=1";
    const std::string cmd = e + " timeout 1800 " + bin + " " + tmpCfg + " > " + log + " 2>&1";
    const bool ok = runCmd(cmd);
    std::filesystem::remove(tmpCfg);
    return ok;
}

// Re-encode the V3 artifact with one failing boundary round disabled.  This is the
// writer-side safety loop: the encoder may hit replay-order drift for some rounds on
// some meshes; instead of falling back to plain, we verify-decode, prune the round
// that failed, and re-encode.  Every emitted artifact is decode-verified.
bool runPrunedV3Reencode(const std::string& configPath, const std::string& toolDir,
                         const std::string& outPath, const std::string& roundsList,
                         const std::string& preList) {
    std::ifstream in(configPath);
    std::stringstream ss;
    ss << in.rdbuf();
    const std::string text = ss.str();
    const std::regex rout("\"output\"\\s*:\\s*\"[^\"]*\"");
    const std::string childCfgText =
        std::regex_replace(text, rout, "\"output\": \"" + outPath + "\"");
    const std::string tmpCfg = "/tmp/pmsf_v3_prune_" + std::to_string(::getpid()) + ".json";
    {
        std::ofstream o(tmpCfg);
        o << childCfgText;
    }
    const std::string bin = toolDir + "/PMSFCompress";
    const std::string log = outPath + ".prune.log";
    std::string e = " env";
    e += " PMSF_BND_V3=1";
    if (!envValueOrEmpty("PMSF_SC_BOUNDARY_V2").empty()) e += " PMSF_SC_BOUNDARY_V2=1";
    if (!envValueOrEmpty("PMSF_SC_ALLOW_MULTIROUND").empty()) e += " PMSF_SC_ALLOW_MULTIROUND=1";
    const std::string maxOps = envValueOrEmpty("PMSF_SC_BOUNDARY_MAXOPS");
    if (!maxOps.empty()) e += " PMSF_SC_BOUNDARY_MAXOPS=" + maxOps;
    if (!envValueOrEmpty("PMSF_SC_V2_PREFLIGHT").empty()) e += " PMSF_SC_V2_PREFLIGHT=1";
    const std::string orderList = envValueOrEmpty("PMSF_SC_V2_STAGE_ORDER_LIST");
    if (!orderList.empty()) e += " PMSF_SC_V2_STAGE_ORDER_LIST=" + orderList;
    if (!roundsList.empty()) e += " PMSF_SC_BOUNDARY_ROUNDS=" + roundsList;
    if (!preList.empty()) e += " PMSF_SC_BOUNDARY_PRE_ROUNDS=" + preList;
    e += " PMSF_BND_V3_VERIFY=0 PMSF_BND_V3_PRUNING=0 PMSF_BND_V3_PRUNED_NAME=" + outPath;
    const std::string cmd = e + " timeout 1800 " + bin + " " + tmpCfg + " > " + log + " 2>&1";
    const bool ok = runCmd(cmd);
    std::filesystem::remove(tmpCfg);
    if (!ok) {
        std::fprintf(stderr, "[bnd-v3-prune] re-encode FAILED for %s\n", outPath.c_str());
        return false;
    }
    return true;
}

}  // namespace

namespace {
}  // namespace

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "用法: " << argv[0] << " <config.json>\n";
        return 1;
    }

    try {
        pmsf::PMSFConfig cfg = pmsf::parseConfigFromFile(argv[1]);
        if (cfg.input.empty() || cfg.output.empty()) {
            std::cerr << "[PMSFCompress] config 必须包含 input 与 output 字段\n";
            return 1;
        }
        for (const auto& kv : cfg.extra) {
            if (!kv.first.empty()) {
                setenv(kv.first.c_str(), kv.second.c_str(), 1);
            }
        }
        TimeRecorder recorder("PMSFCompress total");
        pmsf::dispatchCompressByName(cfg);

        // V3 writer replay verification + clean-process bpv candidate selection:
        // emit = argmin(bytes) over {verified V3 artifact, verified clean plain baseline}.
        // This structurally guarantees bytes(emitted) <= bytes(plain), i.e. bpv does not
        // increase; if a compact V3 payload ever wins, bpv decreases.
        const bool v3On = std::getenv("PMSF_BND_V3") != nullptr;
        const char* verifyEnv = std::getenv("PMSF_BND_V3_VERIFY");
        const bool verifyOn = v3On && !(verifyEnv && verifyEnv[0] == '0');
        if (verifyOn) {
            const std::string toolDir = dirOf(argv[0]);
            const std::string plainTmp = cfg.output + ".plain.bin";
            const bool plainOk =
                makeCleanPlainCandidate(argv[1], toolDir, plainTmp, cfg.input, "baseline");
            int failRound = -1;
            bool v3Ok = v3VerifyArtifact(argv[1], cfg.output, cfg.input, toolDir, "v3", &failRound);
            uintmax_t plainBytes = 0;
            uintmax_t v3Bytes = 0;
            // Writer-side pruning: boundary ops can hit replay-order drift on some
            // rounds/meshes; prune the failing round and re-encode instead of
            // degrading the whole artifact to plain.  Each attempt is verified.
            const char* pruneEnv = std::getenv("PMSF_BND_V3_PRUNING");
            const bool pruneOn = (pruneEnv && pruneEnv[0] != '0');
            if (!v3Ok && pruneOn) {
                int totalRounds = cfg.rounds;
                if (totalRounds <= 0) {
                    std::ifstream in(argv[1]);
                    std::stringstream ss;
                    ss << in.rdbuf();
                    std::smatch m;
                    const std::string text = ss.str();
                    if (std::regex_search(text, m, std::regex("\"rounds\"\\s*:\\s*([0-9]+)")))
                        totalRounds = std::stoi(m[1].str());
                }
                std::set<int> rset =
                    parseRoundSet(std::getenv("PMSF_SC_BOUNDARY_ROUNDS"), totalRounds);
                std::set<int> pset =
                    parseRoundSet(std::getenv("PMSF_SC_BOUNDARY_PRE_ROUNDS"), totalRounds);
                std::fprintf(stderr, "[bnd-v3-prune] initial decode failed at round=%d, %zu rounds\n",
                             failRound, rset.size());
                for (int attempt = 0; attempt < 10 && !v3Ok; ++attempt) {
                    if (failRound < 0 || !rset.count(failRound)) break;
                    rset.erase(failRound);
                    pset.erase(failRound);
                    const std::string prunedOut = cfg.output + ".pruned.bin";
                    if (!runPrunedV3Reencode(argv[1], toolDir, prunedOut, roundSetString(rset),
                                             roundSetString(pset))) {
                        break;
                    }
                    int nextFail = -1;
                    v3Ok = v3VerifyArtifact(argv[1], prunedOut, cfg.input, toolDir, "pruned",
                                            &nextFail);
                    if (v3Ok) {
                        std::filesystem::copy_file(
                            prunedOut, cfg.output,
                            std::filesystem::copy_options::overwrite_existing);
                        std::fprintf(stderr,
                                     "[bnd-v3-prune] ACCEPT pruned artifact rounds=%zu attempt=%d\n",
                                     rset.size(), attempt + 1);
                    } else {
                        std::fprintf(stderr,
                                     "[bnd-v3-prune] attempt=%d failed at round=%d rounds=%zu\n",
                                     attempt + 1, nextFail, rset.size());
                        failRound = nextFail;
                    }
                    std::filesystem::remove(prunedOut);
                    std::filesystem::remove(prunedOut + ".prune.log");
                }
            }
            // Writer-side per-round K adaptation: start from a large per-round budget,
            // and whenever the verified decode fails at round r, halve that round's K
            // and re-encode.  Keeps early rounds aggressive (uniform boundary) while
            // guaranteeing the emitted artifact is decode-verified.
            const bool kadaptOn = std::getenv("PMSF_BND_V3_KADAPT") != nullptr;
            if (kadaptOn && !v3Ok) {
                const char* initEnv = std::getenv("PMSF_BND_V3_KADAPT_INIT");
                int initK = initEnv ? std::atoi(initEnv) : 32;
                if (initK < 1) initK = 32;
                int totalRounds = cfg.rounds;
                if (totalRounds <= 0) totalRounds = 50;
                std::vector<int> perRound(static_cast<std::size_t>(totalRounds), initK);
                std::fprintf(stderr, "[bnd-v3-kadapt] init K=%d rounds=%d\n", initK, totalRounds);
                int kadaptMaxAttempts = 40;
                if (const char* ma = std::getenv("PMSF_BND_V3_KADAPT_MAX_ATTEMPTS")) {
                    const int v = std::atoi(ma);
                    if (v > 0) kadaptMaxAttempts = v;
                }
                for (int attempt = 0; attempt < kadaptMaxAttempts && !v3Ok; ++attempt) {
                    std::string list;
                    for (int r = 0; r < totalRounds; ++r) {
                        if (!list.empty()) list += ",";
                        list += std::to_string(perRound[static_cast<std::size_t>(r)]);
                    }
                    const std::string kOut = cfg.output + ".kadapt.bin";
                    if (!runListV3Reencode(argv[1], toolDir, kOut, list)) break;
                    int failRound = -1;
                    v3Ok = v3VerifyArtifact(argv[1], kOut, cfg.input, toolDir, "kadapt", &failRound);
                    if (v3Ok) {
                        std::filesystem::copy_file(
                            kOut, cfg.output,
                            std::filesystem::copy_options::overwrite_existing);
                        std::fprintf(stderr, "[bnd-v3-kadapt] ACCEPT attempt=%d\n", attempt + 1);
                        std::fprintf(stderr, "[bnd-v3-kadapt] FINAL_LIST=%s\n", list.c_str());
                        std::ifstream lf(kOut + ".kadapt.log");
                        std::string line;
                        std::regex rops("ops=([0-9]+)");
                        int sumOps = 0, nRoundsWithOps = 0, maxOpsRound = 0;
                        while (std::getline(lf, line)) {
                            std::smatch m;
                            if (std::regex_search(line, m, rops)) {
                                const int o = std::stoi(m[1].str());
                                sumOps += o;
                                if (o > 0) ++nRoundsWithOps;
                                maxOpsRound = std::max(maxOpsRound, o);
                            }
                        }
                        std::fprintf(stderr,
                                     "[bnd-v3-kadapt] STATS totalOps=%d roundsWithOps=%d maxOpsRound=%d\n",
                                     sumOps, nRoundsWithOps, maxOpsRound);
                    } else {
                        // Aggressive suffix adaptation: when a decode fails at round r,
                        // halve the whole suffix r..R-1 (plus the two preceding rounds).
                        // Late-round context is the dominant instability source, so this
                        // converges much faster than halving only a small window.
                        int lowered = 0;
                        if (failRound < 0) {
                            for (int rr = 0; rr < totalRounds; ++rr) {
                                int& kk = perRound[static_cast<std::size_t>(rr)];
                                if (kk > 0) {
                                    kk = (kk <= 1) ? 0 : kk / 2;
                                    ++lowered;
                                }
                            }
                        } else {
                            for (int rr = std::max(0, failRound - 2); rr < totalRounds; ++rr) {
                                int& kk = perRound[static_cast<std::size_t>(rr)];
                                if (kk > 0) {
                                    kk = (kk <= 1) ? 0 : kk / 2;
                                    ++lowered;
                                }
                            }
                        }
                        std::fprintf(stderr,
                                     "[bnd-v3-kadapt] attempt=%d failRound=%d lowered=%d\n",
                                     attempt + 1, failRound, lowered);
                    }
                    std::filesystem::remove(kOut);
                    std::filesystem::remove(kOut + ".kadapt.log");
                }
            }

            // Writer-side AUTO selection: try both candidate ordering modes (plain and
            // uniformity-constrained) over a K list, verify every artifact, and prefer
            // the LARGEST K that is verified with bytes < plain (user wants large K);
            // fall back to the smallest verified V3 artifact if none beats plain.
            const bool autoSel = std::getenv("PMSF_BND_V3_AUTO_SELECT") != nullptr;
            if (autoSel) {
                uintmax_t plainBytesAuto = static_cast<uintmax_t>(-1);
                const bool plainOkAuto = plainOk && std::filesystem::exists(plainTmp);
                if (plainOkAuto) plainBytesAuto = std::filesystem::file_size(plainTmp);
                std::vector<int> kList;
                const char* kListEnvAuto = std::getenv("PMSF_BND_V3_K_LIST");
                if (kListEnvAuto && kListEnvAuto[0]) {
                    const std::string ks(kListEnvAuto);
                    std::size_t p = 0;
                    while (p <= ks.size()) {
                        const std::size_t comma = ks.find(',', p);
                        const std::string tok =
                            ks.substr(p, comma == std::string::npos ? std::string::npos : comma - p);
                        if (!tok.empty()) kList.push_back(std::atoi(tok.c_str()));
                        if (comma == std::string::npos) break;
                        p = comma + 1;
                    }
                }
                std::sort(kList.begin(), kList.end(), std::greater<int>());
                std::vector<std::string> modes;
                const char* modesEnv = std::getenv("PMSF_BND_V3_MODES");
                const std::string modesStr = (modesEnv && modesEnv[0]) ? std::string(modesEnv)
                                                                       : std::string("plain,u1.2");
                std::size_t mp = 0;
                while (mp <= modesStr.size()) {
                    const std::size_t comma = modesStr.find(',', mp);
                    const std::string tok = modesStr.substr(
                        mp, comma == std::string::npos ? std::string::npos : comma - mp);
                    if (!tok.empty()) modes.push_back(tok);
                    if (comma == std::string::npos) break;
                    mp = comma + 1;
                }
                const bool exhaustive = std::getenv("PMSF_BND_V3_EXHAUSTIVE") != nullptr;
                uintmax_t fallbackBytes = static_cast<uintmax_t>(-1);
                bool haveFallback = false;
                bool selected = false;
                std::string selInfo;
                // Mode-priority order: the caller lists modes in priority order.  Within
                // one mode, pick the LARGEST K verified with bytes < plain; only if that
                // mode has no such K fall through to the next mode.  This lets a user say
                // "uniform first, then plain" so large-K uneven schedules do not win.
                for (const std::string& mode : modes) {
                    for (int k : kList) {
                        const std::string out =
                            cfg.output + ".auto_k" + std::to_string(k) + "_" + mode + ".bin";
                        if (!runKV3Reencode(argv[1], toolDir, out, k, mode)) continue;
                        const std::string tag = "auto" + std::to_string(k) + "_" + mode;
                        const bool ok = v3VerifyArtifact(argv[1], out, cfg.input, toolDir, tag);
                        if (!ok) {
                            std::filesystem::remove(out);
                            std::filesystem::remove(out + ".kauto.log");
                            continue;
                        }
                        const uintmax_t bytes = std::filesystem::file_size(out);
                        std::fprintf(stderr,
                                     "[bnd-v3-autosel] K=%d mode=%s verified bytes=%llu plain=%llu\n",
                                     k, mode.c_str(), static_cast<unsigned long long>(bytes),
                                     static_cast<unsigned long long>(plainBytesAuto));
                        if (const char* keepDir = std::getenv("PMSF_BND_V3_KEEP_CANDIDATES")) {
                            if (keepDir && keepDir[0]) {
                                std::filesystem::create_directories(keepDir);
                                const std::string keepPath =
                                    std::string(keepDir) + "/k" + std::to_string(k) + "_" +
                                    mode + ".bin";
                                std::filesystem::copy_file(
                                    out, keepPath,
                                    std::filesystem::copy_options::overwrite_existing);
                                std::ofstream idx(std::string(keepDir) + "/index.txt",
                                                  std::ios::app);
                                idx << argv[1] << "\t" << k << "\t" << mode << "\t" << bytes
                                    << "\t" << plainBytesAuto << "\t" << keepPath << "\n";
                            }
                        }
                        if (!selected && plainOkAuto && bytes < plainBytesAuto) {
                            std::filesystem::copy_file(
                                out, cfg.output,
                                std::filesystem::copy_options::overwrite_existing);
                            v3Ok = true;
                            v3Bytes = bytes;
                            selected = true;
                            selInfo = "K=" + std::to_string(k) + " mode=" + mode;
                            std::fprintf(stderr, "[bnd-v3-autosel] SELECT %s bytes=%llu\n",
                                         selInfo.c_str(),
                                         static_cast<unsigned long long>(bytes));
                            std::filesystem::remove(out);
                            std::filesystem::remove(out + ".kauto.log");
                            if (!exhaustive) break;
                            continue;
                        }
                        if (bytes < fallbackBytes) {
                            fallbackBytes = bytes;
                            haveFallback = true;
                            std::filesystem::copy_file(
                                out, cfg.output,
                                std::filesystem::copy_options::overwrite_existing);
                        }
                        std::filesystem::remove(out);
                        std::filesystem::remove(out + ".kauto.log");
                    }
                    if (selected && !exhaustive) break;
                }
                if (!selected && haveFallback) {
                    v3Ok = true;
                    v3Bytes = fallbackBytes;
                    std::fprintf(stderr,
                                 "[bnd-v3-autosel] SELECT fallback (no improvement over plain) bytes=%llu\n",
                                 static_cast<unsigned long long>(fallbackBytes));
                }
            }

            // Writer-side K autotune: try several boundary maxOps values, verify each
            // artifact, and keep the verified V3 artifact with the smallest bytes.
            const char* kListEnv = std::getenv("PMSF_BND_V3_K_LIST");
            if (kListEnv && kListEnv[0] && !autoSel) {
                uintmax_t bestBytes = v3Ok ? std::filesystem::file_size(cfg.output) : 0;
                bool haveBest = v3Ok;
                int bestK = -1;
                const std::string ks(kListEnv);
                std::size_t p = 0;
                while (p <= ks.size()) {
                    const std::size_t comma = ks.find(',', p);
                    const std::string tok =
                        ks.substr(p, comma == std::string::npos ? std::string::npos : comma - p);
                    if (!tok.empty()) {
                        const int k = std::atoi(tok.c_str());
                        if (k > 0) {
                            const std::string kOut = cfg.output + ".k" + std::to_string(k) + ".bin";
                            if (runKV3Reencode(argv[1], toolDir, kOut, k)) {
                                const bool ok = v3VerifyArtifact(argv[1], kOut, cfg.input, toolDir,
                                                                 "kauto" + std::to_string(k));
                                if (ok) {
                                    const uintmax_t bytes = std::filesystem::file_size(kOut);
                                    std::fprintf(stderr,
                                                 "[bnd-v3-kauto] K=%d verified bytes=%llu best=%llu\n",
                                                 k, static_cast<unsigned long long>(bytes),
                                                 static_cast<unsigned long long>(bestBytes));
                                    if (!haveBest || bytes < bestBytes) {
                                        bestBytes = bytes;
                                        bestK = k;
                                        haveBest = true;
                                        std::filesystem::copy_file(
                                            kOut, cfg.output,
                                            std::filesystem::copy_options::overwrite_existing);
                                    }
                                }
                            }
                            std::filesystem::remove(kOut);
                            std::filesystem::remove(kOut + ".kauto.log");
                        }
                    }
                    if (comma == std::string::npos) break;
                    p = comma + 1;
                }
                if (haveBest) {
                    v3Ok = true;
                    v3Bytes = bestBytes;
                    std::fprintf(stderr, "[bnd-v3-kauto] SELECT K=%d bytes=%llu\n", bestK,
                                 static_cast<unsigned long long>(bestBytes));
                }
            }

            if (plainOk) plainBytes = std::filesystem::file_size(plainTmp);
            if (v3Ok) v3Bytes = std::filesystem::file_size(cfg.output);
            if (!v3Ok) v3Bytes = 0;
            bool emitPlain = false;
            if (!v3Ok) {
                emitPlain = plainOk;
            } else if (plainOk && plainBytes < v3Bytes) {
                emitPlain = true;
            }
            if (emitPlain) {
                std::filesystem::copy_file(plainTmp, cfg.output,
                                           std::filesystem::copy_options::overwrite_existing);
                std::fprintf(stderr,
                             "[bnd-v3-emit] emit=plain bytes=%llu (v3 bytes=%llu v3ok=%d)\n",
                             static_cast<unsigned long long>(plainBytes),
                             static_cast<unsigned long long>(v3Bytes), v3Ok ? 1 : 0);
            } else if (v3Ok) {
                std::fprintf(stderr,
                             "[bnd-v3-emit] emit=v3 bytes=%llu (plain bytes=%llu plainok=%d)\n",
                             static_cast<unsigned long long>(v3Bytes),
                             static_cast<unsigned long long>(plainBytes), plainOk ? 1 : 0);
            } else {
                std::fprintf(stderr,
                             "[bnd-v3-emit] neither candidate verified -> in-process plain fallback\n");
                disableStageEnvs();
                pmsf::dispatchCompressByName(cfg);
                const bool fallbackOk =
                    v3VerifyArtifact(argv[1], cfg.output, cfg.input, toolDir, "fallback");
                if (!fallbackOk) {
                    std::fprintf(stderr, "[bnd-v3-emit] fallback artifact ALSO failed\n");
                    return 2;
                }
                std::fprintf(stderr, "[bnd-v3-emit] emit=fallback-plain (verified)\n");
            }
            std::filesystem::remove(plainTmp);
            std::filesystem::remove(plainTmp + ".log");
        }

        // Conservative boundary compression (no progressive inverse): boundary
        // simplification happens once on the terminal/base mesh, so the decoder has
        // nothing to invert.  Emit = argmin(verified clean plain, verified base-stage).
        const bool conservativeOn = std::getenv("PMSF_BND_CONSERVATIVE") != nullptr;
        if (conservativeOn) {
            const std::string toolDir = dirOf(argv[0]);
            const std::string plainTmp = cfg.output + ".cons_plain.bin";
            const std::string baseTmp = cfg.output + ".cons_base.bin";
            std::string baseExtra = " PMSF_BND_BASE_STAGE=1";
            if (const char* mo = std::getenv("PMSF_BND_BASE_MAXOPS"))
                baseExtra += std::string(" PMSF_BND_BASE_MAXOPS=") + mo;
            const bool plainOk = makeCleanPlainCandidate(argv[1], toolDir, plainTmp, cfg.input,
                                                         "cons_plain", std::string());
            const bool baseOk = makeCleanPlainCandidate(argv[1], toolDir, baseTmp, cfg.input,
                                                        "cons_base", baseExtra);
            uintmax_t plainBytes = 0, baseBytes = 0;
            if (plainOk) plainBytes = std::filesystem::file_size(plainTmp);
            if (baseOk) baseBytes = std::filesystem::file_size(baseTmp);
            bool emitBase = false;
            if (baseOk && plainOk) emitBase = baseBytes < plainBytes;
            else if (!plainOk && baseOk) emitBase = true;
            if (emitBase) {
                std::filesystem::copy_file(baseTmp, cfg.output,
                                           std::filesystem::copy_options::overwrite_existing);
                std::fprintf(stderr,
                             "[bnd-cons-emit] emit=base-stage bytes=%llu plain_bytes=%llu plainok=%d\n",
                             static_cast<unsigned long long>(baseBytes),
                             static_cast<unsigned long long>(plainBytes), plainOk ? 1 : 0);
            } else if (plainOk) {
                std::filesystem::copy_file(plainTmp, cfg.output,
                                           std::filesystem::copy_options::overwrite_existing);
                std::fprintf(stderr,
                             "[bnd-cons-emit] emit=plain bytes=%llu base_bytes=%llu baseok=%d\n",
                             static_cast<unsigned long long>(plainBytes),
                             static_cast<unsigned long long>(baseBytes), baseOk ? 1 : 0);
            } else {
                std::fprintf(stderr, "[bnd-cons-emit] ERROR: no verified candidate\n");
                return 4;
            }
            std::filesystem::remove(plainTmp);
            std::filesystem::remove(baseTmp);
            std::filesystem::remove(plainTmp + ".log");
            std::filesystem::remove(baseTmp + ".log");
        }

    } catch (const std::exception& e) {
        std::cerr << "[PMSFCompress] error: " << e.what() << "\n";
        return 1;
    }
    return 0;
}
