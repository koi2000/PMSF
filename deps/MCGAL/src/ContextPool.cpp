/*
 * @Author: koi
 * @Date: 2025-03-23 18:00:20
 * @Description: 
 */
#include "ContextPool.h"
#include "string.h"
#include <cerrno>
#include <climits>
#include <cstdlib>
#include <utility>
namespace MCGAL {

ContextValue::ContextValue() : type_(Type::None) {}

ContextValue::ContextValue(int v) : type_(Type::Int) {
    data_.intValue = v;
}

ContextValue::ContextValue(float v) : type_(Type::Float) {
    data_.floatValue = v;
}

ContextValue::ContextValue(double v) : type_(Type::Double) {
    data_.doubleValue = v;
}

ContextValue::ContextValue(long v) : type_(Type::Long) {
    data_.longValue = v;
}

ContextValue::ContextValue(const std::string& v) : type_(Type::String) {
    new (&data_.stringValue) std::string(v);
}

ContextValue::ContextValue(const char* v) : ContextValue(std::string(v ? v : "")) {}

ContextValue::ContextValue(const ContextValue& other) : type_(Type::None) {
    copyFrom(other);
}

ContextValue::ContextValue(ContextValue&& other) noexcept : type_(Type::None) {
    moveFrom(std::move(other));
}

ContextValue& ContextValue::operator=(const ContextValue& other) {
    if (this != &other) {
        destroy();
        copyFrom(other);
    }
    return *this;
}

ContextValue& ContextValue::operator=(ContextValue&& other) noexcept {
    if (this != &other) {
        destroy();
        moveFrom(std::move(other));
    }
    return *this;
}

ContextValue::~ContextValue() {
    destroy();
}

int ContextValue::asInt(int defaultValue) const {
    switch (type_) {
        case Type::Int:
            return data_.intValue;
        case Type::Float:
            return static_cast<int>(data_.floatValue);
        case Type::Double:
            return static_cast<int>(data_.doubleValue);
        case Type::Long:
            return static_cast<int>(data_.longValue);
        default:
            return defaultValue;
    }
}

float ContextValue::asFloat(float defaultValue) const {
    switch (type_) {
        case Type::Int:
            return static_cast<float>(data_.intValue);
        case Type::Float:
            return data_.floatValue;
        case Type::Double:
            return static_cast<float>(data_.doubleValue);
        case Type::Long:
            return static_cast<float>(data_.longValue);
        default:
            return defaultValue;
    }
}

double ContextValue::asDouble(double defaultValue) const {
    switch (type_) {
        case Type::Int:
            return static_cast<double>(data_.intValue);
        case Type::Float:
            return static_cast<double>(data_.floatValue);
        case Type::Double:
            return data_.doubleValue;
        case Type::Long:
            return static_cast<double>(data_.longValue);
        default:
            return defaultValue;
    }
}

long ContextValue::asLong(long defaultValue) const {
    switch (type_) {
        case Type::Int:
            return static_cast<long>(data_.intValue);
        case Type::Float:
            return static_cast<long>(data_.floatValue);
        case Type::Double:
            return static_cast<long>(data_.doubleValue);
        case Type::Long:
            return data_.longValue;
        default:
            return defaultValue;
    }
}

const std::string& ContextValue::asString() const {
    static const std::string empty;
    if (type_ == Type::String) {
        return data_.stringValue;
    }
    return empty;
}

bool ContextValue::isNumeric() const {
    return type_ == Type::Int || type_ == Type::Float || type_ == Type::Double || type_ == Type::Long;
}

void ContextValue::destroy() {
    if (type_ == Type::String) {
        data_.stringValue.~basic_string();
    }
    type_ = Type::None;
}

void ContextValue::copyFrom(const ContextValue& other) {
    switch (other.type_) {
        case Type::Int:
            data_.intValue = other.data_.intValue;
            break;
        case Type::Float:
            data_.floatValue = other.data_.floatValue;
            break;
        case Type::Double:
            data_.doubleValue = other.data_.doubleValue;
            break;
        case Type::Long:
            data_.longValue = other.data_.longValue;
            break;
        case Type::String:
            new (&data_.stringValue) std::string(other.data_.stringValue);
            break;
        case Type::None:
            break;
    }
    type_ = other.type_;
}

void ContextValue::moveFrom(ContextValue&& other) {
    switch (other.type_) {
        case Type::Int:
            data_.intValue = other.data_.intValue;
            break;
        case Type::Float:
            data_.floatValue = other.data_.floatValue;
            break;
        case Type::Double:
            data_.doubleValue = other.data_.doubleValue;
            break;
        case Type::Long:
            data_.longValue = other.data_.longValue;
            break;
        case Type::String:
            new (&data_.stringValue) std::string(std::move(other.data_.stringValue));
            break;
        case Type::None:
            break;
    }
    type_ = other.type_;
    other.destroy();
}

ContextPool::ContextPool() {}

void ContextPool::initPoolSize(int size) {
    vindexs.resize(size);
    hindexs.resize(size);
    findexs.resize(size);
}

int ContextPool::registerPool(int estimateVSize, int estimateHSize, int estimateFSize) {
    // estimateVSize *= 10;
    // estimateHSize *= 10;
    // estimateFSize *= 10;

    vpoolSizes.push_back(estimateVSize);
    hpoolSizes.push_back(estimateHSize);
    fpoolSizes.push_back(estimateFSize);

    vprops_.push_back(OpenMesh::PropertyContainer());
    hprops_.push_back(OpenMesh::PropertyContainer());
    fprops_.push_back(OpenMesh::PropertyContainer());

    const bool poolProf = std::getenv("PMSF_POOL_PROF") != nullptr;
    const auto poolT0 = std::chrono::steady_clock::now();
    MCGAL::Vertex* vpool = new MCGAL::Vertex[estimateVSize];
    const auto poolT1 = std::chrono::steady_clock::now();
    // perf（见 ContextPool.h hraw_/fraw_ 注释）：H/F 池只申请裸内存，槽位在
    // allocate*/preAlloc* 时 placement-new；V 池保持整体构造（成本仅 ~1.6 ms）。
    MCGAL::Halfedge* hpool = static_cast<MCGAL::Halfedge*>(
        ::operator new[](static_cast<std::size_t>(estimateHSize) * sizeof(MCGAL::Halfedge)));
    const auto poolT2 = std::chrono::steady_clock::now();
    MCGAL::Facet* fpool = static_cast<MCGAL::Facet*>(
        ::operator new[](static_cast<std::size_t>(estimateFSize) * sizeof(MCGAL::Facet)));
    const auto poolT3 = std::chrono::steady_clock::now();
    hraw_.push_back(1);
    fraw_.push_back(1);
    if (poolProf) {
        auto dms = [](const std::chrono::steady_clock::time_point& a,
                      const std::chrono::steady_clock::time_point& b) {
            return std::chrono::duration<double, std::milli>(b - a).count();
        };
        std::fprintf(stderr,
                     "[POOL_PROF] V=%d (%.2f ms, sizeof=%zu) H=%d (%.2f ms, sizeof=%zu) "
                     "F=%d (%.2f ms, sizeof=%zu) total=%.2f ms\n",
                     estimateVSize, dms(poolT0, poolT1), sizeof(MCGAL::Vertex),
                     estimateHSize, dms(poolT1, poolT2), sizeof(MCGAL::Halfedge),
                     estimateFSize, dms(poolT2, poolT3), sizeof(MCGAL::Facet),
                     dms(poolT0, poolT3));
    }

    vpools.push_back(vpool);
    hpools.push_back(hpool);
    fpools.push_back(fpool);
    return poolIndex++;
}

bool ContextPool::hasPool(int index) const {
    return index >= 0 &&
           index < static_cast<int>(vpools.size()) &&
           index < static_cast<int>(hpools.size()) &&
           index < static_cast<int>(fpools.size()) &&
           vpools[index] != nullptr &&
           hpools[index] != nullptr &&
           fpools[index] != nullptr;
}

int ContextPool::poolSizeFromEnv(const char* name, int fallback) {
    const char* raw = std::getenv(name);
    if (!raw || raw[0] == '\0') {
        return fallback;
    }

    errno = 0;
    char* end = nullptr;
    long value = std::strtol(raw, &end, 10);
    if (errno != 0 || end == raw || *end != '\0' || value <= 0 || value > INT_MAX) {
        return fallback;
    }
    return static_cast<int>(value);
}

void ContextPool::releasePool(int index) {}

ContextPool::~ContextPool() {
    // for (size_t i = 0; i < poolIndex; ++i) {
    //     delete vpools[i];
    // }

    // for (size_t i = 0; i < poolIndex; ++i) {
    //     delete hpools[i];
    // }

    // for (size_t i = 0; i < poolIndex; ++i) {
    //     delete fpools[i];
    // }
}

void ContextPool::setMetadata(const std::string& key, ContextValue value) {
    std::lock_guard<std::mutex> guard(metadataMutex_);
    // 热路径同步：如果写的是 "level"（PMSF / CompressPipeline 每轮设一次），
    // 把值 store 到无锁原子，`Level2DegreeN*::isRemovable` 可以用
    // `contextPool.levelFast()` 免 mutex + hashmap + string 读取，
    // 在 bunny 上把 MMSE/APTM/PCLTTM/RD/VQMMC 的压缩时间降 30–50%。
    if (key == "level") {
        cachedLevel_.store(value.asInt(), std::memory_order_relaxed);
    }
    metadata_[key] = std::move(value);
}

bool ContextPool::hasMetadata(const std::string& key) const {
    std::lock_guard<std::mutex> guard(metadataMutex_);
    return metadata_.find(key) != metadata_.end();
}

ContextValue ContextPool::getMetadataOrDefault(const std::string& key, const ContextValue& defaultValue) const {
    std::lock_guard<std::mutex> guard(metadataMutex_);
    auto iter = metadata_.find(key);
    return iter != metadata_.end() ? iter->second : defaultValue;
}

bool ContextPool::tryGetMetadata(const std::string& key, ContextValue& outValue) const {
    std::lock_guard<std::mutex> guard(metadataMutex_);
    auto iter = metadata_.find(key);
    if (iter == metadata_.end()) {
        return false;
    }
    outValue = iter->second;
    return true;
}

void ContextPool::removeMetadata(const std::string& key) {
    std::lock_guard<std::mutex> guard(metadataMutex_);
    metadata_.erase(key);
}

void ContextPool::clearMetadata() {
    std::lock_guard<std::mutex> guard(metadataMutex_);
    metadata_.clear();
}

}  // namespace MCGAL
