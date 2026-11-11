/*
 * @file SimplificationAlgorithmRegistry.h
 * @brief 字符串键算法注册表 —— 取代原先的 preset 枚举 + 手工清单。
 *
 * 接入一个算法的全部工作：
 *   1. 新建 `impl/<ALGO>/<Algo>Algorithm.cpp`，继承 `pmsf::ISimplificationAlgorithm`；
 *   2. 文件末尾写 `PMSF_REGISTER_SIMPLIFICATION_ALGORITHM(<Algo>Algorithm);`
 *   3. 结束。不需要改 src/、不需要改 CMakeLists（impl/ 目录由 GLOB 收集）。
 *
 * 键 = `info().name`，也就是写进 metadata 的算法名；解压时框架按这个名从注册表
 * 再实例化，因此**解压侧零算法代码**（设计文档 §2.1.2）。
 */
#ifndef PMSF_ALGORITHM_ALGORITHM_REGISTRY_H
#define PMSF_ALGORITHM_ALGORITHM_REGISTRY_H

#include "algorithm/ISimplificationAlgorithm.h"

#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <vector>

namespace pmsf {

class SimplificationAlgorithmRegistry {
  public:
    using Factory = std::function<std::shared_ptr<ISimplificationAlgorithm>()>;

    static SimplificationAlgorithmRegistry& instance();

    /// @brief 注册算法。重复注册同一 name 会抛异常（防止静默覆盖）。
    void add(const std::string& name, Factory factory);

    /// @brief 按 name 实例化。未注册返回 nullptr。
    std::shared_ptr<ISimplificationAlgorithm> create(const std::string& name) const;

    bool has(const std::string& name) const;

    /// @brief 已注册算法名（升序），用于诊断与 CLI 帮助。
    std::vector<std::string> names() const;

  private:
    SimplificationAlgorithmRegistry() = default;
    std::unordered_map<std::string, Factory> factories_;
};

/// @brief 静态自注册器。`PMSF_REGISTER_SIMPLIFICATION_ALGORITHM(T)` 展开为文件作用域的一个静态对象。
template <typename AlgoT>
struct SimplificationAlgorithmRegistrar {
    SimplificationAlgorithmRegistrar() {
        static_assert(std::is_base_of<ISimplificationAlgorithm, AlgoT>::value,
                      "PMSF_REGISTER_SIMPLIFICATION_ALGORITHM 的参数必须继承 pmsf::ISimplificationAlgorithm");
        // 实例化一次以取 name；注册工厂以便每次运行拿到全新实例（算法层可持有状态）。
        SimplificationAlgorithmRegistry::instance().add(
            AlgoT{}.info().name,
            []() -> std::shared_ptr<ISimplificationAlgorithm> {
                return std::make_shared<AlgoT>();
            });
    }
};

/// @brief 按 `cfg.algorithmName` 分发压缩。
///        名字在父类契约注册表中 ⇒ 走 ISimplificationAlgorithm；否则回落旧的
///        IAlgorithmEntry 路径（Phase 1 过渡桥，Phase 4 移除）。
void dispatchCompressByName(const PMSFConfig& cfg);
/// @brief 按 `cfg.algorithmName` 分发解压。语义同 dispatchCompressByName。
void dispatchDecompressByName(const PMSFConfig& cfg);

/// @brief Phase 1 过渡桥：把尚未迁移的算法转发给旧的 IAlgorithmEntry 注册表。
///        Phase 4 删除旧路径后随之移除。

}  // namespace pmsf

// 注册宏：在文件作用域放一个静态对象，链接时自动完成注册。
// 匿名命名空间 + 行号后缀 ⇒ 同一个 TU 可注册多个算法且不冲突。
#define PMSF_ALGO_REGISTRAR_CAT_(a, b) a##b
#define PMSF_ALGO_REGISTRAR_CAT(a, b) PMSF_ALGO_REGISTRAR_CAT_(a, b)

#define PMSF_REGISTER_SIMPLIFICATION_ALGORITHM(T)                                             \
    namespace {                                                                \
    [[maybe_unused]] const ::pmsf::SimplificationAlgorithmRegistrar<T>                       \
        PMSF_ALGO_REGISTRAR_CAT(pmsf_algo_registrar_, __LINE__){};             \
    }

#endif  // PMSF_ALGORITHM_ALGORITHM_REGISTRY_H
