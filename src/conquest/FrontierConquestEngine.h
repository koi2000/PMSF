/*
 * @file FrontierConquestEngine.h
 * @brief 前沿式顶点删减引擎（框架原生，**算法无关**）。
 *
 * **验证状态（2026-09-22 更新）**：引擎已与验证器（`tools/AdConquestVerify.cpp`）
 * 在三个网格上**逐符号对拍一致**：fandisk 30000/30000、cow_simp 669/669、
 * cow_open 695/695，0 处差异（含清理层）。
 * 曾经的阻塞点（迁移版清理层不一致）根因是 `decimateLayer` 兜底分支**漏了
 * `nullPatchTail`** —— 符号流上看不出来（该支符号恒为 4），但 sign 不再传播
 * ⇒ `findType` 全面退化为 0 ⇒ 拓扑分岔。详见
 * `docs/refactor/P13_3_ENGINE_HOOKS_2026-09-22.md`。
 * **教训：符号流一致 ≠ 状态一致。**
 *
 * 解码端镜像（P13.4 C）进展见
 * `docs/refactor/P13_4_DECODER_BLUEPRINT_2026-09-22.md`。
 *
 *
 * 机制（与任何具体算法无关）：
 *   1. 种子：一条半边入队；
 *   2. 每层取出队首 gate，看它"前方"的顶点（`target(next(gate))`）；
 *   3. 前端点若可删（**H2** 判据），则删掉它、把其环上的边反向入队（前沿扩张），
 *      并按**sign 对**决定重三角化类型，用 ⊕/⊖ 查表切开；
 *   4. 不可删则按 null-patch 处理，同样扩张前沿；
 *   5. 队列空即本层结束。外层反复「删减层 + 清理层」直到顶点数不再下降。
 *
 * **本文件不认识任何具体算法名**（lint 纪律）：所有可调之处都走
 * `scheme/EngineHooks.h` 的五个引擎级 hook，留空即回落默认行为。
 *
 * 与外部参考实现的两处**非显然**但必须保持的细节（都是踩过坑换来的）：
 *   - `Vertex::vertex_degree()` 在 MCGAL 里是**缓存**，而 `split_facet` /
 *     `erase_center_vertex` 都不维护它。参考实现每次现算 ⇒ 本引擎读 valence
 *     前一律 `recompute_degree()`。漏掉会让**跨层** valence 系统性错误。
 *   - 每次进入删减层/清理层都要**复位三种 flag**（对应参考实现的 `Init()`），
 *     否则第 2 层起一个点都删不掉。
 */
#ifndef PMSF_CONQUEST_FRONTIER_CONQUEST_ENGINE_H
#define PMSF_CONQUEST_FRONTIER_CONQUEST_ENGINE_H

#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Vertex.h"

#include "scheme/EngineHooks.h"

#include <cstdint>
#include <cstdio>
#include <unordered_map>
#include <vector>

namespace pmsf {

/// @brief 引擎的三值状态（对齐参考实现的 FREE / CONQUERED / TO_BE_REMOVED）。
enum class ConquestFlag : std::uint8_t { Free = 0, Conquered = 1, ToBeRemoved = 2 };

/// @brief 顶点/面标志位与符号位。用局部表，**不污染 MCGAL 自身状态**。
struct ConquestState {
    std::unordered_map<MCGAL::Vertex*, std::uint8_t> vertexFlag;
    std::unordered_map<MCGAL::Vertex*, std::uint8_t> vertexSign;  // 0=NOSIGN 1=PLUS 2=MINUS
    std::unordered_map<MCGAL::Facet*, std::uint8_t> facetFlag;

    std::uint8_t flag(MCGAL::Vertex* v) const {
        auto it = vertexFlag.find(v);
        return it == vertexFlag.end() ? 0u : it->second;
    }
    void setFlag(MCGAL::Vertex* v, std::uint8_t f) {
        vertexFlag[v] = f;
    }

    std::uint8_t sign(MCGAL::Vertex* v) const {
        auto it = vertexSign.find(v);
        return it == vertexSign.end() ? 0u : it->second;
    }
    /// @brief 仅在 NOSIGN 时写入（与参考实现一致）。
    void setSign(MCGAL::Vertex* v, std::uint8_t s) {
        if (sign(v) == 0u)
            vertexSign[v] = s;
    }

    std::uint8_t fflag(MCGAL::Facet* f) const {
        auto it = facetFlag.find(f);
        return it == facetFlag.end() ? 0u : it->second;
    }
    void setFFlag(MCGAL::Facet* f, std::uint8_t v) {
        facetFlag[f] = v;
    }

    /// @brief 复位三种 flag（= 参考实现每次进层调用的 `Init()`）。
    /// **`seed` 标记不复位** —— 它在最外层只写一次。
    void reset() {
        vertexFlag.clear();
        vertexSign.clear();
        facetFlag.clear();
    }
};

/// @brief 几何残差的**汇**（引擎只负责算，不负责编码/存储）。
/// 留 nullptr 表示本层不收集几何（连通性对拍就是这种模式）。
struct GeometrySink {
    virtual ~GeometrySink() = default;
    /// @param cellDelta 量化整数域的残差（已过 H4 的 transform）
    virtual void onRemovedVertex(const int cellDelta[3]) = 0;
};

/// @brief 一层的结果统计。
struct ConquestLayerResult {
    int removedVertices = 0;           ///< 本层删掉的顶点数
    std::uint64_t symbolsEmitted = 0;  ///< 本层发出的连通性符号数
};

/// @brief 前沿式顶点删减引擎。
class FrontierConquestEngine {
  public:
    /// @param hooks 五个引擎级 hook；nullptr 或留空项一律回落默认行为。
    explicit FrontierConquestEngine(EngineHooksPtr hooks = nullptr) : hooks_(std::move(hooks)) {}

    /// @brief 删减层：只处理 `3 ≤ valence ≤ 6`（由 **H2** 决定）的前端点。
    /// 发出的符号 = `valence - 3`（删除）或 `4`（null patch）。
    ConquestLayerResult decimateLayer(MCGAL::Mesh* mesh,
                                      ConquestState& st,
                                      MCGAL::Halfedge* seed,
                                      std::vector<int>* connectivityOut = nullptr,
                                      GeometrySink* geometry = nullptr);

    /// @brief 清理层：只处理 `valence == 3` 的前端点，**不做可删性判据**
    /// （参考实现在这一层的判据是注释掉的：度为 3 的点删除不会插入新边，
    /// 本来也不会破坏流形性）。
    /// 发出的符号是**二元**的：`0` = 删除，`1` = null。
    /// 两段式：先遍历**只打标记**，再按顶点顺序统一删。
    ConquestLayerResult regulateLayer(MCGAL::Mesh* mesh,
                                      ConquestState& st,
                                      MCGAL::Halfedge* seed,
                                      std::vector<int>* connectivityOut = nullptr,
                                      GeometrySink* geometry = nullptr);

    /// @brief 量化器配置（H4 之外的网格参数：包围盒与步长）。
    struct QuantGrid {
        double bboxMin[3] = {0, 0, 0};
        double step = 0.0;
        bool valid() const {
            return step > 0.0;
        }
    };
    void setQuantGrid(const QuantGrid& q) {
        grid_ = q;
    }

    /// @brief 逐 gate 的**前端点位置**轨迹出口（诊断用，**默认 nullptr，零默认影响**）。
    ///
    /// 每处理一个 gate 写一行 `<x> <y> <z>`，恒为 `target(next(gate))` 的坐标
    /// （删点分支与 null-patch 分支里它都存在）。
    ///
    /// 这是**验证"两端 gate 序列是否对齐"的唯一观测量**：编码端删掉的前端点，
    /// 解码端要插回同一个点；序列若在某个下标处分岔，错位就定位在那里。
    /// 由调用方打开文件并传入（`src/` 不读环境变量、不自己开文件）。
    void setGateTrace(std::FILE* f) {
        gateTrace_ = f;
    }

    /// @brief 逐删点的**预测格号**轨迹（诊断用，默认 nullptr，零默认影响）。
    /// 每删一个点写一行 `<cx> <cy> <cz>`（`toCell(pred)`）。
    /// 由调用方开文件并传入（`src/` 不读环境变量、不自己开文件）。
    /// 用途：与解码端 `decodeLayer` 的同名轨迹并排，定位两侧 pred 从第几点分开。
    void setPredTrace(std::FILE* f) {
        predTrace_ = f;
    }

    const QuantGrid& quantGrid() const {
        return grid_;
    }

    const EngineHooks& hooks() const {
        return hooksOrDefault();
    }

    /// @brief type == 0 的累计次数（诊断用；默认运行点下应为 0）。
    std::uint64_t unhandledType0Count() const {
        return unhandledType0_;
    }

    /// Independent patch operator used by the frontier driver. Exposing the
    /// operator lets a paper fixture validate its actual diagonals and tags.
    void retriangulate(MCGAL::Mesh* mesh, ConquestState& st, MCGAL::Halfedge* ch, int valence, int type);

  private:
    const EngineHooks& hooksOrDefault() const {
        static const EngineHooks kDefault;
        return hooks_ ? *hooks_ : kDefault;
    }



    /// @brief 流形性判据（参考实现 `Is_Manifold_Property_Violated` 的忠实翻译）。
    /// 四种 type 组合各有不同的**环心**与**检查下标**，见 .cpp 里的表。
    bool manifoldViolated(MCGAL::Halfedge* h, int type, int valence);

    /// @brief 记录一次几何残差（若配置了量化网格且 `geometry != nullptr`）。
    void recordGeometry(MCGAL::Mesh* mesh, MCGAL::Vertex* vdel, GeometrySink* geometry);

    EngineHooksPtr hooks_;
    QuantGrid grid_;

    /// @brief `Find_Type` 落到未覆盖组合（type == 0）的次数。
    /// 默认运行点下应为 0；引擎**不打印**，由调用方查询（保持 `src/` 无输出副作用）。
    std::uint64_t unhandledType0_ = 0;
    std::FILE* predTrace_ = nullptr;
    std::FILE* gateTrace_ = nullptr;
};

}  // namespace pmsf

#endif  // PMSF_CONQUEST_FRONTIER_CONQUEST_ENGINE_H
