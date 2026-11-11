/*
 * @file ConquestSymbolCollectOperator.cpp
 * @brief 编码端薄壳的实现（设计定案见蓝图 §62.2 / §65 / §67）。
 *
 * 分工：**引擎跑遍历**（`FrontierConquestEngine`），本类只做
 * 「引擎 → 符号表」的桥 —— 与 `PomarSymbolCollectOperator` 同形（那边 `collect` 是空壳、
 * 真负载走边信道），差别只在本类主动驱动引擎并把它的两样输出收下。
 */
#include "Conquest/compress/ConquestSymbolCollectOperator.h"

#include "Conquest/hooks/AnchorVetoPredicate.h"
#include "conquest/ConquestBlobFormat.h"
#include "conquest/ConquestDecodeLayer.h"  // defaultEngineHooks()
#include "conquest/FrontierConquestEngine.h"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <unordered_set>
#include <utility>

namespace pmsf {

namespace {

/// 收下引擎的几何残差（整数域 `cellDelta`）。
class DeltaSink : public pmsf::GeometrySink {
  public:
    explicit DeltaSink(std::vector<std::array<int, 3>>* out) : out_(out) {}

    void onRemovedVertex(const int cellDelta[3]) override {
        out_->push_back({cellDelta[0], cellDelta[1], cellDelta[2]});
    }

  private:
    std::vector<std::array<int, 3>>* out_;
};

}  // namespace

void ConquestSymbolCollectOperator::init(std::shared_ptr<MCGAL::Mesh> mesh,
                                         std::vector<MCGAL::Halfedge*> seeds,
                                         PMSFHeader header) {
    mesh_ = std::move(mesh);
    SymbolCollectOperator::init(mesh_, seeds, header);
    computeGridFromMesh();
}

// 各向同性单步长 = 包围盒**最长边** / 2^bits。
// ⚠️ 定义必须与解码端**逐位相同** —— 蓝图 §54 的教训：量化器的定义经有损
// 中间表示会伪装成"被测对象错了"，所以它随流一起写（见 ConquestBlobFormat.h）。
void ConquestSymbolCollectOperator::computeGridFromMesh() {
    stream_.layers.clear();
    if (gridForced_ || !mesh_)
        return;
    double lo[3] = {0, 0, 0}, hi[3] = {0, 0, 0};
    bool first = true;
    for (MCGAL::Vertex* v : mesh_->vertices()) {
        if (!v || v->isRemoved())
            continue;
        const double p[3] = {v->x(), v->y(), v->z()};
        for (int k = 0; k < 3; ++k) {
            if (first) {
                lo[k] = hi[k] = p[k];
            } else {
                lo[k] = std::min(lo[k], p[k]);
                hi[k] = std::max(hi[k], p[k]);
            }
        }
        first = false;
    }
    if (first) {
        grid_ = pmsf::FrontierConquestEngine::QuantGrid{};
        return;  // 空网格
    }
    double extent = 0.0;
    for (int k = 0; k < 3; ++k)
        extent = std::max(extent, hi[k] - lo[k]);
    // ★ **优先用框架那一把尺子**（`Mesh::bboxMin` / `Mesh::f_quantStep`）。
    //
    // 容器把 base mesh 按**框架的**量化器存成量化坐标；解码端手上的点全在
    // 那个量化器的格中心。conquest 若自算一套，两侧就是**两把尺子** ——
    // 实测两者 `bboxMin` 三轴逐位相同，但 `step` 从**第 8 位有效数字**起分开
    // （框架 float 0.00038357224548 vs 自算 double 0.00038357226003），
    // 格边界上就会翻格（蓝图 §85.3）。
    // 这正是 §54 的老坑：**量化器的定义过了有损中间表示**。别再过一次。
    if (mesh_->enableQuantization && mesh_->f_quantStep > 0.0f) {
        grid_.bboxMin[0] = (double)mesh_->bboxMin[0];
        grid_.bboxMin[1] = (double)mesh_->bboxMin[1];
        grid_.bboxMin[2] = (double)mesh_->bboxMin[2];
        grid_.step = (double)mesh_->f_quantStep;
        quantBits_ = mesh_->i_nbQuantBits;
        stream_.bboxMin[0] = grid_.bboxMin[0];
        stream_.bboxMin[1] = grid_.bboxMin[1];
        stream_.bboxMin[2] = grid_.bboxMin[2];
        stream_.step = grid_.step;
        snapMeshToCells();
        return;
    }
    grid_.bboxMin[0] = lo[0];
    grid_.bboxMin[1] = lo[1];
    grid_.bboxMin[2] = lo[2];
    grid_.step = extent / (double)(1u << quantBits_);
    stream_.bboxMin[0] = grid_.bboxMin[0];
    stream_.bboxMin[1] = grid_.bboxMin[1];
    stream_.bboxMin[2] = grid_.bboxMin[2];
    stream_.step = grid_.step;
    snapMeshToCells();
}

// ★ **闭环预测的前提**：让编码端与解码端看到**同一个世界**。
//
// 容器把 base mesh 存成**量化坐标**（cell 中心），所以解码端手上的每一个点
// 都落在格中心。而编码端原先用的是**原始坐标** ⇒ 预测子输入与解码端不同
// ⇒ 重插点开环漂移（实测 A2 = 2.0–3.0×step，而 default/rd 只有 0.77×step）。
//
// 把工作网格先吸附到格中心，两侧的邻居坐标就一致了；此后重插点的格号
// 与编码端算出的完全相同。剩下的只是"原网格本身不在格中心"那 0.87×step，
// 那是所有算法共有的容器量化代价。
// 诊断（env 门控，默认零输出）：把**吸附后的原始网格**落盘。
// 用途见蓝图 §86.5 —— 与解码结果并排，一次判定"存活点是否逐位相同"。

void ConquestSymbolCollectOperator::snapMeshToCells() {
    if (!mesh_ || !grid_.valid())
        return;
    // 诊断（env 门控）：把**吸附当刻**网格自带的锚点打出来。
    // 与解码端读到的 metadata 锚点并排，判定"锚点在中途变过"是否成立（蓝图 §87.4）。
    // 字典序最小（x,y,z）的顶点 —— 与解码端同一选法，可直接比位串
        for (MCGAL::Vertex* v : mesh_->vertices()) {
        if (!v || v->isRemoved())
            continue;
        // ★ **直接调网格自己那一对函数**，不要另写一套取整。
        //
        // 实测（蓝图 §86.5）：自写的 `toCell=ceil((x−min)/step)−1` 与
        // `Mesh::floatPosToInt`（**浮点除法 + 截断**）在格边界上会差**整整一格**
        // —— 148 个点里有 97 个。容器存 base mesh 走的就是后者，
        // 两侧一旦不同，重插点的预测子就跟着错。
        // 这正是硬约束那条：「验的判据与被验的东西**必须用同一套量化器**」。
        // ★ 容器写 base mesh 时还会把格号**夹进 [0, 2^quantBits − 1]**
        //   （`ColumnarSerializeOperator.cpp:139` 的 clampBaseMeshQuantCoord）。
        //   裸 `floatPosToInt` 在 bbox 最大角给 `q = extent/step = 2^bits`，
        //   夹持把它压回 `2^bits − 1` ⇒ 那个角上的点被拉回**一格**。
        //   实测（蓝图 §89.3 的逐轴直方图）：444 个轴里恰好 1 个差一格，正对应这里。
        //   编码端必须照做同一套，否则它假设的世界与解码端拿到的**差一格**。
        MCGAL::PointInt q = mesh_->floatPosToInt(v->point());
        const int bits = (int)mesh_->i_nbQuantBits;
        if (bits > 0 && bits < 31) {
            const int32_t maxV = (1 << bits) - 1;
            for (int k = 0; k < 3; ++k) {
                if (q[k] < 0) q[k] = 0;
                if (q[k] > maxV) q[k] = maxV;
            }
        }
        // ★★ **复刻解码端 base mesh 读回时那条公式，逐字一致。**
        //
        // 解码端（`ColumnarDeserializeOperator.cpp:1930`）算的是：
        //     bboxMinX_ + (q + 0.5f) * quantStep_        ← **全程 float**
        // 而 `Mesh::intPosToFloat` 算的是：
        //     bboxMin[0] + (p[0] + 0.5) * f_quantStep     ← `0.5` 是 double ⇒ **全程 double**
        // 两条路差在末位（1.7e-8，实测 base mesh 83% 的顶点不满足往返）。
        // 编码端必须用**解码端那条**，否则它以为的世界与解码端拿到的**差 1.7e-8**
        // ⇒ pred 落在格边界时翻格 ⇒ A2 残留。
        {
            double w[3];
            for (int k = 0; k < 3; ++k) {
                const float bm = (float)grid_.bboxMin[k];
                const float st = (float)grid_.step;
                w[k] = (double)(bm + ((float)q[k] + 0.5f) * st);
            }
            v->setPoint(MCGAL::Point(w[0], w[1], w[2]));
        }
    }
}

void ConquestSymbolCollectOperator::collect(std::vector<MCGAL::Halfedge*> seeds) {
    if (!seeds.empty())
        collect(seeds[0]);
}

void ConquestSymbolCollectOperator::collect(MCGAL::Halfedge* seed) {
    if (!seed || !mesh_)
        return;

    // ⚠️ **遍历留在引擎里**：引擎的前沿式 FIFO 是本次唯一被逐层验证过的遍历
    // （四网格 × 8 层链式，层往返 topoGeomOk=1）。不要把它搬到这里重写。
    //
    // S5 hook 注入（impl 侧 env 门控，默认零影响）：PMSF_CONQUEST_ANCHOR_IDS
    // 给出锚点 poolId 集合（逗号分隔）时，本层用 AnchorVetoPredicate 换出
    // 默认 H2。锚点只**留在 base mesh** 随基网格传输，解码端零改动。
    pmsf::EngineHooksPtr hooks = hooks_;
    std::unordered_set<std::uint32_t> anchors;
    if (const char* env = std::getenv("PMSF_CONQUEST_ANCHOR_IDS")) {
        if (anchors.empty()) {
            const char* p = env;
            while (*p) {
                while (*p == ',' || *p == ' ')
                    ++p;
                if (!*p)
                    break;
                char* end = nullptr;
                const unsigned long id = std::strtoul(p, &end, 10);
                if (end == p)
                    break;
                anchors.insert((std::uint32_t)id);
                p = end;
            }
        }
        if (!anchors.empty()) {
            auto h = std::make_shared<pmsf::EngineHooks>(pmsf::defaultEngineHooks());
            h->decimation = std::make_shared<pmsf::AnchorVetoPredicate>(anchors);
            hooks = h;
        }
    }
    pmsf::FrontierConquestEngine engine(hooks);
    engine.setQuantGrid(grid_);

    // 一层 = 一条 LayerRecord。后续 gate 的 seed 由框架在下一轮重新给出，
    // 这里只负责"跑完这一层、把它的输出收好"。
    conquest_blob::LayerRecord layer;
    // 种子的**坐标**随层记下（池号在解码端不可比，见 ConquestBlobFormat.h 的说明）。
    // 次序与探针一致：`seedPos[0] = tgt(seed)`、`seedPos[1] = tgt(opp(seed))`。
    if (MCGAL::Vertex* a = seed->end_vertex()) {
        layer.seedPos[0][0] = a->x();
        layer.seedPos[0][1] = a->y();
        layer.seedPos[0][2] = a->z();
    }
    if (MCGAL::Halfedge* o = seed->opposite()) {
        if (MCGAL::Vertex* b = o->end_vertex()) {
            layer.seedPos[1][0] = b->x();
            layer.seedPos[1][1] = b->y();
            layer.seedPos[1][2] = b->z();
        }
    }
    pmsf::ConquestState st;
    DeltaSink sink(&layer.deltas);
        engine.decimateLayer(mesh_.get(), st, seed, &layer.symbols, &sink);
    // 探针（env 门控，默认零影响）：把本层残差 int32 三元组追加到文件，
    // 供离线熵测量（如 H4 极坐标变换的收益上界证伪）。格式：每行 "x y z"。
        if (!gridForced_)
        stream_.step = grid_.step;
    stream_.layers.push_back(std::move(layer));
}

int ConquestSymbolCollectOperator::exportToBufferWithRangeCoder(char* buffer, bool /*enableQuantization*/) {
    // 布局的唯一来源是 `conquest_blob`（编解码同一个头）。
    // 容量契约：框架侧给的是 `BUFFER_SIZE` 的 scratch（`Configuration.h`，
    // 600 MB）。**必须显式比对**，因为越界在这里表现为静默截断/踩内存 ——
    // 是这一层最难查的失败模式。
    if (buffer == nullptr)
        return -1;
    // ⚠️ **层序必须逆序写**：编码按 0→N 逐层删减（148→114→84→…），
    // 解码必须 LIFO 回放（…→84→114→148）—— 第 i 层的符号只能在
    // "第 i 层删减之后的那个网格"上还原。顺序写会让解码端把第 0 层
    // 的符号重放在**最小**的网格上（实测：60 点的网格上还原出 V=83/F=1177 的垃圾）。
    // 逆序写之后解码端可以简单地顺序消费。
    conquest_blob::Stream out;
    out.step = stream_.step;
    out.bboxMin[0] = stream_.bboxMin[0];
    out.bboxMin[1] = stream_.bboxMin[1];
    out.bboxMin[2] = stream_.bboxMin[2];
    out.layers.assign(stream_.layers.rbegin(), stream_.layers.rend());

    const std::size_t need = conquest_blob::detail::serializedSize(out);
    if (need > (std::size_t)BUFFER_SIZE)
        return -1;
    return conquest_blob::writeStream(out, buffer, (std::size_t)BUFFER_SIZE);
}

int ConquestSymbolCollectOperator::exportToBuffer(char* buffer, bool enableQuantization, bool enableCompress) {
    // 照 `PomarSymbolCollectOperator` 的形：真负载走边信道，这里只做前置检查。
    (void)buffer;
    (void)enableQuantization;
    (void)enableCompress;
    return 0;
}

const pmsf::PMSFSymbolTable* ConquestSymbolCollectOperator::pmsfSymbolTable() const {
    paperTable_.clear();
    // 照 `PomarSymbolCollectOperator` 的形：记录本身只占位，
    // **真负载（连通性符号 + 几何残差）走边信道**，由 storage 层取走。
    pmsf::PMSFSymbolRecord& row = pmsf::appendPMSFSymbolRecord(paperTable_, 0, 0, 0);
    (void)row;
    return &paperTable_;
}

}  // namespace pmsf
