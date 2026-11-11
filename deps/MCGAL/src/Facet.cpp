#include "Facet.h"
#include "Global.h"
#include "Halfedge.h"
#include "Point.h"
#include "Vertex.h"
#include "stdio.h"
#include <cstdint>
#include <fstream>
#include <iostream>
#include <tuple>
#include <vector>

namespace MCGAL {

MCGAL::Vector3 Facet::computeNormal() const {
    // ⚠️ 2026-05-04 性能优化：去掉 `std::vector<Point>` 堆分配 —— 仅需前 3 个顶点
    //   即可算法向量（叉积基于 p0→p1、p0→p2）。面 degree ≥ 3 时多余顶点不参与计算。
    //   bunny 上 QEM/POMAR/IsFlip 热路径反复调用此函数（百万级），原 `vector::push_back`
    //   每次触发堆 alloc/free；直接栈读 3 个顶点把 `~20%` 的时间省下。
    //   注：面 degree < 3 的情况在 Mesh 构造时已被拦截（Facet::reset asserts vs.size() >= 3）。
    MCGAL::Halfedge* h = proxyHalfedge_;
    if (!h || !h->next()) return MCGAL::Vector3(0, 0, 0);
    const MCGAL::Point& p1 = h->vertex()->point();
    const MCGAL::Point& p2 = h->next()->vertex()->point();
    const MCGAL::Point& p3 = (h->next()->next() ? h->next()->next()->vertex()
                                                 : h->next()->end_vertex())->point();
    // 计算两个向量
    Vector3 v1(p2.x() - p1.x(), p2.y() - p1.y(), p2.z() - p1.z());
    Vector3 v2(p3.x() - p1.x(), p3.y() - p1.y(), p3.z() - p1.z());

    // 计算叉积得到法向量
    Vector3 normal = v1.cross(v2);

    // 归一化法向量
    normal = normal.normalize();

    return normal;
}

Facet::Facet(const Facet& face) {
    this->proxyHalfedge_ = face.proxyHalfedge();
    this->updateNormal();
}

Facet::~Facet() {}

Facet::Facet(Halfedge* hit) {
    Halfedge* st(hit);
    Halfedge* ed(hit);
    std::vector<Halfedge*> edges;
    do {
        edges.push_back(st);
        st = st->next();
    } while (st != ed);
    this->reset(edges);
    this->updateNormal();
}

Facet* Facet::clone() {
    return new Facet(*this);
}

Facet::Facet(const std::vector<Vertex*>& vs) {
    Halfedge* prev = nullptr;
    Halfedge* head = nullptr;
    for (int i = 0; i < vs.size(); i++) {
        // vertices_.push_back(vs[i]);
        Vertex* nextv = vs[(i + 1) % vs.size()];
        Halfedge* hf = MCGAL::contextPool.allocateHalfedgeFromPool(DEFAULT_MESH_ID, vs[i], nextv);
        // halfedges_.push_back(hf);
        hf->setFace(this);
        if (prev != NULL) {
            prev->setNext(hf);
        } else {
            head = hf;
        }
        if (i == vs.size() - 1) {
            hf->setNext(head);
        }
        prev = hf;
    }
    proxyHalfedge_ = prev;
    this->updateNormal();
}

void Facet::reset(const std::vector<Vertex*>& vs) {
    Halfedge* prev = nullptr;
    Halfedge* head = nullptr;
    // vertices_.clear();
    // vertices_.shrink_to_fit();
    // halfedges_.clear();
    // halfedges_.shrink_to_fit();
    for (int i = 0; i < vs.size(); i++) {
        // vertices_.push_back(vs[i]);
        Vertex* nextv = vs[(i + 1) % vs.size()];
        Halfedge* hf = MCGAL::contextPool.allocateHalfedgeFromPool(DEFAULT_MESH_ID, vs[i], nextv);
        // halfedges_.push_back(hf);
        hf->setFace(this);
        if (prev != NULL) {
            prev->setNext(hf);
        } else {
            head = hf;
        }
        if (i == vs.size() - 1) {
            hf->setNext(head);
        }
        prev = hf;
    }
    this->proxyHalfedge_ = prev;
    this->facet_degree_ = vs.size();
    this->updateNormal();
}

void Facet::reset(const std::vector<Vertex*>& vs, int meshId) {
    Halfedge* prev = nullptr;
    Halfedge* head = nullptr;
    for (int i = 0; i < vs.size(); i++) {
        Vertex* nextv = vs[(i + 1) % vs.size()];
        Halfedge* hf = MCGAL::contextPool.allocateHalfedgeFromPool(meshId, vs[i], nextv);
        hf->setFace(this);
        if (prev != NULL) {
            prev->setNext(hf);
        } else {
            head = hf;
        }
        if (i == vs.size() - 1) {
            hf->setNext(head);
        }
        prev = hf;
    }
    this->facet_degree_ = vs.size();
    this->proxyHalfedge_ = prev;
    this->updateNormal();
}

// =====================================================================
// PCLTTM/PCATM Phase 2 专用：reset_proxy_first(vs)
// =====================================================================
//
// 与 reset(vs) 唯一差异：proxyHalfedge_ 设为**第一条** halfedge (vs[0]→vs[1])，
// 而不是 reset(vs) 默认的最后一条 (vs[N-1]→vs[0])。
//
// 必要性：encoder 端 Columnar 序列化 faceVids 时按
//   `for (st = fit->proxyHalfedge(); ; st = st->next())` 写入，所以
//   faceVids[0] = encoder 的 proxyHalfedge.vertex() = encoder 的 v0。
// decoder 端默认 reset(vs) 把 proxyHalfedge 设为最后一条 → vertex = vs[N-1]，
// 与 encoder v0 错位，导致 PCLTTM Phase 2 的 v0-fan 拓扑推断失败。
//
// 用 reset_proxy_first 让 decoder 端 proxyHalfedge.vertex = vs[0] = 对齐
// encoder v0，PCLTTM/PCATM 路径调本方法即可。
//
// 9 个旧算法仍调 reset(vs)，本方法零影响（本方法不被旧路径调用）。
void Facet::reset_proxy_first(const std::vector<Vertex*>& vs) {
    Halfedge* prev = nullptr;
    Halfedge* head = nullptr;
    for (int i = 0; i < (int)vs.size(); i++) {
        Vertex* nextv = vs[(i + 1) % vs.size()];
        Halfedge* hf = MCGAL::contextPool.allocateHalfedgeFromPool(DEFAULT_MESH_ID, vs[i], nextv);
        hf->setFace(this);
        if (prev != nullptr) {
            prev->setNext(hf);
        } else {
            head = hf;
        }
        if (i == (int)vs.size() - 1) {
            hf->setNext(head);
        }
        prev = hf;
    }
    this->proxyHalfedge_ = head;  // 关键差异：第一条而非最后一条
    this->facet_degree_ = (int)vs.size();
    this->updateNormal();
}

void Facet::reset_proxy_first(const std::vector<Vertex*>& vs, int meshId) {
    Halfedge* prev = nullptr;
    Halfedge* head = nullptr;
    for (int i = 0; i < (int)vs.size(); i++) {
        Vertex* nextv = vs[(i + 1) % vs.size()];
        Halfedge* hf = MCGAL::contextPool.allocateHalfedgeFromPool(meshId, vs[i], nextv);
        hf->setFace(this);
        if (prev != nullptr) {
            prev->setNext(hf);
        } else {
            head = hf;
        }
        if (i == (int)vs.size() - 1) {
            hf->setNext(head);
        }
        prev = hf;
    }
    this->proxyHalfedge_ = head;  // 关键差异：第一条而非最后一条
    this->facet_degree_ = (int)vs.size();
    this->updateNormal();
}

// =====================================================================
// PCLTTM/PCATM Phase 2 专用：normalize_proxy_to_min_position_vertex
// =====================================================================
//
// 把 proxyHalfedge 重置为 face 中**vertex 量化整数坐标 (qx, qy, qz)
// 字典序最小**的那个 vertex 出发的 outgoing halfedge。
//
// 用途：encoder/decoder 双端共用的"标准化 proxyHalfedge"规则——保证两端
// 在 collect/collectFromDecoded 阶段拿到同一条 halfedge 作为 polyFace
// 的入口，从而 polyFace.getVertices() 顺序一致，v0-fan 拓扑推断对齐。
//
// 关键：用 floatPosToInt(bboxMin, f_quantStep) 把 vertex 物理坐标量化到
// 整数后再做字典序——PMSF 量化保证两端 bit-exact 一致；vertex.point()
// 直接的 float 字典序在边界值上会发散（encoder 端 vertex 来自 OFF parse，
// decoder 端来自 intPosToFloat 反量化，浮点末位不同）。
//
// 不调 mesh 几何修改 API，只改 proxyHalfedge_ 字段，O(N) 时间。
// 9 个旧算法不调本方法，零影响。
void Facet::normalize_proxy_to_min_position_vertex(const Point& bboxMin, float f_quantStep) {
    if (proxyHalfedge_ == nullptr) return;

    auto qpos = [&](Vertex* v) -> std::tuple<int64_t, int64_t, int64_t> {
        // 与 Mesh::floatPosToInt 同源 trunc（PointInt 构造从 double 隐式 trunc）
        return {
            static_cast<int64_t>((v->x() - bboxMin.x()) / f_quantStep),
            static_cast<int64_t>((v->y() - bboxMin.y()) / f_quantStep),
            static_cast<int64_t>((v->z() - bboxMin.z()) / f_quantStep)
        };
    };
    auto less_q = [&](Vertex* a, Vertex* b) -> bool {
        if (a == nullptr) return false;
        if (b == nullptr) return true;
        auto [ax, ay, az] = qpos(a);
        auto [bx, by, bz] = qpos(b);
        if (ax != bx) return ax < bx;
        if (ay != by) return ay < by;
        return az < bz;
    };

    Halfedge* st = proxyHalfedge_;
    Halfedge* best = st;
    Halfedge* it = st->next();
    // ── PHASE64：**步数上限**（修 dragon 上的活锁） ────────────────────────
    //
    // 实测（附 88）：`pclttm` 解压 dragon 时在此处 **100% CPU 自旋不退出**
    // （core 栈 `Facet.cpp:221/230 ← normalize_proxy_to_min_position_vertex:240`，
    // `/proc` 采样 800→2000 jiffies / 12s，state=R）。原因：`next()` 链
    // **既不回到 `st`、也不为 null** ⇒ `while (it != nullptr && it != st)`
    // 永不退出。这是 D17「环遍历死循环」同一个族，只是这里绕的是 **facet 环**。
    //
    // 修法与本仓库既有 idiom 一致（`pmsf::boundedRing(v, maxSteps)`、
    // D17 的 `vertexRingCloses`）：给环走一个**步数上限**。
    // 正常面上从 `st->next()` 走 2 步必然回到 `st`（三角面）
    // ⇒ 固定上限 **永不触发** ⇒ 门禁网格逐字节不变。
    //
    // ⚠️ 本方法只有 PCLTTM 调（见上文注释「9 个旧算法不调本方法，零影响」）。
    // ⚠️ PHASE65/D18：上限**不得**取 `facet_degree()` —— 本仓库源码自己写着
    //    不能信它（`src/conquest/ConquestTraversal.h:38-45`：「它是**存量计数器**
    //    …只要某个面的 degree 在某次拓扑改动后没被刷新…**这个信任也不该保留**」）。
    //    若它偏小 ⇒ 环游**提前截断** ⇒ `best` 选错 ⇒ `proxyHalfedge_` 错位
    //    ⇒ BFS 序发散。改用**固定常量**，与本仓库 `pmsf::boundedRing(v, maxSteps=256)`
    //    的 idiom 一致，**完全不碰任何可能陈旧的计数器**。
    //    正常三角面走 2 步即回到 `st` ⇒ 256 永不触发 ⇒ 门禁逐字节不变。
    constexpr int kMaxRingSteps = 256;
    int steps = 0;
    while (it != nullptr && it != st && steps < kMaxRingSteps) {
        if (less_q(it->vertex(), best->vertex())) {
            best = it;
        }
        it = it->next();
        ++steps;
    }
    proxyHalfedge_ = best;
}

// 浮点版（已废弃，仅保留作向后兼容）：直接对 vertex.x()/y()/z() 浮点字典序排序。
// 风险：encoder/decoder 两端浮点不 bit-exact 一致时会发散。请改用上面的
// 量化整数版（接收 bboxMin/f_quantStep）。
void Facet::normalize_proxy_to_min_position_vertex() {
    if (proxyHalfedge_ == nullptr) return;

    auto less_xyz = [](Vertex* a, Vertex* b) -> bool {
        // 字典序 (x, y, z)：a < b
        if (a == nullptr) return false;
        if (b == nullptr) return true;
        if (a->x() != b->x()) return a->x() < b->x();
        if (a->y() != b->y()) return a->y() < b->y();
        return a->z() < b->z();
    };

    Halfedge* st = proxyHalfedge_;
    Halfedge* ed = proxyHalfedge_;
    Halfedge* best = st;
    Halfedge* it = st->next();
    while (it != nullptr && it != ed) {
        if (less_xyz(it->vertex(), best->vertex())) {
            best = it;
        }
        it = it->next();
    }
    proxyHalfedge_ = best;
}

void Facet::reset(Halfedge* h) {
    // 为什么会有两个面都把你设置为了proxy
    if(h->poolId() == 208976) {
        int i = 0;
    }
    if (!h) {
        this->proxyHalfedge_ = nullptr;
        this->facet_degree_ = 0;
        return;
    }
    Halfedge* st = h;
    Halfedge* ed = h;
    int deg = 0;
    int guard = 0;
    do {
        deg++;
        st->setFace(this);
        st = st->next();
    } while (st && st != ed && ++guard <= 8192);
    if (!st) ++deg;
    this->proxyHalfedge_ = h;
    this->updateNormal();
    this->facet_degree_ = deg;
}

/**
 * lazy init
 */
void Facet::reset_without_init(Halfedge* h) {
    if (!h) {
        this->proxyHalfedge_ = nullptr;
        this->facet_degree_ = 0;
        return;
    }
    Halfedge* st = h;
    Halfedge* ed = h;
    this->proxyHalfedge_ = h;
    int deg = 0;
    int guard = 0;
    do {
        deg++;
        st->setFace(this);
        st = st->next();
    } while (st && st != ed && ++guard <= 8192);
    if (!st) ++deg;
    this->facet_degree_ = deg;
    // this->updateNormal();
}

void Facet::reset(std::vector<Halfedge*>& hs) {
    for (int i = 0; i < hs.size(); i++) {
        hs[i]->setFace(this);
    }
    this->updateNormal();
}

std::vector<Vertex*> Facet::getVertices() {
    std::vector<Vertex*> vs;
    getVertices(vs);
    return vs;
}

void Facet::getVertices(std::vector<Vertex*>& vs) {
    vs.clear();
    Halfedge* st = proxyHalfedge_;
    Halfedge* ed = proxyHalfedge_;

    int guard = 0;
    do {
        vs.push_back(st->vertex());
        if (st->next() == nullptr) {
            vs.push_back(st->end_vertex());
        }
        st = st->next();
    } while (st != ed && st != nullptr && ++guard <= 8192);
}

double Facet::areas() {
    std::vector<MCGAL::Vertex*> points = getVertices();
    double abx = points[1]->x() - points[0]->x();
    double aby = points[1]->y() - points[0]->y();
    double abz = points[1]->z() - points[0]->z();
    double acx = points[2]->x() - points[0]->x();
    double acy = points[2]->y() - points[0]->y();
    double acz = points[2]->z() - points[0]->z();
    double nx = aby * acz - abz * acy;
    double ny = abz * acx - abx * acz;
    double nz = abx * acy - aby * acx;
    double norm = std::sqrt(nx * nx + ny * ny + nz * nz);
    return 0.5 * norm;
}

std::vector<Halfedge*> Facet::getHalfedges() {
    std::vector<Halfedge*> hs;
    Halfedge* st = proxyHalfedge_;
    Halfedge* ed = proxyHalfedge_;
    // hs.push_back(st);
    int guard = 0;
    do {
        hs.push_back(st);
        st = st->next();
    } while (st != ed && st != nullptr && ++guard <= 8192);
    return hs;
}

void Facet::print() {
    std::vector<MCGAL::Vertex*> vs;
    for (auto it = proxyHalfedge_; it != proxyHalfedge_->next(); it = it->next()) {
        vs.push_back(it->vertex());
    }
    printf("totally %ld vertices_:\n", vs.size());
    int idx = 0;
    for (Vertex* v : vs) {
        printf("%d:\t", idx++);
        v->print();
    }
}

void Facet::print_off() {
    // TODO: 待完善
    // printf("OFF\n%ld 1 0\n", vertices_.size());
    // for (Vertex* v : vertices_) {
    //     v->print();
    // }
    // printf("%ld\t", vertices_.size());
    // for (int i = 0; i < vertices_.size(); i++) {
    //     printf("%d ", i);
    // }
    // printf("\n");
}

void Facet::dumpTo(std::string path) {
    // TODO: 待完善
    // std::ofstream offFile(path);
    // offFile << "OFF\n";
    // offFile << vertices_.size() << " " << 1 << " 0\n";
    // offFile << "\n";

    // for (Vertex* v : vertices_) {
    //     offFile << v->x() << " " << v->y() << " " << v->z() << "\n";
    // }
    // offFile << vertices_.size() << " ";
    // for (int i = 0; i < vertices_.size(); i++) {
    //     offFile << i << " ";
    // }
    // printf("\n");
}

bool Facet::equal(const Facet& rhs) const {
    return this->poolId_ == rhs.poolId();
}
bool Facet::operator==(const Facet& rhs) const {
    return this->equal(rhs);
}

Point Facet::getRemovedVertexPos() const {
    return removedVertexPos;
};

PointInt Facet::getRemovedVertexPosInt() const {
    return removedVertexPosInt;
};

void Facet::setRemovedVertexPos(Point p) {
    removedVertexPos = p;
};

void Facet::setRemovedVertexPosInt(PointInt p) {
    removedVertexPosInt = p;
};

// Facet::facet_degree 已移到 Facet.h 作为 inline（性能优化，perf 显示
// 三角面特化的 Halfedge::find_prev 调用占 RD/MMSE/APTM 27% CPU）。

}  // namespace MCGAL
