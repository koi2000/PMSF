/*
 * @Author: koi
 * @Date: 2025-09-26 10:22:00
 * @Description:
 */
#pragma ONCE
#include "Halfedge.h"
#include "common/BoundedRingTraversal.h"
#include "Vertex.h"
#include "core.h"
#include <cstdlib>
#include <deque>
#include <fstream>
#include <iostream>
#include <set>
#include <string>
#include <vector>

// #define DEBUG

// 检测是否为容器（具有 begin/end）
template <typename T> class is_container {
    template <typename U> static auto test(int) -> decltype(std::declval<U>().begin(), std::declval<U>().end(), std::true_type{});
    template <typename> static std::false_type test(...);

  public:
    static constexpr bool value = decltype(test<T>(0))::value;
};

#ifdef DEBUG

constexpr std::array<std::array<int, 3>, 68> colors = {{
    // {0, 0, 0},        // Black
    {0, 255, 0},  // Green
    {255, 0, 0},  // Red

    {0, 0, 255},      // Blue
    {255, 255, 0},    // Yellow
    {255, 0, 255},    // Magenta
    {0, 255, 255},    // Cyan
    {128, 128, 128},  // Gray
    {255, 165, 0},    // Orange
    {128, 0, 128},    // Purple
    {64, 224, 208},   // Turquoise
    {255, 250, 205},  // Lemon Yellow
    {128, 0, 0},      // Maroon
    {127, 255, 212},  // Aquamarine
    {75, 0, 130},     // Indigo
    {255, 160, 122},  // Light Salmon
    {32, 178, 170},   // Light Sea Green
    {135, 206, 250},  // Light Sky Blue
    {119, 136, 153},  // Light Slate Gray
    {176, 196, 222},  // Light Steel Blue
    {255, 255, 224},  // Light Yellow
    {250, 128, 114},  // Salmon
    {106, 90, 205},   // Slate Blue
    {112, 128, 144},  // Slate Gray
    {0, 0, 128},      // Navy
    {189, 183, 107},  // Dark Khaki
    {153, 50, 204},   // Dark Orchid
    {205, 133, 63},   // Peru
    {128, 128, 0},    // Olive
    {160, 82, 45},    // Sienna
    {102, 205, 170},  // Medium Aquamarine
    {50, 205, 50},    // Lime Green
    {70, 130, 180},   // Steel Blue
    {210, 105, 30},   // Chocolate
    {154, 205, 50},   // Yellow Green
    {219, 112, 147},  // Pale Violet Red
    {173, 255, 47},   // Green Yellow
    {240, 255, 255},  // Azure
    {255, 127, 80},   // Coral
    {238, 130, 238},  // Violet
    {75, 0, 130},     // Indigo
    {255, 105, 180},  // Hot Pink
    {0, 250, 154},    // Medium Spring Green
    {0, 191, 255},    // Deep Sky Blue
    {255, 99, 71},    // Tomato
    {245, 222, 179},  // Wheat
    {255, 228, 196},  // Bisque
    {255, 215, 0},    // Gold
    {139, 69, 19},    // Saddle Brown
    {255, 228, 181},  // Moccasin
    {152, 251, 152},  // Pale Green
    {238, 232, 170},  // Pale Goldenrod
    {255, 140, 0},    // Dark Orange
    {255, 140, 105},  // Light Salmon
    {0, 128, 128},    // Teal
    {255, 228, 225},  // Blanched Almond
    {192, 192, 192},  // Silver
    {0, 0, 139},      // Dark Blue
    {173, 216, 230},  // Light Blue
    {0, 255, 255},    // Aqua
    {139, 0, 139},    // Dark Magenta
    {255, 250, 250},  // Snow
    {245, 245, 220},  // Beige
    {0, 255, 127},    // Spring Green
    {255, 105, 180},  // Hot Pink
    {255, 255, 255}   // White
}};

static void dumpTo(std::string path, std::set<MCGAL::Vertex*> vertices, std::set<MCGAL::Facet*> facets) {
    std::ofstream offFile(path);
    offFile << "OFF\n";
    offFile << vertices.size() << " " << facets.size() << " 0\n";
    offFile << "\n";
    int id = 0;
    std::set<int> idset;
    for (MCGAL::Vertex* v : vertices) {
        offFile << v->x() << " " << v->y() << " " << v->z() << "\n";
        v->setVid(id++);
        idset.insert(v->poolId());
    }
    offFile << "\n";
    for (MCGAL::Facet* face : facets) {
        if (face->isRemoved())
            continue;
        // offFile << "3 ";
        std::map<int, int> mpcnt;
        int idx = 0;
        std::vector<int> vids;
        for (auto it = face->halfedges_begin(); it != face->halfedges_end(); it++) {
            // mpcnt[(*it)->vertex()->groupId()]++;
            // offFile << (*it)->vertex()->vid() << " ";
            vids.push_back((*it)->vertex()->vid());
            assert(idset.count((*it)->vertex()->poolId()));
        }
        offFile << vids.size() << " ";
        for (int id : vids) {
            offFile << id << " ";
        }

        // if (face->groupId() >= 0) {
        //     offFile << colors[face->groupId()][0] << " " << colors[face->groupId()][1] << " " << colors[face->groupId()][2] << " ";
        // } else {
        //     offFile << 0 << " " << 0 << " " << 0 << " ";
        // }
        // if (mpcnt.size() == 1) {
        //     offFile << colors[mpcnt.begin()->first][0] << " " << colors[mpcnt.begin()->first][1] << " " << colors[mpcnt.begin()->first][2] << " ";
        // } else {
        //     offFile << 0 << " " << 0 << " " << 0 << " ";
        // }

        offFile << "\n";
    }
}

static void DEBUG_DUMP_HALFEDGE(MCGAL::Halfedge* h) {
    h->face()->dumpTo("./submesh1/h_" + std::to_string(h->poolId()) + ".off");
}

static void DEBUG_DUMP_FACE(MCGAL::Facet* f) {
    f->dumpTo("./submesh1/f_" + std::to_string(f->poolId()) + ".off");
}

static void DEBUG_DUMP_VERTEX_ONE_RING(MCGAL::Vertex* v, std::string path) {
    std::set<MCGAL::Vertex*> vone_ring;
    std::set<MCGAL::Facet*> fone_ring;
    vone_ring.insert(v);
    for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
        vone_ring.insert(h->end_vertex());
        fone_ring.insert(h->face());
    }
    dumpTo(path, vone_ring, fone_ring);
}

static void DEBUG_DUMP_VERTEX_TWO_RING(MCGAL::Vertex* v, std::string path) {
    std::set<MCGAL::Vertex*> vone_ring;
    std::vector<MCGAL::Vertex*> one_ring;
    std::set<MCGAL::Facet*> fone_ring;
    vone_ring.insert(v);
    for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
        vone_ring.insert(h->end_vertex());
        fone_ring.insert(h->face());
        one_ring.push_back(h->end_vertex());
    }
    for (MCGAL::Vertex* v : one_ring) {
        for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
            vone_ring.insert(h->end_vertex());
            fone_ring.insert(h->face());
        }
    }
    dumpTo(path, vone_ring, fone_ring);
}

static void DEBUG_DUMP_VERTEX_THREE_RING(MCGAL::Vertex* v, std::string path) {
    std::set<MCGAL::Vertex*> vone_ring;
    std::vector<MCGAL::Vertex*> one_ring;
    std::vector<MCGAL::Vertex*> two_ring;
    std::set<MCGAL::Facet*> fone_ring;
    vone_ring.insert(v);
    for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
        vone_ring.insert(h->end_vertex());
        fone_ring.insert(h->face());
        one_ring.push_back(h->end_vertex());
    }
    for (MCGAL::Vertex* v : one_ring) {
        for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
            vone_ring.insert(h->end_vertex());
            fone_ring.insert(h->face());
            two_ring.push_back(h->end_vertex());
        }
    }
    for (MCGAL::Vertex* v : two_ring) {
        for (MCGAL::Halfedge* h : pmsf::boundedRing(v)) {
            vone_ring.insert(h->end_vertex());
            fone_ring.insert(h->face());
        }
    }
    dumpTo(path, vone_ring, fone_ring);
}

template <typename Container>
typename std::enable_if<is_container<Container>::value>::type DEBUG_DUMP_CONTAINER(const Container& container, const std::string& filename) {
    std::ofstream file(filename);
    if (!file)
        throw std::runtime_error("Cannot open file: " + filename);
    for (const auto& elem : container) {
        int int_elem = (int)elem;
        file << int_elem << '\n';
    }
}

// template <typename T>
// static void DEBUG_DUMP_CONTAINER(const std::vector<T>& container, const std::string& filename) {
//     std::ofstream file(filename);
//     if (!file)
//         throw std::runtime_error("Cannot open file: " + filename);
//     for (const auto& elem : container) {
//         int int_elem = (int)elem;
//         file << int_elem << '\n';
//     }
// }

static void DEBUG_DUMP_VERTEX_CONTAINER(std::vector<MCGAL::Vertex*> vs, const std::string& filename) {
    std::ofstream file(filename);
    if (!file)
        throw std::runtime_error("Cannot open file: " + filename);
    for (const auto& elem : vs) {
        file << elem->x() << " " << elem->y() << " " << elem->z() << "\n";
    }
}

template <typename T> static void DEBU_DUMP_SYMBOL(std::vector<std::deque<T>>& symbol, const std::string& filename) {
    std::ofstream file(filename);
    for (const auto& elem : symbol) {
        for (const auto& elem2 : elem) {
            file << elem2 << ' ';
        }
        file << '\n';
    }
}

static void DEBUG_DUMP_PERFORMED_OPERATION(MCGAL::PerformedOperation& op, std::string path) {
    std::ofstream file(path);
    file << "v1: " << op.v1->x() << " " << op.v1->y() << " " << op.v1->z() << "\n";
    file << "v2: " << op.v2->x() << " " << op.v2->y() << " " << op.v2->z() << "\n";
    file << "vr: " << op.vr->x() << " " << op.vr->y() << " " << op.vr->z() << "\n";
    file << "vhPivotL: " << op.vhPivotL->x() << " " << op.vhPivotL->y() << " " << op.vhPivotL->z() << "\n";
    file << "vhPivotR: " << op.vhPivotR->x() << " " << op.vhPivotR->y() << " " << op.vhPivotR->z() << "\n";
}

static void DEBUG_DUMP_PERFORMED_OPERATION(std::queue<MCGAL::PerformedOperation> opToPerform, std::string path) {
    std::ofstream file(path);
    int idx = 0;
    while (!opToPerform.empty()) {
        file << "operation: " << idx++ << ":\n";
        MCGAL::PerformedOperation op = opToPerform.front();
        file << "v1: " << op.v1->x() << " " << op.v1->y() << " " << op.v1->z() << "\n";
        file << "v2: " << op.v2->x() << " " << op.v2->y() << " " << op.v2->z() << "\n";
        file << "vr: " << op.vr->x() << " " << op.vr->y() << " " << op.vr->z() << "\n";
        file << "vhPivotL: " << op.vhPivotL->x() << " " << op.vhPivotL->y() << " " << op.vhPivotL->z() << "\n";
        file << "vhPivotR: " << op.vhPivotR->x() << " " << op.vhPivotR->y() << " " << op.vhPivotR->z() << "\n";
        // file << "\n";
        opToPerform.pop();
    }
}

static void DEBUG_DUMP_HALFEDGES(std::vector<MCGAL::Halfedge*> halfedges, std::string path) {
    std::ofstream file(path);
    for (MCGAL::Halfedge* h : halfedges) {
        file << h->vertex()->x() << " " << h->vertex()->y() << " " << h->vertex()->z() << " ";
        file << h->end_vertex()->x() << " " << h->end_vertex()->y() << " " << h->end_vertex()->z() << "\n";
    }
}

static void DEBUG_DUMP_FACETS(std::vector<MCGAL::Facet*> facets, std::string path) {
    std::ofstream file(path);
    for (MCGAL::Facet* f : facets) {
        std::vector<float> fs;
        for (MCGAL::Halfedge* h : *f) {
            fs.push_back(h->vertex()->x());
            fs.push_back(h->vertex()->y());
            fs.push_back(h->vertex()->z());
        }
        std::sort(fs.begin(), fs.end());
        for (float f : fs) {
            file << f << " ";
        }
        file << "\n";
    }
}
#else

static void DEBUG_DUMP_HALFEDGE(MCGAL::Halfedge* h) {};

static void DEBUG_DUMP_FACE(MCGAL::Facet* f) {};

static void DEBUG_DUMP_VERTEX_ONE_RING(MCGAL::Vertex* v, std::string path) {};

static void DEBUG_DUMP_VERTEX_TWO_RING(MCGAL::Vertex* v, std::string path) {};

static void DEBUG_DUMP_VERTEX_THREE_RING(MCGAL::Vertex* v, std::string path) {};

template <typename Container>
typename std::enable_if<is_container<Container>::value>::type DEBUG_DUMP_CONTAINER(const Container& container, const std::string& filename){};
// template <typename T>
// static void DEBUG_DUMP_CONTAINER(const std::vector<T>& container, const std::string& filename) {};

static void DEBUG_DUMP_PERFORMED_OPERATION(MCGAL::PerformedOperation& op, std::string path){};

static void DEBUG_DUMP_PERFORMED_OPERATION(std::queue<MCGAL::PerformedOperation> opToPerform, std::string path){};

template <typename T> static void DEBU_DUMP_SYMBOL(std::vector<std::deque<T>>& symbol, const std::string& filename){};

static void DEBUG_DUMP_VERTEX_CONTAINER(std::vector<MCGAL::Vertex*> vs, const std::string& filename){};

static void DEBUG_DUMP_HALFEDGES(std::vector<MCGAL::Halfedge*> halfedges, std::string path){};

static void DEBUG_DUMP_FACETS(std::vector<MCGAL::Facet*> facets, std::string path){};
#endif

// ---------------------------------------------------------------------------
// 环境变量门控的符号序列 dump（**Release 亦可用**）。
//
// 背景：上面这套 DEBUG_DUMP_* / DEBU_DUMP_SYMBOL 被 `#ifdef DEBUG` **整段**门控，
// 于是默认（门禁所用的 Release）构建下它们是空实现 —— 想做"编解码两侧对拍"时
// 会发现**根本产不出文件**。本函数补上这条通道，且不改变任何既有行为。
//
// 语义（严格）：
//   · env 未设置 / 为空串 ⇒ **立即返回**：不建文件、不写字节、不碰任何产物。
//   · env 值 = 输出目录；文件写在 `<dir>/<filename>`。
//   · 与算法无关：本函数只认 (环境变量名, 相对文件名)，不含任何算法名。
//
// 调用点约定：只在**默认关闭**的诊断分支里调用，且调用不得影响被测对象
// （即写入内容必须来自"本来就会算出来"的量，不得为 dump 而额外计算）。
template <typename T>
static void DUMP_SYMBOL_IF_ENV(const std::vector<std::deque<T>>& symbol,
                               const char* envName,
                               const std::string& filename) {
    const char* dir = std::getenv(envName);
    if (dir == nullptr || dir[0] == '\0') return;
    std::string path(dir);
    if (!path.empty() && path.back() != '/') path.push_back('/');
    path += filename;
    std::ofstream file(path);
    if (!file) {
        std::cerr << "[DUMP_SYMBOL_IF_ENV] 无法写入: " << path << std::endl;
        return;
    }
    for (const auto& row : symbol) {
        for (const auto& v : row) {
            file << v << ' ';
        }
        file << '\n';
    }
}
