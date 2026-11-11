#include "Mesh.h"
#include "BFSVersionManager.h"
#include "Configuration.h"
#include "Facet.h"
#include "Global.h"
#include "Halfedge.h"
#include "Point.h"
#include "Vertex.h"
#include "VertexSplitNode.h"
#include "biops.h"
#include "string.h"
#include <algorithm>
#include <array>
#include <assert.h>
#include <charconv>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <map>
#include <set>
#include <sstream>
#include <unordered_set>
#include <utility>
#include <vector>

namespace MCGAL {

// #define FACE_COLOR

constexpr std::array<std::array<int, 3>, 200> colors = {{
    // Original colors (deduplicated and kept as-is, ~66 entries)
    {255, 0, 0},      // Red
    {0, 255, 0},      // Green
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
    {255, 140, 105},  // Light Salmon (different shade)
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
    {255, 255, 255},  // White

    // --- Additional 134 colors to reach 200 ---
    {255, 192, 203},  // Pink
    {255, 182, 193},  // Light Pink
    {216, 191, 216},  // Thistle
    {221, 160, 221},  // Plum
    {238, 130, 238},  // Violet (already present, but keep if needed — actually already above, so skip duplicate)
    // Let's avoid duplicates — use new ones below:

    {148, 0, 211},   // Dark Violet
    {72, 61, 139},   // Dark Slate Blue
    {106, 90, 205},  // Slate Blue (already present — skip)
    // Replace with new:

    {123, 104, 238},  // Medium Slate Blue
    {138, 43, 226},   // Blue Violet
    {160, 32, 240},   // Purple (different from {128,0,128})
    {147, 112, 219},  // Medium Purple
    {218, 112, 214},  // Orchid
    {255, 20, 147},   // Deep Pink
    {255, 105, 180},  // Hot Pink (already present — skip)
    // Replace:

    {255, 110, 180},  // New Hot Pink variant
    {255, 69, 0},     // Red Orange
    {255, 140, 0},    // Dark Orange (already present — skip)
    // Replace:

    {230, 120, 0},    // Vivid Orange
    {255, 218, 185},  // Peach Puff
    {255, 222, 173},  // Navajo White
    {255, 228, 181},  // Moccasin (present — skip)
    // Replace:

    {255, 235, 205},  // Papaya Whip
    {253, 245, 230},  // Old Lace
    {250, 240, 230},  // Linen
    {245, 255, 250},  // Mint Cream
    {240, 255, 240},  // Honeydew
    {245, 245, 245},  // White Smoke
    {255, 250, 240},  // Floral White
    {255, 248, 220},  // Cornsilk
    {255, 245, 238},  // Seashell
    {248, 248, 255},  // Ghost White
    {255, 239, 213},  // Papaya Whisper
    {176, 224, 230},  // Powder Blue
    {175, 238, 238},  // Pale Turquoise
    {0, 206, 209},    // Dark Turquoise
    {95, 158, 160},   // Cadet Blue
    {46, 139, 87},    // Sea Green
    {32, 178, 170},   // Light Sea Green (present — skip)
    // Replace:

    {34, 139, 34},   // Forest Green
    {0, 100, 0},     // Dark Green
    {85, 107, 47},   // Dark Olive Green
    {107, 142, 35},  // Olive Drab
    {124, 252, 0},   // Lawn Green
    {127, 255, 0},   // Chartreuse
    {173, 255, 47},  // Green Yellow (present — skip)
    // Replace:

    {50, 205, 50},  // Lime Green (present — skip)
    // Replace:

    {152, 251, 152},  // Pale Green (present — skip)
    // So add new:

    {143, 188, 143},  // Dark Sea Green
    {193, 255, 193},  // Light Green
    {0, 128, 0},      // Green (standard)
    {255, 127, 80},   // Coral (present — skip)
    // Replace:

    {240, 128, 128},  // Light Coral
    {205, 92, 92},    // Indian Red
    {178, 34, 34},    // Fire Brick
    {139, 0, 0},      // Dark Red
    {165, 42, 42},    // Brown
    {210, 180, 140},  // Tan
    {210, 105, 30},   // Chocolate (present — skip)
    // Replace:

    {188, 143, 143},  // Rosy Brown
    {192, 192, 192},  // Silver (present — skip)
    // Replace:

    {128, 128, 128},  // Gray (present — skip)
    // Add:

    {112, 128, 144},  // Slate Gray (present — skip)
    // So add new grays:

    {105, 105, 105},  // Dim Gray
    {169, 169, 169},  // Dark Gray
    {211, 211, 211},  // Light Gray
    {220, 220, 220},  // Gainsboro
    {245, 245, 245},  // White Smoke (already listed — skip duplicate)

    // Blues
    {0, 0, 205},  // Medium Blue
    {0, 0, 139},  // Dark Blue (present — skip)
    // Replace:

    {25, 25, 112},  // Midnight Blue
    {0, 191, 255},  // Deep Sky Blue (present — skip)
    // Replace:

    {135, 206, 250},  // Light Sky Blue (present — skip)
    // Add:

    {176, 226, 255},  // Light Sky Blue (lighter variant)
    {99, 184, 255},   // Dodger Blue
    {30, 144, 255},   // Dodger Blue (standard)
    {100, 149, 237},  // Cornflower Blue
    {65, 105, 225},   // Royal Blue
    {0, 0, 255},      // Blue (present — skip)
    // Add:

    {72, 209, 204},   // Medium Turquoise
    {0, 201, 87},     // Emerald Green
    {255, 204, 0},    // Amber
    {255, 87, 51},    // Red-Orange
    {255, 69, 100},   // Crimson Red
    {156, 39, 176},   // Deep Purple (Material)
    {103, 58, 183},   // Indigo (Material)
    {63, 81, 181},    // Blue (Material)
    {33, 150, 243},   // Light Blue (Material)
    {0, 188, 212},    // Cyan (Material)
    {0, 150, 136},    // Teal (Material)
    {76, 175, 80},    // Green (Material)
    {139, 195, 74},   // Light Green (Material)
    {205, 220, 57},   // Lime (Material)
    {255, 235, 59},   // Yellow (Material)
    {255, 193, 7},    // Amber (Material)
    {255, 152, 0},    // Orange (Material)
    {255, 87, 34},    // Deep Orange (Material)
    {121, 85, 72},    // Brown (Material)
    {158, 158, 158},  // Gray (Material)
    {96, 125, 139},   // Blue Gray (Material)

    // Additional vibrant colors
    {255, 0, 127},  // Rose
    {255, 0, 255},  // Magenta (present — skip)
    // Replace:

    {225, 0, 225},  // Vivid Magenta
    {0, 255, 0},    // Green (present — skip)
    // Add:

    {0, 225, 0},      // Bright Green
    {0, 0, 225},      // Bright Blue
    {225, 225, 0},    // Bright Yellow
    {255, 174, 170},  // Light Red
    {170, 255, 174},  // Light Green
    {170, 174, 255},  // Light Blue
    {255, 174, 255},  // Light Magenta
    {174, 255, 255},  // Light Cyan
    {255, 215, 170},  // Light Orange
    {200, 150, 100},  // Coffee
    {100, 50, 0},     // Espresso
    {255, 240, 245},  // Lavender Blush
    {250, 235, 215},  // Antique White
    {255, 235, 205},  // Blanched Almond variant
    {255, 222, 173},  // Navajo White (already listed — skip)
    // Add unique:

    {255, 183, 117},  // Pastel Orange
    {173, 216, 230},  // Light Blue (present — skip)
    // Replace:

    {135, 206, 235},  // Sky Blue
    {0, 150, 200},    // Ocean Blue
    {100, 200, 150},  // Mint
    {200, 100, 150},  // Raspberry
    {150, 100, 200},  // Lavender
    {200, 150, 100},  // Tan (already listed — skip)
    // Add:

    {255, 200, 200},  // Misty Rose
    {200, 255, 200},  // Honeydew Light
    {200, 200, 255},  // Lavender Mist
    {255, 200, 255},  // Thistle Light
    {200, 255, 255},  // Light Cyan (new)
    {255, 255, 200},  // Light Yellow (new)
    {240, 230, 140},  // Khaki
    {210, 180, 140},  // Tan (present — skip)
    // Add:

    {189, 183, 107},  // Dark Khaki (present — skip)
    // Add:

    {240, 230, 140},  // Khaki (standard)
}};

namespace {

inline void appendInt(std::string& out, int value) {
    char buf[32];
    auto [ptr, ec] = std::to_chars(buf, buf + sizeof(buf), value);
    if (ec == std::errc()) {
        out.append(buf, ptr);
    }
}

inline void appendSize(std::string& out, std::size_t value) {
    char buf[32];
    auto [ptr, ec] = std::to_chars(buf, buf + sizeof(buf), value);
    if (ec == std::errc()) {
        out.append(buf, ptr);
    }
}

inline void appendFloat10(std::string& out, float value) {
    char buf[64];
    auto [ptr, ec] = std::to_chars(buf, buf + sizeof(buf), value,
                                   std::chars_format::general, 10);
    if (ec == std::errc()) {
        out.append(buf, ptr);
    }
}

}  // namespace

Mesh::~Mesh() {}

Facet* Mesh::add_face(std::vector<Vertex*>& vs) {
    Facet* f = MCGAL::contextPool.allocateFaceFromPool(DEFAULT_MESH_ID, vs);
    faces_.push_back(f);
    return f;
}

Facet* Mesh::add_face(Facet* face) {
    faces_.push_back(face);
    return face;
}

Vertex* Mesh::add_vertex(Vertex* vertex) {
    vertices_.push_back(vertex);
    return vertex;
}

void Mesh::eraseFacetByPointer(Facet* facet) {
    for (auto it = faces_.begin(); it != faces_.end();) {
        if ((*it) == facet) {
            faces_.erase(it);
            break;
        } else {
            it++;
        }
    }
}

void Mesh::eraseVertexByPointer(Vertex* vertex) {
    for (auto it = vertices_.begin(); it != vertices_.end();) {
        if ((*it) == vertex) {
            vertices_.erase(it);
            break;
        } else {
            it++;
        }
    }
}

void Mesh::resetState() {
    for (MCGAL::Vertex*& vertex : vertices_) {
        vertex->resetState();
    }
    for (MCGAL::Facet*& face : faces_) {
        if (face->isRemoved()) {
            continue;
        }
        face->resetState();
        MCGAL::Halfedge* st = face->proxyHalfedge();
        MCGAL::Halfedge* ed = st;
        while (st) {
            st->resetState();
            if (!st->next()) break;
            st = st->next();
            if (st == ed) break;
        }
    }
    // MCGAL::contextPool.halfedgeResetState();
}

void Mesh::invalidateAllVertexDegrees() {
    for (MCGAL::Vertex*& vertex : vertices_) {
        if (!vertex->isRemoved()) {
            vertex->invalidate_degree();
        }
    }
}

void Mesh::recomputeAllVertexDegrees() {
    for (MCGAL::Vertex*& vertex : vertices_) {
        if (!vertex->isRemoved()) {
            vertex->recompute_degree();
        }
    }
}

// 计算边坍缩代价（可以使用各种度量标准）
float calculateEdgeCollapseCost(Halfedge* edge) {
    // 这里使用简单的边长作为代价
    return edge->length();
}

#ifdef USE_DEPRECATED
// 需要确定方向
MCGAL::Vertex* Mesh::halfedge_collapse(MCGAL::Halfedge* h, MCGAL::Point newp) {
    static int order = 0;
    MCGAL::Vertex* v1 = h->vertex();
    MCGAL::Vertex* v2 = h->end_vertex();
    MCGAL::Point c = v1->point();
    MCGAL::Point d = v2->point();
    std::set<MCGAL::Vertex*> belongv2;
    for (MCGAL::Halfedge* hit : v2->halfedges()) {
        belongv2.insert(hit->end_vertex());
    }

#    ifdef DEBUG
    if (v1->isRemoved() || v2->isRemoved()) {
        std::cout << "error" << std::endl;
    }
#    endif
    v1->setPoint(newp);

    MCGAL::Vertex* banned1 = nullptr;
    MCGAL::Vertex* banned2 = nullptr;

    v2->setRemoved();

    if (h->face()->facet_degree() == 3) {
        Halfedge* h1 = h->next();
        banned1 = h1->end_vertex();
        Halfedge* h2 = find_prev(h);
        MCGAL::Halfedge* oh1 = h1->opposite();
        MCGAL::Halfedge* oh2 = h2->opposite();
        oh1->setOpposite(oh2);
        oh2->setOpposite(oh1);

        // banned1 = h->next()->end_vertex();

        h->face()->setRemoved();
        MCGAL::Halfedge* st = h1->face()->proxyHalfedge();
        MCGAL::Halfedge* ed = h1->face()->proxyHalfedge();
        do {
            // st->vertex()->eraseHalfedgeByPointer(st);
            if (st->vertex() == v2) {
                st->opposite()->setEndVertex(v1);
            }
            st->setRemoved();
            st = st->next();
        } while (st != ed);

    } else if (h->face()->facet_degree() > 3) {
        Halfedge* h1 = h->next();
        Halfedge* h2 = find_prev(h);
        h2->setNext(h1);
        h2->face()->reset(h2);
    }
    // 获得one ring
    MCGAL::Halfedge* around = h->next()->opposite()->next()->next();

    h = h->opposite();

    if (h->face()->facet_degree() == 3) {
        Halfedge* h1 = h->next();
        banned2 = h1->end_vertex();
        Halfedge* h2 = find_prev(h);
        MCGAL::Halfedge* oh1 = h1->opposite();
        MCGAL::Halfedge* oh2 = h2->opposite();
        oh1->setOpposite(oh2);
        oh2->setOpposite(oh1);

        // banned2 = h->next()->end_vertex();

        h->face()->setRemoved();
        MCGAL::Halfedge* st = h1->face()->proxyHalfedge();
        MCGAL::Halfedge* ed = h1->face()->proxyHalfedge();
        do {
            // st->vertex()->eraseHalfedgeByPointer(st);
            if (st->vertex() == v2) {
                st->opposite()->setEndVertex(v1);
            }
            st->setRemoved();
            st = st->next();
        } while (st != ed);

    } else if (h->face()->facet_degree() > 3) {
        Halfedge* h1 = h->next();
        Halfedge* h2 = find_prev(h);
        h2->setNext(h1);
        h2->face()->reset(h2);
    }

    for (MCGAL::Halfedge* hit : v2->halfedges()) {
        // hit->end_vertex()->eraseHalfedgeByPointer(hit->opposite());
        hit->setVertex(v1);
        hit->opposite()->setEndVertex(v1);
        // if (hit != h->opposite() && hit != h->next()) {
        v1->halfedges().push_back(hit);
        // }
    }
    MCGAL::Halfedge* st = around;
    MCGAL::Halfedge* ed = around;
    MCGAL::Halfedge* minEdge = st;
    do {
        if (st->vertex()->point() < minEdge->vertex()->point()) {
            minEdge = st;
        }
        st = st->next()->opposite()->next();
    } while (st != ed);
    uint16_t bitmap = 0;
    // memset(bitmap, 0, 2);
    st = minEdge;
    ed = st;
    int idx = 0;
    int bannedIdx1 = -1;
    int bannedIdx2 = -1;
    int flag = 0;
    // 需要一个人bit来存储先banned1还是banned2
    // 记录一下是从1->2还是从2->1
    do {
        if (st->vertex() == banned1) {
            bannedIdx1 = idx;
            setBit(reinterpret_cast<char*>(&bitmap), idx);
        } else if (st->vertex() == banned2) {
            bannedIdx2 = idx;
            setBit(reinterpret_cast<char*>(&bitmap), idx);
        }
        if (bannedIdx1 != -1 && bannedIdx2 != -1) {
            break;
        }
        // 说明是从1->2
        if (flag == 0 && bannedIdx1 != -1) {
            if (belongv2.count(st->end_vertex())) {
                flag = 1;
            } else {
                flag = 3;
            }
        }
        // 说明是从2->1
        if (flag == 0 && bannedIdx2 != -1) {
            if (belongv2.count(st->end_vertex())) {
                flag = 2;
            } else {
                flag = 3;
            }
        }
        idx++;
        st = st->next()->opposite()->next();
    } while (st != ed);
    // 说明是从小到大
    if (flag == 1 && bannedIdx1 < bannedIdx2) {
        // if(banned1.next()->vertex() == banned2)
        setBit(reinterpret_cast<char*>(&bitmap), 15);
    }
    if (flag == 2 && bannedIdx2 < bannedIdx1) {
        setBit(reinterpret_cast<char*>(&bitmap), 15);
    }

    // 需要把自己的孩子也都记下来
    VertexSplitNode* vsn = new VertexSplitNode();
    vsn->order = order++;
    // vsn->c = c;
    // vsn->d = d;
    vsn->c = c;
    vsn->d = d;
    if (v1->isCollapsed()) {
        // setBit(bitmap, 13);
        vsn->left = v1->vsplitNode();
    }
    if (v2->isCollapsed()) {
        // setBit(bitmap, 14);
        vsn->right = v2->vsplitNode();
    }
    v1->setCollapsed();
    vsn->bitmap = bitmap;
    v1->setVertexSplitNode(vsn);
    if (v2->isBoundary()) {
        v1->setBoundary();
    }
    return v1;
}

MCGAL::Halfedge* Mesh::vertex_split(MCGAL::Vertex* v, MCGAL::VertexSplitNode* node) {
    // 获取到one ring的halfedge
    MCGAL::Halfedge* st = v->halfedges()[0]->next();
    MCGAL::Halfedge* ed = st;
    MCGAL::Halfedge* minEdge = st;
    do {
        if (st->vertex()->point() < minEdge->vertex()->point()) {
            minEdge = st;
        }
        st = st->next()->opposite()->next();
    } while (st != ed);
    char* bitmap = reinterpret_cast<char*>(&node->bitmap);
    bool ordered = getBit(bitmap, 15);
    bool isFirst = true;
    st = minEdge;
    ed = st;
    int idx = 0;
    MCGAL::Halfedge* banned1 = nullptr;
    MCGAL::Halfedge* banned2 = nullptr;
    // 需要一个bit来存储先banned1还是banned2
    do {
        if (getBit(bitmap, idx)) {
            if (isFirst && ordered) {
                banned1 = st;
                isFirst = false;
            } else if (!isFirst && ordered) {
                banned2 = st;
            }
            if (isFirst && !ordered) {
                banned2 = st;
                isFirst = false;
            } else if (!isFirst && !ordered) {
                banned1 = st;
            }
            // if (isBanned1) {
            //     banned1 = st;
            //     isBanned1 = false;
            // } else {
            //     banned2 = st;
            //     isBanned1 = true;
            // }
        }
        idx++;
        st = st->next()->opposite()->next();
    } while (st != ed);
    if (v->poolId() == 8869 || banned1->vertex()->poolId() == 8869 || banned2->vertex()->poolId() == 8869) {
        int i = 0;
    }
    // MCGAL::Halfedge* op = banned1->next()->next()->opposite();
    // op->setOpposite(nullptr);
    // banned1->next()->next()->setOpposite(nullptr);
    // op = banned2->next()->next()->opposite();
    // op->setOpposite(nullptr);
    // banned2->next()->next()->setOpposite(nullptr);

    MCGAL::Point c = node->c;
    MCGAL::Point d = node->d;
    // MCGAL::Point v1p = c - d / 2;
    // MCGAL::Point v2p = c + d / 2;
    MCGAL::Point v1p = c;
    MCGAL::Point v2p = d;

    v->setPoint(v1p);
    MCGAL::Vertex* v2 = MCGAL::contextPool.allocateVertexFromPool(meshId_, v2p);
    // if (node->order == 4333) {
    //     std::swap(banned1, banned2);
    // }

    st = banned1;
    ed = banned2;

    do {
        MCGAL::Halfedge* hit1 = st->next()->next();
        MCGAL::Halfedge* hit2 = st->next()->next()->opposite();
        MCGAL::Halfedge* next = st->next()->opposite()->next();
        if (st->vertex() == banned1->vertex() || st->vertex() == banned2->vertex()) {
            MCGAL::Halfedge* op = hit1->opposite();
            op->setOpposite(nullptr);
            hit1->setOpposite(nullptr);
        }
        // if (next == ed) {
        //     st->next()->setEndVertex(v2);
        //     st = next;
        //     break;
        // }
        // hit1->vertex()->eraseHalfedgeByPointer(hit1);
        hit1->setVertex(v2);
        v2->halfedges().push_back(hit1);
        if (st->vertex() != banned1->vertex()) {
            hit2->setEndVertex(v2);
        }
        st = next;
    } while (st != ed);
    st->next()->next()->opposite()->setEndVertex(v2);
    if (st->vertex() == banned2->vertex()) {
        MCGAL::Halfedge* op = st->next()->next()->opposite();
        op->setOpposite(nullptr);
        st->next()->next()->setOpposite(nullptr);
    }
    // for (MCGAL::Halfedge* hit : v->halfedges()) {
    //     if (hit->end_vertex() == banned1->vertex() || hit->end_vertex() == banned2->vertex()) {
    //         MCGAL::Halfedge* op = hit->opposite();
    //         op->setOpposite(nullptr);
    //         hit->setOpposite(nullptr);
    //     }
    // }
    MCGAL::Facet* fit1 = MCGAL::contextPool.allocateFaceFromPool(meshId_, {banned1->vertex(), v2, v});
    MCGAL::Facet* fit2 = MCGAL::contextPool.allocateFaceFromPool(meshId_, {banned2->vertex(), v, v2});

    if (node->left) {
        v->setVertexSplitNode(node->left);
    } else {
        v->setVertexSplitNode(nullptr);
    }
    if (node->right) {
        v2->setVertexSplitNode(node->right);
    }
    vertices_.push_back(v2);
    faces_.push_back(fit1);
    faces_.push_back(fit2);
    MCGAL::Halfedge* res = nullptr;
    for (MCGAL::Halfedge* hit : v->halfedges()) {
        if (hit->end_vertex() == v2) {
            return hit;
        }
    }

    return nullptr;
}

bool Mesh::IsFlipped(MCGAL::Halfedge* edge, const MCGAL::Point& ptTarget) {
    MCGAL::Vertex* v0 = edge->vertex();
    MCGAL::Vertex* v1 = edge->end_vertex();
    // // 检查坍缩后是否会导致重叠面片或非流形网格
    // // 坍缩后确保其他点的邻居数不能少于三个

    for (MCGAL::Halfedge* h : v0->halfedges()) {
        if (h->face()->isRemoved()) {
            continue;
        }
        std::vector<MCGAL::Vertex*> vs;
        MCGAL::Halfedge* st = h->face()->proxyHalfedge();
        MCGAL::Halfedge* ed = st;
        do {
            vs.push_back(st->vertex());
            st = st->next();
        } while (st != ed);
        if (vs[0] == v1 || vs[1] == v1 || vs[2] == v1) {
            continue;
        }
        int idxV0 = 0;
        for (int i = 0; i < vs.size(); i++) {
            MCGAL::Vertex* v = vs[i];
            if (vs[i] == v0) {
                idxV0 = i;
            }
        }
        MCGAL::Point pt0 = v0->point();
        MCGAL::Point pt1 = vs[(idxV0 + 1) % 3]->point();
        MCGAL::Point pt2 = vs[(idxV0 + 2) % 3]->point();

        MCGAL::Point dir1 = pt1 - ptTarget;
        dir1.normalize();
        MCGAL::Point dir2 = pt2 - ptTarget;
        dir2.normalize();

        // The angle below can be adjusted, but now is enough
        // if the angle between dir1 and dir2 small than 2.6 angle, return true
        if (fabs(dir1.dot(dir2)) > 0.999)
            return true;
        MCGAL::Point normold;
        try {
            normold = h->face()->computeNormal();
        } catch (std::string exp) { return true; }
        MCGAL::Point norm = dir1.cross(dir2);
        norm.normalize();

        // if the angle between normold and norm large than 78.5 angle, return true
        if ((normold.dot(norm)) < 0.2)
            return true;
    }
    return false;
}

bool Mesh::is_collapse_ok(MCGAL::Halfedge* v0v1) {
    // 检查边是否已删除
    if (v0v1->isRemoved()) {
        return false;
    }

    MCGAL::Halfedge* v1v0 = v0v1->opposite();
    MCGAL::Vertex* v0 = v1v0->vertex();
    MCGAL::Vertex* v1 = v0v1->vertex();
    if (v1->halfedges().size() + v0->halfedges().size() >= 16) {
        return false;
    }

    bool v0v1_triangle = false;
    bool v1v0_triangle = false;

    // 检查是否为三角形面
    if (!v0v1->isBoundary()) {
        v0v1_triangle = v0v1->face()->facet_degree() == 3;
    }
    if (!v1v0->isBoundary()) {
        v1v0_triangle = v1v0->face()->facet_degree() == 3;
    }

    // 获取下一个顶点
    MCGAL::Vertex* v_01_n = v0v1->next()->end_vertex();
    MCGAL::Vertex* v_10_n = v1v0->next()->end_vertex();
    if (v_01_n->halfedges().size() <= 3 || v_10_n->halfedges().size() <= 3) {
        return false;
    }

    // 检查顶点是否已删除
    if (v0->isRemoved() || v1->isRemoved()) {
        return false;
    }

    // 检查三角形面的边界情况
    MCGAL::Vertex* vl = nullptr;
    if (!v0v1->isBoundary()) {
        if (v0v1_triangle) {
            MCGAL::Halfedge* h1 = v0v1->next();
            MCGAL::Halfedge* h2 = h1->next();

            vl = h1->vertex();
            if (h1->opposite()->isBoundary() && h2->opposite()->isBoundary()) {
                return false;
            }
        }
    }

    MCGAL::Vertex* vr = nullptr;
    if (!v1v0->isBoundary()) {
        if (v1v0_triangle) {
            MCGAL::Halfedge* h1 = v1v0->next();
            MCGAL::Halfedge* h2 = h1->next();

            vr = h1->vertex();
            if (h1->opposite()->isBoundary() && h2->opposite()->isBoundary()) {
                return false;
            }
        }
    }

    // 检查 vl 和 vr 是否相同
    if (vl && vr && vl == vr) {
        return false;
    }

    // 检查边界顶点
    // if (v0->isBoundary() && v1->isBoundary() && !v0v1->isBoundary() && !v1v0->isBoundary()) {
    //     return false;
    // }

    // 检查一环邻域交集
    std::set<MCGAL::Vertex*> v0_neighbors;
    std::set<MCGAL::Vertex*> v1_neighbors;

    // 收集 v0 的邻居
    for (MCGAL::Halfedge* hit : v0->halfedges()) {
        if (!hit->end_vertex()->isRemoved() && hit->face() != v0v1->face() && hit->face() != v1v0->face()) {
            v0_neighbors.insert(hit->end_vertex());
        }
    }

    // 收集 v1 的邻居
    for (MCGAL::Halfedge* hit : v1->halfedges()) {
        if (!hit->end_vertex()->isRemoved() && hit->face() != v0v1->face() && hit->face() != v1v0->face()) {
            v1_neighbors.insert(hit->end_vertex());
        }
    }

    // 检查交集
    std::set<MCGAL::Vertex*> cneighbor;
    for (MCGAL::Vertex* neighbor : v0_neighbors) {
        if (v1_neighbors.count(neighbor) > 0) {
            // 排除特殊三角形情况
            if (!(neighbor == v_01_n && v0v1_triangle) && !(neighbor == v_10_n && v1v0_triangle)) {
                return false;
            }
            cneighbor.insert(neighbor);
        }
    }
    MCGAL::Vertex* cv1 = v0v1->next()->end_vertex();
    MCGAL::Vertex* cv2 = v0v1->opposite()->next()->end_vertex();
    for (MCGAL::Halfedge* hit : cv1->halfedges()) {
        if (hit->end_vertex() == cv2) {
            return false;
        }
    }

    // for (MCGAL::Vertex* neighbor : cneighbor) {
    //     for (MCGAL::Halfedge* hit : neighbor->halfedges()) {
    //         if (cneighbor.count(hit->end_vertex())) {
    //             return false;
    //         }
    //     }
    // }

    // 检查三角形面的退化情况
    if (v0v1_triangle) {
        MCGAL::Halfedge* one = v0v1->next();
        MCGAL::Halfedge* two = one->next();

        one = one->opposite();
        two = two->opposite();

        if (one->face() == two->face() && one->face()->facet_degree() != 3) {
            return false;
        }
    }

    if (v1v0_triangle) {
        MCGAL::Halfedge* one = v1v0->next();
        MCGAL::Halfedge* two = one->next();

        one = one->opposite();
        two = two->opposite();

        if (one->face() == two->face() && one->face()->facet_degree() != 3) {
            return false;
        }
    }

    // 额外的特殊情况检查
    if (v_01_n == v_10_n && v0v1_triangle && v1v0_triangle) {
        return false;
    }

    // 通过所有检查
    return true;
}

MCGAL::Vertex* Mesh::halfedge_collapse(MCGAL::Halfedge* h) {
    MCGAL::Vertex* v1 = h->vertex();
    MCGAL::Vertex* v2 = h->end_vertex();
    v1->setPoint({(v1->point().x() + v2->point().x()) / 2, (v1->point().y() + v2->point().y()) / 2, (v1->point().z() + v2->point().z()) / 2});
    v1->setProcessed();
    std::set<Vertex*> vset;

    // for (MCGAL::Halfedge* hit : v1->halfedges()) {
    //     hit->face()->reset_without_init(hit);
    // }

    v2->setRemoved();
    // v2->halfedges().clear();

    if (h->face()->facet_degree() == 3) {
        Halfedge* h1 = h->next();
        Halfedge* h2 = find_prev(h);
        MCGAL::Halfedge* oh1 = h1->opposite();
        MCGAL::Halfedge* oh2 = h2->opposite();
        oh1->setOpposite(oh2);
        oh2->setOpposite(oh1);

        h->face()->setRemoved();
        MCGAL::Halfedge* st = h1->face()->proxyHalfedge();
        MCGAL::Halfedge* ed = h1->face()->proxyHalfedge();
        do {
            // st->vertex()->eraseHalfedgeByPointer(st);
            st->setRemoved();
            st = st->next();
        } while (st != ed);

    } else if (h->face()->facet_degree() > 3) {
        Halfedge* h1 = h->next();
        Halfedge* h2 = find_prev(h);
        h2->setNext(h1);
        h2->face()->reset(h2);
    }
    h = h->opposite();

    if (h->face()->facet_degree() == 3) {
        Halfedge* h1 = h->next();
        Halfedge* h2 = find_prev(h);
        MCGAL::Halfedge* oh1 = h1->opposite();
        MCGAL::Halfedge* oh2 = h2->opposite();
        oh1->setOpposite(oh2);
        oh2->setOpposite(oh1);

        h->face()->setRemoved();
        MCGAL::Halfedge* st = h1->face()->proxyHalfedge();
        MCGAL::Halfedge* ed = h1->face()->proxyHalfedge();
        do {
            // st->vertex()->eraseHalfedgeByPointer(st);
            st->setRemoved();
            st = st->next();
        } while (st != ed);

    } else if (h->face()->facet_degree() > 3) {
        Halfedge* h1 = h->next();
        Halfedge* h2 = find_prev(h);
        h2->setNext(h1);
        h2->face()->reset(h2);
    }

    for (MCGAL::Halfedge* hit : v2->halfedges()) {
        // hit->end_vertex()->eraseHalfedgeByPointer(hit->opposite());
        hit->setVertex(v1);
        hit->opposite()->setEndVertex(v1);
        if (hit != h->opposite() && hit != h->next()) {
            v1->halfedges().push_back(hit);
        }
    }
    return v1;
}
#endif

bool Mesh::new_is_collapse_ok(Halfedge* v0v1) const {
    Halfedge* v1v0 = v0v1->opposite();
    Vertex* v0 = v1v0->vertex();
    Vertex* v1 = v0v1->vertex();
    Vertex* vl = v0v1->next()->end_vertex();
    Vertex* vr = v1v0->next()->end_vertex();

    if (vl == vr)
        return false;

    for (Halfedge* hit : *v0) {
        // OpenMesh link condition: a neighbour of v0 other than vl/vr must not
        // be shared with v1, otherwise the collapse creates a non-manifold link.
        if (hit->end_vertex() != vl && hit->end_vertex() != vr && v1->isConnected(hit->end_vertex()))
            return false;
    }
    return true;
}

void Mesh::new_collapse(MCGAL::Halfedge* h) {
    MCGAL::Halfedge* hn = h->next();
    MCGAL::Halfedge* hp = h->find_prev();
    MCGAL::Halfedge* hpo = hp->opposite();

    MCGAL::Halfedge* o = h->opposite();
    MCGAL::Halfedge* on = o->next();
    MCGAL::Halfedge* op = o->find_prev();
    MCGAL::Halfedge* ono = on->opposite();

    MCGAL::Facet* fh = h->face();
    MCGAL::Facet* fo = o->face();

    MCGAL::Vertex* vh = h->end_vertex();
    MCGAL::Vertex* vo = o->end_vertex();

    MCGAL::Vertex* vl = hn->end_vertex();
    MCGAL::Vertex* vr = on->end_vertex();

    // halfedge -> vertex
    for (MCGAL::Halfedge* hit : *vo) {
        hit->opposite()->setEndVertex(vh);
        hit->setVertex(vh);
    }

    // vertex -> halfedge
    if (vh->proxyHalfedge() == o) {
        vh->setProxyHalfedge(hn);
    }
    if (vl->proxyHalfedge() == hp) {
        vl->setProxyHalfedge(hpo->next());
    }

    if (vr->proxyHalfedge() == ono) {
        vr->setProxyHalfedge(op);
    }

    // halfedge -> next halfedge
    hn->setNext(hpo->next());
    hn->next()->next()->setNext(hn);

    op->setNext(ono->next());
    op->next()->next()->setNext(op);

    // face -> halfedge
    hpo->face()->reset(hn);
    ono->face()->reset(op);

    // halfedge -> face
    // hn->setFace(hpo->face());
    // op->setFace(ono->face());

    // delete stuff
    h->setRemoved();
    h->opposite()->setRemoved();
    on->setRemoved();
    on->opposite()->setRemoved();
    hp->setRemoved();
    hp->opposite()->setRemoved();
    vo->setRemoved();
    fh->setRemoved();
    fo->setRemoved();

    // status(edge_handle(h)).set_deleted(true);
    // status(edge_handle(on)).set_deleted(true);
    // status(edge_handle(hp)).set_deleted(true);
    // status(vo).set_deleted(true);
    // status(fh).set_deleted(true);
    // status(fo).set_deleted(true);
}

void adjust_outgoing_halfedge(Vertex* _vh) {
    for (Halfedge* hit : *_vh) {
        if (hit->face() != nullptr) {
            _vh->setProxyHalfedge(hit);
            break;
        }
    }
}

Halfedge* Mesh::insert_loop(Halfedge* _hh) {
    Halfedge* h0(_hh);
    Halfedge* o0(h0->opposite());

    Vertex* v0(o0->end_vertex());
    Vertex* v1(h0->end_vertex());

    Halfedge* h1 = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(meshId_, v1, v0);
    Halfedge* o1 = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(meshId_, v0, v1);
    h1->setOpposite(o1);
    o1->setOpposite(h1);
    Facet* f0 = h0->face();
    Facet* f1 = MCGAL::contextPool.allocateFaceFromPool(meshId_);
    faces_.push_back(f1);

    // halfedge -> halfedge
    h0->find_prev()->setNext(o1);
    o1->setNext(h0->next());

    h1->setNext(h0);
    h0->setNext(h1);

    // halfedge -> face

    o1->setFace(f0);
    h0->setFace(f1);
    h1->setFace(f1);

    // face -> halfedge
    f1->reset(h0);
    if (f0 != nullptr)
        f0->reset(o1);

    // vertex -> halfedge
    adjust_outgoing_halfedge(v0);
    adjust_outgoing_halfedge(v1);

    return h1;
}

Halfedge* Mesh::insert_edge(Vertex* _vh, Halfedge* _h0, Halfedge* _h1) {
    assert(_h0 != nullptr && _h1 != nullptr);

    Vertex* v0 = _vh;
    Vertex* v1 = _h0->end_vertex();

    assert(v1 == _h1->end_vertex());

    Halfedge* v0v1 = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(meshId_, v0, v1);
    Halfedge* v1v0 = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(meshId_, v1, v0);
    v0v1->setOpposite(v1v0);
    v1v0->setOpposite(v0v1);

    // vertex -> halfedge
    v0->setProxyHalfedge(v0v1);
    v1->setProxyHalfedge(v1v0);

    // halfedge -> halfedge
    v0v1->setNext(_h0->next());
    _h0->setNext(v0v1);
    v1v0->setNext(_h1->next());
    _h1->setNext(v1v0);

    // halfedge -> vertex
    for (Halfedge* hit : *v0) {
        hit->setVertex(v0);
        hit->opposite()->setEndVertex(v0);
    }

    // halfedge -> face
    v0v1->setFace(_h0->face());
    v1v0->setFace(_h1->face());

    // face -> halfedge
    if (v0v1->face() != nullptr)
        v0v1->face()->reset(v0v1);
    if (v1v0->face() != nullptr)
        v1v0->face()->reset(v1v0);

    // vertex -> halfedge
    adjust_outgoing_halfedge(v0);
    adjust_outgoing_halfedge(v1);

    return v0v1;
}

void Mesh::edge_flip(MCGAL::Halfedge* hit) {
    if (hit->poolId() == 208976) {
        int i = 0;
    }
    MCGAL::Facet* f1 = hit->face();
    MCGAL::Facet* f2 = hit->opposite()->face();
    MCGAL::Vertex* v1 = hit->next()->end_vertex();
    MCGAL::Vertex* v2 = hit->opposite()->next()->end_vertex();
    MCGAL::Halfedge* ohit = hit->opposite();
    MCGAL::Halfedge* h1 = hit->next()->next();
    MCGAL::Halfedge* h2 = hit->opposite()->next();

    MCGAL::Halfedge* h3 = hit->next();
    MCGAL::Halfedge* h4 = ohit->next()->next();
    if (hit->vertex()->proxyHalfedge() == hit) {
        assert(hit->vertex() == h2->vertex());
        hit->vertex()->setProxyHalfedge(h2);
    }
    if (hit->end_vertex()->proxyHalfedge() == ohit) {
        assert(hit->end_vertex() == h3->vertex());
        hit->end_vertex()->setProxyHalfedge(h3);
    }

    h1->setNext(h2);
    h2->setNext(hit);
    hit->setVertex(v2);
    hit->setEndVertex(v1);
    hit->setNext(h1);

    h3->setNext(ohit);
    ohit->setNext(h4);
    h4->setNext(h3);

    ohit->setEndVertex(v2);
    ohit->setVertex(v1);

    f1->reset(h1);
    f2->reset(h3);
}

Halfedge* Mesh::new_vertex_split(MCGAL::Vertex* v0, MCGAL::Vertex* v1, MCGAL::Vertex* vl, MCGAL::Vertex* vr) {
    Halfedge *v1vl, *vlv1, *vrv1, *v0v1;

    // build loop from halfedge v1->vl
    // if (!vl->isRemoved()) {
    v1vl = v1->find_halfedge(vl);
    assert(!v1vl->isRemoved());
    vlv1 = insert_loop(v1vl);
    // }

    // build loop from halfedge vr->v1
    // if (!vr->isRemoved()) {
    vrv1 = vr->find_halfedge(v1);
    assert(!vrv1->isRemoved());
    insert_loop(vrv1);
    // }

    // handle boundary cases
    // TODO: boundary case
    // if (!vl.is_valid())
    //     vlv1 = prev_halfedge_handle(halfedge_handle(v1));
    // if (!vr.is_valid())
    //     vrv1 = prev_halfedge_handle(halfedge_handle(v1));

    // split vertex v1 into edge v0v1
    v0v1 = insert_edge(v0, vlv1, vrv1);

    return v0v1;
}

/**
 * 半边折叠的逆操作
 * @param v 需要被split的点
 * @param p 需要被新插入的边
 * @param conn 需要被连接到新点的边
 * @param vid1 vid2
 */
// MCGAL::Halfedge* Mesh::vertex_split(MCGAL::Vertex* v, MCGAL::Point p, std::vector<Halfedge*> conn, int vid1, int vid2) {
//     // TODO: 完善该方法
//     MCGAL::Vertex* vnew = MCGAL::contextPool.tryAllocVertexFromPool(p);
//     // MCGAL::Halfedge* newh = MCGAL::contextPool.allocateHalfedgeFromPool(v, vnew);
//     MCGAL::Halfedge* split1 = nullptr;
//     MCGAL::Halfedge* split2 = nullptr;
//     std::set<int> poolIds;
//     // 某个点，谁两边的边 groupId不一样谁就被分割
//     // conn里只处理groupId属于该半边的
//     /**
//      * 四种case，每种都要单独处理
//      * 最好是分的一人一边，然后直接创建新的face
//      */
//     std::vector<MCGAL::Vertex*> vts;
//     for (MCGAL::Halfedge* hit : conn) {
//         // 分割边被分割后仍属于同一个group
//         // 需要维护opposite
//         if (hit->end_vertex->id == vid1 && hit->opposite != nullptr) {
//             hit->vertex->eraseHalfedgeByPointer(hit);
//             hit->vertex = vnew;
//             vnew->halfedges.push_back(hit);
//             poolIds.insert(hit->end_vertex->poolId);
//             vts.push_back(hit->end_vertex);
//             vts.push_back(vnew);
//             vts.push_back(v);
//         }
//         // 边界情况
//         if (hit->end_vertex->id == vid1 && hit->opposite == nullptr) {
//             hit->vertex = vnew;
//             poolIds.insert(hit->end_vertex->poolId);
//         }
//         // 该group只有end_vertex指向自己的边
//         if (hit->vertex->id == vid1) {
//             vnew->halfedges.push_back(hit);
//             vts.push_back(vnew);
//             vts.push_back(hit->vertex);
//             vts.push_back(v);
//         }
//         if (hit->vertex == v) {
//             hit->vertex = vnew;
//         }
//         if (hit->opposite != nullptr && hit->opposite->face->groupId == hit->face->groupId) {
//             hit->opposite->end_vertex = vnew;
//         }
//     }
//     // 删除v中不属于他的边
//     for (auto it = v->halfedges.begin(); it != v->halfedges.end();) {
//         if (poolIds.count((*it)->poolId)) {
//             vnew->halfedges.push_back(*it);
//             it = v->halfedges.erase(it);
//         } else {
//             it++;
//         }
//     }
//     return nullptr;
// }

Halfedge* Mesh::split_facet_non_meshId(Halfedge* h, Halfedge* g) {
    Facet* origin = h->face();
    // early expose
    Facet* fnew = MCGAL::contextPool.allocateFaceFromPool(meshId_);
    // create new halfedge
    Halfedge* hnew = MCGAL::contextPool.allocateHalfedgeFromPool(meshId_, h->end_vertex(), g->end_vertex());
    Halfedge* oppo_hnew = MCGAL::contextPool.allocateHalfedgeFromPool(meshId_, g->end_vertex(), h->end_vertex());
    // set the opposite
    // set the connect information

    hnew->setNext(g->next());
    oppo_hnew->setNext(h->next());

    h->setNext(hnew);
    g->setNext(oppo_hnew);
    // create new face depend on vertexs
    origin->reset(hnew);
    fnew->reset(oppo_hnew);
    fnew->setProcessedFlag(origin->processedFlag());
    fnew->setGroupId(origin->groupId());
    // fnew->removedFlag = origin->removedFlag;
    // add halfedge and face to mesh
    this->faces_.push_back(fnew);
    return hnew;
}

Halfedge* Mesh::split_facet(Halfedge* h, Halfedge* g) {
    Facet* origin = h->face();
    // early expose
    Facet* fnew = MCGAL::contextPool.allocateFaceFromPool(meshId_);
    // create new halfedge
    Halfedge* hnew = MCGAL::contextPool.allocateHalfedgeFromPool(meshId_, h->end_vertex(), g->end_vertex());
    Halfedge* oppo_hnew = MCGAL::contextPool.allocateHalfedgeFromPool(meshId_, g->end_vertex(), h->end_vertex());
    // set the opposite
    // set the connect information

    hnew->setNext(g->next());
    oppo_hnew->setNext(h->next());

    h->setNext(hnew);
    g->setNext(oppo_hnew);
    // create new face depend on vertexs
    origin->reset(hnew);
    fnew->reset(oppo_hnew);
    fnew->setProcessedFlag(origin->processedFlag());
    fnew->setGroupId(origin->groupId());
    // fnew->removedFlag = origin->removedFlag;
    // add halfedge and face to mesh
    this->faces_.push_back(fnew);
    return hnew;
}

// Halfedge* Mesh::erase_center_vertex(Halfedge* h) {
//     Halfedge* g = h->next()->opposite();
//     Halfedge* gstart = g;
//     Halfedge* hret = find_prev(h);
//     while (g != h) {
//         Halfedge* gprev = find_prev(g);
//         remove_tip(gprev);
//         if (g->face() != h->face()) {
//             // eraseFacetByPointer(g->facet());
//             g->face()->setRemoved();
//         }
//         Halfedge* gnext = g->next()->opposite();
//         // g->vertex()->eraseHalfedgeByPointer(g);
//         // g->end_vertex->eraseHalfedgeByPointer(g->opposite);
//         g->setRemoved();
//         g->opposite()->setRemoved();
//         if(gprev->vertex()->proxyHalfedge() == gnext) {
//             gprev->vertex()->setProxyHalfedge(gprev);
//         }
//         g = gnext;
//     }
//     h->setRemoved();
//     h->opposite()->setRemoved();
//     if(hret->vertex()->proxyHalfedge() == gstart) {
//         hret->vertex()->setProxyHalfedge(hret);
//     }
//     remove_tip(hret);
//     // h->vertex()->eraseHalfedgeByPointer(h);
//     // h->end_vertex()->halfedges().clear();
//     h->end_vertex()->setRemoved();
//     // eraseVertexByPointer(h->end_vertex);
//     h->face()->reset(hret);
//     return hret;
// }

Halfedge* Mesh::erase_center_vertex(Halfedge* h) {
    Halfedge* g = h->next()->opposite();
    Halfedge* gstart = g;
    Halfedge* hret = find_prev(h);
    while (g != h) {
        Halfedge* gprev = find_prev(g);
        remove_tip(gprev);
        if (g->face() != h->face()) {
            g->face()->setRemoved();
        }
        Halfedge* gnext = g->next()->opposite();
        g->setRemoved();
        g->opposite()->setRemoved();
        // if (gprev->vertex()->proxyHalfedge() == gnext) {
        gprev->vertex()->setProxyHalfedge(gprev);
        // }
        g = gnext;
    }
    h->setRemoved();
    h->opposite()->setRemoved();
    // if (hret->vertex()->proxyHalfedge() == gstart) {
    hret->vertex()->setProxyHalfedge(hret);
    // }
    remove_tip(hret);
    h->end_vertex()->setRemoved();
    h->face()->reset(hret);
    return hret;
}

Halfedge* Mesh::erase_center_vertex_plus(Halfedge* h) {
    bool isPlus = h->end_vertex()->isBoundary();
#ifdef DEBUG
    std::vector<MCGAL::Halfedge*> origin_around_halfedge;
    for (MCGAL::Halfedge* rhit : *h->end_vertex()) {
        // if (rhit->face()->poolId() == 27807) {
        //     int i = 0;
        // }
        origin_around_halfedge.push_back(rhit);
    }
#endif
    if (isPlus) {
        while (h->opposite() != nullptr) {
            h = h->opposite()->next()->next();
        }
    }

    Halfedge* g = h->next()->opposite();
    Halfedge* gstart = g;
    Halfedge* hret = find_prev(h);
    Halfedge* gprev = nullptr;
    while (g != h) {
        gprev = find_prev(g);
        remove_tip(gprev);
        if (g->face() != h->face()) {
            g->face()->setRemoved();
        }
        Halfedge* gnext = g->next()->opposite();
        g->setRemoved();
        // g->opposite()->setRemoved();
        g->opposite()->setRemoved();
        // if (gprev->vertex()->proxyHalfedge() == gnext) {
        gprev->vertex()->setProxyHalfedge(gprev);
        // }
        if (gnext == nullptr) {
            g->next()->setRemoved();
            break;
        }
        g = gnext;
    }
    h->setRemoved();
    if (!isPlus)
        h->opposite()->setRemoved();
    // if (hret->vertex()->proxyHalfedge() == gstart) {
    hret->vertex()->setProxyHalfedge(hret);
    // }

    // h->vertex()->setProxyHalfedge(hret->opposite());
    if (!isPlus) {
        remove_tip(hret);
    }
    if (!isPlus) {
        h->vertex()->setProxyHalfedge(hret->next());
    } else {
        h->vertex()->setProxyHalfedge(hret->opposite());
    }
    h->end_vertex()->setRemoved();
    if (!isPlus) {
        h->face()->reset(hret);
    }
#ifdef DEBUG
    for (MCGAL::Halfedge* rhit : origin_around_halfedge) {
        assert(rhit->isRemoved());
    }
#endif
    if (isPlus) {
        hret->setNext(nullptr);
        hret->face()->setProxyHalfedge(gprev);
        gprev->setFace(h->face());
        return gprev;
    }

    return hret;
}

Halfedge* Mesh::create_center_vertex(Halfedge* h) {
    // Vertex* vnew = new Vertex();
    Vertex* vnew = MCGAL::contextPool.allocateVertexFromPool(meshId_);
    this->vertices_.push_back(vnew);
    Halfedge* hnew = MCGAL::contextPool.allocateHalfedgeFromPool(meshId_, h->end_vertex(), vnew);
    Halfedge* oppo_new = MCGAL::contextPool.allocateHalfedgeFromPool(meshId_, vnew, h->end_vertex());
    // add new halfedge to current mesh and set opposite
    // set the next element
    // now the next of hnew and prev of oppo_new is unknowen
    insert_tip(hnew->opposite(), h);
    Halfedge* g = hnew->opposite()->next();
    std::vector<Halfedge*> origin_around_halfedge;

    Halfedge* hed = hnew;
    while (g->next() != hed) {
        Halfedge* gnew = MCGAL::contextPool.allocateHalfedgeFromPool(meshId_, g->end_vertex(), vnew);
        Halfedge* oppo_gnew = MCGAL::contextPool.allocateHalfedgeFromPool(meshId_, vnew, g->end_vertex());
        origin_around_halfedge.push_back(g);
        gnew->setNext(hnew->opposite());
        insert_tip(gnew->opposite(), g);

        g = gnew->opposite()->next();
        hnew = gnew;
    }
    hed->setNext(hnew->opposite());
    h->face()->reset(h);
    // collect all the halfedge
    for (Halfedge* hit : origin_around_halfedge) {
        Facet* face = MCGAL::contextPool.allocateFaceFromPool(meshId_, hit);
        this->faces_.push_back(face);
    }
    return oppo_new;
}

/**
    目前存在的问题，如果只是三个顶点，会存在问题
 */
Halfedge* Mesh::create_center_vertex(Halfedge* h, Point point) {
    // Vertex* vnew = new Vertex();
    Vertex* vnew = MCGAL::contextPool.allocateVertexFromPool(meshId_, point);
    this->vertices_.push_back(vnew);
    Halfedge* hnew = MCGAL::contextPool.allocateHalfedgeFromPool(meshId_, h->end_vertex(), vnew);
    Halfedge* oppo_new = MCGAL::contextPool.allocateHalfedgeFromPool(meshId_, vnew, h->end_vertex());
    // add new halfedge to current mesh and set opposite
    // set the next element
    // now the next of hnew and prev of oppo_new is unknowen
    insert_tip(hnew->opposite(), h);
    Halfedge* g = hnew->opposite()->next();
    std::vector<Halfedge*> origin_around_halfedge;
    origin_around_halfedge.reserve(5);

    Halfedge* hed = hnew;
    while (g->next() != hed) {
        Halfedge* gnew = MCGAL::contextPool.allocateHalfedgeFromPool(meshId_, g->end_vertex(), vnew);
        Halfedge* oppo_gnew = MCGAL::contextPool.allocateHalfedgeFromPool(meshId_, vnew, g->end_vertex());
        origin_around_halfedge.push_back(g);
        gnew->setNext(hnew->opposite());
        insert_tip(gnew->opposite(), g);

        g = gnew->opposite()->next();
        hnew = gnew;
    }
    hed->setNext(hnew->opposite());
    h->face()->reset_without_init(h);
    // collect all the halfedge
    for (Halfedge* hit : origin_around_halfedge) {
        Facet* face = MCGAL::contextPool.allocateFaceFromPool(meshId_, hit);
        this->faces_.push_back(face);
    }
    return oppo_new;
}

Halfedge* Mesh::create_center_vertex_without_init(Halfedge* h, Point point) {
    // Vertex* vnew = new Vertex();
    const int faceGroupId = h && h->face() ? h->face()->groupId()
                                           : (h && h->end_vertex() ? h->end_vertex()->groupId() : -1);
    Vertex* vnew = MCGAL::contextPool.allocateVertexFromPool(meshId_, point);
    vnew->setGroupId(faceGroupId);
    this->vertices_.push_back(vnew);
    Halfedge* hnew = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(meshId_, h->end_vertex(), vnew);
    Halfedge* oppo_new = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(meshId_, vnew, h->end_vertex());
    hnew->setOpposite(oppo_new);
    oppo_new->setOpposite(hnew);
    // add new halfedge to current mesh and set opposite
    // set the next element
    // now the next of hnew and prev of oppo_new is unknowen
    insert_tip(hnew->opposite(), h);
    Halfedge* g = hnew->opposite()->next();
    // std::vector<Halfedge*> origin_around_halfedge;
    // origin_around_halfedge.reserve(5);

    Halfedge* hed = hnew;
    while (g->next() != hed) {
        Halfedge* gnew = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(meshId_, g->end_vertex(), vnew);
        Halfedge* oppo_gnew = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(meshId_, vnew, g->end_vertex());

        gnew->setOpposite(oppo_gnew);
        oppo_gnew->setOpposite(gnew);

        gnew->setNext(hnew->opposite());
        insert_tip(gnew->opposite(), g);
        Facet* face = MCGAL::contextPool.allocateFaceFromPool(meshId_, g);
        face->setGroupId(faceGroupId);
        // origin_around_halfedge.push_back(g);
        this->faces_.push_back(face);
        g = gnew->opposite()->next();
        hnew = gnew;
    }
    hed->setNext(hnew->opposite());
    h->face()->reset_without_init(h);
    // collect all the halfedge
    // for (Halfedge* hit : origin_around_halfedge) {
    //     Facet* face = MCGAL::contextPool.allocateFaceFromPool(hit, meshId);
    //     this->faces_.push_back(face);
    // }
    return oppo_new;
}

Halfedge* Mesh::create_center_vertex_plus(Halfedge* h, Point point) {
    // Vertex* vnew = new Vertex();
    const int faceGroupId = h && h->face() ? h->face()->groupId()
                                           : (h && h->end_vertex() ? h->end_vertex()->groupId() : -1);
    Vertex* vnew = MCGAL::contextPool.allocateVertexFromPool(meshId_, point);
    vnew->setGroupId(faceGroupId);
    this->vertices_.push_back(vnew);
    Halfedge* hnew = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(meshId_, h->end_vertex(), vnew);
    Halfedge* oppo_new = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(meshId_, vnew, h->end_vertex());
    hnew->setOpposite(oppo_new);
    oppo_new->setOpposite(hnew);
    insert_tip(hnew->opposite(), h);
    Halfedge* g = hnew->opposite()->next();
    // std::vector<Halfedge*> origin_around_halfedge;
    // origin_around_halfedge.reserve(5);

    Halfedge* hed = hnew;
    while (g->next() != hed && g->next() != nullptr) {
        Halfedge* gnew = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(meshId_, g->end_vertex(), vnew);
        Halfedge* oppo_gnew = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(meshId_, vnew, g->end_vertex());

        gnew->setOpposite(oppo_gnew);
        oppo_gnew->setOpposite(gnew);

        gnew->setNext(hnew->opposite());
        insert_tip(gnew->opposite(), g);
        Facet* face = MCGAL::contextPool.allocateFaceFromPool(meshId_, g);
        face->setGroupId(faceGroupId);
        // origin_around_halfedge.push_back(g);
        this->faces_.push_back(face);
        g = gnew->opposite()->next();
        hnew = gnew;
    }
    if (g->next() != nullptr) {
        hed->setNext(hnew->opposite());
        h->face()->reset_without_init(h);
    } else {
        Halfedge* h1 = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(meshId_, hed->end_vertex(), h->vertex());
        hed->setNext(h1);
        h1->setNext(h);
        h->face()->reset_without_init(h);

        Halfedge* h2 = MCGAL::contextPool.allocateHalfedgeFromPoolWithOutInit(meshId_, g->end_vertex(), vnew);
        hnew->opposite()->setNext(g);
        g->setNext(h2);
        h2->setNext(hnew->opposite());
        Facet* face = MCGAL::contextPool.allocateFaceFromPool(meshId_, g);
        face->setGroupId(faceGroupId);
        this->faces_.push_back(face);
    }
    // collect all the halfedge
    // for (Halfedge* hit : origin_around_halfedge) {
    //     Facet* face = MCGAL::contextPool.allocateFaceFromPool(hit, meshId);
    //     this->faces_.push_back(face);
    // }
    return oppo_new;
}
// #endif

inline void Mesh::close_tip(Halfedge* h, Vertex* v) const {
    h->setNext(h->opposite());
    h->setVertex(v);
}

inline void Mesh::insert_tip(Halfedge* h, Halfedge* v) const {
    h->setNext(v->next());
    v->setNext(h->opposite());
}

Halfedge* Mesh::find_prev(Halfedge* h) const {
    Halfedge* g = h;
    while (g->next() != h) {
        g = g->next();
    }
    return g;
}

inline void Mesh::remove_tip(Halfedge* h) const {
    h->setNext(h->next()->opposite()->next());
}

Halfedge* Mesh::join_face(Halfedge* h) {
    Halfedge* hprev = find_prev(h);
    Halfedge* gprev = find_prev(h->opposite());
    remove_tip(hprev);
    remove_tip(gprev);
    h->opposite()->setRemoved();
    h->setRemoved();

    gprev->face()->setRemoved();
    hprev->face()->reset_without_init(hprev);
    return hprev;
}

bool Mesh::loadOFF(std::string path) {
    // perf（2026-11-08 perf record：load 里 num_get/sentry/strtof 合计 ~6.7% cycles，
    // 另有逐面 std::stringstream/getline 的分配）：整文件读入缓冲区后手写解析。
    // 语义按原实现逐条复刻：
    //   * 头：token("OFF") + nb_vertices/nb_faces/nb_edges；
    //   * 顶点：连续 3 个 float 一个顶点（与原 `file >> x >> y >> z` 相同的 token 流；
    //     顶点之间不按行切分；最后一个顶点之后整行丢弃，与原 getline 一致）；
    //   * 面：一行一个面，跳过空行与 '#' 注释行，行首 int 为度数，随后为顶点索引；
    //   * 顶点/面分配顺序、bbox 更新与原来完全一致 ⇒ 池 poolId 与遍历顺序不变。
    std::ifstream fp(path, std::ios::binary);
    if (!fp.is_open()) {
        std::cerr << "Error: Unable to open file " << path << std::endl;
        return false;
    }
    fp.seekg(0, std::ios::end);
    const std::streamoff fileSize = fp.tellg();
    fp.seekg(0, std::ios::beg);
    std::string data;
    if (fileSize > 0) {
        data.resize(static_cast<std::size_t>(fileSize));
        fp.read(&data[0], fileSize);
    }
    fp.close();

    const char* p = data.c_str();
    const char* const endp = p + data.size();
    auto skipWs = [&]() {
        while (p < endp && (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r')) ++p;
    };
    auto parseLong = [&](long& out) -> bool {
        skipWs();
        if (p >= endp) return false;
        char* e = nullptr;
        const long v = std::strtol(p, &e, 10);
        if (e == p) return false;
        out = v;
        p = e;
        return true;
    };
    auto parseFloat = [&](float& out) -> bool {
        skipWs();
        if (p >= endp) return false;
        char* e = nullptr;
        const float v = std::strtof(p, &e);
        if (e == p) return false;
        out = v;
        p = e;
        return true;
    };

    skipWs();
    std::string format;
    while (p < endp && *p != ' ' && *p != '\t' && *p != '\n' && *p != '\r') {
        format.push_back(*p++);
    }
    long nv = 0, nf = 0, ne = 0;
    if (!parseLong(nv) || !parseLong(nf) || !parseLong(ne)) {
        std::cerr << "Error: Invalid OFF file header" << std::endl;
        return false;
    }
    nb_vertices = static_cast<int>(nv);
    nb_faces = static_cast<int>(nf);
    nb_edges = static_cast<int>(ne);
    meshId_ = DEFAULT_MESH_ID;
    if (format != "OFF") {
        std::cerr << "Error: Invalid OFF file format" << std::endl;
        return false;
    }

    vertices_.reserve(static_cast<std::size_t>(nb_vertices));
    faces_.reserve(static_cast<std::size_t>(nb_faces));

    for (int i = 0; i < nb_vertices; ++i) {
        float x = 0.0f, y = 0.0f, z = 0.0f;
        if (!parseFloat(x) || !parseFloat(y) || !parseFloat(z)) {
            std::cerr << "Error: Unexpected end of OFF vertex list" << std::endl;
            return false;
        }
        Vertex* vt = MCGAL::contextPool.allocateVertexFromPool(DEFAULT_MESH_ID, x, y, z);
        if (enableQuantization) {
            bboxMin = MCGAL::Point(std::min(bboxMin.x(), x), std::min(bboxMin.y(), y), std::min(bboxMin.z(), z));
            bboxMax = MCGAL::Point(std::max(bboxMax.x(), x), std::max(bboxMax.y(), y), std::max(bboxMax.z(), z));
        }
        this->vertices_.push_back(vt);
    }
    if (enableQuantization) {
        float f_maxRange = 0;
        for (unsigned i = 0; i < 3; ++i) {
            float range = bboxMax[i] - bboxMin[i];
            f_maxRange = std::max(range, f_maxRange);
        }
        f_quantStep = f_maxRange / (1 << i_nbQuantBits);
    }

    // 与原实现的 `std::getline(file, faceLine)` 对称：丢弃最后一个顶点行的剩余部分。
    while (p < endp && *p != '\n') ++p;
    if (p < endp) ++p;

    std::vector<Vertex*> vts;
    for (int i = 0; i < nb_faces; ++i) {
        const char* lineStart = nullptr;
        const char* lineEnd = nullptr;
        for (;;) {
            if (p >= endp) {
                std::cerr << "Error: Unexpected end of OFF face list" << std::endl;
                return false;
            }
            lineStart = p;
            while (p < endp && *p != '\n') ++p;
            lineEnd = p;
            if (p < endp) ++p;
            if (lineStart == lineEnd) continue;  // 空行
            if (*lineStart == '#') continue;     // 注释行
            break;
        }
        char* e = nullptr;
        const long nfv = std::strtol(lineStart, &e, 10);
        if (e == lineStart) {
            std::cerr << "Error: Invalid OFF face line" << std::endl;
            return false;
        }
        const int num_face_vertices = static_cast<int>(nfv);
        vts.clear();
        vts.reserve(static_cast<std::size_t>(num_face_vertices));
        const char* q = e;
        for (int j = 0; j < num_face_vertices; ++j) {
            while (q < lineEnd && (*q == ' ' || *q == '\t')) ++q;
            if (q >= lineEnd) {
                std::cerr << "Error: Invalid OFF face line" << std::endl;
                return false;
            }
            char* e2 = nullptr;
            const long idx = std::strtol(q, &e2, 10);
            if (e2 == q) {
                std::cerr << "Error: Invalid OFF face line" << std::endl;
                return false;
            }
            q = e2;
            vts.push_back(vertices_[static_cast<std::size_t>(idx)]);
        }
        MCGAL::Facet* fit = MCGAL::contextPool.allocateFaceFromPool(DEFAULT_MESH_ID, vts);
        this->faces_.push_back(fit);
    }
    return true;
}

inline bool isFacetRemovable(MCGAL::Facet* fit) {
    return fit->isRemoved();
}

inline bool isVertexRemovable(MCGAL::Vertex* vit) {
    return vit->isRemoved();
}

/* floatPosToInt 已搬到 Mesh.h 作 inline（PHASE64 物品2）。 */

MCGAL::Point Mesh::intPosToFloat(PointInt p) {
    return Point(bboxMin[0] + (p[0] + 0.5) * f_quantStep, bboxMin[1] + (p[1] + 0.5) * f_quantStep, bboxMin[2] + (p[2] + 0.5) * f_quantStep);
}

MCGAL::Vector3 Mesh::computeNormal() {
    MCGAL::Vector3 normal(0, 0, 0);
    int len = faces_.size();
    for (int i = 0; i < len; i++) {
        normal = normal + faces_[i]->computeNormal();
    }
    return MCGAL::Vector3(normal.x() / len, normal.y() / len, normal.z() / len);
}

void Mesh::driftAlongNormal(int step) {
    MCGAL::Vector3 normal = computeNormal();
    for (int i = 0; i < vertices_.size(); i++) {
        vertices_[i]->setPoint({vertices_[i]->x() + step * normal.x(), vertices_[i]->y() + step * normal.y(), vertices_[i]->z() + step * normal.z()});
    }
}

#ifdef DECODE
void Mesh::dumpto(std::string path) {
    auto newEnd = std::remove_if(faces_.begin(), faces_.end(), isFacetRemovable);
    faces_.resize(std::distance(faces_.begin(), newEnd));

    auto newVEnd = std::remove_if(vertices_.begin(), vertices_.end(), isVertexRemovable);
    vertices_.resize(std::distance(vertices_.begin(), newVEnd));

    std::ofstream offFile(path);
    if (!offFile.is_open()) {
        std::cerr << "Error opening file: " << path << std::endl;
        return;
    }
    offFile << std::setprecision(10);
    // write header
    offFile << "OFF\n";
    offFile << this->vertices_.size() << " " << this->faces_.size() << " 0\n";
    // write vertex
    int id = 0;
    for (Vertex* vertex : this->vertices_) {
        if (vertex->isRemoved())
            continue;
        offFile << vertex->x() << " " << vertex->y() << " " << vertex->z() << "\n";
        vertex->setVid(id++);
    }

    for (Facet* face : this->faces_) {
        if (face->isRemoved())
            continue;

        offFile << face->vertices_.size() << " ";
        Halfedge* hst = face->halfedges[0];
        Halfedge* hed = hst;
        do {
            offFile << hst->vertex->getVid() << " ";
            hst = hst->next;
        } while (hst != hed);

#    ifdef COLOR
        if (face->groupId >= 0) {
            offFile << colors[face->groupId][0] << " " << colors[face->groupId][1] << " " << colors[face->groupId][2] << " ";
        } else {
            offFile << 0 << " " << 0 << " " << 0 << " ";
        }
#    endif
        offFile << "\n";
    }
    offFile.close();
}
#else

void Mesh::garbage_collection() {
    auto newEnd = std::remove_if(faces_.begin(), faces_.end(), isFacetRemovable);
    faces_.resize(std::distance(faces_.begin(), newEnd));

    auto newVEnd = std::remove_if(vertices_.begin(), vertices_.end(), isVertexRemovable);
    vertices_.resize(std::distance(vertices_.begin(), newVEnd));
}

void Mesh::vertex_based_submesh_dumpto_oldtype(std::string path, int groupId) {
    auto newEnd = std::remove_if(faces_.begin(), faces_.end(), isFacetRemovable);
    faces_.resize(std::distance(faces_.begin(), newEnd));

    auto newVEnd = std::remove_if(vertices_.begin(), vertices_.end(), isVertexRemovable);
    vertices_.resize(std::distance(vertices_.begin(), newVEnd));

    std::set<Vertex*> vertices;
    std::vector<Facet*> faces;
    std::copy_if(std::begin(vertices_), std::end(vertices_), std::inserter(vertices, vertices.end()),
                 [&](Vertex* v) { return v->groupId() == groupId; });
    std::copy_if(std::begin(faces_), std::end(faces_), std::back_inserter(faces), [&](Facet* f) {
        // std::set<int> sts;
        for (Halfedge* it : *f) {
            if (it->vertex()->groupId() != groupId) {
                // sts.insert(it->vertex()->groupId());
                return false;
            }
        }
        // assert(sts.size() < 2);
        return true;
    });
    // for (Facet* f : faces) {
    //     for (Facet::halfedge_iterator it = f->halfedges_begin(); it != f->halfedges_end(); ++it) {
    //         vertices.insert((*it)->vertex());
    //     }
    // }
    std::ofstream offFile(path);
    if (!offFile.is_open()) {
        std::cerr << "Error opening file: " << path << std::endl;
        return;
    }
    offFile << std::setprecision(10);
    // write header
    offFile << "OFF\n";
    offFile << vertices.size() << " " << faces.size() << " 0\n";
    offFile << "\n";
    // write vertex
    int id = 0;
    for (Vertex* vertex : vertices) {
        if (vertex->isRemoved())
            continue;
        offFile << vertex->x() << " " << vertex->y() << " " << vertex->z() << "\n";
        vertex->setVid(id++);
    }

    for (Facet* face : faces) {
        if (face->isRemoved())
            continue;

        int num = 0;
        // offFile << face->vertices_.size() << " ";
        Halfedge* hst = face->proxyHalfedge();
        Halfedge* hed = face->proxyHalfedge();
        bool fg = false;
        do {
            // offFile << hst->vertex->getVid() << " ";
            num++;
            if (hst->vertex()->isRemoved()) {
                fg = true;
            }
            hst = hst->next();
        } while (hst != hed);
        if (fg) {
            offFile << 3 << " ";
            offFile << 0 << " " << 0 << " " << 0 << " ";
            offFile << "\n";
            continue;
        }
        offFile << num << " ";
#    ifdef VERTEX_COLOR
        std::map<int, int> mpcnt;
#    endif
        do {
            offFile << hst->vertex()->vid() << " ";
#    ifdef VERTEX_COLOR
            mpcnt[hst->vertex()->groupId()]++;
#    endif
            hst = hst->next();
        } while (hst != hed);

#    ifdef VERTEX_COLOR

        if (mpcnt.size() == 1) {
            offFile << colors[mpcnt.begin()->first][0] << " " << colors[mpcnt.begin()->first][1] << " " << colors[mpcnt.begin()->first][2] << " ";
        } else {
            offFile << 255 << " " << 255 << " " << 255 << " ";
        }
#    endif
#    ifdef FACE_COLOR
        if (face->groupId() >= 0) {
            offFile << colors[face->groupId()][0] << " " << colors[face->groupId()][1] << " " << colors[face->groupId()][2] << " ";
        } else {
            offFile << 0 << " " << 0 << " " << 0 << " ";
        }
#    endif
        offFile << "\n";
    }
    offFile.close();
}

void Mesh::facet_based_submesh_dumpto_oldtype(std::string path, int groupId) {
    garbage_collection();
    // auto newEnd = std::remove_if(faces_.begin(), faces_.end(), isFacetRemovable);
    // faces_.resize(std::distance(faces_.begin(), newEnd));

    // auto newVEnd = std::remove_if(vertices_.begin(), vertices_.end(), isVertexRemovable);
    // vertices_.resize(std::distance(vertices_.begin(), newVEnd));

    std::set<Vertex*> vertices;
    std::vector<Facet*> faces;
    // std::copy_if(std::begin(vertices_), std::end(vertices_), std::back_inserter(vertices),
    //              [&](Vertex* v) { return v->groupId() == groupId; });
    std::copy_if(std::begin(faces_), std::end(faces_), std::back_inserter(faces), [&](Facet* f) { return f->groupId() == groupId; });
    for (Facet* f : faces) {
        for (Facet::halfedge_iterator it = f->halfedges_begin(); it != f->halfedges_end(); ++it) {
            vertices.insert((*it)->vertex());
        }
    }

    std::ofstream offFile(path);
    if (!offFile.is_open()) {
        std::cerr << "Error opening file: " << path << std::endl;
        return;
    }
    offFile << std::setprecision(10);
    // write header
    offFile << "OFF\n";
    offFile << vertices.size() << " " << faces.size() << " 0\n";
    offFile << "\n";
    // write vertex
    int id = 0;
    for (Vertex* vertex : vertices) {
        if (vertex->isRemoved())
            continue;
        offFile << vertex->x() << " " << vertex->y() << " " << vertex->z() << "\n";
        vertex->setVid(id++);
    }
    // TODO: 记得处理这种情况，怀疑是在三顶点时遇到了问题
    for (Facet* face : faces) {
        if (face->isRemoved())
            continue;

        int num = 0;
        // offFile << face->vertices_.size() << " ";
        Halfedge* hst = face->proxyHalfedge();
        Halfedge* hed = face->proxyHalfedge();
        bool fg = false;
        do {
            // offFile << hst->vertex->getVid() << " ";
            num++;
            assert(!hst->vertex()->isRemoved());
            if (hst->vertex()->isRemoved()) {
                fg = true;
            }
            hst = hst->next();
        } while (hst != hed);
        if (fg) {
            offFile << 3 << " ";
            offFile << 0 << " " << 0 << " " << 0 << " ";
            offFile << "\n";
            continue;
        }
        offFile << num << " ";
#    ifdef VERTEX_COLOR
        std::map<int, int> mpcnt;
#    endif
        do {
            offFile << hst->vertex()->vid() << " ";
#    ifdef VERTEX_COLOR
            mpcnt[hst->vertex()->groupId()]++;
#    endif
            hst = hst->next();
        } while (hst != hed);

#    ifdef VERTEX_COLOR

        if (mpcnt.size() == 1) {
            offFile << colors[mpcnt.begin()->first][0] << " " << colors[mpcnt.begin()->first][1] << " " << colors[mpcnt.begin()->first][2] << " ";
        } else {
            offFile << 255 << " " << 255 << " " << 255 << " ";
        }
#    endif
#    ifdef FACE_COLOR
        if (face->groupId() >= 0) {
            offFile << colors[face->groupId()][0] << " " << colors[face->groupId()][1] << " " << colors[face->groupId()][2] << " ";
        } else {
            offFile << 0 << " " << 0 << " " << 0 << " ";
        }
#    endif
        offFile << "\n";
    }
    offFile.close();
}

void Mesh::checkMesh() {
    std::set<std::pair<int, int>> edges;
    for (Facet* face : faces_) {
        if (face->isRemoved())
            continue;

        // offFile << face->vertices_.size() << " ";
        Halfedge* hst = face->proxyHalfedge();
        Halfedge* hed = face->proxyHalfedge();
        bool fg = false;
        do {
            assert(!hst->vertex()->isRemoved());
            auto p = std::make_pair(hst->vertex()->poolId(), hst->end_vertex()->poolId());
            assert(!edges.count(p));
            edges.insert(p);
            hst = hst->next();
        } while (hst != hed);
    }
}

void Mesh::checkMesh(int groupId) {
    std::set<Vertex*> vertices;
    std::vector<Facet*> faces;
    std::copy_if(std::begin(faces_), std::end(faces_), std::back_inserter(faces),
                 [&](Facet* f) { return !f->isRemoved() && f->groupId() == groupId; });
    for (Facet* f : faces) {
        for (Facet::halfedge_iterator it = f->halfedges_begin(); it != f->halfedges_end(); ++it) {
            vertices.insert((*it)->vertex());
        }
    }
    std::map<std::pair<int, int>, MCGAL::Halfedge*> edges;
    for (Facet* face : faces) {
        if (face->isRemoved())
            continue;

        // offFile << face->vertices_.size() << " ";
        Halfedge* hst = face->proxyHalfedge();
        Halfedge* hed = face->proxyHalfedge();
        bool fg = false;
        do {
            assert(!hst->vertex()->isRemoved());
            auto p = std::make_pair(hst->vertex()->poolId(), hst->end_vertex()->poolId());
            assert(!edges.count(p));
            edges[p] = hst;
            hst = hst->next();
        } while (hst != hed);
    }
}

void Mesh::markGroupId(MCGAL::Halfedge* seed, int groupId) {
    std::deque<int> gateQueue;
    gateQueue.push_back(seed->face()->poolId());
    int current_version = MCGAL::BFSVersionManager::current_version++;
    while (!gateQueue.empty()) {
        int fid = gateQueue.front();
        gateQueue.pop_front();
        Facet* f = MCGAL::contextPool.getFacetByIndexInSubPool(meshId_, fid);
        if (f->isVisited(current_version)) {
            continue;
        }
        f->setVisited(current_version);
        f->setGroupId(groupId);
        MCGAL::Halfedge* st = f->proxyHalfedge();
        MCGAL::Halfedge* ed = f->proxyHalfedge();
        do {
            MCGAL::Halfedge* hOpp = st->opposite();
            if (!hOpp->face()->isVisited(current_version) && !st->isBoundary() && !st->opposite()->isBoundary()) {
                gateQueue.push_back(hOpp->face()->poolId());
            }
            st = st->next();
        } while (st != ed);
    }
}

void Mesh::updateBboxMax(MCGAL::Point bMax) {
    bboxMax[0] = std::max(bboxMax[0], bMax[0]);
    bboxMax[1] = std::max(bboxMax[1], bMax[1]);
    bboxMax[2] = std::max(bboxMax[2], bMax[2]);
}

void Mesh::updateBboxMin(MCGAL::Point bMin) {
    bboxMin[0] = std::min(bboxMin[0], bMin[0]);
    bboxMin[1] = std::min(bboxMin[1], bMin[1]);
    bboxMin[2] = std::min(bboxMin[2], bMin[2]);
}

void Mesh::dumpto_oldtype(std::string path) {
    // auto newEnd = std::remove_if(faces_.begin(), faces_.end(), isFacetRemovable);
    // faces_.resize(std::distance(faces_.begin(), newEnd));

    // auto newVEnd = std::remove_if(vertices_.begin(), vertices_.end(), isVertexRemovable);
    // vertices_.resize(std::distance(vertices_.begin(), newVEnd));
    garbage_collection();
    const char* strictTriEnv = std::getenv("PMSF_STRICT_TRIANGULATE_OUTPUT");
    const bool strictTriangulateOutput =
        strictTriEnv && strictTriEnv[0] != '\0' && std::strcmp(strictTriEnv, "0") != 0;

    std::string out;
    out.reserve(vertices_.size() * 48 + faces_.size() * 32 + 64);

    auto collectFaceVids = [](Facet* face, std::vector<int>& vids) -> bool {
        vids.clear();
        if (!face || face->isRemoved() || !face->proxyHalfedge()) return false;
        Halfedge* hst = face->proxyHalfedge();
        Halfedge* hed = face->proxyHalfedge();
        int guard = 0;
        while (hst) {
            if (!hst->vertex() || hst->vertex()->isRemoved()) return false;
            vids.push_back(hst->vertex()->vid());
            if (!hst->next()) {
                if (!hst->end_vertex() || hst->end_vertex()->isRemoved()) return false;
                vids.push_back(hst->end_vertex()->vid());
                break;
            }
            hst = hst->next();
            if (hst == hed) break;
            if (++guard > 8192) return false;
        }
        return vids.size() >= 3;
    };

    std::size_t outputFaceCount = faces_.size();
    bool hasPolygonFace = false;
    if (strictTriangulateOutput) {
        outputFaceCount = 0;
        std::vector<int> vids;
        for (Facet* face : this->faces_) {
            if (face->isRemoved()) continue;
            if (!collectFaceVids(face, vids)) {
                ++outputFaceCount;
            } else {
                if (vids.size() > 3) hasPolygonFace = true;
                outputFaceCount += vids.size() > 3 ? vids.size() - 2 : 1;
            }
        }
    }

    out.append("OFF\n");
    appendSize(out, vertices_.size());
    out.push_back(' ');
    appendSize(out, outputFaceCount);
    out.append(" 0\n");

    // write vertex
    std::vector<Vertex*> strictVertexByVid;
    if (strictTriangulateOutput) {
        strictVertexByVid.reserve(vertices_.size());
    }
    int id = 0;
    for (Vertex* vertex : this->vertices_) {
        if (vertex->isRemoved())
            continue;
        appendFloat10(out, vertex->x());
        out.push_back(' ');
        appendFloat10(out, vertex->y());
        out.push_back(' ');
        appendFloat10(out, vertex->z());
        out.push_back('\n');
        vertex->setVid(id++);
        if (strictTriangulateOutput) {
            strictVertexByVid.push_back(vertex);
        }
    }

    auto strictEdgeKey = [](int a, int b) -> std::uint64_t {
        if (a > b) std::swap(a, b);
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(a)) << 32) |
               static_cast<std::uint32_t>(b);
    };
    std::unordered_set<std::uint64_t> strictBoundaryEdges;
    std::unordered_set<std::uint64_t> strictInternalDiags;
    auto strictTriangleArea2 = [&](int a, int b, int c) -> double {
        if (a < 0 || b < 0 || c < 0 ||
            static_cast<std::size_t>(a) >= strictVertexByVid.size() ||
            static_cast<std::size_t>(b) >= strictVertexByVid.size() ||
            static_cast<std::size_t>(c) >= strictVertexByVid.size()) {
            return 0.0;
        }
        Vertex* va = strictVertexByVid[static_cast<std::size_t>(a)];
        Vertex* vb = strictVertexByVid[static_cast<std::size_t>(b)];
        Vertex* vc = strictVertexByVid[static_cast<std::size_t>(c)];
        if (!va || !vb || !vc) return 0.0;
        const double ux = static_cast<double>(vb->x()) - static_cast<double>(va->x());
        const double uy = static_cast<double>(vb->y()) - static_cast<double>(va->y());
        const double uz = static_cast<double>(vb->z()) - static_cast<double>(va->z());
        const double vx = static_cast<double>(vc->x()) - static_cast<double>(va->x());
        const double vy = static_cast<double>(vc->y()) - static_cast<double>(va->y());
        const double vz = static_cast<double>(vc->z()) - static_cast<double>(va->z());
        const double cx = uy * vz - uz * vy;
        const double cy = uz * vx - ux * vz;
        const double cz = ux * vy - uy * vx;
        return cx * cx + cy * cy + cz * cz;
    };
    if (strictTriangulateOutput && hasPolygonFace) {
        strictBoundaryEdges.reserve(faces_.size() * 3 + 16);
        std::vector<int> vids;
        for (Facet* face : this->faces_) {
            if (face->isRemoved() || !collectFaceVids(face, vids)) continue;
            for (std::size_t i = 0; i < vids.size(); ++i) {
                strictBoundaryEdges.insert(
                    strictEdgeKey(vids[i], vids[(i + 1) % vids.size()]));
            }
        }
    }

    auto triangulateStrictPolygon =
        [&](const std::vector<int>& vids) -> std::vector<std::array<int, 3>> {
        std::vector<std::array<int, 3>> fallback;
        if (vids.size() < 3) return fallback;
        for (std::size_t i = 1; i + 1 < vids.size(); ++i) {
            fallback.push_back({vids[0], vids[i], vids[i + 1]});
        }
        if (vids.size() <= 3 || !hasPolygonFace) return fallback;

        std::unordered_set<std::uint64_t> polygonBoundary;
        polygonBoundary.reserve(vids.size() * 2 + 1);
        for (std::size_t i = 0; i < vids.size(); ++i) {
            polygonBoundary.insert(strictEdgeKey(vids[i], vids[(i + 1) % vids.size()]));
        }

        std::vector<std::array<int, 3>> current;
        std::vector<std::array<int, 3>> chosen;
        std::function<bool(std::vector<int>&)> search = [&](std::vector<int>& poly) -> bool {
            if (poly.size() == 3) {
                if (strictTriangleArea2(poly[0], poly[1], poly[2]) <= 1e-30) {
                    return false;
                }
                current.push_back({poly[0], poly[1], poly[2]});
                std::unordered_set<std::uint64_t> internal;
                internal.reserve(current.size() * 2 + 1);
                for (const auto& tri : current) {
                    const int ev[3][2] = {{tri[0], tri[1]},
                                          {tri[1], tri[2]},
                                          {tri[2], tri[0]}};
                    for (const auto& e : ev) {
                        std::uint64_t key = strictEdgeKey(e[0], e[1]);
                        if (polygonBoundary.count(key) == 0) {
                            internal.insert(key);
                        }
                    }
                }
                bool ok = true;
                for (std::uint64_t key : internal) {
                    if (strictBoundaryEdges.count(key) != 0 ||
                        strictInternalDiags.count(key) != 0) {
                        ok = false;
                        break;
                    }
                }
                if (ok) {
                    chosen = current;
                    for (std::uint64_t key : internal) {
                        strictInternalDiags.insert(key);
                    }
                }
                current.pop_back();
                return ok;
            }

            const std::size_t n = poly.size();
            for (std::size_t i = 0; i < n; ++i) {
                const int prev = poly[(i + n - 1) % n];
                const int cur = poly[i];
                const int next = poly[(i + 1) % n];
                if (strictTriangleArea2(prev, cur, next) <= 1e-30) {
                    continue;
                }
                current.push_back({prev, cur, next});
                std::vector<int> rest;
                rest.reserve(n - 1);
                for (std::size_t j = 0; j < n; ++j) {
                    if (j != i) rest.push_back(poly[j]);
                }
                if (search(rest)) return true;
                current.pop_back();
            }
            return false;
        };

        std::vector<int> poly = vids;
        if (search(poly)) return chosen;
        return fallback;
    };

    for (Facet* face : this->faces_) {
        if (face->isRemoved())
            continue;

        if (strictTriangulateOutput) {
            std::vector<int> vids;
            if (!collectFaceVids(face, vids)) {
                out.append("3 0 0 0 \n");
                continue;
            }
            std::vector<std::array<int, 3>> tris = triangulateStrictPolygon(vids);
            for (const auto& tri : tris) {
                out.append("3 ");
                appendInt(out, tri[0]);
                out.push_back(' ');
                appendInt(out, tri[1]);
                out.push_back(' ');
                appendInt(out, tri[2]);
                out.push_back('\n');
            }
            continue;
        }

        int num = 0;
        // offFile << face->vertices_.size() << " ";
        Halfedge* hst = face->proxyHalfedge();
        Halfedge* hed = face->proxyHalfedge();
        bool fg = false;
        int guard = 0;
        while (hst) {
            // offFile << hst->vertex->getVid() << " ";
            num++;
            if (!hst->vertex() || hst->vertex()->isRemoved()) {
                fg = true;
            }
            if (!hst->next()) {
                num++;
                if (hst->end_vertex() && hst->end_vertex()->isRemoved()) {
                    fg = true;
                }
                break;
            }
            hst = hst->next();
            if (hst == hed) break;
            if (++guard > 8192) {
                fg = true;
                break;
            }
        }
        if (fg) {
            out.append("3 0 0 0 \n");
            continue;
        }
        appendInt(out, num);
        out.push_back(' ');
#    ifdef VERTEX_COLOR
        std::map<int, int> mpcnt;
#    endif
        hst = face->proxyHalfedge();
        guard = 0;
        while (hst) {
            if (!hst->vertex()) break;
            appendInt(out, hst->vertex()->vid());
            out.push_back(' ');
#    ifdef VERTEX_COLOR
            mpcnt[hst->vertex()->groupId()]++;
#    endif
            if (!hst->next()) {
                if (hst->end_vertex()) {
                    appendInt(out, hst->end_vertex()->vid());
                    out.push_back(' ');
#    ifdef VERTEX_COLOR
                    mpcnt[hst->end_vertex()->groupId()]++;
#    endif
                }
                break;
            }
            hst = hst->next();
            if (hst == hed) break;
            if (++guard > 8192) break;
        }

#    ifdef VERTEX_COLOR

        if (mpcnt.size() == 1) {
            appendInt(out, colors[mpcnt.begin()->first][0]);
            out.push_back(' ');
            appendInt(out, colors[mpcnt.begin()->first][1]);
            out.push_back(' ');
            appendInt(out, colors[mpcnt.begin()->first][2]);
            out.push_back(' ');
        } else {
            out.append("255 255 255 ");
        }
#    endif
#    ifdef FACE_COLOR
        if (face->groupId() >= 0) {
            appendInt(out, colors[face->groupId()][0]);
            out.push_back(' ');
            appendInt(out, colors[face->groupId()][1]);
            out.push_back(' ');
            appendInt(out, colors[face->groupId()][2]);
            out.push_back(' ');
        } else {
            out.append("0 0 0 ");
        }
#    endif
        out.push_back('\n');
    }

    std::ofstream offFile(path, std::ios::binary);
    if (!offFile.is_open()) {
        std::cerr << "Error opening file: " << path << std::endl;
        return;
    }
    offFile.write(out.data(), static_cast<std::streamsize>(out.size()));
    offFile.close();
}

// void Mesh::dumpto(std::string path) {
//     int subVPoolSize = MCGAL::contextPool.getSubVPoolSize(meshId);
//     int subFPoolSize = MCGAL::contextPool.getSubFPoolSize(meshId);
//     // 重置未移除的面和其半边的状态
//     // auto newEnd = std::remove_if(faces_.begin(), faces_.end(), isFacetRemovable);
//     // faces_.resize(std::distance(faces_.begin(), newEnd));
//     // auto newVEnd = std::remove_if(vertices_.begin(), vertices_.end(), isVertexRemovable);
//     // vertices_.resize(std::distance(vertices_.begin(), newVEnd));
//     std::ofstream offFile(path);
//     if (!offFile.is_open()) {
//         std::cerr << "Error opening file: " << path << std::endl;
//         return;
//     }
//     // 计算实际的有效顶点数量
//     int validVCount = 0;
//     for (int i = 0; i < subVPoolSize; i++) {
//         MCGAL::Vertex* vertex = MCGAL::contextPool.getVertexByIndexInSubPool(meshId, i);
//         if (!vertex->isRemoved()) {
//             validVCount++;
//         }
//     }
//     // 计算实际的有效面数量
//     int validFCount = 0;
//     for (int i = 0; i < subFPoolSize; i++) {
//         MCGAL::Facet* face = MCGAL::contextPool.getFacetByIndexInSubPool(meshId, i);
//         if (!face->isRemoved()) {
//             validFCount++;
//         }
//     }
//     // write header
//     offFile << "OFF\n";
//     offFile << validVCount << " " << validFCount << " 0\n";
//     offFile << "\n";
//     // write vertex
//     int id = 0;
//     for (int i = 0; i < subVPoolSize; i++) {
//         MCGAL::Vertex* vertex = MCGAL::contextPool.getVertexByIndexInSubPool(meshId, i);
//         if (vertex->isRemoved()) {
//             continue;
//         }
//         offFile << vertex->x() << " " << vertex->y() << " " << vertex->z() << "\n";
//         vertex->setVid(id++);
//     }
//     for (int i = 0; i < subFPoolSize; i++) {
//         MCGAL::Facet* face = MCGAL::contextPool.getFacetByIndexInSubPool(meshId, i);
//         if (face->isRemoved())
//             continue;
//         int num = 0;
//         // offFile << face->vertices_.size() << " ";
//         Halfedge* hst = face->proxyHalfedge;
//         Halfedge* hed = face->proxyHalfedge;
//         do {
//             // offFile << hst->vertex->getVid() << " ";
//             num++;
//             hst = hst->next;
//         } while (hst != hed);
//         offFile << num << " ";
//         do {
//             offFile << hst->vertex->getVid() << " ";
//             hst = hst->next;
//         } while (hst != hed);
// #    ifdef COLOR
//         if (face->groupId >= 0) {
//             offFile << colors[face->groupId][0] << " " << colors[face->groupId][1] << " " << colors[face->groupId][2] << " ";
//         } else {
//             offFile << 0 << " " << 0 << " " << 0 << " ";
//         }
// #    endif
//         // else if(face->groupId == -1) {
//         //     std::cout << 1 << std::endl;
//         //     offFile << 255 << " " << 255 << " " << 255 << " ";
//         // }
//         // if (face->groupId >= 0 && face->groupId < 10) {
//         //     offFile << colors[face->level][0] << " " << colors[face->level][1] << " " << colors[face->level][2] << " ";
//         // } else {
//         //     offFile << 255 << " " << 255 << " " << 255 << " ";
//         // }
//         // if (face->level <10) {
//         //     offFile << colors[face->level][0] << " " << colors[face->level][1] << " " << colors[face->level][2] << " ";
//         // } else {
//         //     offFile << 255 << " " << 255 << " " << 255 << " ";
//         // }
//         offFile << "\n";
//     }
//     offFile.close();
// }

/**
 * Quantize the positions of all the vertices of a mesh.
 */
void Mesh::quantizeVertexPositions() {
    int i_maxCoord = 1 << i_nbQuantBits;

    for (MCGAL::Vertex* vh : vertices_) {
        Point p = vh->point();

        PointInt p_quant = floatPosToInt(p);

        /* The max value is the unique that have to to be reassigned
           because else it would get out the range. */
        p_quant = PointInt(p_quant[0] == i_maxCoord ? i_maxCoord - 1 : p_quant[0], p_quant[1] == i_maxCoord ? i_maxCoord - 1 : p_quant[1],
                           p_quant[2] == i_maxCoord ? i_maxCoord - 1 : p_quant[2]);

        Point new_p = intPosToFloat(p_quant);

        vh->setPoint(new_p);
    }
}

#endif

}  // namespace MCGAL
