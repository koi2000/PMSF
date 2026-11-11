#include "PPMC/compress/PPMCSegmentationSerializeOperator.h"

#include "BufferUtils.h"
#include "Facet.h"
#include "Halfedge.h"
#include "Mesh.h"
#include "Vertex.h"
#include <set>
#include <vector>

void PPMCSegmentationSerializeOperator::serializeBaseMeshWithSeed(
    std::shared_ptr<MCGAL::Mesh> mesh,
    std::vector<MCGAL::Halfedge*> seeds) {
    mesh->garbage_collection();
    writeInt(buffer_, dataOffset_, static_cast<int>(mesh->size_of_vertices()));
    writeInt(buffer_, dataOffset_, static_cast<int>(mesh->size_of_facets()));

    int seedVertexIndex = 0;
    int groupIndex = 0;
    int vertexIndex = static_cast<int>(seeds.size()) * 2;
    std::vector<int> groupCounts;
    std::vector<std::vector<MCGAL::Vertex*>> vertexGroups;
    std::set<MCGAL::Vertex*> seedVertices;

    for (MCGAL::Halfedge* seed : seeds) {
        seedVertices.insert(seed->vertex());
        seedVertices.insert(seed->end_vertex());
    }
    for (MCGAL::Halfedge* seed : seeds) {
        seed->vertex()->setVid(seedVertexIndex++);
        seed->end_vertex()->setVid(seedVertexIndex++);
        std::vector<MCGAL::Vertex*> vertices;
        calculateGroupSize(mesh, vertices, seed, seedVertices, groupIndex);
        ++groupIndex;
        groupCounts.push_back(static_cast<int>(vertices.size()));
        vertexGroups.push_back(vertices);
    }

    for (int count : groupCounts) {
        writeInt(buffer_, dataOffset_, count);
    }
    for (MCGAL::Halfedge* seed : seeds) {
        writePoint(buffer_, dataOffset_, seed->vertex());
        writePoint(buffer_, dataOffset_, seed->end_vertex());
    }
    for (auto& group : vertexGroups) {
        for (MCGAL::Vertex* vertex : group) {
            vertex->setVid(vertexIndex++);
            writePoint(buffer_, dataOffset_, vertex);
        }
    }
    for (MCGAL::Facet* face : mesh->faces()) {
        if (face->isRemoved()) {
            continue;
        }
        MCGAL::Halfedge* start = face->proxyHalfedge();
        MCGAL::Halfedge* it = start;
        int degree = 0;
        do {
            ++degree;
            it = it->next();
        } while (it != start);
        writeInt(buffer_, dataOffset_, degree);
        it = start;
        do {
            writeInt(buffer_, dataOffset_, it->vertex()->vid());
            it = it->next();
        } while (it != start);
    }
}
