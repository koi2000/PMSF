#include "Vertex.h"
#include "Halfedge.h"
#include "Facet.h"
#include <iostream>

int main() {
    std::cout << "Vertex size: " << sizeof(MCGAL::Vertex) << std::endl;
    std::cout << "Halfedge size: " << sizeof(MCGAL::Halfedge) << std::endl;
    std::cout << "Facet size: " << sizeof(MCGAL::Facet) << std::endl;

    std::cout << "Vertex size: " << sizeof(MCGAL::Vertex*) << std::endl;
    std::cout << "Halfedge size: " << sizeof(MCGAL::Halfedge*) << std::endl;
    std::cout << "Facet size: " << sizeof(MCGAL::Facet*) << std::endl;
    return 0;
}