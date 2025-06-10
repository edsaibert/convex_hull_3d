#include "ConvexHull.h"

int main() {
    Mesh mesh;

    // Cria os vértices do tetraedro
    Vertice* v1 = mesh.createNewVertex(0, 0, 0);
    Vertice* v2 = mesh.createNewVertex(1, 0, 0);
    Vertice* v3 = mesh.createNewVertex(0, 1, 0);
    Vertice* v4 = mesh.createNewVertex(0, 0, 1);

    mesh.loadTetrahedron(v1, v2, v3, v4);

    return 0;
}