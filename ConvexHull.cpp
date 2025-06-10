#include "ConvexHull.h"

void constructNewVertice(Mesh mesh, int x, int y, int z){
    Vertice* v = mesh.createNewVertex(x, y, z, nVertices);

    for (Face* f : mesh.faces){
        if (f->HalfEdge == NULL) break;

        // Se o ponto está à esquerda da face atual, será necessário criar mais três faces
        if (mesh.isPointLeft(f, v)) {
            constructPartitionedFace(f, v);
            continue;
        }

        // Se o ponto é colinar, a solução é simples, basta criar uma nova semi-aresta e garantir que ela é inserida na face
        HalfEdge* he = mesh.isPointColinearWithFace(f, v);
        if (he){
            constructNewColinearHalfEdge(f, v, he);    
            continue;
        }
    }
}
