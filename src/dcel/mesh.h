#ifndef mesh_h
#define mesh_h

/*
    mesh.h

    Este arquivo contém a implementação da classe Mesh, que representa uma malha poligonal
    e fornece métodos para carregar a malha, imprimir a representação DCEL (Doubly Connected
    Edge List), construir semi-arestas e arestas, e verificar a validade da topologia da malha.
    A classe também contém métodos auxiliares para criar novos vértices, faces e semi-arestas,
    além de verificar se um vértice já existe na malha.

*/

#include "sweepLine.h"

class Mesh {
    public: 
        Mesh() {};
        ~Mesh() {
            for (auto v : vertices) delete v;
            for (auto f : faces) delete f;
            for (auto he : halfEdges) delete he;
        }
        void loadTetrahedron(Vertice* v1, Vertice* v2, Vertice* v3, Vertice* v4);
        void printDCEL();
        void removeFace(Face* f);
        void removeHalfEdge(HalfEdge* he);
        void removeFaceKeepHalfEdges(Face* f);
        void findTwin(HalfEdge* he);

        Vertice* createNewVertex(int x, int y, int z);
        Face* createNewFace(Vertice* v1, Vertice* v2, Vertice* v3);
        Face* createNewFace(Vertice* v1, Vertice* v2, Vertice* v3, Vertice* v4);
        void printHalfEdge(HalfEdge* he);
        void unmergeFace(Face* quad);
        void triangulateFaces();

        Face* findFace(int idx);
        VERTICES getVertices() { return vertices; }
        FACES getFaces() { return faces; }
        HALF_EDGES getHalfEdges() { return halfEdges; }

        
    private:
        unsigned int nVertices = 0;
        unsigned int nFaces = 0;
        unsigned int nHalfEdges = 0;
        
        bool faceDoesExist(int idx);
        bool checkIfVertexExists(int x, int y, int z);
        void addVertexToMesh(int x, int y, int z);
        void defineFace(Vertice *v1, Vertice *v2, Vertice *v3, int idx);
        void constructExternalFace(Vertice* v1, Vertice* v2, Vertice* v3, Vertice* v4);

        bool isOpen();
        bool isSubdivPlanar();
        bool isOverlapped();
        bool autoIntersect(Face* f);

        Face* createNewFace(int idx);
        HalfEdge* createHalfEdgeNode(Vertice* origin, int faceIdx, int idx);

        void findNext(HalfEdge* he);
        void findPrev(HalfEdge* he);

        VERTICES vertices;                
        FACES faces;
        HALF_EDGES halfEdges;
        FACE_VERTICES faceVertices; 
        // EDGES edges;                   
		// unordered_map<int, vector<Edge*>> edgesMap;
};

#endif
