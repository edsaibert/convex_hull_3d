#ifndef CONVEXHULL_H
#define CONVEXHULL_H

#include "../dcel/mesh.h"
#include "../grafo/graph.h"
#include <algorithm>
#include <random>
#include <array>

using namespace std;

struct IndexedPoint {
    int x;
    int y;
    int z;
};

/* Estrutura que representa um triângulo na triangulação final do fecho convexo */
struct TriangleNode {
    int idx;
    int v1;
    int v2;
    int v3;
    int t1;
    int t2;
    int t3;
};

using TRIANGLES = vector<TriangleNode*>;

class ConvexHull {
    public:
        ConvexHull() {centroid = new Vertice;};
        ~ConvexHull() {delete centroid;};
        
        void readCloud();
        void createConvexHull(Mesh& mesh);
        void loadTetrahedron(Mesh& mesh);
        void printConvexHull(Mesh& mesh);

    private:
        unordered_map<int, IndexedPoint> pointCloud;
        void findNotCoplanarPoints(Mesh &mesh, Vertice* v1, Vertice*& v2, Vertice*& v3, Vertice*& v4);
        void addPairsToConflictList(Mesh& mesh);
        void constructConflictList(Mesh& mesh);
        void constructConflictList(Mesh& mesh, Face* face, int idx);
        bool pointIsAboveFace(Face* face, IndexedPoint& point);
        void swapIfNegativePlane(Vertice* v1, Vertice*& v2, Vertice*& v3, Vertice*& v4);
        void permutePointCloud();
        void findTwinsForFace(Mesh& mesh, Face* face);
        void mergeFaces(Mesh& mesh, Face*& face1, Face* face2);

        TriangleNode* createTriangleNode(int v1, int v2, int v3, int idx);
        void setNeighbors(unordered_map<Face*, TriangleNode*>& faceToTriangleMap);
        void setTriangulation(Mesh& mesh);
        void freeTriangulation(TRIANGLES& triangulation);

        vector<HalfEdge*> get_horizon_from_faces(Mesh& mesh, FACES& visibleFaces);
        vector<HalfEdge*> updateHorizon(vector<HalfEdge*>& horizon, Face* old, Face* newFace);
        FACES collectVisibleFaces(Mesh& mesh, IndexedPoint& pr, int pointIndex);

        BipartiteGraph* conflictList = nullptr;
        Vertice* centroid = nullptr; 
        TRIANGLES triangulation;
};

#endif