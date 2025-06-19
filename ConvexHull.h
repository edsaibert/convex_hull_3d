#ifndef CONVEXHULL_H
#define CONVEXHULL_H

#include "./dcel/mesh.h"
#include "./grafo/graph.h"
#include <algorithm>
#include <random>

using namespace std;

// Define a struct to hold a point and its index
struct IndexedPoint {
    int x;
    int y;
    int z;
};

void constructNewVertice(Mesh mesh, int x, int y, int z);

class ConvexHull {
    public:
        ConvexHull() {centroid = new Vertice;};
        ~ConvexHull() {delete centroid;};
        
        void readCloud();
        void createConvexHull(Mesh& mesh);
        void loadTetrahedron(Mesh& mesh);

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

        vector<HalfEdge*> get_horizon_from_faces(Mesh& mesh, FACES& visibleFaces);
        vector<HalfEdge*> updateHorizon(vector<HalfEdge*>& horizon, Face* old, Face* newFace);
        FACES collectVisibleFaces(Mesh& mesh, IndexedPoint& pr, int pointIndex);

        BipartiteGraph* conflictList = nullptr;
        Vertice* centroid = nullptr; 
};

#endif