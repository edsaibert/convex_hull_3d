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
        ConvexHull() {};
        ~ConvexHull() {};
        
        void readCloud();
        void createConvexHull(Mesh& mesh);
        void loadTetrahedron(Mesh& mesh);
        void addToConvexHull(Mesh& mesh, Vertice* v);
        void constructPartitionedFace(Face* f, Vertice* v);

    private:
        unordered_map<int, IndexedPoint> pointCloud;
        void findNotCoplanarPoints(Mesh &mesh, Vertice* v1, Vertice*& v2, Vertice*& v3, Vertice*& v4);
        void addPairsToConflictList(Mesh& mesh);
        void constructConflictList(Mesh& mesh);
        bool pointIsAboveFace(Face* face, IndexedPoint& point);
        void swapIfNegativePlane(Vertice* v1, Vertice* v2, Vertice*& v3, Vertice*& v4);
        void permutePointCloud();
        void findTwinsForFace(Mesh& mesh, Face* face);
        void merge(Mesh& mesh, Face* newFace, HalfEdge* he);

        vector<HalfEdge*> get_horizon_from_faces(Mesh& mesh, FACES& visibleFaces);
        FACES collectVisibleFaces(Mesh& mesh, IndexedPoint& pr, int pointIndex);

        BipartiteGraph* conflictList = nullptr;
};

#endif