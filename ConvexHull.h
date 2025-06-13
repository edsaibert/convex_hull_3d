#ifndef CONVEXHULL_H
#define CONVEXHULL_H

#include "./dcel/mesh.h"
#include "./grafo/graph.h"
#include <algorithm>
#include <random>

using namespace std;
using PointCloud = vector<vector<int>>;
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
        PointCloud pointCloud;
        void findNotCoplanarPoints(Mesh &mesh, Vertice* v1, Vertice*& v2, Vertice*& v3, Vertice*& v4);
        void addPairsToConflictList(Mesh& mesh);
        void constructConflictList(Mesh& mesh);
        bool pointIsAboveFace(Face* face, vector<int>& point);
        void swapIfNegativePlane(Vertice* v1, Vertice* v2, Vertice*& v3, Vertice*& v4);
        void permutePointCloud();

        BipartiteGraph* conflictList = nullptr;
};

#endif