#ifndef CONVEXHULL_H
#define CONVEXHULL_H

#include "./dcel/mesh.h"


using namespace std;
using PointCloud = vector<vector<int>>;
void constructNewVertice(Mesh mesh, int x, int y, int z);

class ConvexHull {
    public:
        ConvexHull() {};
        ~ConvexHull() {};
        
        void readCloud();
        void loadTetrahedron(Mesh& mesh);
        void addToConvexHull(Mesh& mesh, Vertice* v);
        void constructPartitionedFace(Face* f, Vertice* v);

    private:
        PointCloud pointCloud;
        void findNotCoplanarPoints(Mesh &mesh, Vertice* v1, Vertice*& v2, Vertice*& v3, Vertice*& v4);
};

#endif