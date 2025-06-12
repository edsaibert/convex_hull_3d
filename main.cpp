#include "ConvexHull.h"

int main() {
    Mesh mesh;
    ConvexHull convexHull;

    convexHull.readCloud();
    // convexHull.loadTetrahedron(mesh);
    convexHull.createConvexHull(mesh);

    return 0;
}