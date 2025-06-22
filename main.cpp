#include "ConvexHull.h"

int main() {
    Mesh mesh;
    ConvexHull convexHull;

    convexHull.readCloud();
    convexHull.createConvexHull(mesh);
    convexHull.printConvexHull(mesh);

    return 0;
}