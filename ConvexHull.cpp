#include "ConvexHull.h"

// void constructNewVertice(Mesh mesh, int x, int y, int z){
//     Vertice* v = mesh.createNewVertex(x, y, z, nVertices);

//     for (Face* f : mesh.faces){
//         if (f->HalfEdge == NULL) break;

//         // Se o ponto está à esquerda da face atual, será necessário criar mais três faces
//         if (mesh.isPointLeft(f, v)) {
//             constructPartitionedFace(f, v);
//             continue;
//         }

//         // Se o ponto é colinar, a solução é simples, basta criar uma nova semi-aresta e garantir que ela é inserida na face
//         HalfEdge* he = mesh.isPointColinearWithFace(f, v);
//         if (he){
//             constructNewColinearHalfEdge(f, v, he);    
//             continue;
//         }
//     }
// }

void ConvexHull::readCloud(){
    int x, y, z;
    while(cin >> x >> y >> z) {
        pointCloud.push_back({x, y, z});
    }
}

bool isCoplanar(Vertice* v1, Vertice* v2, Vertice* v3, vector<int>& point){
    // Verifica se o ponto está no plano definido pelos três vértices
    // A equação do plano é: Ax + By + Cz + D = 0
    // Onde A, B, C são os coeficientes normais do plano e D é a constante
    int A = (v2->y - v1->y) * (v3->z - v1->z) - (v2->z - v1->z) * (v3->y - v1->y);
    int B = (v2->z - v1->z) * (v3->x - v1->x) - (v2->x - v1->x) * (v3->z - v1->z);
    int C = (v2->x - v1->x) * (v3->y - v1->y) - (v2->y - v1->y) * (v3->x - v1->x);
    int D = -(A * v1->x + B * v1->y + C * v1->z);
    int result = A * point[0] + B * point[1] + C * point[2] + D;

    // Se o resultado for zero, o ponto está no plano
    if (result == 0) {
        return true; // Ponto está no plano
    }
    return false; // Ponto não está no plano
}

bool isColinear(Vertice* v1, Vertice* v2, vector<int>& point){
    int x1 = v1->x, y1 = v1->y, z1 = v1->z;
    int x2 = v2->x, y2 = v2->y, z2 = v2->z;
    int x = point[0], y = point[1], z = point[2];

    int dx1 = x2 - x1, dy1 = y2 - y1, dz1 = z2 - z1;
    int dx2 = x - x1, dy2 = y - y1, dz2 = z - z1;

    int cx = dy1 * dz2 - dz1 * dy2;
    int cy = dz1 * dx2 - dx1 * dz2;
    int cz = dx1 * dy2 - dy1 * dx2;

    return (cx == 0 && cy == 0 && cz == 0);
}

bool isTheSamePoint(Vertice* v, vector<int>& point){
    // Verifica se o ponto é o mesmo que o vértice
    if (v->x == point[0] && v->y == point[1] && v->z == point[2]) {
        return true; // É o mesmo ponto
    }
    return false; // Não é o mesmo ponto
}

void ConvexHull::findNotCoplanarPoints(Mesh& mesh, Vertice* v1, Vertice*& v2, Vertice*& v3, Vertice*& v4){
    unsigned int i = 0;
    while (i < pointCloud.size()) {
        if (v3 != nullptr){
            if (isCoplanar(v1, v2, v3, pointCloud[i])) {
                ++i;
                continue;
            }
            v4 = mesh.createNewVertex(pointCloud[i][0], pointCloud[i][1], pointCloud[i][2]);
            pointCloud.erase(pointCloud.begin() + i);
            break;
        } else if (v2 != nullptr) {
            if (isColinear(v1, v2, pointCloud[i]) || isTheSamePoint(v1, pointCloud[i]) || isTheSamePoint(v2, pointCloud[i])){
                ++i;
                continue; 
            }
            v3 = mesh.createNewVertex(pointCloud[i][0], pointCloud[i][1], pointCloud[i][2]);
            pointCloud.erase(pointCloud.begin() + i);
        } else {
            if (isTheSamePoint(v1, pointCloud[i])){
                ++i;
                continue; 
            }
            v2 = mesh.createNewVertex(pointCloud[i][0], pointCloud[i][1], pointCloud[i][2]);
            pointCloud.erase(pointCloud.begin() + i);
        }
    }
}

float signedVolume(Vertice* v1, Vertice* v2, Vertice* v3, Vertice* v4) {
    // Compute vectors
    float x1 = v2->x - v1->x;
    float y1 = v2->y - v1->y;
    float z1 = v2->z - v1->z;

    float x2 = v3->x - v1->x;
    float y2 = v3->y - v1->y;
    float z2 = v3->z - v1->z;

    float x3 = v4->x - v1->x;
    float y3 = v4->y - v1->y;
    float z3 = v4->z - v1->z;

    // Compute determinant / scalar triple product
    float volume = (x1 * (y2 * z3 - y3 * z2)
                  - y1 * (x2 * z3 - x3 * z2)
                  + z1 * (x2 * y3 - x3 * y2)) / 6.0f;

    return volume;
}

void ConvexHull::loadTetrahedron(Mesh& mesh){
    // necessário encontrar quatro pontos não coplanares
    if (pointCloud.size() < 4){
        cout << "Point cloud size is too small" << endl;
        return;
    }

    // escolher um ponto aleatório
    int random = rand() % pointCloud.size();
    Vertice* v1 = mesh.createNewVertex(pointCloud[random][0], pointCloud[random][1], pointCloud[random][2]);
    pointCloud.erase(pointCloud.begin() + random);

    Vertice* v2 = nullptr;
    Vertice* v3 = nullptr;
    Vertice* v4 = nullptr;

    // encontrar três pontos não coplanares
    findNotCoplanarPoints(mesh, v1, v2, v3, v4);
    if (!v2 || !v3 || !v4) {
        cout << "Could not find four non-coplanar points" << endl;
        return;
    }

    printf("Loaded tetrahedron with vertices: (%d, %d, %d), (%d, %d, %d), (%d, %d, %d), (%d, %d, %d)\n",
           v1->x, v1->y, v1->z,
           v2->x, v2->y, v2->z,
           v3->x, v3->y, v3->z,
           v4->x, v4->y, v4->z);

    // Carrega o tetraedro na malha
    mesh.loadTetrahedron(v1, v2, v3, v4);
}