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

/*
    Encontra quatro pontos não coplanares na nuvem de pontos
*/
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

void ConvexHull::permutePointCloud(){
    // Embaralha a nuvem de pontos
    std::random_device rd;
    std::mt19937 g(rd());
    shuffle(pointCloud.begin(), pointCloud.end(), g);
}

vector<int> subtractVectors(const vector<int>& v1, const vector<int>& v2) {
    return {v1[0] - v2[0], v1[1] - v2[1], v1[2] - v2[2]};
}

vector<int> crossProduct(const vector<int>& v1, const vector<int>& v2) {
    return {
        v1[1] * v2[2] - v1[2] * v2[1],
        v1[2] * v2[0] - v1[0] * v2[2],
        v1[0] * v2[1] - v1[1] * v2[0]
    };
}

void computePlaneEquation(Face* face, int& A, int& B, int& C, int& D) {
    Vertice* v1 = face->halfEdge->origin;
    Vertice* v2 = face->halfEdge->next->origin;
    Vertice* v3 = face->halfEdge->prev->origin;

    // A equação do plano é Ax + By + Cz + D = 0
    vector<int> ab = subtractVectors({v2->x, v2->y, v2->z}, {v1->x, v1->y, v1->z});
    vector<int> ac = subtractVectors({v3->x, v3->y, v3->z}, {v1->x, v1->y, v1->z});
    vector<int> normal = crossProduct(ab, ac);

    A = normal[0];
    B = normal[1];
    C = normal[2];

    D = -(A * v1->x + B * v1->y + C * v1->z);
}

bool ConvexHull::pointIsAboveFace(Face* face, vector<int>& point){
    int A, B, C, D;
    computePlaneEquation(face, A, B, C, D);

    int result = A * point[0] + B * point[1] + C * point[2] + D;

    if (result >= 0) {
        return true;
    }
    return false;
}

void ConvexHull::constructConflictList(Mesh& mesh){
    for (Face* face : mesh.getFaces()){
        for (Node* node = conflictList->nodes; node != nullptr; node = node->next) {
            if (node->is_in_set_A) {
                // Adiciona a face ao conflito se o ponto é visível para a face
                if (pointIsAboveFace(face, pointCloud[node->refId])) {
                    cout << "Point " << pointCloud[node->refId][0] << ", "
                         << pointCloud[node->refId][1] << ", "
                         << pointCloud[node->refId][2] << " is visible to face " << face->idx << endl;

                    Node* faceNode = get_node_by_ref_id(conflictList, face->idx, false);
                    Node* pointNode = get_node_by_ref_id(conflictList, node->id, true);
                    add_conflict(conflictList, pointNode->id, faceNode->id);
                }
            }
        }
    }
}

void ConvexHull::addPairsToConflictList(Mesh& mesh){
    unsigned int pointCloudSize = pointCloud.size();
    for (unsigned int i = 0; i < pointCloudSize; ++i) {
        Node* node = add_node(conflictList, i, i, true);

        if (!node) {
            cout << "Failed to add node for point " << i << endl;
            continue;
        }
    }

    for (Face* face : mesh.getFaces()){
        Node* faceNode = add_node(conflictList, face->idx + pointCloudSize, face->idx, false);
        if (!faceNode) {
            cout << "Failed to add node for face " << face->idx << endl;
            continue;
        }
    }

    constructConflictList(mesh); 
}

/*
    Algoritmo principal de construção do Convex Hull
*/
void ConvexHull::createConvexHull(Mesh& mesh){
    // encontre quatro pontos não coplanares para formar um tetraedro inicial
    loadTetrahedron(mesh);
    // computar uma permutação dos pontos restantes
    permutePointCloud();
    // inicializar o grafo G com todos as duplas visiveis (p, f), onde f é uma faceta do convexHull e t > 4
    conflictList = create_bipartite_graph();
    addPairsToConflictList(mesh);
    
    // enquanto houver pontos na nuvem de pontos, faça:
    // while (pointCloud.size() > 0){
        // adicionar o ponto pr à malha 
        // se pr é visível à uma face f (é exterior), então:
            // deletar todas as faces do conflito com pr da malha
            // caminhar pela margem da área visível de pr, e criar uma lista das L arestas (em ordem)
            // para cada aresta l em L, faça:
                // conecte pr à aresta l, criando uma nova face triangular f'
                // se f' é coplanar com uma vizinha face f, então faça um merge f' e f 
                // se não, crie um nodo no grafo de visibilidade G para f' 
            // delete o nodo correspondente a pr e os nodos correspondentes ao grafo de visibilidade G com pr
            // atualize a malha e o grafo de visibilidade conforme necessário
    // }
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