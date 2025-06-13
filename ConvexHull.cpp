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
    int id = 0;
    while(cin >> x >> y >> z) {
        IndexedPoint point;
        point.x = x;
        point.y = y;
        point.z = z;
        pointCloud[id] = point;
        ++id;
    }
}

bool isCoplanar(Vertice* v1, Vertice* v2, Vertice* v3, IndexedPoint& point){
    // Verifica se o ponto está no plano definido pelos três vértices
    // A equação do plano é: Ax + By + Cz + D = 0
    // Onde A, B, C são os coeficientes normais do plano e D é a constante
    int A = (v2->y - v1->y) * (v3->z - v1->z) - (v2->z - v1->z) * (v3->y - v1->y);
    int B = (v2->z - v1->z) * (v3->x - v1->x) - (v2->x - v1->x) * (v3->z - v1->z);
    int C = (v2->x - v1->x) * (v3->y - v1->y) - (v2->y - v1->y) * (v3->x - v1->x);
    int D = -(A * v1->x + B * v1->y + C * v1->z);
    int result = A * point.x + B * point.y + C * point.z + D;

    // Se o resultado for zero, o ponto está no plano
    if (result == 0) {
        return true; // Ponto está no plano
    }
    return false; // Ponto não está no plano
}

bool isColinear(Vertice* v1, Vertice* v2, IndexedPoint& point){
    int x1 = v1->x, y1 = v1->y, z1 = v1->z;
    int x2 = v2->x, y2 = v2->y, z2 = v2->z;
    int x = point.x, y = point.y, z = point.z;

    int dx1 = x2 - x1, dy1 = y2 - y1, dz1 = z2 - z1;
    int dx2 = x - x1, dy2 = y - y1, dz2 = z - z1;

    int cx = dy1 * dz2 - dz1 * dy2;
    int cy = dz1 * dx2 - dx1 * dz2;
    int cz = dx1 * dy2 - dy1 * dx2;

    return (cx == 0 && cy == 0 && cz == 0);
}

bool isTheSamePoint(Vertice* v, IndexedPoint& point){
    // Verifica se o ponto é o mesmo que o vértice
    if (v->x == point.x && v->y == point.y && v->z == point.z) {
        return true; // É o mesmo ponto
    }
    return false; // Não é o mesmo ponto
}

/*
    Encontra quatro pontos não coplanares na nuvem de pontos
*/
void ConvexHull::findNotCoplanarPoints(Mesh& mesh, Vertice* v1, Vertice*& v2, Vertice*& v3, Vertice*& v4){
    for (auto it = pointCloud.begin(); it != pointCloud.end(); ) {
        IndexedPoint& point = it->second;
        if (v3 != nullptr){
            if (isCoplanar(v1, v2, v3, point)) {
                ++it;
                continue;
            }
            v4 = mesh.createNewVertex(point.x, point.y, point.z);
            it = pointCloud.erase(it);
            break;
        } else if (v2 != nullptr) {
            if (isColinear(v1, v2, point) || isTheSamePoint(v1, point) || isTheSamePoint(v2, point)){
                ++it;
                continue; 
            }
            v3 = mesh.createNewVertex(point.x, point.y, point.z);
            it = pointCloud.erase(it);
        } else {
            if (isTheSamePoint(v1, point)){
                ++it;
                continue; 
            }
            v2 = mesh.createNewVertex(point.x, point.y, point.z);
            it = pointCloud.erase(it);
        }
    }
}

// void ConvexHull::permutePointCloud(){
//     // Embaralha a nuvem de pontos
//     std::random_device rd;
//     std::mt19937 g(rd());
//     shuffle(pointCloud.begin(), pointCloud.end(), g);
// }

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

void computePlaneEquation(Vertice* v1, Vertice* v2, Vertice* v3, int& A, int& B, int& C, int& D) {
    // A equação do plano é Ax + By + Cz + D = 0
    vector<int> ab = subtractVectors({v2->x, v2->y, v2->z}, {v1->x, v1->y, v1->z});
    vector<int> ac = subtractVectors({v3->x, v3->y, v3->z}, {v1->x, v1->y, v1->z});
    vector<int> normal = crossProduct(ab, ac);

    A = normal[0];
    B = normal[1];
    C = normal[2];

    D = -(A * v1->x + B * v1->y + C * v1->z);
}

bool ConvexHull::pointIsAboveFace(Face* face, IndexedPoint& point){
    int A, B, C, D;
    Vertice* v1 = face->halfEdge->origin;
    Vertice* v2 = face->halfEdge->next->origin;
    Vertice* v3 = face->halfEdge->next->next->origin;
    computePlaneEquation(v1, v2, v3, A, B, C, D);

    int result = A * point.x + B * point.y + C * point.z + D;

    if (result > 0) {
        return true;
    }
    return false;
}

void ConvexHull::constructConflictList(Mesh& mesh){
    for (Face* face : mesh.getFaces()){
        for (Node* node = conflictList->nodes; node != nullptr; node = node->next) {
            if (node->is_in_set_A) {
                // Adiciona a face ao conflito se o ponto é visível para a face
                if (pointIsAboveFace(face, pointCloud.at(node->refId))) {
                    cout << "Point " << pointCloud.at(node->refId).x << ", "
                         << pointCloud.at(node->refId).y << ", "
                         << pointCloud.at(node->refId).z << " is visible to face " << face->idx << endl;

                    Node* faceNode = get_node_by_ref_id(conflictList, face->idx, false);
                    Node* pointNode = get_node_by_ref_id(conflictList, node->id, true);
                    add_conflict(conflictList, pointNode->id, faceNode->id);
                }
            }
        }
    }
}

void ConvexHull::addPairsToConflictList(Mesh& mesh){
    for (const auto& [id, pt] : pointCloud) {
        Node* node = add_node(conflictList, id, id, true);
        if (!node) {
            cout << "Failed to add node for point " << id << endl;
            continue;
        }
    }

    for (Face* face : mesh.getFaces()){
        Node* faceNode = add_node(conflictList, face->idx + pointCloud.size(), face->idx, false); 
        if (!faceNode) {
            cout << "Failed to add node for face " << face->idx << endl;
            continue;
        }
    }

    constructConflictList(mesh); 
}

FACES ConvexHull::collectVisibleFaces(Mesh& mesh, IndexedPoint& pr, int pointIndex){
    FACES visibleFaces;
    for (Face* face : mesh.getFaces()){
        int pointIdx = get_node_by_ref_id(conflictList, pointIndex, true)->id;
        int faceIdx = get_node_by_ref_id(conflictList, face->idx, false)->id;
        if (is_edge_in_graph(conflictList, pointIdx, faceIdx)){
            remove_conflict(conflictList, pointIdx, faceIdx);
            visibleFaces.push_back(face);
        }
    }

    return visibleFaces;
}

vector<HalfEdge*> ConvexHull::get_horizon_from_faces(Mesh& mesh, FACES& visibleFaces){
    vector<HalfEdge*> horizon;
    std::unordered_set<Face*> visibleSet(visibleFaces.begin(), visibleFaces.end());

    for (Face* face : visibleFaces) {
        HalfEdge* start = face->halfEdge;
        HalfEdge* he = start;
        do {
            // Only one side of the edge should be visible
            if (he->twin && visibleSet.find(he->twin->leftFace) == visibleSet.end()) {
                // This edge is on the horizon (face is visible, twin's face is not)
                horizon.push_back(he);
            }
            he = he->next;
        } while (he != start);
    }
    return horizon;
}

/*
    Algoritmo principal de construção do Convex Hull
*/
void ConvexHull::createConvexHull(Mesh& mesh){
    // encontre quatro pontos não coplanares para formar um tetraedro inicial
    loadTetrahedron(mesh);

    for (HalfEdge* he : mesh.getHalfEdges()) {
        mesh.printHalfEdge(he);
    }
    cout << "____________________________________________________________" << endl;
    // computar uma permutação dos pontos restantes
    // permutePointCloud();
    // inicializar o grafo G com todos as duplas visiveis (p, f), onde f é uma faceta do convexHull e t > 4
    conflictList = create_bipartite_graph();
    addPairsToConflictList(mesh);
    
    // enquanto houver pontos na nuvem de pontos, faça:
    vector<int> facesIdx = {};
    vector<HalfEdge*> horizon = {};
    while (!pointCloud.empty()) {
        // adicionar o ponto pr à malha 
        auto it = pointCloud.begin();
        int id = it->first;
        IndexedPoint& point = it->second;

        Vertice* pr = mesh.createNewVertex(point.x, point.y, point.z);
        // se pr é visível à uma face f (é exterior), então:
        FACES visibleFaces = collectVisibleFaces(mesh, point, id);
        horizon = get_horizon_from_faces(mesh, visibleFaces);

        for (Face* face : visibleFaces) {
            // deletar todas as faces do conflito com pr da malha
            facesIdx.push_back(face->idx);
            mesh.removeFace(face);
        }

        // para cada aresta l em L, faça:
        for (HalfEdge* he : horizon) {
            // conecte pr à aresta l, criando uma nova face triangular f'
            Face* newFace = mesh.createNewFace(pr, he);
            // se f' é coplanar com uma vizinha face f, então faça um merge f' e f 
            // se não, crie um nodo no grafo de visibilidade G para f' 
        }
        
        for (HalfEdge* he : mesh.getHalfEdges()){
            mesh.printHalfEdge(he);
        }
        break;

        // delete o nodo correspondente a pr e os nodos correspondentes ao grafo de visibilidade G com pr
        // atualize a malha e o grafo de visibilidade conforme necessário
        pointCloud.erase(it);
    }
}

void ConvexHull::swapIfNegativePlane(Vertice* v1, Vertice* v2, Vertice*& v3, Vertice*& v4) {
    // Verifica se o ponto v3 está abaixo do plano definido por v1 e v2
    int a, b, c, d;
    computePlaneEquation(v1, v2, v3, a, b, c, d);
    vector<int> toV4 = {v4->x - v1->x, v4->y - v1->y, v4->z - v1->z};
    
    if (a * toV4[0] + b * toV4[1] + c * toV4[2] + d >= 0){
        cout << "Point v3 is above the plane defined by v1 and v2." << endl;
        // Se v3 está acima do plano, não é necessário trocar
        return;
    }

    cout << "Point v3 is below the plane defined by v1 and v2. Swapping vertices." << endl;

    swap(v3, v4); // Troca v3 e v4 para garantir que v4 esteja acima do plano
}

void ConvexHull::loadTetrahedron(Mesh& mesh){
    // necessário encontrar quatro pontos não coplanares
    if (pointCloud.size() < 4){
        cout << "Point cloud size is too small" << endl;
        return;
    }

    // escolher um ponto aleatório
    int random = rand() % pointCloud.size();
    Vertice* v1 = mesh.createNewVertex(pointCloud[random].x, pointCloud[random].y, pointCloud[random].z);
    pointCloud.erase(random);

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

    swapIfNegativePlane(v1, v2, v3, v4);
    // Carrega o tetraedro na malha
    mesh.loadTetrahedron(v1, v2, v3, v4);
}