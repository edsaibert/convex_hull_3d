#include "ConvexHull.h"

void ConvexHull::readCloud(){
    int x, y, z;
    int nVertices;
    int id = 0;
    cin >> nVertices;
    while(cin >> x >> y >> z) {
        IndexedPoint point;
        point.x = x;
        point.y = y;
        point.z = z;
        pointCloud[id] = point;
        ++id;
    }
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

double angleBetweenNormals(Face* f1, Face* f2){
    int A1, B1, C1, D1;
    int A2, B2, C2, D2;

    computePlaneEquation(f1->halfEdge->prev->origin, f1->halfEdge->origin, f1->halfEdge->next->origin, A1, B1, C1, D1);
    computePlaneEquation(f2->halfEdge->prev->origin, f2->halfEdge->origin, f2->halfEdge->next->origin, A2, B2, C2, D2);

    double dot = A1 * A2 + B1 * B2 + C1 * C2;
    double norm1 = sqrt(A1 * A1 + B1 * B1 + C1 * C1);
    double norm2 = sqrt(A2 * A2 + B2 * B2 + C2 * C2);

    double cosine = dot / (norm1 * norm2);
    if (cosine < -1.0) cosine = -1.0; 
    if (cosine > 1.0) cosine = 1.0; 
    
    double angle = acos(cosine);
    return angle * 180 / M_PI;
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



bool ConvexHull::pointIsAboveFace(Face* face, IndexedPoint& point){
    int A, B, C, D;
    Vertice* v1 = face->halfEdge->origin;
    Vertice* v2 = face->halfEdge->next->origin;
    Vertice* v3 = face->halfEdge->next->next->origin;
    computePlaneEquation(v1, v2, v3, A, B, C, D);

    int result = A * point.x + B * point.y + C * point.z + D;

    if (result <= 0) {
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
                    Node* faceNode = get_node_by_ref_id(conflictList, face->idx, false);
                    add_conflict(conflictList, node->id, faceNode->id);
                }
            }
        }
    }
}

void ConvexHull::constructConflictList(Mesh& mesh, Face* face, int idx){
    for (Node* node = conflictList->nodes; node != nullptr; node = node->next) {
        if (node->is_in_set_A && node->refId != idx) {
            // Adiciona a face ao conflito se o ponto é visível para a face
            if (pointIsAboveFace(face, pointCloud.at(node->refId))) {
                Node* faceNode = get_node_by_ref_id(conflictList, face->idx, false);
                add_conflict(conflictList, node->id, faceNode->id);
            }
        }
    }
}

void ConvexHull::addPairsToConflictList(Mesh& mesh){
    unsigned int curr_id = 0;
    for (const auto& [id, pt] : pointCloud) {
        Node* node = add_node(conflictList, curr_id++, id, true);
        if (!node) {
            cout << "Failed to add node for point " << id << endl;
            continue;
        }
    }

    for (Face* face : mesh.getFaces()){
        Node* faceNode = add_node(conflictList, curr_id++, face->idx, false); 
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

vector<HalfEdge*> ConvexHull::updateHorizon(vector<HalfEdge*>& horizon, Face* old, Face* newFace) {
    for (HalfEdge* he : horizon) {
        if (he->leftFace == old) {
            he->leftFace = newFace; 
        }
    }
    return horizon;
}

void ConvexHull::findTwinsForFace(Mesh& mesh, Face* face) {
    HalfEdge* he = face->halfEdge;
    do {
        mesh.findTwin(he);
        he = he->next;
    } while (he != face->halfEdge);
}

void ConvexHull::swapIfNegativePlane(Vertice* v1, Vertice*& v2, Vertice*& v3, Vertice*& v4) {
    // Verifica se o ponto v3 está abaixo do plano definido por v1 e v2
    int a, b, c, d;
    computePlaneEquation(v1, v2, v3, a, b, c, d);
    vector<int> toV4 = {v4->x - v1->x, v4->y - v1->y, v4->z - v1->z};
    
    if (a * toV4[0] + b * toV4[1] + c * toV4[2] + d >= 0){
        // cout << "Point v3 is above the plane defined by v1 and v2." << endl;
        // Se v3 está acima do plano, não é necessário trocar
        return;
    }
    
    // cout << "Point v3 is below the plane defined by v1 and v2. Swapping vertices." << endl;
    
    swap(v3, v4); // Troca v3 e v4 para garantir que v4 esteja acima do plano
}

HalfEdge* findDivisorHalfEdge(Mesh& mesh, Face* f1, Face* f2){
    HalfEdge* he = f1->halfEdge;
    do {
        if (he->twin && he->twin->leftFace == f2) {
            return he;
        }
        he = he->next;
    } while (he != f1->halfEdge);
    return nullptr; 
}

void ConvexHull::mergeFaces(Mesh& mesh, Face*& face1, Face* face2){
    HalfEdge* he = findDivisorHalfEdge(mesh, face1, face2);
    if (!he) 
        return;

    // Conecta as semi-arestas de face1 e face2, excluindo a semi-aresta divisor
    HalfEdge* b = he->next;
    HalfEdge* c = he->prev;
    HalfEdge* d = he->twin;    
    HalfEdge* e = d->next;
    HalfEdge* f = d->prev;

    c->next = e;
    e->prev = c;
    b->prev = f;
    f->next = b;

    HalfEdge* temp = c;
    do {
        temp->leftFace = face1; 
        temp = temp->next;
    } while (temp != c);

    face1->halfEdge = c; 

    Node* node1 = get_node_by_ref_id(conflictList, face1->idx, false);
    Node* node2 = get_node_by_ref_id(conflictList, face2->idx, false);
    for (Edge* edge = node2->edges; edge != nullptr; edge = edge->next) {
        add_conflict(conflictList, node1->id, edge->to->id);
    }
    remove_node(conflictList, node2->id);

    findTwinsForFace(mesh, face1);

    mesh.removeFaceKeepHalfEdges(face2);
    mesh.removeHalfEdge(he);
    mesh.removeHalfEdge(d);
}

void addHalfEdgesToDelete(vector<HalfEdge*>& toDelete, HalfEdge* he) {
    if (!he) return;
    HalfEdge* start = he;
    do {
        toDelete.push_back(he);
        he = he->next;
    } while (he != start);
}

void ConvexHull::loadTetrahedron(Mesh& mesh){
    // necessário encontrar quatro pontos não coplanares
    if (pointCloud.size() < 4){
        cout << "Point cloud size is too small" << endl;
        return;
    }

    int random = rand() % pointCloud.size();
    auto it = pointCloud.begin();
    std::advance(it, random);
    Vertice* v1 = mesh.createNewVertex(it->second.x, it->second.y, it->second.z);
    pointCloud.erase(it);

    Vertice* v2 = nullptr;
    Vertice* v3 = nullptr;
    Vertice* v4 = nullptr;
    
    // encontrar três pontos não coplanares
    findNotCoplanarPoints(mesh, v1, v2, v3, v4);
    if (!v2 || !v3 || !v4) {
        cout << "Could not find four non-coplanar points" << endl;
        return;
    }
        
    swapIfNegativePlane(v1, v2, v3, v4);

    centroid->x = (v1->x + v2->x + v3->x + v4->x) / 4;
    centroid->y = (v1->y + v2->y + v3->y + v4->y) / 4;
    centroid->z = (v1->z + v2->z + v3->z + v4->z) / 4;

    // Carrega o tetraedro na malha
    mesh.loadTetrahedron(v1, v2, v3, v4);
}

void ConvexHull::printConvexHull(Mesh& mesh){
    // mesh.triangulateFaces();
    return;
}


/*
Algoritmo principal de construção do Convex Hull
*/
void ConvexHull::createConvexHull(Mesh &mesh)
{
    // encontre quatro pontos não coplanares para formar um tetraedro inicial
    loadTetrahedron(mesh);

    // computar uma permutação dos pontos restantes
    // permutePointCloud();
    // inicializar o grafo G com todos as duplas visiveis (p, f), onde f é uma faceta do convexHull e t > 4
    conflictList = create_bipartite_graph();
    addPairsToConflictList(mesh);
    
    // enquanto houver pontos na nuvem de pontos, faça:
    vector<HalfEdge *> horizon = {};
    vector<pair<Face*, Face*>> toMerge;
    vector<HalfEdge*> toDelete;
    while (!pointCloud.empty())
    {
        toMerge.clear();
        toDelete.clear();
        // adicionar o ponto pr à malha
        auto it = pointCloud.begin();
        int id = it->first;
        IndexedPoint &point = it->second;
        
        Vertice *pr = mesh.createNewVertex(point.x, point.y, point.z);
        // se pr é visível à uma face f (é exterior), então:
        FACES visibleFaces = collectVisibleFaces(mesh, point, id);
        horizon = get_horizon_from_faces(mesh, visibleFaces);
        
        for (Face *face : visibleFaces)
        {
            // deletar todas as faces do conflito com pr da malha
            Node* faceNode = get_node_by_ref_id(conflictList, face->idx, false);
            remove_node(conflictList, faceNode->id);
            addHalfEdgesToDelete(toDelete, face->halfEdge);
            
            mesh.removeFace(face);
        }
        
        // para cada aresta l em L, faça:
        for (HalfEdge *he : horizon)
        {
            if (!he) continue;
            // conecte pr à aresta l, criando uma nova face triangular f'
            Vertice *v1 = he->origin;
            Vertice *v2 = he->next->origin;
            Face *newFace = mesh.createNewFace(v1, v2, pr);
            findTwinsForFace(mesh, newFace);

            // se f' é coplanar com uma vizinha face f, então faça um merge f' e f
            // se não, crie um nodo no grafo de visibilidade G para f'
            
            add_node(conflictList, conflictList->idx, newFace->idx, false);
            HalfEdge* temp = newFace->halfEdge;
            do {
                if (temp->twin && temp->twin->leftFace) {
                    Face* neighbor = temp->twin->leftFace;
                    double angle = angleBetweenNormals(newFace, neighbor);
                    if (angle == 0) {
                        toMerge.push_back({newFace, neighbor});
                    }
                }
                temp = temp->next;
            } while (temp != newFace->halfEdge);
            
            for (auto& pair : toMerge) {
                mergeFaces(mesh, pair.first, pair.second);
            }
            toMerge.clear();

            constructConflictList(mesh, newFace, id);
        }

        for (HalfEdge* he : toDelete){
            delete he;
        }

        Node* pointNode = get_node_by_ref_id(conflictList, id, true);
        remove_node(conflictList, pointNode->id);
        pointCloud.erase(it);
    }

    free_bipartite_graph(conflictList);
    delete conflictList;
    mesh.printDCEL();
    
}