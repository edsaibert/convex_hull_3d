#include "graph.h"

// Creates an empty bipartite graph
BipartiteGraph* create_bipartite_graph(){
    BipartiteGraph* graph = new BipartiteGraph;
    graph->nodes = nullptr;
    graph->num_nodes = 0;
    graph->idx = 0;
    return graph;
}

void free_bipartite_graph(BipartiteGraph* graph){
    if (!graph) {
        return; // Nothing to free
    }
    Node* current_node = graph->nodes;
    while (current_node != nullptr) {
        Edge* current_edge = current_node->edges;
        while (current_edge != nullptr) {
            Edge* temp_edge = current_edge;
            current_edge = current_edge->next;
            delete temp_edge; // Free the edge
        }
        Node* temp_node = current_node;
        current_node = current_node->next;
        delete temp_node; // Free the node
    }
}

Node* add_node(BipartiteGraph* graph, int id, int refId, bool is_in_set_A){
    if (get_node_by_id(graph, id) != nullptr){
        return nullptr;
    }

    Node* new_node = new Node;
    new_node->id = id;
    new_node->refId = refId;
    new_node->is_in_set_A = is_in_set_A;
    new_node->next = nullptr;
    new_node->edges = nullptr;
    graph->num_nodes++;
    graph->idx++;

    // Insert the new node at the beginning of the linked list
    if (graph->nodes == nullptr) {
        graph->nodes = new_node;
    } else {
        new_node->next = graph->nodes;
        graph->nodes = new_node;
    }
    return new_node;
}

int add_conflict(BipartiteGraph* graph, int from_id, int to_id){
    
    if(!graph || graph->num_nodes == 0){
        return -1; // Graph is empty
    }
    
    Node* from_node = get_node_by_id(graph, from_id);
    Node* to_node = get_node_by_id(graph, to_id);
    
    if (!from_node || !to_node) {
        return -1; // One or both nodes do not exist
    }

    // Check if the edge already exists
    Edge* current_edge = from_node->edges;
    
    while (current_edge != nullptr) {
        if (current_edge->to->id == to_id) {
            return -2; // Edge already exists
        }
        current_edge = current_edge->next;
    }

    // Create a new edge    
    Edge* new_edge = new Edge;
    new_edge->to = to_node;
    new_edge->next = nullptr;
    // Insert the new edge at the beginning of the linked list
    if (from_node->edges == nullptr) {
        from_node->edges = new_edge;
    } else {
        new_edge->next = from_node->edges;
        from_node->edges = new_edge;
    }
    return 0; // Edge added successfully
}

///// DEBUG FUNCTIONS /////

bool is_node_in_graph(const BipartiteGraph* graph, int id){
    if(get_node_by_id(graph, id) != nullptr){
        return true;
    }
    return false;
}

Node* get_node_by_ref_id(const BipartiteGraph* graph, int refId, bool is_in_set_A){
    Node* current = graph->nodes;
    while (current != nullptr) {
        if (current->refId == refId && current->is_in_set_A == is_in_set_A) {
            return current;
        }
        current = current->next;
    }
    return nullptr; 
}

Node* get_node_by_id(const BipartiteGraph* graph, int id){
    Node* current = graph->nodes;
    while (current != nullptr) {
        if (current->id == id) {
            return current;
        }
        current = current->next;
    }
    return nullptr; 
}

void remove_node(BipartiteGraph* graph, int id){
    if (!graph || graph->num_nodes == 0) {
        return; // Graph is empty
    }

    // remove all edges that conflict with the node being removed
    Node* current = graph->nodes;
    while (current != nullptr) {
        if (current->id != id) {
            remove_conflict(graph, current->id, id);
        }
        current = current->next;
    }

    // removing the node itself
    current = graph->nodes;
    Node* previous = nullptr;

    while (current != nullptr) {
        if (current->id == id) {
            if (previous == nullptr) {
                // Node is the first in the list
                graph->nodes = current->next;
            } else {
                previous->next = current->next;
            }
            // Free the edges associated with the node
            Edge* edge = current->edges;
            while (edge != nullptr) {
                Edge* temp_edge = edge;
                edge = edge->next;
                delete temp_edge;
            }
            delete current; 
            graph->num_nodes--;
            return; 
        }
        previous = current;
        current = current->next;
    }
}

void remove_conflict(BipartiteGraph* graph, int from_id, int to_id){
    if (!graph || graph->num_nodes == 0) {
        return; // Graph is empty
    }
    
    Node* from_node = get_node_by_id(graph, from_id);
    if (!from_node) {
        return; // From node does not exist
    }
    
    Edge* current_edge = from_node->edges;
    Edge* previous_edge = nullptr;

    while (current_edge != nullptr) {
        if (current_edge->to->id == to_id) {
            // Found the edge to remove
            if (previous_edge == nullptr) {
                // Edge is the first in the list
                from_node->edges = current_edge->next;
            } else {
                previous_edge->next = current_edge->next;
            }
            delete current_edge; // Free the edge
            return; // Edge removed successfully
        }
        previous_edge = current_edge;
        current_edge = current_edge->next;
    }
}

bool is_edge_in_graph(const BipartiteGraph* graph, int from_id, int to_id){
    Node* from_node = get_node_by_id(graph, from_id);
    if (!from_node) {
        return false; // From node does not exist
    }
    
    Edge* current_edge = from_node->edges;
    while (current_edge != nullptr) {
        if (current_edge->to->id == to_id) {
            return true; // Edge exists
        }
        current_edge = current_edge->next;
    }
    return false; // Edge does not exist
}

///// PRINT FUNCTIONS /////
void print_bipartite_graph(const BipartiteGraph* graph){
    if(!graph || graph->num_nodes == 0){
        std::cout << "------------------------" << std::endl;
        std::cout << "The bipartite graph is empty." << std::endl;
        std::cout << "------------------------" << std::endl;
        return;
    }
    std::cout << "------------------------" << std::endl;
    std::cout << "Bipartite Graph:" << std::endl;
    print_bipartite_sets(graph);
    std::cout << "Total nodes: " << graph->num_nodes << std::endl;
    std::cout << "------------------------" << std::endl;
    Node* current = graph->nodes;
    while (current != nullptr) {
        print_node(current);
        current = current->next;
    }
    std::cout << "------------------------" << std::endl;
    std::cout << "End of Bipartite Graph." << std::endl;
}

void print_node(const Node* node){
    if (!node) {
        std::cout << "Node is null." << std::endl;
        return;
    }
    std::cout << "Node ID: " << node->id << ", Set: " << (node->is_in_set_A ? "A" : "B") << std::endl;
    if (node->edges) {
        print_edges(node->edges);
    } else {
        std::cout << "No edges for this node." << std::endl;
    }
}

void print_edge(const Edge* edge){
    if (!edge) {
        std::cout << "Edge is null." << std::endl;
        return;
    }
    std::cout << "Edge to Node ID: " << edge->to->id << std::endl;
    if (edge->next) {
        print_edge(edge->next);
    }
}

void print_edges(const Edge* edge){
    if(!edge) {
        std::cout << "No edges." << std::endl;
        return;
    }
    std::cout << "Edges: " << std::endl;
    const Edge* current = edge;
    while (current != nullptr) {
        print_edge(current);
        current = current->next;
    }
}

void print_nodes(const BipartiteGraph* graph){
    if (!graph || graph->num_nodes == 0) {
        std::cout << "No nodes in the bipartite graph." << std::endl;
        return;
    }
    std::cout << "Nodes in the bipartite graph:" << std::endl;
    Node* current = graph->nodes;
    while (current != nullptr) {
        std::cout << "Node ID: " << current->id << ", Set: " << (current->is_in_set_A ? "A" : "B") << std::endl;
        current = current->next;
    }
}

void print_bipartite_sets(const BipartiteGraph* graph){
    if (!graph || graph->num_nodes == 0) {
        std::cout << "No nodes in the bipartite graph." << std::endl;
        return;
    }
    std::cout << "Bipartite Sets:" << std::endl;
    Node* current = graph->nodes;
    while (current != nullptr) {
        std::cout << "Node ID: " << current->id << ", Set: " << (current->is_in_set_A ? "A" : "B") << std::endl;
        current = current->next;
    }
}
