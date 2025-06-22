#ifndef GRAPH_H
#define GRAPH_H
#include <string>
#include <vector>
#include <iostream>

using namespace std;

typedef struct Node {
    int id;
    int refId;  // Reference ID for the node, used to identify the node in the original context
    bool is_in_set_A; // T if in set A, F if in set B
    struct Node* next; // Pointer to the next node in the adjacency list
    struct Edge* edges; // Pointer to the first edge in the adjacency list
} Node;

typedef struct Edge {
    struct Node* to; // Pointer to the destination node
    struct Edge* next; // Pointer to the next edge in the adjacency list
} Edge;


typedef struct BipartiteGraph{
    Node* nodes;
    size_t num_nodes;
    size_t idx;
} BipartiteGraph;

// Creates an empty bipartite graph
BipartiteGraph* create_bipartite_graph();

// Free the memory allocated for the bipartite graph
void free_bipartite_graph(BipartiteGraph* graph);

/*
    Adds a node to the bipartite graph.
    Returns a pointer to the newly created node, or NULL if id is already in use.
*/
Node* add_node(BipartiteGraph* graph, int id, int refId, bool is_in_set_A);

/*
    Adds an edge between two nodes
    Returns 0 on success, -1 if the nodes do not exist, or -2 if the edge already exists.
*/
int add_conflict(BipartiteGraph* graph, int from_id, int to_id);

/* Removes a node from the bipartite graph */
void remove_node(BipartiteGraph* graph, int id);

/*
    Removes an edge between two nodes.
*/
void remove_conflict(BipartiteGraph* graph, int from_id, int to_id);

// Prints the bipartite graph
void print_bipartite_graph(const BipartiteGraph* graph);

///// DEBUG FUNCTIONS /////
bool is_node_in_graph(const BipartiteGraph* graph, int id);
Node* get_node_by_id(const BipartiteGraph* graph, int id);
Node* get_node_by_ref_id(const BipartiteGraph* graph, int refId, bool is_in_set_A);
void print_node(const Node* node);
void print_edge(const Edge* edge);
void print_edges(const Edge* edge);
void print_nodes(const BipartiteGraph* graph);
void print_bipartite_sets(const BipartiteGraph* graph);
bool is_edge_in_graph(const BipartiteGraph* graph, int from_id, int to_id);

#endif // GRAPH_H