#include <iostream>
#include "graph.h"



int main() {
    // Create a bipartite graph
    BipartiteGraph* graph = create_bipartite_graph();
    if (graph){
        cout << "Deu boa a criação do grafo bipartido!" << endl;
    }
    else {
        cout << "Erro ao criar o grafo bipartido!" << endl;
        return 1; // Exit if graph creation fails
    }

    print_bipartite_graph(graph);


    // Add nodes to the graph
    add_node(graph, 1, true);  // Node in set A
    add_node(graph, 2, false); // Node in set B
    add_node(graph, 3, true);  // Node in set A

    // Check if nodes were added successfully
    for(int i = 1; i <= 3; ++i) {
        if (is_node_in_graph(graph, i)) {
            cout << "Deu boa o nodo " << i << " foi colocado." << endl;
        } else {
            cout << "Deu ruim " << i << "." << endl;
        }
    }

    print_bipartite_graph(graph);


    // Add conflicts (edges) between nodes
    add_conflict(graph, 1, 2);
    add_conflict(graph, 2, 3);

    for(int i = 1; i <= 3; ++i) {
        for(int j = 1; j <= 3; ++j) {
            if (i != j) {
                if (is_edge_in_graph(graph, i, j)) {
                    cout << "Aresta entre " << i << " e " << j << " existe (deu boa)." << endl;
                }
            }
        }
    }

    print_bipartite_graph(graph);

    // Free the memory allocated for the bipartite graph
    free_bipartite_graph(graph);

    cout << "Grafo bipartido liberado com sucesso!" << endl;
    return 0;
}