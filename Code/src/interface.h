#ifndef INTERFACE_H
#define INTERFACE_H

#define ClearScreen system("clear")
#include <string>

#include "graph.h"

enum option_t {
    quit,
    load,
    backTracingAlgorithm,
    triangularApproximationHeuristic,
    otherHeuristic,
    RWtsp,
};

enum datasets {
    extraFullyConnected = 1,
    realWorldGraphs = 2,
    toyGraphs = 3,
};

enum datasets_toy_graphs {
    shipping = 1,
    stadiums = 2,
    tourism = 3,
};

enum datasets_real_world_graphs {
    graph1 = 1,
    graph2 = 2,
    graph3 = 3,
};

enum datasets_extra_fully_connected_graphs {
    edges_25 = 1,
    edges_50 = 2,
    edges_75 = 3,
    edges_100 = 4,
    edges_200 = 5,
    edges_300 = 6,
    edges_400 = 7,
    edges_500 = 8,
    edges_600 = 9,
    edges_700 = 10,
    edges_800 = 11,
    edges_900 = 12,
};

class interface {
public:
    /**
     * Show function, do display the menu interface
     */
    void show();
private:
    /**
     * Graph where we are working at the moment
     */
    Graph<int> graph;

    /**
     * Variable representing if we are facing a real world graph or not
     */
    bool realWorldGraph = false;

    /**
     * Variable that indicates if we have a graph loaded or not
     */
    bool loaded = false;

    /**
     * Loading a graph interface menu
     */
    void show_load();
    /**
     * Loading a toy graph interface menu
     */
    void show_load_toy_graphs();

    /**
     * Loading a Real World graph interface menu
     */
    void show_load_real_world_graphs();

    /**
     * Loading an Extra Fully Connected Graph interface menu
     */
    void show_load_extra_fully_connected_graphs();
};



#endif //INTERFACE_H
