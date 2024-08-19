#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include "graph.h"


class algorithms {
public:

    /**
     * Algorithm that performs the backtracking on the graph passed in the parameters
     * @param graph
     */
    static void back_tracking_algorithm(Graph<int> graph);

    /**
     * Algorithm that performs the triangular approximation on the graph passed in the parameters
     * @param graph
     */
    static void triangular_approximation_heuristic(Graph<int> graph, bool realWorldGraph);

    /**
     * Algorithm that performs the nearest neighbor heuristic on the graph passed in the parameters
     * @param graph
     */
    static void nearestNeighborHeuristic(Graph<int> graph, bool realWorldGraph);

    /**
     * Algorithm that performs the find TSP solution, given an origin node chosen by the user
     * @param graph
     */
    static void findTSPSolution(Graph<int> graph, int origin);
};



#endif //ALGORITHMS_H
