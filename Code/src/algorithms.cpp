
#include "algorithms.h"



void algorithms::back_tracking_algorithm(Graph<int> graph) {

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    double val = graph.back_tracking_algorithm();

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "The cost is: " << val << std::endl;
    std::cout << "Running time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << " µs" << std::endl;
}

void algorithms::triangular_approximation_heuristic(Graph<int> graph, bool realWorldGraph) {

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    double cost;
    std::vector<Vertex<int>*> path = graph.tspTriangular(&cost, realWorldGraph);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();




    std::cout << "Path is: ";
    for(auto it = path.begin(); it < path.end(); ++it){
        std::cout << (*it)->getInfo();
        if(it+1 != path.end()) {
            std::cout << " -> ";
        }
    }
    std::cout << "\n" << "The cost is: " << cost << std::endl;

    std::cout << "Running time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << " µs" << std::endl;
}

void algorithms::nearestNeighborHeuristic(Graph<int> graph, bool realWorldGraph) {


    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    double cost;
    std::vector<Vertex<int>*> path = graph.nearestNeighborHeuristic(0, cost, realWorldGraph);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "Path is: ";
    for(auto it = path.begin(); it < path.end(); ++it){
        std::cout << (*it)->getInfo();
        if(it+1 != path.end()) {
            std::cout << " -> ";
        }

    }
    std::cout << std::endl;
    std::cout << "The cost is: " << cost << std::endl;
    std::cout << "Running time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << " µs" << std::endl;



}

void algorithms::findTSPSolution(Graph<int> graph, int origin) {


    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

    bool isConnected = graph.isConnected(origin);
    if (!isConnected) {
        std::cout << "Error: Graph is not fully connected from the origin node.\n";
        return;
    }

    // Use your heuristic algorithm to find the TSP solution
    double cost;
    std::vector<Vertex<int>*> path = graph.nearestNeighborHeuristic(origin, cost, false);

    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();

    std::cout << "Path is: ";
    for(auto it = path.begin(); it < path.end(); ++it){
        std::cout << (*it)->getInfo();
        if(it+1 != path.end()) {
            std::cout << " -> ";
        }

    }
    std::cout << std::endl;
    std::cout << "The cost is: " << cost << std::endl;
    std::cout << "Running time: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << " µs" << std::endl;


}
