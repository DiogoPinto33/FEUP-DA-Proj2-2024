#include <iostream>

#include "graphBuilder.h"
#include "interface.h"
#include "algorithms.h"

void interface::show() {
    ClearScreen;
    bool error = false;
    while (true) {
        std::cout << "+-----------------------------------------------------------+" << "\n";
        std::cout << "| Routing Algorithm for Ocean Shipping and Urban Deliveries |" << "\n";
        std::cout << "+-----------------------------------------------------------+" << "\n";
        std::cout << "| 0 - Quit                                                  |" << "\n";
        std::cout << "| 1 - Load Dataset                                          |" << "\n";
        std::cout << "| 2 - Back Tracking Algorithm                               |" << "\n";
        std::cout << "| 3 - Triangular Approximation Heuristic                    |" << "\n";
        std::cout << "| 4 - Nearest Neighbor Heuristic                            |" << "\n";
        std::cout << "| 5 - TSP in the Real World                                 |" << "\n";
        std::cout << "+-----------------------------------------------------------+" << std::endl;

        if (error)
            std::cerr << "Invalid selection, please select a value within the menu!" << std::endl;
        error = false;
        std::cout << "Indicate a value: ";
        fflush(stdout);
        int received;
        std::cin >> received;
        auto option = static_cast<option_t>(received);

        if(!loaded && option > 1) {
            std::cerr << "\n[Error] Please select a graph before running an algorithm" << std::endl;
            fflush(stderr);
            ClearScreen;
            continue;
        }

        switch (option) {
            case quit:
                return;
            case load:
                graph = Graph<int>();

                show_load();
                break;
            case backTracingAlgorithm:
                std::cout << "Selected Back Tracking Algorithm" << std::endl;
                algorithms::back_tracking_algorithm(graph);
                break;
            case triangularApproximationHeuristic:
                std::cout << "Selected Triangular Approximation" << std::endl;
                algorithms::triangular_approximation_heuristic(graph, realWorldGraph);
                break;
            case otherHeuristic:
                std::cout << "Selected Other Heuristic" << std::endl;
                algorithms::nearestNeighborHeuristic(graph, realWorldGraph);
                break;
            case RWtsp:
                std::cout << "Selected Real World TSP" << std::endl;
                std::cout << "Indicate the origin: ";
                fflush(stdout);
                int origin;
                std::cin >> origin;
                algorithms::findTSPSolution(graph, origin);
                break;

            default:
                error = true;
                ClearScreen;
                break;
        }
    }
}

void interface::show_load() {
    ClearScreen;
    bool error = false;
    while (true) {
        std::cout << "+-----------------------------------------------------------+" << "\n";
        std::cout << "|                       Load Datasets                       |" << "\n";
        std::cout << "+-----------------------------------------------------------+" << "\n";
        std::cout << "| 0 - Back                                                  |" << "\n";
        std::cout << "| 1 - Extra Fully Connected                                 |" << "\n";
        std::cout << "| 2 - Real World Graphs                                     |" << "\n";
        std::cout << "| 3 - Toy Graphs                                            |" << "\n";
        std::cout << "+-----------------------------------------------------------+" << "\n";

        if (error)
            std::cout << "Invalid selection, please select a value within the menu!\n";
        error = false;
        std::cout << "Indicate a value: ";
        fflush(stdout);
        int received;
        std::cin >> received;
        auto option = static_cast<datasets>(received);

        switch (option) {
            case 0:
                return;
            case extraFullyConnected:
                show_load_extra_fully_connected_graphs();
                return;;
            case realWorldGraphs:
                show_load_real_world_graphs();
                return;
            case toyGraphs:
                show_load_toy_graphs();
                return;
            default:
                error = true;
                ClearScreen;
                break;
        }
    }
}

void interface::show_load_toy_graphs() {
    ClearScreen;
    bool error = false;
    while (true) {
        std::cout << "+-----------------------------------------------------------+" << "\n";
        std::cout << "|                        Toy Graphs                         |" << "\n";
        std::cout << "+-----------------------------------------------------------+" << "\n";
        std::cout << "| 0 - Back                                                  |" << "\n";
        std::cout << "| 1 - Shipping                                              |" << "\n";
        std::cout << "| 2 - Stadiums                                              |" << "\n";
        std::cout << "| 3 - Tourism                                               |" << "\n";
        std::cout << "+-----------------------------------------------------------+" << "\n";

        if (error)
            std::cout << "Invalid selection, please select a value within the menu!\n";
        error = false;
        std::cout << "Indicate a value: ";
        fflush(stdout);
        int received;
        std::cin >> received;
        auto option = static_cast<datasets_toy_graphs>(received);

        switch (option) {
            case 0:
                return;
            case shipping:
            case stadiums:
            case tourism:
                graphBuilder::load_graph(&graph, toyGraphs, option);
                loaded = true;
                return;
            default:
                error = true;
                ClearScreen;
                break;
        }
    }
}

void interface::show_load_real_world_graphs() {
    ClearScreen;
    bool error = false;
    while (true) {
        std::cout << "+-----------------------------------------------------------+" << "\n";
        std::cout << "|                    Real World Graphs                      |" << "\n";
        std::cout << "+-----------------------------------------------------------+" << "\n";
        std::cout << "| 0 - Back                                                  |" << "\n";
        std::cout << "| 1 - Graph 1                                               |" << "\n";
        std::cout << "| 2 - Graph 2                                               |" << "\n";
        std::cout << "| 3 - Graph 3                                               |" << "\n";
        std::cout << "+-----------------------------------------------------------+" << "\n";

        if (error)
            std::cout << "Invalid selection, please select a value within the menu!\n";
        error = false;
        std::cout << "Indicate a value: ";
        fflush(stdout);
        int received;
        std::cin >> received;
        auto option = static_cast<datasets_real_world_graphs>(received);

        switch (option) {
            case 0:
                return;
            case graph1:

            case graph2:

            case graph3:
                graphBuilder::load_graph(&graph, realWorldGraphs, option);
                loaded = true;
                realWorldGraph = true;
                return;
            default:
                error = true;
                ClearScreen;
                break;
        }
    }
}

void interface::show_load_extra_fully_connected_graphs() {
    ClearScreen;
    bool error = false;
    while (true) {
        std::cout << "+-----------------------------------------------------------+" << "\n";
        std::cout << "|                    Real World Graphs                      |" << "\n";
        std::cout << "+-----------------------------------------------------------+" << "\n";
        std::cout << "|  0 - Back                                                 |" << "\n";
        std::cout << "|  1 - Edges 25                                             |" << "\n";
        std::cout << "|  2 - Edges 50                                             |" << "\n";
        std::cout << "|  3 - Edges 75                                             |" << "\n";
        std::cout << "|  4 - Edges 100                                            |" << "\n";
        std::cout << "|  5 - Edges 200                                            |" << "\n";
        std::cout << "|  6 - Edges 300                                            |" << "\n";
        std::cout << "|  7 - Edges 400                                            |" << "\n";
        std::cout << "|  8 - Edges 500                                            |" << "\n";
        std::cout << "|  9 - Edges 600                                            |" << "\n";
        std::cout << "| 10 - Edges 700                                            |" << "\n";
        std::cout << "| 11 - Edges 800                                            |" << "\n";
        std::cout << "| 12 - Edges 900                                            |" << "\n";
        std::cout << "+-----------------------------------------------------------+" << "\n";

        if (error)
            std::cout << "Invalid selection, please select a value within the menu!\n";
        error = false;
        std::cout << "Indicate a value: ";
        fflush(stdout);
        int received;
        std::cin >> received;
        auto option = static_cast<datasets_extra_fully_connected_graphs>(received);

        switch (option) {
            case edges_25:
            case edges_50:
            case edges_75:
            case edges_100:
            case edges_200:
            case edges_300:
            case edges_400:
            case edges_500:
            case edges_600:
            case edges_700:
            case edges_800:
            case edges_900:
                graphBuilder::load_graph(&graph, extraFullyConnected, option);
                loaded = true;
                return;
            default:
                error = true;
                ClearScreen;
                break;
        }
    }
}
