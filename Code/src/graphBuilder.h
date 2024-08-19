#ifndef GRAPHBUILDER_H
#define GRAPHBUILDER_H

#include "graph.h"
#include "interface.h"

/**
 * @brief class that auxiliaries the graph creation
 *
 */


/**
 * path variable, where to find the directory of the datasets
 */
const std::string path = "../datasets/";

/**
 * Paths for all the toy graphs
 */
const std::map<datasets_toy_graphs, std::string> paths_toy_graphs = {
        {shipping, "Toy-Graphs/shipping.csv"},
        {stadiums, "Toy-Graphs/stadiums.csv"},
        {tourism, "Toy-Graphs/tourism.csv"},
};

/**
 * Paths for all the real world graphs
 */
const std::map<datasets_real_world_graphs, std::string> paths_real_world_graphs = {
        {graph1, "Real-world Graphs/graph1/"},
        {graph2, "Real-world Graphs/graph2/"},
        {graph3, "Real-world Graphs/graph3/"},
};

/**
 * Paths for all the extra fully connected graphs
 */
const std::string path_nodes_extra_fully_connected_graphs = "Extra_Fully_Connected_Graphs/nodes.csv";

const std::map<datasets_extra_fully_connected_graphs, std::string> paths_edges_extra_fully_connected_graphs = {
        {edges_25, "Extra_Fully_Connected_Graphs/edges_25.csv"},
        {edges_50, "Extra_Fully_Connected_Graphs/edges_50.csv"},
        {edges_75, "Extra_Fully_Connected_Graphs/edges_75.csv"},
        {edges_100, "Extra_Fully_Connected_Graphs/edges_100.csv"},
        {edges_200, "Extra_Fully_Connected_Graphs/edges_200.csv"},
        {edges_300, "Extra_Fully_Connected_Graphs/edges_300.csv"},
        {edges_400, "Extra_Fully_Connected_Graphs/edges_400.csv"},
        {edges_500, "Extra_Fully_Connected_Graphs/edges_500.csv"},
        {edges_600, "Extra_Fully_Connected_Graphs/edges_600.csv"},
        {edges_700, "Extra_Fully_Connected_Graphs/edges_700.csv"},
        {edges_800, "Extra_Fully_Connected_Graphs/edges_800.csv"},
        {edges_900, "Extra_Fully_Connected_Graphs/edges_900.csv"},
};

class graphBuilder {
public:
    /**
     * Function that loads and creates a graph from a csv file
     * @param graph
     * @param type
     * @param dataset
     */
    static void load_graph(Graph<int>* graph, datasets type, int dataset);
private:

    /**
     * Given the nodes, this function builds the nodes from the graph
     * @param graph
     * @param nodes
     */
    static void build_nodes(Graph<int>* graph, std::string nodes);

    /**
     * Given the edges string, this function builds all the edges from the graph
     * @param graph
     * @param edges
     */
    static void build_edges_no_start_line(Graph<int>* graph, std::string edges);

    /**
     * Given the edges string, this function builds all the edges from the graph
     * @param graph
     * @param edges
     */
    static void build_edges(Graph<int>* graph, std::string edges);

    /**
     * Given the edges string, this function builds all the edges from the graph, like the above one, but with a type of file that has associated with the node, a name and a number
     * @param graph
     * @param edges
     */
    static void build_edges_with_label(Graph<int>* graph, std::string edges);
};

#endif //GRAPHBUILDER_H
