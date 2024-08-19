#include "graphBuilder.h"
#include <fstream>
#include <sstream>
#include <chrono>

void graphBuilder::load_graph(Graph<int>* graph, datasets type, int dataset) {

    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point end;

    switch (type) {
        case extraFullyConnected: {
            begin = std::chrono::steady_clock::now();
            auto edges = static_cast<datasets_extra_fully_connected_graphs>(dataset);
            build_nodes(graph, path + path_nodes_extra_fully_connected_graphs);
            build_edges(graph, path + paths_edges_extra_fully_connected_graphs.at(edges));

            end = std::chrono::steady_clock::now();
            std::cout  << "Extra fully connected dataset nº" << edges << " selected" << std::endl;
            break;
        }
        case realWorldGraphs: {
            begin = std::chrono::steady_clock::now();
            auto folder = static_cast<datasets_real_world_graphs>(dataset);
            build_nodes(graph, path + paths_real_world_graphs.at(folder) + "nodes.csv");
            build_edges(graph, path + paths_real_world_graphs.at(folder) + "edges.csv");

            end = std::chrono::steady_clock::now();
            std::cout << folder << " dataset selected" << std::endl;
            break;
        }
        case toyGraphs: {
            auto dt_set = static_cast<datasets_toy_graphs>(dataset);
            switch (dt_set) {
                case shipping: {
                    begin = std::chrono::steady_clock::now();
                    build_edges(graph, path + paths_toy_graphs.at(dt_set));

                    end = std::chrono::steady_clock::now();
                    std::cout << "Shipping dataset selected" << std::endl;
                    break;
                }
                case stadiums: {
                    begin = std::chrono::steady_clock::now();
                    build_edges(graph, path + paths_toy_graphs.at(dt_set));

                    end = std::chrono::steady_clock::now();
                    std::cout << "Stadiums dataset selected" << std::endl;
                    break;
                }
                case tourism: {
                    begin = std::chrono::steady_clock::now();

                    build_edges_with_label(graph, path + paths_toy_graphs.at(tourism));

                    end = std::chrono::steady_clock::now();
                    std::cout << "Tourism dataset selected" << std::endl;


                    break;
                }
            }
        }
    }


    std::cout << "Number of NODES (cities): " << graph->getNumVertex() << std::endl;

    std::cout << "Time spent: " << std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count() << " µs" <<  std::endl;
}


// Function (1)
// Load nodes type "id,longitude,latitude"
void graphBuilder::build_nodes(Graph<int>* graph, std::string nodes) {
    std::fstream fin;
    std::string row;
    std::map<int, Coordinates> coordinatesMap;
    std::map<int, std::string> cityMap;

    // Load nodes
    fin.open(nodes, std::ios::in);

    // Remove
    getline(fin, row,'\n');


    while (!fin.eof()) {
        getline(fin, row, '\n');
        std::string node_str, coords;
        int node;
        double longitude, latitude;
        std::stringstream line(row);

        getline(line, node_str, ',');
        getline(line, coords, '\n');

        if(node_str.empty() || coords.empty()) continue;

        node = std::stoi(node_str);

        // Extract longitude and latitude from coordinates string
        std::stringstream c(coords);
        c >> longitude; // Read longitude
        c.ignore(1); // Ignore the comma
        c >> latitude; // Read latitude

        Coordinates coordinates;
        coordinates.latitude = latitude;
        coordinates.longitude = longitude;

        coordinatesMap[node] = coordinates;

        graph->addVertex(node);

        cityMap.insert_or_assign(node, coords);



    }

    // Set coordinates for each node in the graph
    for (auto& pair : coordinatesMap) {
        int nodeId = pair.first;
        Coordinates coords = pair.second;
        Vertex<int>* vertex = graph->findVertex(nodeId);
        if (vertex != nullptr) {
            // Set coordinates for the vertex
            vertex->setCoords(coords);
        }
    }
    graph->setCityMap(cityMap);
}

// Function (2)
// Load edges type "src,dst,weight no start line"
void graphBuilder::build_edges_no_start_line(Graph<int>* graph, std::string edges) {
    std::fstream fin;
    std::string row;

    // Load nodes
    fin.open(edges, std::ios::in);

    while (!fin.eof()) {
        getline(fin, row , '\n');
        std::string src_str, dst_str, weight_str;
        int src, dst;
        double weight;
        std::stringstream line(row);

        getline(line, src_str, ',');
        getline(line, dst_str, ',');
        getline(line, weight_str, '\n');

        if(src_str.empty() || dst_str.empty() || weight_str.empty()) continue;

        src = std::stoi(src_str);
        dst = std::stoi(dst_str);
        weight = std::stod(weight_str);

        if (graph->findVertex(src) == nullptr)
            graph->addVertex(src);
        if (graph->findVertex(dst) == nullptr)
            graph->addVertex(dst);
        graph->addBidirectionalEdge(src, dst, weight);
    }
}

// Function (3)
// Load edges type "origem,destino,haversine_distance"
void graphBuilder::build_edges(Graph<int>* graph, std::string edges) {
    std::fstream fin;
    std::string row;

    // Load nodes
    fin.open(edges, std::ios::in);

    // Remove
    getline(fin, row,'\n');


    while (!fin.eof()) {
        getline(fin, row, '\n');
        std::string src_str, dst_str, weight_str;
        int src, dst;
        double weight;
        std::stringstream line(row);

        getline(line, src_str, ',');
        getline(line, dst_str, ',');
        getline(line, weight_str, '\n');

        if(src_str.empty() || dst_str.empty() || weight_str.empty()) continue;

        src = std::stoi(src_str);
        dst = std::stoi(dst_str);
        weight = std::stod(weight_str);

        if (graph->findVertex(src) == nullptr)
            graph->addVertex(src);
        if (graph->findVertex(dst) == nullptr)
            graph->addVertex(dst);
        if(!graph->addBidirectionalEdge(src, dst, weight)){
            std::cerr << "Vertex not known (" << src << ", " << dst << ", " << weight << ")" << std::endl;
        }
    }
}

// Function (4)
// Load edges type "origem,destino,distancia,label origem, label destino
void graphBuilder::build_edges_with_label(Graph<int>* graph, std::string edges) {
    std::fstream fin;
    std::string row;
    std::map<int, std::string> cityMap;

    // Load nodes
    fin.open(edges, std::ios::in);

    // Remove
    getline(fin, row,'\n');


    while (!fin.eof()) {
        getline(fin, row , '\n');
        std::string src_str, dst_str, weight_str, src_lbl, dst_lbl;
        int src, dst;
        double weight;
        std::stringstream line(row);

        getline(line, src_str, ',');
        getline(line, dst_str, ',');
        getline(line, weight_str, ',');
        getline(line, src_lbl, ',');
        getline(line, dst_lbl, '\n');

        if(src_str.empty() || dst_str.empty() || weight_str.empty() || src_lbl.empty() || dst_lbl.empty()) continue;

        src = std::stoi(src_str);
        dst = std::stoi(dst_str);
        weight = std::stod(weight_str);

        cityMap.insert_or_assign(src, src_lbl);
        cityMap.insert_or_assign(dst, dst_lbl);

        if (graph->findVertex(src) == nullptr)
            graph->addVertex(src);
        if (graph->findVertex(dst) == nullptr)
            graph->addVertex(dst);
        graph->addBidirectionalEdge(src, dst, weight);
    }
    graph->setCityMap(cityMap);
}


// Extra Fully Connected Graphs
// Nodes: id,longitude,latitude (1)
// Edges: src,dst,weight (2)


// Real World Graphs
// Nodes: id,longitude,latitude (1)
// Edges: origem,destino,haversine_distance (3)

// shipping.csv
// Edges: origem,destino,distancia (3)

// statiums.csv
// Edges: origem,destino,distancia (3)

// tourism.csv
// Edges: origem,destino,distancia,label origem,label destino (4)