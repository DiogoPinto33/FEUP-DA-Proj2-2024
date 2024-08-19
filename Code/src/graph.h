#ifndef GRAPH_H
#define GRAPH_H


#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <limits>
#include <unordered_set>

template <class T>
class Edge;

#define INF std::numeric_limits<int>::max()
#define DOUBLE_INF std::numeric_limits<double>::max()

/************************* Vertex  **************************/

/**
 * Struct that indicates the coordinates in Latitude and Longitude of a Vertex
 */

struct Coordinates {
    double latitude;
    double longitude;
};

template <class T>
class Vertex {
public:
    Vertex(T in);
    bool operator<(Vertex<T> & vertex) const; // // required by mutable_priority_queue

    /**
     *
     * @return the vertex number identifier
     */
    T getInfo() const;

    /**
     *
     * @return a vector of the adjacent edges of a vertex
     */
    std::vector<Edge<T> *> getAdj() const;

    /**
     *
     * @return if a vertex is set visited or not
     */
    bool isVisited() const;

    /**
     *
     * @return if a Vertex is in state of processing or not
     */
    bool isProcessing() const;

    /**
     *
     * @return the vertex indegree
     */
    unsigned int getIndegree() const;

    /**
     *
     * @return the vertex distance
     */
    double getDist() const;

    /**
     *
     * @return the vertex path
     */
    Edge<T> *getPath() const;

    /**
     * Sets a vertex as visited
     * @param visited
     */
    void setVisited(bool visited);

    /**
     * Sets a vertex with processing
     * @param processing
     */
    void setProcesssing(bool processing);

    /**
     * Sets a vertex with indegree
     * @param indegree
     */
    void setIndegree(unsigned int indegree);

    /**
     * Sets a vertex distance
     * @param dist
     */
    void setDist(double dist);

    /**
     * Sets a vertex path
     * @param path
     */
    void setPath(Edge<T> *path);

    /**
     * Adds an edge to the vertex, with a destination Vertex and a weight w
     * @param dest
     * @param w
     * @return
     */
    Edge<T> * addEdge(Vertex<T> *dest, double w);

    /**
     * Removes a vertex edge
     * @param in
     * @return
     */
    bool removeEdge(T in);

    /**
     * Removes vertex outgoing edges
     */
    void removeOutgoingEdges();

    /**
     * Gets the vertex coordinates
     * @return
     */
    const Coordinates &getCoords() const;

    /**
     * Sets the vertex coordinates
     * @param coords
     */
    void setCoords(const Coordinates &coords);

    std::vector<Edge<T> *> getIncoming() const;
    void setInfo(T info);

protected:

    /**
     * info node
     */
    T info;
    /**
     * outgoing edges
     */
    std::vector<Edge<T> *> adj;
    /**
     * If a vertex is visited or not
     * used by DFS, BFS, Prim ...
     */
    bool visited = false;
    /**
     * If a vertex is processing or not
     * used by isDAG (in addition to the visited attribute)
     */
    bool processing = false;

    /**
     * if a vertex is indegree or not
     * used in topsort
     */
    unsigned int indegree;

    /**
     * Vertex distance
     * default value is 0
     */
    double dist = 0;

    /**
     * Vertex path
     * Default value is nullptr indicating that the path doesn t exists
     */
    Edge<T> *path = nullptr;

    /**
     * Vertex Latitude and Longitude coordinates
     */
    Coordinates coords;

protected:

    /**
     * incoming edges
     */
    std::vector<Edge<T> *> incoming;

    /**
     * required by mutable_priority_queue and UFDS
     */
    int queueIndex = 0;

    /**
     * Deletes an edge from the vertex
     * @param edge
     */
    void deleteEdge(Edge<T> *edge);
};

/********************** Edge  ****************************/

template <class T>
class Edge {
public:
    /**
     * Edge constructor, receives an origin Vertex, a destination Vertex and a weight
     * @param orig Origin vertex
     * @param dest Destination vertex
     * @param w weight
     */
    Edge(Vertex<T> *orig, Vertex<T> *dest, double w);

    /**
     * Get edge destination
     * @return edge destination
     */
    Vertex<T> * getDest() const;

    /**
     * Get edge weight
     * @return edge weight
     */
    double getWeight() const;

    /**
     * Is edge selected?
     * @return if edge is selected
     */
    bool isSelected() const;

    /**
     * Get edge origin
     * @return origin edge
     */
    Vertex<T> * getOrig() const;

    /**
     * Get reversed edge
     * @return reversed edge
     */
    Edge<T> *getReverse() const;

    /**
     * Sets edge as selected
     * @param selected bool indicating if edge is selected or not
     */
    void setSelected(bool selected);

    /**
     * Sets edge as reverse
     * @param reverse bool indicating if edge is reversed or not
     */
    void setReverse(Edge<T> *reverse);


    void setFlow(double flow);
    double getFlow() const;
protected:
    /**
     * Destination vertex
     */
    Vertex<T> * dest;

    /**
     * edge weight, can also be used for capacity
     */
    double weight;

    /**
     * Selected auxiliry field
     */
    bool selected = false;

    // used for bidirectional edges
    /**
     * Edge origin
     */
    Vertex<T> *orig;
    /**
     * reverse edge
     */
    Edge<T> *reverse = nullptr;

    /**
     * Edge flow, for flow-related problems
     */
    double flow;
};

/********************** Graph  ****************************/



template <class T>
class Graph {
public:
    /**
     * Graph constructor
     */
    ~Graph();

    /**
    * Auxiliary function to find a vertex with a given the content.
    */
    Vertex<T> *findVertex(const T &in) const;

    /**
     *  Adds a vertex with a given content or info (in) to a graph (this).
     *  Returns true if successful, and false if a vertex with that content already exists.
     */
    bool addVertex(const T &in);

    /**
     * Gets the distance between two vertexes
     * @param src vertex
     * @param dst destination vertex
     * @param useHaversine if not connect, do you want to make a connection using the vertexes coordinates?
     * @return the distances between the two vertexes
     */
    double getDistFromTo(Vertex<T>* src, Vertex<T>* dst, bool useHaversine) const;


    /**
     * Adds an edge to a graph (this), given the contents of the source and
     * destination vertices and the edge weight (w).
     * Returns true if successful, and false if the source or destination vertex does not exist.
     */
    bool addEdge(const T &sourc, const T &dest, double w);

    /**
     * Removes an edge from the graph
     * @param source vertex
     * @param dest vertex
     * @return true if successful and false if not
     */
    bool removeEdge(const T &source, const T &dest);

    /**
     * Adds a bidirectional edge
     * @param sourc source vertex
     * @param dest destination vertex
     * @param w edge weight
     * @return true if successful
     */
    bool addBidirectionalEdge(const T &sourc, const T &dest, double w);

    /**
     *
     * @return the number of vertexes in the graph
     */
    int getNumVertex() const;


    /**
    * Auxiliary function that visits a vertex (v) and its adjacent, recursively.
    * Updates a parameter with the list of visited node contents.
     * @param res
     * @param v
     * @return
    */

    void dfsVisit(Vertex<T> *v,  std::vector<T> & res) const;

    /**
     * Function that determines if a graph is connected in TSP from an origin node
     * Runs a type of BFS to examine it
     * @param origin origin node
     * @return true if he graph if connected and false otherwise
     */
    bool isConnected (T origin);

    /**
     * Auxiliary function that visits a vertex (v) and its adjacent, recursively.
     * Returns false (not acyclic) if an edge to a vertex in the stack is found.
     * @param v
     * @return
     */
    bool dfsIsDAG(Vertex<T> *v) const;


    /**
     * Sets the graph city map
     * We are working with cities and transportation, so it os useful to have a city map
     * @param cityMap
     */
    void setCityMap(std::map<int, std::string> cityMap);

    /**
     * Algorithm that performs backtracking to get to the TSP solution
     * @return
     */
    double back_tracking_algorithm();

    /**
     * Algorithm that performs a triangular approximation heuristic to get to the TSP solution
     * @param cost path cost
     * @param realWorldGraph are we using a real world graph, so we can decide if we use the Haversine code to get the distances
     * @return a vector with the TSP path solution
     */
    std::vector<Vertex<T>*> tspTriangular(double* cost, bool realWorldGraph);


    /**
     * Algorithm taht performs a nearest neighbour heuristic to get to the TSP solution
     * @param origin starting node
     * @param cost path cost
     * @param useHaversine use haversine distance calculation
     * @return  a vector with the TSP path solution
     */
    std::vector<Vertex<T>*> nearestNeighborHeuristic(int origin, double &cost, bool useHaversine) const;

    /**
     * Calculates the haversine distance between two locations
     * @param lat1 latitude location 1
     * @param lon1 longitude location 1
     * @param lat2 latitude location 2
     * @param lon2 longitude location 2
     * @return the haversine distance
     */
    static double haversineDistance(double lat1, double lon1, double lat2, double lon2);


    bool removeVertex(const T &in);
    std::vector<Vertex<T> *> getVertexSet() const;
    std:: vector<T> dfs() const;
    std:: vector<T> dfs(const T & source) const;
    std::vector<T> bfs(const T & source) const;
    bool isDAG() const;
    std::vector<T> topsort() const;
    std::map<int, std::string> getCityMap();


protected:

    /**
     * Vertex set
     */
    std::vector<Vertex<T> *> vertexSet;

    /**
     * dist matrix for Floyd-Warshall
     */
    double ** distMatrix = nullptr;

    /**
     * path matrix for Floyd-Warshall
     */
    int **pathMatrix = nullptr;

    /**
     * City map of the graph
     */
    std::map<int, std::string> cityMap;




    /**
     * Backtracking recursive algorithm
     * @param currPos
     * @param n
     * @param count
     * @param cost
     * @param ans
     * @param path
     * @param bestPath
     */
    void back_tracking_algorithm_rec(Vertex<T>* currPos, int n, int count, double cost, double* ans, std::vector<T>& path, std::vector<T>& bestPath);

    /**
     *Performs a depth-first search on the graph, starting from the given vertex,
     *    and returns the path of vertices in the triangular structure.
     * @param v A pointer to the starting vertex for the DFS.
     * @return A vector of pointers to vertices representing the path traversed in the DFS.
     */
    std::vector<Vertex<T>*> dfsTriangular(Vertex<T>* v);

    /**
     * prims algorithm
     */
    void prim_algorithm();


    /**
     * Finds the index of the vertex with a given content.
     */
    int findVertexIdx(const T &in) const;
};

/**
 * Deletes graph matrix
 * @param m
 * @param n
 */
void deleteMatrix(int **m, int n);

/**
 * Deletes graph matrix
 * @param m
 * @param n
 */
void deleteMatrix(double **m, int n);

/************************* Vertex  **************************/

template <class T>
Vertex<T>::Vertex(T in): info(in) {}
/*
 * Auxiliary function to add an outgoing edge to a vertex (this),
 * with a given destination vertex (d) and edge weight (w).
 */
template <class T>
Edge<T> * Vertex<T>::addEdge(Vertex<T> *d, double w) {
    auto newEdge = new Edge<T>(this, d, w);
    adj.push_back(newEdge);
    d->incoming.push_back(newEdge);
    return newEdge;
}

/*
 * Auxiliary function to remove an outgoing edge (with a given destination (d))
 * from a vertex (this).
 * Returns true if successful, and false if such edge does not exist.
 */
template <class T>
bool Vertex<T>::removeEdge(T in) {
    bool removedEdge = false;
    auto it = adj.begin();
    while (it != adj.end()) {
        Edge<T> *edge = *it;
        Vertex<T> *dest = edge->getDest();
        if (dest->getInfo() == in) {
            it = adj.erase(it);
            deleteEdge(edge);
            removedEdge = true; // allows for multiple edges to connect the same pair of vertices (multigraph)
        }
        else {
            it++;
        }
    }
    return removedEdge;
}

/*
 * Auxiliary function to remove an outgoing edge of a vertex.
 */
template <class T>
void Vertex<T>::removeOutgoingEdges() {
    auto it = adj.begin();
    while (it != adj.end()) {
        Edge<T> *edge = *it;
        it = adj.erase(it);
        deleteEdge(edge);
    }
}

template <class T>
bool Vertex<T>::operator<(Vertex<T> & vertex) const {
    return this->dist < vertex.dist;
}

template <class T>
T Vertex<T>::getInfo() const {
    return this->info;
}

template <class T>
std::vector<Edge<T>*> Vertex<T>::getAdj() const {
    return this->adj;
}

template <class T>
bool Vertex<T>::isVisited() const {
    return this->visited;
}

template <class T>
bool Vertex<T>::isProcessing() const {
    return this->processing;
}

template <class T>
unsigned int Vertex<T>::getIndegree() const {
    return this->indegree;
}

template <class T>
double Vertex<T>::getDist() const {
    return this->dist;
}

template <class T>
struct VertexComparator {
    bool operator()(T v1, T v2) {
        return v1->getDist() > v2->getDist();
    }
};


template<class T>
double Graph<T>::getDistFromTo(Vertex<T>* src, Vertex<T>* dst, bool useHaversine) const {
    if(src->getInfo() == dst->getInfo()) return 0;

    std::priority_queue<Vertex<T>*, std::vector<Vertex<T>*>, VertexComparator<Vertex<T>*>> q;

    for(auto v : vertexSet) {
        v->setDist(DOUBLE_INF);
        v->setPath(nullptr);
        q.push(v);
    }
    src->setDist(0);

    while (!q.empty()) {
        Vertex<T>* u = q.top();
        q.pop();

        for(Edge<T>* e : u->getAdj()) {
            auto w = u->getDist() + e->getWeight();
            if(w < e->getDest()->getDist()) {
                e->getDest()->setDist(w);
                e->getDest()->setPath(e);
            }
        }
    }

    if(dst->getDist() == DOUBLE_INF && useHaversine) {
        return haversineDistance(src->getCoords().latitude, src->getCoords().longitude, dst->getCoords().latitude, dst->getCoords().longitude);
    }


    double cost = 0;
    Vertex<T>* curr = dst;
    while(curr != nullptr && curr->getPath() != nullptr) {
        auto e = curr->getPath();
        cost += e->getWeight();
        curr = e->getOrig();
    }
    return cost;
}


template <class T>
Edge<T> *Vertex<T>::getPath() const {
    return this->path;
}

template <class T>
std::vector<Edge<T> *> Vertex<T>::getIncoming() const {
    return this->incoming;
}

template <class T>
void Vertex<T>::setInfo(T in) {
    this->info = in;
}

template <class T>
void Vertex<T>::setVisited(bool visited) {
    this->visited = visited;
}

template <class T>
void Vertex<T>::setProcesssing(bool processing) {
    this->processing = processing;
}

template <class T>
void Vertex<T>::setIndegree(unsigned int indegree) {
    this->indegree = indegree;
}

template <class T>
void Vertex<T>::setDist(double dist) {
    this->dist = dist;
}

template <class T>
void Vertex<T>::setPath(Edge<T> *path) {
    this->path = path;
}

template <class T>
void Vertex<T>::deleteEdge(Edge<T> *edge) {
    Vertex<T> *dest = edge->getDest();
    // Remove the corresponding edge from the incoming list
    auto it = dest->incoming.begin();
    while (it != dest->incoming.end()) {
        if ((*it)->getOrig()->getInfo() == info) {
            it = dest->incoming.erase(it);
        }
        else {
            it++;
        }
    }
    delete edge;
}

template<class T>
const Coordinates &Vertex<T>::getCoords() const {
    return coords;
}

template<class T>
void Vertex<T>::setCoords(const Coordinates &coords) {
    Vertex::coords = coords;
}

/********************** Edge  ****************************/

template <class T>
Edge<T>::Edge(Vertex<T> *orig, Vertex<T> *dest, double w): orig(orig), dest(dest), weight(w) {}

template <class T>
Vertex<T> * Edge<T>::getDest() const {
    return this->dest;
}

template <class T>
double Edge<T>::getWeight() const {
    return this->weight;
}

template <class T>
Vertex<T> * Edge<T>::getOrig() const {
    return this->orig;
}

template <class T>
Edge<T> *Edge<T>::getReverse() const {
    return this->reverse;
}

template <class T>
bool Edge<T>::isSelected() const {
    return this->selected;
}

template <class T>
double Edge<T>::getFlow() const {
    return flow;
}

template <class T>
void Edge<T>::setSelected(bool selected) {
    this->selected = selected;
}

template <class T>
void Edge<T>::setReverse(Edge<T> *reverse) {
    this->reverse = reverse;
}

template <class T>
void Edge<T>::setFlow(double flow) {
    this->flow = flow;
}

/********************** Graph  ****************************/

template <class T>
int Graph<T>::getNumVertex() const {
    return vertexSet.size();
}

template <class T>
std::vector<Vertex<T> *> Graph<T>::getVertexSet() const {
    return vertexSet;
}

/*
 * Auxiliary function to find a vertex with a given content.
 */
template <class T>
Vertex<T> * Graph<T>::findVertex(const T &in) const {
    for (auto v : vertexSet){
        if (v->getInfo() == in){
            return v;
        }
    }

    return nullptr;
}

/*
 * Finds the index of the vertex with a given content.
 */
template <class T>
int Graph<T>::findVertexIdx(const T &in) const {
    for (unsigned i = 0; i < vertexSet.size(); i++)
        if (vertexSet[i]->getInfo() == in)
            return i;
    return -1;
}
/*
 *  Adds a vertex with a given content or info (in) to a graph (this).
 *  Returns true if successful, and false if a vertex with that content already exists.
 */
template <class T>
bool Graph<T>::addVertex(const T &in) {
    if (findVertex(in) != nullptr)
        return false;
    vertexSet.push_back(new Vertex<T>(in));
    return true;
}

/*
 *  Removes a vertex with a given content (in) from a graph (this), and
 *  all outgoing and incoming edges.
 *  Returns true if successful, and false if such vertex does not exist.
 */
template <class T>
bool Graph<T>::removeVertex(const T &in) {
    for (auto it = vertexSet.begin(); it != vertexSet.end(); it++) {
        if ((*it)->getInfo() == in) {
            auto v = *it;
            v->removeOutgoingEdges();
            for (auto u : vertexSet) {
                u->removeEdge(v->getInfo());
            }
            vertexSet.erase(it);
            delete v;
            return true;
        }
    }
    return false;
}

/*
 * Adds an edge to a graph (this), given the contents of the source and
 * destination vertices and the edge weight (w).
 * Returns true if successful, and false if the source or destination vertex does not exist.
 */
template <class T>
bool Graph<T>::addEdge(const T &sourc, const T &dest, double w) {

    auto v1 = findVertex(sourc);
    auto v2 = findVertex(dest);

    if (v1 == nullptr || v2 == nullptr)
        return false;
    v1->addEdge(v2, w);
    return true;
}

/*
 * Removes an edge from a graph (this).
 * The edge is identified by the source (sourc) and destination (dest) contents.
 * Returns true if successful, and false if such edge does not exist.
 */
template <class T>
bool Graph<T>::removeEdge(const T &sourc, const T &dest) {
    Vertex<T> * srcVertex = findVertex(sourc);
    if (srcVertex == nullptr) {
        return false;
    }
    return srcVertex->removeEdge(dest);
}

template <class T>
bool Graph<T>::addBidirectionalEdge(const T &sourc, const T &dest, double w) {
    auto v1 = findVertex(sourc);
    auto v2 = findVertex(dest);
    if (v1 == nullptr || v2 == nullptr)
        return false;
    auto e1 = v1->addEdge(v2, w);
    auto e2 = v2->addEdge(v1, w);
    e1->setReverse(e2);
    e2->setReverse(e1);
    return true;
}




/****************** DFS ********************/

/*
 * Performs a depth-first search (dfs) traversal in a graph (this).
 * Returns a vector with the contents of the vertices by dfs order.
 */
template <class T>
std::vector<T> Graph<T>::dfs() const {
    std::vector<T> res;
    for (auto v : vertexSet)
        v->setVisited(false);
    for (auto v : vertexSet)
        if (!v->isVisited())
            dfsVisit(v, res);
    return res;
}

/*
 * Performs a depth-first search (dfs) in a graph (this) from the source node.
 * Returns a vector with the contents of the vertices by dfs order.
 */
template <class T>
std::vector<T> Graph<T>::dfs(const T & source) const {
    std::vector<int> res;
    // Get the source vertex
    auto s = findVertex(source);
    if (s == nullptr) {
        return res;
    }
    // Set that no vertex has been visited yet
    for (auto v : vertexSet) {
        v->setVisited(false);
    }
    // Perform the actual DFS using recursion
    dfsVisit(s, res);

    return res;
}

/**
 * Auxiliary function that visits a vertex (v) and its adjacent, recursively.
 * Updates a parameter with the list of visited node contents.
 */
template <class T>
void Graph<T>::dfsVisit(Vertex<T> *v, std::vector<T> & res) const {
    v->setVisited(true);
    res.push_back(v->getInfo());
    for (auto & e : v->getAdj()) {
        auto w = e->getDest();
        if (!w->isVisited()) {
            dfsVisit(w, res);
        }
    }
}

/****************** BFS ********************/
/*
 * Performs a breadth-first search (bfs) in a graph (this), starting
 * from the vertex with the given source contents (source).
 * Returns a vector with the contents of the vertices by bfs order.
 */
template <class T>
std::vector<T> Graph<T>::bfs(const T & source) const {
    std::vector<int> res;
    // Get the source vertex
    auto s = findVertex(source);
    if (s == nullptr) {
        return res;
    }

    // Set that no vertex has been visited yet
    for (auto v : vertexSet) {
        v->setVisited(false);
    }

    // Perform the actual BFS using a queue
    std::queue<Vertex<T> *> q;
    q.push(s);
    s->setVisited(true);
    while (!q.empty()) {
        auto v = q.front();
        q.pop();
        res.push_back(v->getInfo());
        for (auto & e : v->getAdj()) {
            auto w = e->getDest();
            if ( ! w->isVisited()) {
                q.push(w);
                w->setVisited(true);
            }
        }
    }
    return res;
}

template <class T>
std::map<int, std::string> Graph<T>::getCityMap(){
    return cityMap;
}

template <class T>
void Graph<T>::setCityMap(std::map<int, std::string> cityMap) {
    this->cityMap = cityMap;
}

/****************** isDAG  ********************/
/*
 * Performs a depth-first search in a graph (this), to determine if the graph
 * is acyclic (acyclic directed graph or DAG).
 * During the search, a cycle is found if an edge connects to a vertex
 * that is being processed in the stack of recursive calls (see theoretical classes).
 * Returns true if the graph is acyclic, and false otherwise.
 */

template <class T>
bool Graph<T>::isDAG() const {
    for (auto v : vertexSet) {
        v->setVisited(false);
        v->setProcesssing(false);
    }
    for (auto v : vertexSet) {
        if (! v->isVisited()) {
            if ( ! dfsIsDAG(v) ) return false;
        }
    }
    return true;
}

/**
 * Auxiliary function that visits a vertex (v) and its adjacent, recursively.
 * Returns false (not acyclic) if an edge to a vertex in the stack is found.
 */
template <class T>
bool Graph<T>::dfsIsDAG(Vertex<T> *v) const {
    v->setVisited(true);
    v->setProcesssing(true);
    for (auto e : v->getAdj()) {
        auto w = e->getDest();
        if (w->isProcessing()) return false;
        if (! w->isVisited()) {
            if (! dfsIsDAG(w)) return false;
        }
    }
    v->setProcesssing(false);
    return true;
}
/********** back_tracking_algorithm ************/

template <class T>
double Graph<T>::back_tracking_algorithm() {
    int n = vertexSet.size();

    distMatrix = (double**) malloc(n*sizeof(double*));
    for(int i = 0; i < n; ++i)
        distMatrix[i] = (double*) malloc(n*sizeof(double));

    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            distMatrix[i][j] = INF;

    for (auto vert : vertexSet) {
        int u = vert->getInfo();
        for (Edge<T>* edge : vert->getAdj()) {
            int v = edge->getDest()->getInfo();
            distMatrix[u][v] = edge->getWeight();
        }
    }


    for(auto vert : vertexSet)
        vert->setVisited(false);

    double ans = INF;
    Vertex<T>* startNode = findVertex(0);

    std::vector<T> path;
    std::vector<T> bestPath;
    path.push_back(startNode->getInfo());

    startNode->setVisited(true);
    back_tracking_algorithm_rec(startNode, n, 1, 0, &ans, path, bestPath);




    //deleteMatrix(distMatrix, n);
    for(auto vert : vertexSet)
        vert->setVisited(false);



    std::cout << "Optimal path: ";
    for (const auto& node : bestPath) {
        std::cout << node << " -> ";
    }
    std::cout << startNode->getInfo() << std::endl;



    return ans;
}

template <class T>
void Graph<T>::back_tracking_algorithm_rec(Vertex<T>* curr, int n, int count, double cost, double* ans, std::vector<T>& path, std::vector<T>& bestPath) {
    auto startNode = findVertex(0);
    Edge<T>* edgeCurrStart = nullptr;
    for (Edge<T>* e: curr->getAdj()){
        if (e->getDest()->getInfo() == startNode->getInfo()) {
            edgeCurrStart = e;
            break;
        }
    }

    // If reached last node and edge connects to start
    if(count == n && edgeCurrStart != nullptr) {
        if (*ans >= cost + edgeCurrStart->getWeight()) {
            *ans = cost + edgeCurrStart->getWeight();
            bestPath = path;
        }
        return;
    }


    for(Edge<T>* e : curr->getAdj()){
        if(!e->getDest()->isVisited()){
            e->getDest()->setVisited(true);
            path.push_back(e->getDest()->getInfo());
            back_tracking_algorithm_rec(e->getDest(), n, count + 1, cost + e->getWeight(), ans, path, bestPath);

            path.pop_back();
            e->getDest()->setVisited(false);
        }
    }
}


/********** triangular_approximation_heuristic ************/
template <class T>
std::vector<Vertex<T>*> Graph<T>::tspTriangular(double* cost, bool realWorldGraph) {



    for(auto v : vertexSet) {
        v->setVisited(false);
        v->setPath(nullptr);
        v->setDist(DOUBLE_INF);
        for(auto e : v->getAdj())
            e->setSelected(false);
    }
    prim_algorithm();

    for(auto v : vertexSet)
        v->setVisited(false);

    auto startNode = findVertex(0);
    std::vector<Vertex<T>*> path = dfsTriangular(startNode);
    path.push_back(startNode);

    *cost = 0;
    for (int i = 0; i < path.size() - 1 ; i++)
        *cost += getDistFromTo(path[i], path[i + 1], realWorldGraph);

    return path;
}

template <class T>
std::vector<Vertex<T>*> Graph<T>::dfsTriangular(Vertex<T>* v){
    v->setVisited(true);
    std::vector<Vertex<T>*> path = {v};

    for (Edge<T>* e : v->getAdj()) {
        if (e->isSelected() && !e->getDest()->isVisited()) {

                std::vector<Vertex<T>*> rec_path = dfsTriangular(e->getDest());
                path.insert(path.end(), rec_path.begin(), rec_path.end());

        }
    }

    return path;
}


/********** Nearest Neighbor Heuristic ************/


template<class T>
std::vector<Vertex<T> *> Graph<T>::nearestNeighborHeuristic(int origin, double &cost, bool useHaversine) const {

    std::vector<Vertex<T>*> path;

    auto startNode = findVertex(origin);

    for(auto v : vertexSet)
        v->setVisited(false);

    path.push_back(startNode);

    startNode->setVisited(true);
    int numNodes = vertexSet.size();

    Vertex<T>* current = startNode;
    for (int i = 0; i < numNodes - 1; i++) {
        int minDistance = std::numeric_limits<int>::max();
        Vertex<T>* nextNode = nullptr;
        for (auto v : vertexSet) {
            if (!v->isVisited() && getDistFromTo(current, v, useHaversine) < minDistance) {
                minDistance = getDistFromTo(current, v, useHaversine);
                nextNode = v;
            }
        }
        if (nextNode != nullptr) {
            nextNode->setVisited(true);
            path.push_back(nextNode);
            cost = cost + minDistance;
            current = nextNode;
        }

    }
    cost += getDistFromTo(current, startNode, useHaversine);
    path.push_back(startNode); // Return to the start node
    return path;

}




template<class T>
double Graph<T>::haversineDistance(double lat1, double lon1, double lat2, double lon2) {
    const double R = 6371.0; // Radius of the Earth in km
    double dLat = (lat2 - lat1) * M_PI / 180.0;
    double dLon = (lon2 - lon1) * M_PI / 180.0;

    double a = sin(dLat / 2) * sin(dLat / 2) +
               cos(lat1 * M_PI / 180.0) * cos(lat2 * M_PI / 180.0) *
               sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return R * c;
}







template <class T>
struct EdgeComparator {
    bool operator()(T e1, T e2) {
        return e1->getWeight() > e2->getWeight();
    }
};

template <class T>
void Graph<T>::prim_algorithm() {

    std::priority_queue<Edge<T>*, std::vector<Edge<T>*>, EdgeComparator<Edge<T>*> > pq;

    Vertex<T>* startNode = findVertex(0);
    startNode->setVisited(true);

    for (auto edge : startNode->getAdj())
        pq.push(edge);

    while (!pq.empty()) {
        auto minEdge = pq.top();
        pq.pop();

        if(minEdge->getOrig()->isVisited() && minEdge->getDest()->isVisited()) continue;

        minEdge->setSelected(true);
        minEdge->getReverse()->setSelected(true);

        minEdge->getDest()->setVisited(true);
        for (auto edge : minEdge->getDest()->getAdj()) {
            if(!edge->getDest()->isVisited()){
                pq.push(edge);
            }
        }
    }

}


/****************** toposort ********************/
//=============================================================================
// Exercise 1: Topological Sorting
//=============================================================================
// TODO
/*
 * Performs a topological sorting of the vertices of a graph (this).
 * Returns a vector with the contents of the vertices by topological order.
 * If the graph has cycles, returns an empty vector.
 * Follows the algorithm described in theoretical classes.
 */

template<class T>
std::vector<T> Graph<T>::topsort() const {
    std::vector<int> res;

    for (auto v : vertexSet) {
        v->setIndegree(0);
    }
    for (auto v : vertexSet) {
        for (auto e : v->getAdj()) {
            unsigned int indegree = e->getDest()->getIndegree();
            e->getDest()->setIndegree(indegree + 1);
        }
    }

    std::queue<Vertex<T> *> q;
    for (auto v : vertexSet) {
        if (v->getIndegree() == 0) {
            q.push(v);
        }
    }

    while( !q.empty() ) {
        Vertex<T> * v = q.front();
        q.pop();
        res.push_back(v->getInfo());
        for(auto e : v->getAdj()) {
            auto w = e->getDest();
            w->setIndegree(w->getIndegree() - 1);
            if(w->getIndegree() == 0) {
                q.push(w);
            }
        }
    }

    if ( res.size() != vertexSet.size() ) {
        //std::cout << "Impossible topological ordering!" << std::endl;
        res.clear();
        return res;
    }

    return res;
}

inline void deleteMatrix(int **m, int n) {
    if (m != nullptr) {
        for (int i = 0; i < n; i++)
            if (m[i] != nullptr)
                delete [] m[i];
        delete [] m;
    }
}

inline void deleteMatrix(double **m, int n) {
    if (m != nullptr) {
        for (int i = 0; i < n; i++)
            if (m[i] != nullptr)
                delete [] m[i];
        delete [] m;
    }
}

template <class T>
Graph<T>::~Graph() {
    deleteMatrix(distMatrix, vertexSet.size());
    deleteMatrix(pathMatrix, vertexSet.size());
}

template<class T>
bool Graph<T>::isConnected(T origin) {

    std::unordered_set<Vertex<T>*> visited;
    std::queue<Vertex<T>*> queue;

    Vertex<T>* start = findVertex(origin);
    if (!start) return false;

    queue.push(start);
    visited.insert(start);

    while (!queue.empty()) {
        Vertex<T>* current = queue.front();
        queue.pop();

        for (Edge<T>* edge : current->getAdj()) {
            Vertex<T>* neighbor = edge->getDest();
            if (visited.find(neighbor) == visited.end()) {
                queue.push(neighbor);
                visited.insert(neighbor);
            }
        }
    }

    return visited.size() == vertexSet.size();
}




#endif //GRAPH_H
