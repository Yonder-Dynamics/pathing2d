#pragma once
#include <queue>
#include <list>
#include <vector>
#include <list>
#include <utility>
#include <limits>
#include <unordered_map>

using namespace std;



template <typename T>
class Graph {
  int maxEdges;
  typedef pair<size_t, T> tpair;
  // array of nodes for which there is an array of their connections
  priority_queue<tpair> * edges; 
  size_t size;

  struct GraphNode {
    size_t id;
    vector<size_t> connection;
    size_t prev;
    T value;
    T dist;

    void addConnection(size_t v) {
      connection.push_back(v);
    };
  };

  public:
  Graph(size_t size, int maxEdges=-1);
  ~Graph();
  void addEdge(size_t u, size_t v, T w);
  void reconstruct_path(std::vector<size_t>& cameFrom, size_t current);
  std::vector<size_t> shortestPath(int src);
};

template <typename T>
Graph<T>::Graph(size_t size, int maxEdges) : size(size), maxEdges(maxEdges) {
  edges = new priority_queue<tpair>[size];
}

template <typename T>
Graph<T>::~Graph() {
  delete[] edges;
}

template <typename T>
void Graph<T>::addEdge(size_t u, size_t v, T w) {
  edges[u].push(make_pair(v, w));
  edges[v].push(make_pair(u, w));
  if (maxEdges < edges[u].size() && maxEdges > 0)
    edges[u].pop();
  if (maxEdges < edges[v].size() && maxEdges > 0)
    edges[v].pop();
}

template <typename T>
// Prints shortest paths from src to all other vertices
std::vector<size_t> Graph<T>::shortestPath(int src)
{
  /*size_t startNode = edges[src].first;
    unordered_map<size_t, GraphNode> aStarG;

    for (auto it = aStarG.begin(); it != aStarG.end(); it++) {
    it->second.prev = -1;
    }

    aStarG[startNode] */
  size_t src = (size_t)src1;
  std::list<size_t> closedSet;
  std::list<size_t> openSet;
  closedSet.push_back(src);
  std::vector<size_t> cameFrom (size*size, 0); // map
  std::vector<T> gScore (size, std::numeric_limits<T>::infinity()); // map
  gScore[src] = 0;
  std::vector<T> fScore (size, std::numeric_limits<T>::infinity()); // map
  fScore[src] = 0;

  //Heuristic estimate
  size_t current = src;


  for (tpair x : edges[src]) {
    openSet.insert(openSet.begin(), x.first);
  }

  while (!openSet.empty()) {
    //Choose lowest fScore from openSet
    size_t lowest;
    for (size_t x : openSet) {
      if (fScore[x] <= fScore[lowest])
        lowest = x;
    }
    current = lowest;

    openSet.remove(current);
    closedSet.insert(closedSet.begin(), current);

    for (tpair& x : edges[current]) {
      if (find(x.first, closedSet.begin(), closedSet.end()) != closedSet.end())
        continue;
      if (find(x.first, openSet.begin(), openSet.end()) == openSet.end())
        openSet.insert(openSet.begin(), x.first);
      auto it = find(x, edges[current].begin(), edges[current].end());
      T tentative_score = gScore[current] + it->second;

      if (tentative_score >= gScore[x])
        continue;

      cameFrom[x] = current;
      gScore[x] = tentative_score;
      fScore[x] = tentative_score + edges[x].at(edges[x].size - 1).second;
    }
  }
}

template <typename T>
void Graph<T>::reconstruct_path (std::vector<size_t>& cameFrom, size_t current) {

}
  /*std::vector<size_t> path;
  path.push_back(1);
  return path;
}*/
