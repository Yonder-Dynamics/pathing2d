#pragma once
#include <queue>
#include <list>
#include <vector>
#include <list>
#include <utility>
#include <limits>
#include <unordered_map>
#include <pathing2d/util.h>

using namespace std;

template <typename T>
class Graph {
  int maxEdges;
  typedef pair<size_t, T> tpair;
  // array of nodes for which there is an array of their connections
  priority_queue<tpair> * edges; 
  //vector<tpair> * edges;
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
  T heuristic(size_t src, size_t goal, std::vector<WeightedPoint> & points);
  std::vector<size_t> reconstruct_path(std::vector<size_t>& cameFrom, size_t current);
  std::vector<size_t> shortestPath(size_t src, size_t goal,
      std::vector<WeightedPoint> & points);
};

template <typename T>
Graph<T>::Graph(size_t size, int maxEdges) : size(size), maxEdges(maxEdges) {
  edges = new priority_queue<tpair> [size];
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
T Graph<T>::heuristic(size_t src, size_t goal, std::vector<WeightedPoint> & points) {
  // Gives an approximation of how far src is from goal
  return dist3d(points[src], points[goal]);
}

template <typename T>
// Prints shortest paths from src to all other vertices
std::vector<size_t> Graph<T>::shortestPath(size_t src, size_t goal,
    std::vector<WeightedPoint> & points)
{
  // The set of currently discovered nodes that are not evaluated yet.
  // Initially, only the start node is known.
  //std::list<size_t> openSet;
  //openSet.push_back(src);
  priority_queue<tpair, std::vector<tpair>, std::greater<tpair> > openSet;
  openSet.push(make_pair(src, heuristic(src, goal, points)));

  // The set of nodes already evaluated
  std::list<size_t> closedSet;
  priority_queue<tpair, std::vector<tpair>, std::greater<tpair> > closedSetQ;

  // For each node, which node it can most efficiently be reached from.
  // If a node can be reached from many nodes, cameFrom will eventually contain the
  // most efficient previous step.
  std::vector<size_t> cameFrom (size, -1);

  // For each node, the cost of getting from the start node to that node.
  std::vector<T> gScore (size, std::numeric_limits<T>::infinity());
  // The cost of going from start to start is zero.
  gScore[src] = 0;

  // For each node, the total cost of getting from the start node to the goal
  // by passing by that node. That value is partly known, partly heuristic.
  //std::vector<T> fScore (size, std::numeric_limits<T>::infinity());
  //Heuristic estimate of how far the goal is from the rover
  //fScore[src] = heuristic(src, goal, points);

  //tpair current_pair = openSet.top();
  size_t current = openSet.top().first;

  /*
  for (tpair& x : edges[src]) {
    openSet.push_back(x.first);
  }
  */

  while (!openSet.empty()) {
    //Choose lowest fScore from openSet (need to use priority queue here)
    /*
    size_t lowest;
    for (size_t x : openSet) {
      if (fScore[x] <= fScore[lowest])
        lowest = x;
    }
    current = lowest;

    openSet.remove(current);
    */
    current = openSet.top().first;
    closedSet.push_back(current);
    closedSetQ.push(openSet.top());
    openSet.pop();

    if (current == goal) {
      return reconstruct_path(cameFrom, current);
    }

    // Search the neighbors of current
    //for (auto it = edges[current].begin(); it != edges[current].end(); it++) {
    //  tpair& x = *it;
    while (edges[current].size() > 0) {
      tpair x = edges[current].top();
      edges[current].pop();
      // Skip if already evaluated
      if (std::find(closedSet.begin(), closedSet.end(),x.first) != closedSet.end())
        continue;
      // Add to open set if not yet evaluated. Technically unnecessary as we
      // will never reprocess the same point thanks to closed set
      //if (std::find(openSet.begin(), openSet.end(), x.first) == openSet.end())
        //openSet.push_back(x.first);

      /// Then assign that node a gScore based on current distance and
      /// distance from that node to current

      /*
      // Search for the distance
      int found = -1;
      for (int i = 0; i < edges[current].size(); i++) {
        if (edges[current].at(i).first == x.first)
          found = i;
      }
      T tentative_score = gScore[current] + edges[current].at(found).second;
      */

      T tentative_score = gScore[current] + x.second;

      if (tentative_score >= gScore[x.first])
        continue;

      cameFrom[x.first] = current;
      gScore[x.first] = tentative_score;
      //fScore[x.first] = tentative_score + edges[x.first].at(edges[x.first].size() - 1).second;
      //fScore[x.first] = tentative_score + heuristic(x.first, goal, points);
      openSet.push(make_pair(x.first, tentative_score + heuristic(x.first, goal, points)));
    }
  }
  current = closedSetQ.top().first;
  return reconstruct_path(cameFrom, current);
}

template <typename T>
std::vector<size_t> Graph<T>::reconstruct_path (std::vector<size_t>& cameFrom, size_t current) {
  std::vector<size_t> path;
  path.push_back(current);
  while (cameFrom[current] != -1) {
    // Add to the path the shortest path to current and set current to the previous node
    current = cameFrom[current];
    path.insert(path.begin(), current);
  }
  return path;
}
