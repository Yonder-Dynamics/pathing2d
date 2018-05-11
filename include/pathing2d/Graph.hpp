#pragma once
#include <queue>
#include <list>
#include <vector>
#include <list>
#include <utility>
#include <limits>

using namespace std;

template <typename T>
class Graph {
  int maxEdges;
  typedef pair<size_t, T> tpair;
  // array of nodes for which there is an array of their connections
  priority_queue<tpair> * edges; 
  size_t size;
  public:
  Graph(size_t size, int maxEdges=-1);
  ~Graph();
  void addEdge(size_t u, size_t v, T w);
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
  std::list<size_t> closedSet;
  std::list<size_t> openSet;
  openSet.push_back(src);
  std::vector<size_t> cameFrom (size*size, 0); // map
  std::vector<size_t> gScore (size, std::numeric_limits<T>::infinity()); // map
  gScore[src] = 0;
  std::vector<size_t> fScore (size, std::numeric_limits<T>::infinity()); // map
  fScore[src] = 0;
  std::vector<size_t> path;
  path.push_back(1);
  return path;
}
/*
function A*(start, goal)
    // The set of nodes already evaluated
    closedSet := {}

    // The set of currently discovered nodes that are not evaluated yet.
    // Initially, only the start node is known.
    openSet := {start}

    // For each node, which node it can most efficiently be reached from.
    // If a node can be reached from many nodes, cameFrom will eventually contain the
    // most efficient previous step.
    cameFrom := an empty map

    // For each node, the cost of getting from the start node to that node.
    gScore := map with default value of Infinity

    // The cost of going from start to start is zero.
    gScore[start] := 0

    // For each node, the total cost of getting from the start node to the goal
    // by passing by that node. That value is partly known, partly heuristic.
    fScore := map with default value of Infinity

    // For the first node, that value is completely heuristic.
    fScore[start] := heuristic_cost_estimate(start, goal)

    while openSet is not empty
        current := the node in openSet having the lowest fScore[] value
        if current = goal
            return reconstruct_path(cameFrom, current)

        openSet.Remove(current)
        closedSet.Add(current)

        for each neighbor of current
            if neighbor in closedSet
                continue		// Ignore the neighbor which is already evaluated.

            if neighbor not in openSet	// Discover a new node
                openSet.Add(neighbor)
            
            // The distance from start to a neighbor
            //the "dist_between" function may vary as per the solution requirements.
            tentative_gScore := gScore[current] + dist_between(current, neighbor)
            if tentative_gScore >= gScore[neighbor]
                continue		// This is not a better path.

            // This path is the best until now. Record it!
            cameFrom[neighbor] := current
            gScore[neighbor] := tentative_gScore
            fScore[neighbor] := gScore[neighbor] + heuristic_cost_estimate(neighbor, goal) 

    return failure

function reconstruct_path(cameFrom, current)
    total_path := [current]
    while current in cameFrom.Keys:
        current := cameFrom[current]
        total_path.append(current)
    return total_path
*/
