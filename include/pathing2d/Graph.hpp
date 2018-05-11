#pragma once
#include <queue>
#include <list>
#include <vector>
#include <list>
#include <utility>

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
  void shortestPath(int src);
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
void Graph<T>::shortestPath(int src)
{
  /*
  // Create a priority queue to store vertices that
  // are being preprocessed. This is weird syntax in C++.
  // Refer below link for details of this syntax
  // http://geeksquiz.com/implement-min-heap-using-stl/
  priority_queue< iPair, vector <iPair> , greater<iPair> > pq;

  // Create a vector for distances and initialize all
  // distances as infinite (INF)
  vector<int> dist(V, INF);

  // Insert source itself in priority queue and initialize
  // its distance as 0.
  pq.push(make_pair(0, src));
  dist[src] = 0;

  // Looping till priority queue becomes empty (or all
  // distances are not finalized)
  while (!pq.empty())
  {
      // The first vertex in pair is the minimum distance
      // vertex, extract it from priority queue.
      // vertex label is stored in second of pair (it
      // has to be done this way to keep the vertices
      // sorted distance (distance must be first item
      // in pair)
      int u = pq.top().second;
      pq.pop();

      // 'i' is used to get all adjacent vertices of a vertex
      list< pair<int, int> >::iterator i;
      for (i = adj[u].begin(); i != adj[u].end(); ++i)
      {
          // Get vertex label and weight of current adjacent
          // of u.
          int v = (*i).first;
          int weight = (*i).second;

          //  If there is shorted path to v through u.
          if (dist[v] > dist[u] + weight)
          {
              // Updating distance of v
              dist[v] = dist[u] + weight;
              pq.push(make_pair(dist[v], v));
          }
      }
  }

  // Print shortest distances stored in dist[]
  printf("Vertex   Distance from Source\n");
  for (int i = 0; i < V; ++i)
      printf("%d \t\t %d\n", i, dist[i]);
  */
}

