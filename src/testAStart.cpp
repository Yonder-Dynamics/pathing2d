#include <pathing2d/Graph.hpp>
#include <pathing2d/Histogram.hpp>
#include <pathing2d/Pathing2D.h>
#include <pathing2d/util.h>

int main() {
  std::vector<WeightedPoint> points;
  int i[] = {1, 0, 2, 0, 2, 1};
  int j[] = {1, 2, 2, 3, 3, 4};
  int w[] = {0, 0, 1, 1, 4, 1, 2};
  for(int k = 0; k < 6; k++) {
    WeightedPoint p;
    p.x = i[k];
    p.y = j[k];
    p.z = 0;
    p.weight = 0;
    points.push_back(p);
  }

//  int width = 10;
//  int height = 10;

  Graph<int> g(6, 100);
  g.addEdge(0, 1, 0);
  g.addEdge(0, 2, 1);
  g.addEdge(1, 2, 0);
  g.addEdge(1, 3, 1);
  g.addEdge(3, 4, 0);
  g.addEdge(4, 5, 2);
  g.addEdge(3, 5, 1);
  g.addEdge(3, 2, 0);
  g.addEdge(1, 4, 5);
 

  auto results = g.shortestPath(0, 5, points);
  std::cout << results.size() << std::endl;
  for (auto it = results.begin(); it != results.end(); it++)
    std::cout << *it << std::endl;


}
