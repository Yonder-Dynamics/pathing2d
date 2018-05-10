#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <pathing2d/Histogram.hpp>
#include <pathing2d/Pathing2D.h>
#include <opencv2/opencv.hpp>
#include <geometry_msgs/Point.h>
#include <pathing2d/Graph.hpp>

void Pathing2D::poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
}

void Pathing2D::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  goal = msg->pose;
  gotGoal = true;
  if (gotOcto)
    process();
}

void Pathing2D::octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
  std::cout << "Recieved octomap" << std::endl;
  octomap::AbstractOcTree* atree = octomap_msgs::binaryMsgToMap(*msg);
  tree = boost::make_shared<octomap::OcTree>(*(octomap::OcTree *)atree);
  gotOcto = true;
  if (gotGoal)
    process();
  delete(atree);

}

cv::Mat Pathing2D::processOctomap() {
  tree->getMetricMin(ox, oy, oz);
  tree->getMetricMax(mx, my, mz);

  Histogram<float> h (
      int(ceil((mx-ox)/res)),
      int(ceil((my-oy)/res)),
      int(ceil(ox/res)),
      int(ceil(oy/res)));
  for(auto it = tree->begin(); it != tree->end(); it++) {
    octomath::Vector3 pos = it.getCoordinate();
    int x = int(pos.x()/res);
    int y = int(pos.y()/res);
    //std::cout << x << "," << y << std::endl;
    h.add(x, y, pos.z());
  }
  cv::Mat hist = h.findMaxes();
  return hist;
  /* 
  // Display things
  std::cout << hist << std::endl;
  cv::Mat bw;
  cv::normalize(hist, bw, 0, 255, 32, CV_8UC1);
  cv::namedWindow("Hist", 0);
  cv::imshow("Hist", bw);
  cv::waitKey(0);
  std::cout << "Done" << std::endl;
  */
}

std::vector<WeightedPoint> Pathing2D::openPositions(
    cv::Mat & heightHist,
    cv::Mat & edgeHist,
    float maxBumpiness,
    int radius,
    float dangerOfUnknown,
    float res)
{

  // Use convolution of a circle with soft edges with zero padding so only safe points are navigated to
  // Create circlular filter
  cv::Mat filter = circularFilter(radius);

  // Apply exponential function to edgeHistogram to emphasize large obstacles
  // over collections of small obstacles
  cv::Mat exp;
  cv::exp(edgeHist, exp);
  // Apply filter
  cv::Mat occupationHist;
  cv::filter2D(exp, occupationHist, -1, filter,
      cv::Point(-1,-1), 0, BORDER_CONSTANT, dangerOfUnknown);
  occupationHist /= cv::sum(filter); // Average bumpiness

  // Create positions filter by threshold
  std::vector<WeightedPoint> openPoses;
  for (int i=0; i<; i++) {
    for (int j=0; j<2*rad; j++) {
      // Exponentiate threshold to match occupationHist
      if (occupationHist.at<float>(i, j) < exp(threshold)) {
        WeightedPoint p;
        p.x = j * res;
        p.y = i * res;
        p.z = heightHist.at<float>(i, j);
        p.weight = occupationHist.at<float>(i,j);
        openPoses.push_back(p);
        // Maybe find node edges here?
      }
    }
  }
  return openPoses;
}

Graph Pathing2D::buildGraph(
    std::vector<WeightedPoint> & points,
    float roughnessWeight,
    float steepnessWeight,
    float maxSteepness,
    float maxLength)
{
  Graph g(points.size());
  // The steepness of an edge is an average of each node's roughnes
  float dist, slope, roughness;
  bool goalWithinHist = (ox < goal.position.x && goal.position.x < mx) &&
                        (oy < goal.position.y && goal.position.y < my);
  WeightedPoint * p1, p2;
  for (int i=0; i<points.size(); i++) {
    p1 = &points[i];
    // Add goal to each point if goal is within map
    //if (goalWithinHist) {
    if (true) {
      dist = dist2d(*p1, goal);
      slope = abs(p1->z-goal.z)/dist;
      if (dist < maxLength && slope < maxSteepness) {
        g.addEdge(i, j, dist + slope*steepnessWeight + p1->weight*roughnessWeight);
      }
    }
    for (int j=i; j<points.size(); j++) {
      if (i!=j) {
        // Add an edge for every point
        p2 = &points[i];
        dist = dist2d(*p1, *p2);
        slope = abs(p1->z-p2->z)/dist;
        roughness = p1->weight/2+p2->weight/2;
        if (dist < maxLength && slope < maxSteepness) {
          g.addEdge(i, j, dist + slope*steepnessWeight + roughness*roughnessWeight);
        }
      }
    }
  }

  if (!goalWithinHist) {
    //Maybe different behavior?
  }
  return g;
}

void Pathing2D::process() {
    cv::Mat rawHeight = processOctomap();
    cv::Mat edgeHist;

    //--------- Calculate slope histogram (derivative) ------------
    cv::GaussianBlur(rawHeight, edgeHist, Size(3,3), 0, 0, BORDER_CONSTANT, dangerOfUnknown);
    /// Generate grad_x and grad_y
    Mat grad_x, grad_y;
    Mat abs_grad_x, abs_grad_y;

    /// Gradient X
    Scharr( edgeHist, grad_x, -1, 1, 0, 3, 1, 0, BORDER_CONSTANT, dangerOfUnknown);
    convertScaleAbs( grad_x, abs_grad_x );

    /// Gradient Y
    Scharr( edgeHist, grad_y, -1, 0, 1, 3, 1, 0, BORDER_CONSTANT, dangerOfUnknown);
    convertScaleAbs( grad_y, abs_grad_y );

    /// Total Gradient (approximate)
    addWeighted( abs_grad_x, 0.5, abs_grad_y, 0.5, 0, edgeHist );

    std::vector<WeightedPoint> open = openPositions(
        rawHeight, edgeHist, maxBumpiness, robotRadius, dangerOfUnknown, res);
    Graph g = buildGraph(
        open, roughnessWeight, steepnessWeight, maxSteepness, robotRadius-1);
    auto shortestPath = g.shortestPath();
}
