#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseStamped.h>
#include <opencv2/opencv.hpp>
#include <pathing2d/Graph.hpp>

cv::Mat circularFilter(int rad) {
  cv::Mat filter (2*rad, 2*rad, CV_32F);
  for (int i=0; i<2*rad; i++) {
    for (int j=0; j<2*rad; j++) {
      dist = (i - rad)*(i - rad) + (j - rad)*(j - rad);
      if (dist < rad*rad) {
        filter.at<float>(i, j) = 1;
      } else {
        // Add soft edges
        filter.at<float>(i, j) = 0;
      }
    }
  }
  return filter;
}

float dist2d(geometry_msgs::Point & a, geometry_msgs::Point & b) {
   return sqrt((a.x-b.x)*(a.x-b.x) + (a.y-b.y)*(a.y-b.y));
}

float dist3d(geometry_msgs::Point & a, geometry_msgs::Point & b) {
   return sqrt((a.x-b.x)*(a.x-b.x) +
               (a.y-b.y)*(a.y-b.y) +
               (a->z-b->z)*(a->z-b->z));
}

struct WeightedPoint : public geometry_msgs::Point {
  float weight;
  bool operator < (WeightedPoint const &a, WeightedPoint const &b) { 
    return a.weight < b.weight;
  }
}

class Pathing2D {
  bool gotGoal, gotOcto;
  float res;
  double ox, oy, oz;
  double mx, my, mz;
  boost::shared_ptr<octomap::OcTree> tree;
  geometry_msgs::Pose goal;
  public:
    float maxBumpiness, robotRadius, dangerOfUnknown, roughnessWeight, steepnessWeight, maxSteepness;
    Pathing2D(float res) : gotGoal(false), res(res) {};
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg);
    void process();
    cv::Mat processOctomap();
    std::vector<WeightedPoint> openPositions(
        // basic height histogram
        cv::Mat & heightHist,
        // histogram that has been convolved to find edges
        cv::Mat & edgeHist,
        // threshold for maximum amount of edges
        float maxBumpiness,
        // Radius to check around points for occupation
        float radius,
        // bumpiness of unknown things, basically how much to avoid unknown parts of map
        float dangerOfUnknown,
        // Resolution of histogram, ie coords in hist*res = coords in world
        float res);

    Graph buildGraph(
        std::vector<WeightedPoint> & points,
        float roughnessWeight,
        float steepnessWeight,
        float maxSteepness,
        // Should be slightly less than the radius for occupation around robot
        float maxLength);
};
