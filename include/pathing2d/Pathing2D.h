#pragma once
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

cv::Mat circularFilter(int rad);

float dist2d(geometry_msgs::Point & a, geometry_msgs::Point & b);

float dist3d(geometry_msgs::Point & a, geometry_msgs::Point & b);

struct WeightedPoint : public geometry_msgs::Point {
  float weight;
  bool operator < (WeightedPoint const &b) { 
    return weight < b.weight;
  }
};

class Pathing2D {
  bool gotGoal, gotOcto;
  double ox, oy, oz;
  double mx, my, mz;
  boost::shared_ptr<octomap::OcTree> tree;
  geometry_msgs::Pose goal;
  public:
    float res, maxBumpiness, robotRadius, dangerOfUnknown, roughnessWeight, steepnessWeight, maxSteepness;
    int maxEdges;

    Pathing2D(float res, float maxBumpiness, float robotRadius,
              float dangerOfUnknown, float roughnessWeight,
              float steepnessWeight, float maxSteepness, int maxEdges) :
      gotGoal(false), res(res), maxBumpiness(maxBumpiness),
      robotRadius(robotRadius), dangerOfUnknown(dangerOfUnknown),
      roughnessWeight(roughnessWeight), steepnessWeight(steepnessWeight),
      maxSteepness(maxSteepness), maxEdges(maxEdges) {};

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

    Graph<float> buildGraph(
        std::vector<WeightedPoint> & points,
        float roughnessWeight,
        float steepnessWeight,
        float maxSteepness,
        // Should be slightly less than the radius for occupation around robot
        float maxLength,
        int maxEdges);
};
