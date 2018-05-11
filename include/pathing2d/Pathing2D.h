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
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <opencv2/opencv.hpp>
#include <pathing2d/Graph.hpp>

// It's like filter2d but implemented properly
cv::Mat filter2D(cv::Mat src, int ddepth, cv::Mat & kernel,
    const cv::Point & origin=cv::Point(-1,-1), double delta=0,
    int border_type=cv::BORDER_DEFAULT,
    const cv::Scalar & border_value=cv::Scalar());
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
  bool gotGoal, gotOcto, gotRover;
  double ox, oy, oz;
  double mx, my, mz;
  std::string frame_id;
  boost::shared_ptr<octomap::OcTree> tree;
  geometry_msgs::Pose goal, rover;
  ros::Publisher traj_pub, map_pub;
  public:
    float res, maxBumpiness, robotRadius, dangerOfUnknown, roughnessWeight, steepnessWeight, maxSteepness;
    int maxEdges;

    Pathing2D(ros::NodeHandle n, float res, float maxBumpiness, float robotRadius,
              float dangerOfUnknown, float roughnessWeight,
              float steepnessWeight, float maxSteepness, int maxEdges,
              std::string frame_id) :
      gotGoal(false), gotRover(false), res(res), maxBumpiness(maxBumpiness),
      robotRadius(robotRadius), dangerOfUnknown(dangerOfUnknown),
      roughnessWeight(roughnessWeight), steepnessWeight(steepnessWeight),
      maxSteepness(maxSteepness), maxEdges(maxEdges), frame_id(frame_id) {
      traj_pub = n.advertise<nav_msgs::Path>("trajectory", 4);
      map_pub = n.advertise<nav_msgs::OccupancyGrid>("occupancy_grid", 4);
    };

    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg);

    nav_msgs::Path construct(const std::vector<size_t> & path,
        const std::vector<WeightedPoint> & nodes);

    void process();
    cv::Mat processOctomap();
    std::vector<WeightedPoint> openPositions(
        // basic height histogram
        const cv::Mat & heightHist,
        // histogram that has been convolved to find edges
        const cv::Mat & edgeHist,
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

    nav_msgs::OccupancyGrid cvToGrid(cv::Mat & mat);
};
