#pragma once
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#include <octomap_msgs/Octomap.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/Point.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>

#include <pathing2d/Graph.hpp>
#include <pathing2d/Histogram.hpp>
#include <pathing2d/Pathing2D.h>
#include <pathing2d/util.h>

class Pathing2D {
  bool gotGoal, gotOcto, gotRover, isProcessing;
  double ox, oy, oz;
  double mx, my, mz;
  double width, height, depth;
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
      gotOcto(false), gotGoal(false), gotRover(true), isProcessing(false),
      res(res), maxBumpiness(maxBumpiness),
      robotRadius(robotRadius), dangerOfUnknown(dangerOfUnknown),
      roughnessWeight(roughnessWeight), steepnessWeight(steepnessWeight),
      maxSteepness(maxSteepness), maxEdges(maxEdges), frame_id(frame_id) {
      traj_pub = n.advertise<nav_msgs::Path>("trajectory", 4);
      map_pub = n.advertise<nav_msgs::OccupancyGrid>("occupancy_grid", 4);
    };

    void poseCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg);

    nav_msgs::Path construct(const std::vector<size_t> & path,
        const std::vector<WeightedPoint> & nodes);

    void process();
    cv::Mat processOctomap(cv::Mat * unknown);

    Graph<float> buildGraph(
        const cv::Mat & heightHist,
        const cv::Mat & occupancy,
        std::vector<WeightedPoint> & openPoses, // output
        cv::Mat * graphIndexes, // output
        float roverConnectionRad,
        float maxBumpiness,
        float roughnessWeight,
        float steepnessWeight,
        float maxSteepness,
        float maxLength,
        int maxEdges);

    nav_msgs::OccupancyGrid cvToGrid(const  cv::Mat & mat);
};
