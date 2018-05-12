#include <iostream>
#include <ros/ros.h>
#include <pathing2d/Pathing2D.h>


int main(int argc, char *argv[]) {
  ros::init(argc, argv, "pathing2d");
  ros::NodeHandle n;
  float res, maxBumpiness, robotRadius, dangerOfUnknown, roughnessWeight, steepnessWeight, maxSteepness;
  int maxEdges; 
  n.getParam("maxEdge", maxEdges);
  n.getParam("resolution", res);
  n.getParam("maxBump", maxBumpiness);
  n.getParam("robotRadius", robotRadius);
  n.getParam("dangerOfUnknown", dangerOfUnknown);
  n.getParam("roughnessWeight", roughnessWeight);
  n.getParam("steepnessWeight", steepnessWeight);
  n.getParam("maxSteepness", maxSteepness);
  /*res = .05;
  maxBumpiness = 3;
  robotRadius = .15;
  dangerOfUnknown = 100; // use high value to prevent all unknown locations from being checked
  roughnessWeight = .5;
  steepnessWeight = 1;
  maxSteepness = 10;*/
  Pathing2D pather (n, res, maxBumpiness, robotRadius,
                    dangerOfUnknown, roughnessWeight,
                    steepnessWeight, maxSteepness, maxEdges, "map");
  ros::Subscriber sub1 = n.subscribe("/octomap_binary", 3, &Pathing2D::octomapCallback, &pather);
  ros::Subscriber sub2 = n.subscribe("/goal", 3, &Pathing2D::goalCallback, &pather);
  ros::Subscriber sub3 = n.subscribe("/vehiclePose", 3, &Pathing2D::poseCallback, &pather);
  std::cout << "Started pathfinding" << std::endl;
  ros::spin();
}
