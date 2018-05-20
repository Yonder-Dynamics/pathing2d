#include <iostream>
#include <ros/ros.h>
#include <pathing2d/Pathing2D.h>


int main(int argc, char *argv[]) {
  ros::init(argc, argv, "pathing");
  ros::NodeHandle n;
  Pathing2D pather (n);
  ros::Subscriber sub1 = n.subscribe("/octomap_binary", 3, &Pathing2D::octomapCallback, &pather);
  ros::Subscriber sub2 = n.subscribe("/move_base_simple/goal", 3, &Pathing2D::goalCallback, &pather);
  ros::Subscriber sub3 = n.subscribe("/fusion/local_fusion/filtered", 3, &Pathing2D::poseCallback, &pather);
  std::cout << "Started pathfinding" << std::endl;
  ros::spin();
}
