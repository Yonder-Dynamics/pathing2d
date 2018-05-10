#include <iostream>
#include <ros/ros.h>
#include <pathing2d/Pathing2D.h>

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "pathing2d");
  ros::NodeHandle n;
  Pathing2D pather (atof(argv[1]));
  ros::Subscriber sub1 = n.subscribe("/octomap_binary", 3, &Pathing2D::octomapCallback, &pather);
  ros::Subscriber sub2 = n.subscribe("/goal", 3, &Pathing2D::goalCallback, &pather);
  ros::Subscriber sub3 = n.subscribe("/vehiclePose", 3, &Pathing2D::poseCallback, &pather);
  std::cout << "Started pathfinding" << std::endl;
  ros::spin();
}
