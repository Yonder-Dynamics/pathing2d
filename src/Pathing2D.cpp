#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#include <octomap_msgs/Octomap.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>

#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>

#include <pathing2d/Graph.hpp>
#include <pathing2d/Histogram.hpp>
#include <pathing2d/Pathing2D.h>

cv::Mat circularFilter(int rad) {
  std::cout << "Rad " << rad << std::endl;
  cv::Mat filter (2*rad+1, 2*rad+1, CV_32F);
  float dist;
  for (int i=0; i<2*rad+1; i++) {
    for (int j=0; j<2*rad+1; j++) {
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
               (a.z-b.z)*(a.z-b.z));
}

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
  //if (gotGoal)
    process();
  delete(atree);
}

cv::Mat Pathing2D::processOctomap() {
  tree->getMetricMin(ox, oy, oz);
  tree->getMetricMax(mx, my, mz);

  Histogram<float> occupied (
      int(ceil((mx-ox)/res)),
      int(ceil((my-oy)/res)),
      int(ceil(ox/res)),
      int(ceil(oy/res)));
  Histogram<float> empty (
      int(ceil((mx-ox)/res)),
      int(ceil((my-oy)/res)),
      int(ceil(ox/res)),
      int(ceil(oy/res)));
  for(auto it = tree->begin(); it != tree->end(); it++) {
    octomath::Vector3 pos = it.getCoordinate();
    int x = int(pos.x()/res);
    int y = int(pos.y()/res);
    if (it->getValue() > 0) {
      occupied.add(x, y, pos.z());
    } else {
      empty.add(x, y, pos.z());
    }
  }
  cv::Mat occ_min, occ_max;
  occupied.findExtrema(&occ_min, &occ_max, dangerOfUnknown);
  std::cout << "OCC "  << mean(occ_max)[0] << std::endl;
  cv::Mat emp_min, emp_max;
  empty.findExtrema(&emp_min, &emp_max, dangerOfUnknown);
  std::cout << "EMP "  << mean(emp_min)[0] << std::endl;
  cv::Mat hist = cv::min(occ_max, emp_min);
  std::cout << "Hist " << mean(hist)[0] << std::endl;
  //cv::Mat hist = occ_max/2 + emp_min/2;
  // Display things
  cv::namedWindow("Hist", 0);
  cv::Mat bw;
  cv::normalize(occ_max, bw, 0, 255, 32, CV_8UC1);
  cv::imshow("Hist", bw);
  cv::waitKey(0);
  cv::normalize(emp_min, bw, 0, 255, 32, CV_8UC1);
  cv::imshow("Hist", bw);
  cv::waitKey(0);
  cv::normalize(hist, bw, 0, 255, 32, CV_8UC1);
  cv::imshow("Hist", bw);
  cv::waitKey(0);
  std::cout << "Done" << std::endl;
  return hist;
}

std::vector<WeightedPoint> Pathing2D::openPositions(
    cv::Mat & heightHist,
    cv::Mat & edgeHist,
    float maxBumpiness,
    float radius,
    float dangerOfUnknown,
    float res)
{

  // Use convolution of a circle with soft edges with zero padding so only
  // safe points are navigated to
  // Create circlular filter
  cv::Mat filter = circularFilter(radius/res);

  // Apply exponential function to edgeHistogram to emphasize large obstacles
  // over collections of small obstacles
  cv::Mat exp;
  cv::exp(edgeHist, exp);
  float THRESHOLD = 10;
  cv::threshold(exp, exp, THRESHOLD, THRESHOLD, 2);
  // Apply filter
  cv::Mat occupationHist;
  filter /= float(cv::sum(filter)[0]); // Average bumpiness
  //filter = cv::Mat::ones( 3, 3, CV_32F );
  // If your filter is too big you will get Nans
  //cv::filter2D(exp, occupationHist, exp.depth(), filter,
  //    cv::Point(filter.cols/2-1, filter.rows/2-1), 0, cv::BORDER_CONSTANT);//, dangerOfUnknown);
  //filter2D(src, dst, ddepth , kernel, anchor, delta, BORDER_CONSTANT );
  cv::filter2D(exp, occupationHist, -1, filter,
      cv::Point(-1, -1), 0, cv::BORDER_CONSTANT);//, dangerOfUnknown);
  //std::cout << occupationHist << std::endl;
  //* 
  cv::Mat bw;
  cv::normalize(occupationHist, bw, 0, 255, 32, CV_8UC1);
  cv::namedWindow("Traversable", 0);
  cv::imshow("Traversable", bw);
  cv::waitKey(0);
  //*/

  // Create positions filter by threshold
  std::vector<WeightedPoint> openPoses;
  for (int i=0; i<edgeHist.rows; i++) {
    for (int j=0; j<edgeHist.cols; j++) {
      // Exponentiate threshold to match occupationHist
      if (occupationHist.at<float>(i, j) < std::exp(maxBumpiness)) {
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

Graph<float> Pathing2D::buildGraph(
    std::vector<WeightedPoint> & points,
    float roughnessWeight,
    float steepnessWeight,
    float maxSteepness,
    float maxLength,
    int maxEdges)
{
  Graph<float> g(points.size(), maxEdges);
  // The steepness of an edge is an average of each node's roughnes
  float dist, slope, roughness;
  bool goalWithinHist = (ox < goal.position.x && goal.position.x < mx) &&
                        (oy < goal.position.y && goal.position.y < my);
  WeightedPoint * p1, * p2;
  for (int i=0; i<points.size(); i++) {
    p1 = &points[i];
    // Add goal to each point if goal is within map
    //if (goalWithinHist) {
    if (true) {
      dist = dist2d(*p1, goal.position);
      slope = abs(p1->z-goal.position.z)/dist;
      if (dist < maxLength && slope < maxSteepness) {
        g.addEdge(i, points.size(), dist + slope*steepnessWeight + p1->weight*roughnessWeight);
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
  std::cout << "Creating heightmap" << std::endl;
  cv::Mat rawHeight = processOctomap();
  cv::Mat edgeHist;

  std::cout << "Running Scharr" << std::endl;
  //--------- Calculate slope histogram (derivative) ------------
  // Processing
  
  cv::medianBlur (rawHeight, edgeHist, 3);
  cv::pyrDown(rawHeight, edgeHist);
  //cv::GaussianBlur(edgeHist, edgeHist, cv::Size(3,3), 0, 0,
  //    cv::BORDER_CONSTANT);//, dangerOfUnknown);

  cv::Mat bw;
  cv::normalize(edgeHist, bw, 0, 255, 32, CV_8UC1);
  cv::namedWindow("Blur", 0);
  cv::imshow("Blur", bw);
  cv::waitKey(0);
  /// Generate grad_x and grad_y
  cv::Mat grad_x, grad_y;
  cv::Mat abs_grad_x, abs_grad_y;

  /// Gradient X
  cv::Scharr( edgeHist, grad_x, -1, 1, 0, 1, 0,
      cv::BORDER_CONSTANT);//, dangerOfUnknown);
  //cv::convertScaleAbs( grad_x, abs_grad_x );

  /// Gradient Y
  cv::Scharr( edgeHist, grad_y, -1, 0, 1, 1, 0,
      cv::BORDER_CONSTANT);//, dangerOfUnknown);
  //cv::convertScaleAbs( grad_y, abs_grad_y );

  /// Total Gradient (approximate)
  cv::addWeighted( grad_x, 0.5, grad_y, 0.5, 0, edgeHist, -1);
  float avg = float(cv::sum(edgeHist)[0])/edgeHist.rows/edgeHist.cols;
  edgeHist = cv::abs(edgeHist - avg);

  std::cout << "Finding open positions" << std::endl;
  std::vector<WeightedPoint> open = openPositions(
      rawHeight, edgeHist, maxBumpiness, robotRadius, dangerOfUnknown, res);
  std::cout << "Building graph" << std::endl;
  Graph<float> g = buildGraph( open, roughnessWeight, steepnessWeight,
      maxSteepness, robotRadius-1, maxEdges);
  std::cout << "Done" << std::endl;
  //auto shortestPath = g.shortestPath(open.size());
}
