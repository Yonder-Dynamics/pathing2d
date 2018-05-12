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

void Pathing2D::poseCallback(const nav_msgs::Odometry::ConstPtr& msg) {
  if (!isProcessing) {
    std::cout << "Recieved pose" << std::endl;
    rover = msg->pose.pose;
    gotRover = true;
    if (gotOcto && gotGoal && !isProcessing)
      process();
  }
}

void Pathing2D::goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  if (!isProcessing) {
    std::cout << "Recieved goal" << std::endl;
    goal = msg->pose;
    rover = goal;
    rover.position.x -= 1;
    gotGoal = true;
    if (gotOcto && gotRover && !isProcessing)
      process();
  }
}

void Pathing2D::octomapCallback(const octomap_msgs::Octomap::ConstPtr& msg) {
  if (!isProcessing) {
    std::cout << "Recieved octomap" << std::endl;
    octomap::AbstractOcTree* atree = octomap_msgs::binaryMsgToMap(*msg);
    tree = boost::make_shared<octomap::OcTree>(*(octomap::OcTree *)atree);
    gotOcto = true;
    if (gotGoal && gotRover)
      process();
    delete(atree);
  }
}

cv::Mat Pathing2D::processOctomap() {
  tree->getMetricMin(ox, oy, oz);
  tree->getMetricMax(mx, my, mz);
  tree->getMetricSize(width, height, depth);

  mx = ((ox + mx)/2) + 5;
  my = ((oy + my)/2) + 5;
  mz = ((oz + mz)/2) + 5;

  ox = ((ox + mx)/2) - 5;
  oy = ((oy + my)/2) - 5;
  oz = ((oz + mz)/2) - 5;

  Histogram<float> occupied (
      int(ceil((mx-ox)/res)), int(ceil((my-oy)/res)), 0, 0);
  Histogram<float> empty (
      int(ceil((mx-ox)/res)), int(ceil((my-oy)/res)), 0, 0);

  octomath::Vector3 first(ox, oy, oz);
  octomath::Vector3 second(mx, my, mz);

  // Fill
  for(auto it = tree->begin_leafs_bbx(first, second, 14); it != tree->end_leafs_bbx(); it++) {
    octomath::Vector3 pos = it.getCoordinate();
    int x = int((pos.x()-ox)/res);
    int y = int((pos.y()-oy)/res);
    if (it->getValue() > 0) {
      occupied.add(x, y, pos.z());
    } else {
      empty.add(x, y, pos.z());
    }
  }
  cv::Mat occ_min, occ_max;
  occupied.findExtrema(&occ_min, &occ_max, dangerOfUnknown);
  cv::Mat emp_min, emp_max;
  empty.findExtrema(&emp_min, &emp_max, dangerOfUnknown);
  cv::Mat hist = cv::min(occ_max, emp_min);
  //cv::Mat hist = occ_max/2 + emp_min/2;
  /* Display things
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
  */
  return hist;
}

Graph<float> Pathing2D::buildGraph(
    const cv::Mat & heightHist,
    const cv::Mat & occupancy,
    std::vector<WeightedPoint> & openPoses,
    cv::Mat * graphIndexes,
    float maxBumpiness,
    float roughnessWeight,
    float steepnessWeight,
    float maxSteepness,
    float maxLength,
    int maxEdges)
{
  // Create positions filter by threshold
  *graphIndexes = -cv::Mat::ones(occupancy.rows, occupancy.rows, CV_8UC1);
  for (int i=0; i<occupancy.rows; i++) {
    for (int j=0; j<occupancy.cols; j++) {
      // Exponentiate threshold to match occupationHist
      if (occupancy.at<float>(i, j) < maxBumpiness) {
        WeightedPoint p;
        p.x = j * res + ox;
        p.y = i * res + oy;
        p.z = heightHist.at<float>(i, j);
        p.weight = occupancy.at<float>(i,j);
        graphIndexes->at<uint8_t>(i, j) = openPoses.size();
        openPoses.push_back(p);
      }
    }
  }

  float dist, slope, roughness;
  bool goalWithinHist = (ox < goal.position.x && goal.position.x < mx) &&
                        (oy < goal.position.y && goal.position.y < my);
  Graph<float> g(openPoses.size()+2, maxEdges);
  for (int i=0; i<graphIndexes->rows; i++) {
    for (int j=0; j<graphIndexes->cols; j++) {
      if (graphIndexes->at<float>(i, j) != -1) {
        
        uint8_t ind1 = graphIndexes->at<uint8_t>(i, j);
        WeightedPoint & p1 = openPoses[ind1];
        // Add key edges to graph
        // Rover
        dist = dist2d(p1, rover.position);
        slope = abs(p1.z-rover.position.z)/dist;
        g.addEdge(ind1, 0,
            dist + slope*steepnessWeight + p1.weight*roughnessWeight);
        // Goal
        if (goalWithinHist) {
          dist = dist2d(p1, goal.position);
          slope = abs(p1.z-goal.position.z)/dist;
          g.addEdge(ind1, openPoses.size()+1,
              dist + slope*steepnessWeight + p1.weight*roughnessWeight);
        }
        // Add local edges to graph
        for (int dy=-1; dy<=1; dy++) {
          for (int dx=-1; dx<=1; dx++) {
            if ((i+dy > 0 && i+dy < occupancy.rows) &&
                (j+dx > 0 && j+dx < occupancy.cols) &&
                (graphIndexes->at<float>(i+dy, j+dx) != -1)) {
              uint8_t ind2 = graphIndexes->at<uint8_t>(i+dy, j+dx);
              WeightedPoint & p2 = openPoses[ind2];

              dist = dist2d(p1, p2);
              slope = abs(p1.z-p2.z)/dist;
              roughness = p1.weight/2+p2.weight/2;

              if (dist < maxLength && slope < maxSteepness) {
                g.addEdge(ind1, ind2,
                    dist + slope*steepnessWeight + roughness*roughnessWeight);
              }
            }
          }
        }
      }
    }
  }
  return g;
}

void Pathing2D::process() {
  isProcessing = true;
  std::cout << "Creating heightmap" << std::endl;
  cv::Mat rawHeight = processOctomap();
  //--------- Calculate slope histogram (derivative) ------------

  cv::Mat edgeHist;
  /// Generate grad_x and grad_y
  cv::Mat grad_x, grad_y;
  cv::Mat abs_grad_x, abs_grad_y;

  /// Preprocessing
  //cv::medianBlur (rawHeight, edgeHist, 5);
  cv::GaussianBlur(rawHeight, edgeHist, cv::Size(5,5), 10, 10,
      cv::BORDER_CONSTANT);//, dangerOfUnknown);

  /// Gradient X
  cv::Sobel( edgeHist, grad_x, -1, 1, 0, 5, 1, 0,
      cv::BORDER_CONSTANT);//, dangerOfUnknown);

  /// Gradient Y
  cv::Sobel( edgeHist, grad_y, -1, 0, 1, 5, 1, 0,
      cv::BORDER_CONSTANT);//, dangerOfUnknown);

  /// Total Gradient (approximate)
  cv::addWeighted( grad_x, 0.5, grad_y, 0.5, 0, edgeHist, -1);
  float avg = float(cv::sum(edgeHist)[0])/edgeHist.rows/edgeHist.cols;
  edgeHist = cv::abs(edgeHist - avg);
  //---------------------------END-------------------------------


  //--------- Perform radius search around each node ------------
  // Create circlular filter

  cv::Mat prefiltered;
  cv::Mat occupationHist;

  // Apply exponential function to edgeHistogram to emphasize large obstacles
  // over collections of small obstacles
  //cv::exp(edgeHist, exp);
  float THRESHOLD = 10;
  cv::threshold(edgeHist, prefiltered, THRESHOLD, THRESHOLD, 2);

  // Apply filter
  cv::Mat filter = circularFilter(robotRadius/res);
  filter /= float(cv::sum(filter)[0]); // Average bumpiness
  // If your filter is too big you will get Nans
  occupationHist = filter2D(prefiltered, -1, filter,
      cv::Point(-1, -1), 0, cv::BORDER_CONSTANT, dangerOfUnknown);

  /* Display
  cv::Mat bw;
  cv::threshold(occupationHist, bw, maxBumpiness, 255, cv::THRESH_BINARY);
  cv::namedWindow("Traversable", 0);
  cv::imshow("Traversable", bw);
  cv::waitKey(0);
  */
  //---------------------------END-------------------------------

  // Publish map
  nav_msgs::OccupancyGrid grid_msg = cvToGrid(occupationHist);
  map_pub.publish(grid_msg);

  // Find all edges for graph
  std::cout << "Building graph" << std::endl;
  std::vector<WeightedPoint> open;
  cv::Mat graphIndexes;
  Graph<float> g = buildGraph(rawHeight, occupationHist, open, &graphIndexes,
      maxBumpiness, roughnessWeight, steepnessWeight, maxSteepness,
      robotRadius, maxEdges);

  // Find path
  std::cout << "Finding shortest path" << std::endl;
  std::vector<size_t> shortestPath = g.shortestPath(0, open.size()+1, open);
  nav_msgs::Path path = construct(shortestPath, open);
  std::cout << path << std::endl;
  traj_pub.publish(path);

  isProcessing = false;
  std::cout << "Done" << std::endl;
}

nav_msgs::Path Pathing2D::construct(const std::vector<size_t> & path,
    const std::vector<WeightedPoint> & nodes) {
  std::vector<geometry_msgs::PoseStamped> poses;

  for (size_t i=0; i<path.size(); i++) {
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = frame_id;

    if (path[i] == nodes.size()+1) {
      pose.pose = goal;
      poses.push_back(pose);
    } else if (path[i] != 0) {
      pose.pose.position = nodes[path[i]];
      // Fancy calculation to determine orientation to goal
      // Or not
      pose.pose.orientation.x = 0;
      pose.pose.orientation.y = 0;
      pose.pose.orientation.z = 0;
      pose.pose.orientation.w = 1;
      poses.push_back(pose);
    }
  }
  nav_msgs::Path path_msg;
  path_msg.poses = poses;
  path_msg.header.frame_id = frame_id;
  return path_msg;
}

nav_msgs::OccupancyGrid Pathing2D::cvToGrid(cv::Mat & mat) {
  cv::Mat intmat;
  cv::normalize(mat.t(), intmat, 0, 100, 32, CV_8UC1);
  nav_msgs::OccupancyGrid grid;
  grid.data.clear();
  grid.data.resize(mat.rows*mat.cols);

  for (int i=0; i<mat.rows; i++) {
    for (int j=0; j<mat.cols; j++) {
      grid.data[(mat.rows-i-1)*mat.cols+(mat.cols-j-1)] = intmat.at<uint8_t>(i, j);
    }
  }

  // Construct message
  grid.header.frame_id = frame_id;
  grid.info.width = mat.cols;
  grid.info.height = mat.rows;
  grid.info.resolution = res;
  grid.info.origin.position.x = ox;
  grid.info.origin.position.y = oy;
  grid.info.origin.position.z = oz;
  return grid;
}
