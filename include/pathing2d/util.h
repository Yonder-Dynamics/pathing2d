#pragma once
#include <geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>

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
