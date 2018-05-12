#include <geometry_msgs/Point.h>
#include <opencv2/opencv.hpp>
#include <pathing2d/util.h>

cv::Mat filter2D(cv::Mat src, int ddepth, cv::Mat & kernel,
    const cv::Point & origin, double delta,
    int border_type, const cv::Scalar & border_value)
{
  cv::Mat dst;
  int s = int(ceil(float(kernel.rows)/2.0));
  copyMakeBorder(src, dst, s,s,s,s, border_type, border_value);
  cv::filter2D(dst, dst, ddepth, kernel,
      origin, delta, border_type);
  // Remove border
  cv::Rect roi (cv::Point(s,s), src.size());
  return dst(roi);
}

cv::Mat circularFilter(int rad) {
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



