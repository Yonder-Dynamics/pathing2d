#pragma once
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>

template <typename T>
class Histogram {
  std::vector<std::vector<T> > data;
  int width, height, ox, oy;
  public:
    Histogram(int width, int height, int ox=0, int oy=0);
    ~Histogram();
    std::vector<T> & get(int x, int y);
    void set(int x, int y, std::vector<T> val);
    void add(int x, int y, T val);
    void findExtrema(cv::Mat * min, cv::Mat * max, cv::Mat * unknown, float dangerOfUnknown);
};

template <typename T>
Histogram<T>::Histogram(int width, int height, int ox, int oy) : ox(ox), oy(oy), width(width), height(height) {
  data.resize(width*height);
}

template <typename T>
Histogram<T>::~Histogram() {
}

template <typename T>
std::vector<T> & Histogram<T>::get(int x, int y) {
  return data[(x-ox)+(y-oy)*width];
}

template <typename T>
void Histogram<T>::set(int x, int y, std::vector<T> val) {
  data[(x-ox)+(y-oy)*width] = val;
}

template <typename T>
void Histogram<T>::add(int x, int y, T val) {
  //std::cout << x-ox << ", " << y-oy << std::endl;
  //std::cout << "WH: " << width << ", " << height << std::endl;
  data[(x-ox)+(y-oy)*width].push_back(val);
}

// Row major
template <typename T>
void Histogram<T>::findExtrema(cv::Mat * min, cv::Mat * max, cv::Mat * unknown, float dangerOfUnknown) {
  *min = cv::Mat::zeros(height, width, CV_32F);
  *max = cv::Mat::zeros(height, width, CV_32F);
  *unknown = cv::Mat::zeros(height, width, CV_8UC1);
  for (int i=0; i<height; i++) {
    for (int j=0; j<width; j++) {
      std::vector<T> & vals = get(j+ox, i+oy);
      //Find max
      if (vals.size() > 6) {
        // Use inter quartile range then take max value
        std::sort(vals.begin(), vals.end());
        float Q1 = vals.at(int(floor(vals.size() / 4)));
        float Q3 = vals.at(int(floor(3*vals.size() / 4)));
        // Max when discarding outliers
        max->at<float>(i,j) = Q3 + 1.5*(Q3-Q1);
        min->at<float>(i,j) = Q1 - 1.5*(Q3-Q1);
        unknown->at<uint8_t>(i,j) = 0;
      } else if (vals.size() > 0) {
        // Use average
        T sum = 0;
        for (T val : vals) {
          sum += val;
        }
        max->at<float>(i,j) = sum/vals.size();
        min->at<float>(i,j) = sum/vals.size();
        unknown->at<uint8_t>(i,j) = 0;
      } else {
        max->at<float>(i,j) = dangerOfUnknown;
        min->at<float>(i,j) = dangerOfUnknown;
        unknown->at<uint8_t>(i,j) = 1;
      }
    }
  }
}
