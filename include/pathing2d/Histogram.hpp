#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>

template <typename T>
class Histogram {
  std::vector<std::vector<T> > data;
  int width, height, ox, oy;
  public:
    Histogram(int width, int height, int ox=0, int oy=0);
    std::vector<T> * get(int x, int y);
    void set(int x, int y, std::vector<T> val);
    void add(int x, int y, T val);
    cv::Mat findMaxes();
};

template <typename T>
Histogram<T>::Histogram(int width, int height, int ox, int oy) : ox(ox), oy(oy), width(width), height(height) {
  data.resize(width*height);
}

template <typename T>
std::vector<T> * Histogram<T>::get(int x, int y) {
  return &data[(x-ox)+(y-oy)*width];
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
cv::Mat Histogram<T>::findMaxes() {
  cv::Mat out (height, width, CV_32F);
  std::cout << data.size() << std::endl;
  for (int i=0; i<height; i++) {
    for (int j=0; j<width; j++) {
      std::vector<T> * vals = get(i+ox, j+oy);
      //Find max
      std::cout << vals->size() << std::endl;
      if (vals->size() > 6) {
        // Use inter quartile range then take max value
        std::sort(vals->begin(), vals->end());
        float Q1 = vals->at(int(floor(vals->size() / 4)));
        float Q3;
        if (vals->size() % 2) {
          Q3 = vals->at(int(floor(3*(vals->size()+1) / 4)));
        } else {
          Q3 = vals->at(int(floor(3*vals->size() / 4)));
        }
        // Max when discarding outliers
        out.at<float>(i,j) = Q3 + 1.5*(Q3-Q1);
      } else if (vals->size() > 0) {
        // Use average
        T sum;
        for (T val : *vals) {
          sum += val;
        }
        out.at<float>(i,j) = sum/vals->size();
      } else {
        out.at<float>(i,j) = 9999;
      }
    }
  }
  return out;
}
