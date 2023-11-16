/*
栅格地图绘制
*/

#include <iostream>
#include <vector>

#include "opencv2/opencv.hpp"

int bresenham_line(int x1, int y1, int x2, int y2, std::vector<cv::Point>& line_pts) {
  int dx = x2 - x1;
  int dy = y2 - y1;
  int stepX = dx >= 0 ? 1 : -1;
  int stepY = dy >= 0 ? 1 : -1;
  dx = abs(dx);
  dy = abs(dy);

  if (dx > dy) { // |m| < 1
    int p = 2 * dy - dx;
    int y = y1;
    for (int x = x1; x != x2; x += stepX) {
      line_pts.emplace_back(x, y);
      if (p > 0) {
        y += stepY;
        p -= 2 * dx;
      }
      p += 2 * dy;
    }
  }  else { // |m| >= 1
    int p = 2 * dx - dy;
    int x = x1;
    for (int y = y1; y != y2; y += stepY) {
      line_pts.emplace_back(x, y);
      if (p > 0) {
        x += stepX;
        p -= 2 * dy;
      }
      p += 2 * dx;
    }
  }  return 0;
}


int main() {

  cv::Mat canvs = cv::Mat::zeros(400, 400, CV_8UC3);

  std::vector<cv::Point> pts, pts2, pts3, pts4;
  pts.reserve(100);

  bresenham_line(200, 200, 200, 250, pts);
  bresenham_line(200, 200, 100, 200, pts2);
  bresenham_line(200, 200, 300, 100, pts3);
  bresenham_line(200, 200, 100, 100, pts4);

  for(const auto& pt : pts) {
    canvs.at<cv::Vec3b>(pt) = cv::Vec3b(0,0,255);
  }

  for(const auto& pt : pts2) {
    canvs.at<cv::Vec3b>(pt) = cv::Vec3b(0,0,255);
  }

  for(const auto& pt : pts3) {
    canvs.at<cv::Vec3b>(pt) = cv::Vec3b(0,0,255);
  }

  for(const auto& pt : pts4) {
    canvs.at<cv::Vec3b>(pt) = cv::Vec3b(0,0,255);
  }



  return 0;
}