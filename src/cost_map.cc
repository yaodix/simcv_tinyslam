#include "cost_map.h"

#include <iostream>
#include <vector>




int CostMap::DrawScanData(const ScanData& scan_data, int method /*= 0*/) {
  std::vector<cv::Point2d> trans_in_map;
  trans_in_map.reserve(500);
  scan_data.TransUndistort(trans_in_map);

  if(method == 1) {  // 绘制点和对应线
    std::vector<cv::Point> pts(200);
    for (int i = 0; i < trans_in_map.size(); i++) {
      bresenham_line(scan_data.robot_base_pts[i].x, scan_data.robot_base_pts[i].y,
        trans_in_map[i].x, trans_in_map[i].y, pts);
      for (const auto& pt : pts) {
        grid_map_.at<uchar>(pt) = kFreeSpace;
      }
    }  
  } else if(method == 2) {  // 绘制点和对应线，美化线的显示

  }
  for (const auto& pt : trans_in_map) {
    grid_map_.at<uchar>(pt) = kObstacle;
  }
}

int CostMap::bresenham_line(int x1, int y1, int x2, int y2, std::vector<cv::Point>& line_pts) {
  line_pts.clear();
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
