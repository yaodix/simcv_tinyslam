#include "cost_map.h"

#include <iostream>
#include <vector>

int CostMap::DrawScanData(const ScanData& scan_data, int method /*= 0*/, cv::Scalar color) {
  if (map_canvs_.empty()) {
    map_canvs_ = cv::Mat(gt_map_.size(), CV_8UC3, cv::Scalar::all(kUnknown));
  }
  std::vector<cv::Point2d> trans_in_map;
  trans_in_map.reserve(500);
  scan_data.TransUndistort(trans_in_map);
  DrawScanData(trans_in_map, scan_data.robot_base_pts[0], method, color);
}

int CostMap::DrawScanData(const std::vector<cv::Point2d>& trans_in_map,
    cv::Point2d robot_base_pt, int method , cv::Scalar color) {
  if (map_canvs_.empty()) {
    map_canvs_ = cv::Mat(gt_map_.size(), CV_8UC3, cv::Scalar::all(kUnknown));
  }

  if(method == 1) {  // 绘制点和对应线
    std::vector<cv::Point> pts(200);
    for (int i = 0; i < trans_in_map.size(); i++) {
      bresenham_line(robot_base_pt.x, robot_base_pt.y,
        trans_in_map[i].x, trans_in_map[i].y, pts);
      for (const auto& pt : pts) {
        map_canvs_.at<cv::Vec3b>(pt) = cv::Vec3b::all(kFreeSpace);
      }
    }  
  } else if(method == 2) {  // 绘制点和对应线，美化线的显示
    // 对当前线条进行插补
    std::vector<cv::Point2d> fix_gap_pts;
    fix_gap_pts.reserve(trans_in_map.size()/10);
    for (int i = 0; i < trans_in_map.size()-1; i++) {
      if ((abs(trans_in_map[i].x - trans_in_map[i+1].x) + abs(trans_in_map[i].y - trans_in_map[i+1].y)) <= 4) {
        std::vector<cv::Point> tmp_pts;
        bresenham_line(trans_in_map[i], trans_in_map[i+1], tmp_pts);
        fix_gap_pts.insert(fix_gap_pts.end(), tmp_pts.begin(), tmp_pts.end());
      }
    }

    std::vector<cv::Point2d> trans_pts_all = trans_in_map;
    trans_pts_all.insert(trans_pts_all.end(), fix_gap_pts.begin(), fix_gap_pts.end());
    std::vector<cv::Point> pts(200);
    for (int i = 0; i < trans_pts_all.size(); i++) {
      bresenham_line(robot_base_pt.x, robot_base_pt.y, trans_pts_all[i].x, trans_pts_all[i].y, pts);
      for (const auto& pt : pts) {
        map_canvs_.at<cv::Vec3b>(pt) = cv::Vec3b::all(kFreeSpace);
      }
    } 
  }
  for (const auto& pt : trans_in_map) {
    map_canvs_.at<cv::Vec3b>(std::round(pt.y), std::round(pt.x)) = cv::Vec3b(color[0], color[1], color[2]);
  }
}

int CostMap::bresenham_line(cv::Point pt1, cv::Point pt2, std::vector<cv::Point>& line_pts) {
  return bresenham_line(pt1.x, pt1.y, pt2.x, pt2.y, line_pts);
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
