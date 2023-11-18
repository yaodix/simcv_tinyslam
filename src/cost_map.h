#pragma once

#include "opencv2/opencv.hpp"

#include "lidar.h"
#include "robot.h"

enum class DRAWMETHOD : uint8_t {
  RAW_DOT

};

class CostMap {
 public:
  CostMap(int width, int height) {
    gt_map_ = cv::Mat::zeros(height, width, CV_8UC1);
    gt_map_.setTo(kUnknown);
  }

  CostMap(const std::string& map_file_path) {
    gt_map_ = cv::imread(map_file_path, cv::IMREAD_GRAYSCALE);
  }
  CostMap(cv::Mat map_in) {
    gt_map_ = map_in.clone();
  }

  cv::Mat GetGtMap() const {
    return gt_map_;
  }

  cv::Mat GetGridMapCanvs() {
  if (map_canvs_.empty()) {
    map_canvs_ = cv::Mat(gt_map_.size(), CV_8UC3, cv::Scalar::all(kUnknown));
  }
    return map_canvs_;
  }

  int DrawScanData(const ScanData& scan_data, int method = 0);

 private:
  cv::Mat gt_map_;
  cv::Mat map_canvs_;

 int bresenham_line(cv::Point pt1, cv::Point pt2, std::vector<cv::Point>& line_pts);
 int bresenham_line(int x1, int y1, int x2, int y2, std::vector<cv::Point>& line_pts);


};