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
    grid_map_ = cv::Mat::zeros(height, width, CV_8UC1);
    grid_map_canvs_ = cv::Mat::zeros(height, width, CV_8UC3);
  }

  CostMap(const std::string& map_file_path) {
    grid_map_ = cv::imread(map_file_path, cv::IMREAD_GRAYSCALE);
    grid_map_canvs_ = cv::imread(map_file_path, cv::IMREAD_COLOR);
  }
  CostMap(cv::Mat map_in) {
    grid_map_ = map_in.clone();
    grid_map_canvs_.setTo(grid_map_);
  }

  cv::Mat GetGridMap() const {
    return grid_map_;
  }

  cv::Mat GetGridMapCanvs() {
    grid_map_canvs_.setTo(grid_map_);
    return grid_map_canvs_;
  }

  int DrawScanData(const ScanData& scan_data, int method = 0);

 private:
  cv::Mat grid_map_;
  cv::Mat grid_map_canvs_;

 int bresenham_line(int x1, int y1, int x2, int y2, std::vector<cv::Point>& line_pts);


};