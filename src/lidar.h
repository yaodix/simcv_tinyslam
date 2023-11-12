/*
单线激光雷达仿真
性能参数模拟RPLidar A1M8
*/

#include <iostream>
#include <vector>

#include "opencv2/opencv.hpp"

#include "common_config.h"
#include "robot.h"

// 激光雷达每帧扫描数据
struct ScanData {
  ScanData() {
    x.resize(kScanSize);
    y.resize(kScanSize);
    values.resize(kScanSize);
    nb_points = kScanSize;
  }
  std::vector<double> x;  // 激光雷达下的y坐标点
  std::vector<double> y;  // 激光雷达下的y坐标点
  std::vector<int> values;  // 标记为障碍物点和非障碍物点
  int nb_points;
};

class Lidar {
 public:

  int Scan(ScanData& scan_data, const Robot& robot, const cv::Mat& map);
 
 private:
  int ray(double x_start, double y_start, double angle, int& x_end, int& y_end, const cv::Mat& map);
 
 public:
  // 雷达参数
  int scan_size_ = kScanSize;  // 一帧激光雷达的雷达束数量
  double reseluton_ = 2.;  // unit: pixel
  int angle_min_ = 0;  // 激光雷达开始扫描的角度
  int angle_max_ = 360;  // 激光雷达停止扫描的角度

  int detection_min_ = 0.15 * kPixelPerMeter;  // 激光雷达最大探测距离，超过该距离的深度范围0
  int detection_max_ = 8 * kPixelPerMeter;  // 激光雷达最大探测距离，超过该距离的深度范围0

  double std_err_ = 0.2;
};