/*
单线激光雷达仿真
性能参数模拟RPLidar A1M8
*/
#pragma once

#include <iostream>
#include <vector>

#include "opencv2/opencv.hpp"

#include "common_config.h"
#include "robot.h"

// 激光雷达每帧扫描数据
struct ScanData {
  ScanData() {
    laser_distance.resize(kScanSize);
    laser_angle.resize(kScanSize);
    values.resize(kScanSize);

    robot_base_pts.resize(kScanSize);
    robot_base_angle.resize(kScanSize);
    nb_points = kScanSize;
  }

  void TransRaw(cv::Point base_emit_pos, double base_angle,
      std::vector<cv::Point2d>& trans_laser_pts) const {
    trans_laser_pts.clear();
    double x_end, y_end;
    for (int i = 0; i < nb_points; i++) {
      if (values[i] == kObstacle) {
        x_end = base_emit_pos.x + std::cos((base_angle+laser_angle[i])*kDeg2rad)*laser_distance[i];
        y_end = base_emit_pos.y + (-std::sin((base_angle+laser_angle[i])*kDeg2rad)*laser_distance[i]);// 转到图像坐标系
        trans_laser_pts.emplace_back(x_end, y_end);
      }
    }
  }

  void TransUndistort(std::vector<cv::Point2d>& trans_laser_pts) const{
    trans_laser_pts.clear();
    double x_end, y_end;
    for (int i = 0; i < nb_points; i++) {
      if (values[i] == kObstacle) {
        x_end = robot_base_pts[i].x + std::cos((robot_base_angle[i]+laser_angle[i])*kDeg2rad)*laser_distance[i];
        y_end = robot_base_pts[i].y + (-std::sin((robot_base_angle[i]+laser_angle[i])*kDeg2rad)*laser_distance[i]);  // 转到图像坐标系
        trans_laser_pts.emplace_back(x_end, y_end);
      }
    }
  }


  std::vector<double> laser_distance;  // 激光雷达下的激光反射回来的距离
  std::vector<double> laser_angle;  // 激光雷达下的激光坐标反射时的发射角度
  std::vector<int> values;  // 标记为障碍物点和非障碍物点

  std::vector<cv::Point> robot_base_pts;  // 激光雷达发出时候的基座坐标点
  std::vector<double> robot_base_angle;  // 激光雷达所在基座发出的base角度
  int nb_points;
};

class Lidar {
 public:
  Lidar() = default;
  Lidar(double std_err) {
    std_err_ = std_err;
  }
  void SetLaserDetectionMax(int max_len_pixel) {
    detection_max_ = max_len_pixel;
  }
  
  int GetLaserDetectionMax() const {
    return detection_max_;
  }

  int Scan(ScanData& scan_data, const Robot& robot, const cv::Mat& map);
 
  int Ray(double x_start, double y_start, double angle, double& laser_dist, double& laser_angle, const cv::Mat& map);
 private:
 
 public:
  // 雷达参数
  int scan_size_ = kScanSize;  // 一帧激光雷达的雷达束数量
  double reselution_ = 2.;  // unit: pixel
  int angle_min_ = 0;  // 激光雷达开始扫描的角度
  int angle_max_ = 360;  // 激光雷达停止扫描的角度

  int detection_min_ = 0.15 * kPixelPerMeter;  // 激光雷达最大探测距离，超过该距离的深度范围0
  int detection_max_ = 8 * kPixelPerMeter;  // 激光雷达最大探测距离，超过该距离的深度范围0

  double std_err_ = 0.2;
};