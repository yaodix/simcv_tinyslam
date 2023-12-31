/*
单线激光雷达仿真
性能参数模拟RPLidar A1M8
*/
#pragma once

#include <iostream>
#include <vector>
#include <mutex>

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

  // 仅将激光雷达坐标系下物体转到像素坐标系
  void TransRaw(cv::Point base_emit_pos, double base_angle,
      std::vector<cv::Point2d>& trans_laser_pts) const {
    trans_laser_pts.clear();
    double x_end, y_end;
    for (int i = 0; i < nb_points; i++) {
      if (values[i] == kObstacle) {
        x_end = base_emit_pos.x + std::cos((base_angle+laser_angle[i])*kDeg2rad)*laser_distance[i];
        y_end = base_emit_pos.y + (std::sin((base_angle+laser_angle[i])*kDeg2rad)*laser_distance[i]);// 转到图像坐标系
        trans_laser_pts.emplace_back(x_end, y_end);
      }
    }
  }

  // 去运动畸变
  void TransUndistort(std::vector<cv::Point2d>& trans_laser_pts) const{
    trans_laser_pts.clear();
    double x_end, y_end;
    for (int i = 0; i < nb_points; i++) {
      if (values[i] == kObstacle) {
        x_end = robot_base_pts[i].x + std::cos((robot_base_angle[i]+laser_angle[i])*kDeg2rad)*laser_distance[i];
        y_end = robot_base_pts[i].y + (std::sin((robot_base_angle[i]+laser_angle[i])*kDeg2rad)*laser_distance[i]);  // 转到图像坐标系
        trans_laser_pts.emplace_back(x_end, y_end);
      }
    }
  }


  std::vector<double> laser_distance;  // 激光雷达下的激光反射回来的距离,极坐标
  std::vector<double> laser_angle;  // 激光雷达下的激光坐标反射时的发射角度,极坐标
  std::vector<int> values;  // 标记为障碍物点和非障碍物点

  std::vector<cv::Point> robot_base_pts;  // 激光雷达发出时候的基座坐标点
  std::vector<double> robot_base_angle;  // 激光雷达所在基座发出的base角度
  int nb_points;
};

class Lidar {
 public:
  Lidar() = default;
  Lidar(double std_err) {  // 机器人姿态误差
    ray_std_err_ = std_err;
  }
  void SetLaserDetectionMax(int max_len_pixel) {
    range_max_ = max_len_pixel;
  }
  
  int GetLaserDetectionMax() const {
    return range_max_;
  }

  void GetScanData(ScanData& scan_data) {
    sd_mtx.lock();
    scan_data = scan_data_;
    sd_mtx.unlock();
  }
  
  int LoopScan(const Robot& robot, const cv::Mat& map);

  int Scan(const Robot& robot, const cv::Mat& map);
 
  int Ray(double x_start, double y_start, double angle, double& laser_dist, double& laser_angle, const cv::Mat& map);
 
 public:

  ScanData scan_data_;
  // 雷达参数
  int scan_size_ = kScanSize;  // 一帧激光雷达的雷达束数量
  double reselution_ = 2.;  // unit: pixel
  int angle_min_ = 0;  // 激光雷达开始扫描的角度
  int angle_max_ = 360;  // 激光雷达停止扫描的角度

  int range_min_ = 0.15 * kPixelPerMeter;  // 激光雷达最大探测距离，超过该距离的深度范围0
  int range_max_ = 8 * kPixelPerMeter;  // 激光雷达最大探测距离，超过该距离的深度范围0

  double pose_std_err_ = 0;
  double ray_std_err_ = 0.2;
  
  std::mutex sd_mtx;
};