#pragma once

#include <iostream>
#include <math.h>
#include <atomic>

#include "common_config.h"
#include "opencv2/opencv.hpp"


class Robot {
 public:
  // 初始化机器人
  Robot(double x, double y, double theta) {
    SetPose(x, y, theta);
  }

  Robot(double x, double y, double theta, double std_err) {
    SetPose(x, y, theta);
    pose_std_err_ = std_err;
  }

  void SetPoseStdErr(double std_err) {
    pose_std_err_ = std_err;
  }

  void Draw(cv::Mat& show_map);

  void SetSpeed(double linear_speed, double twist_speed) {
    linear_speed_ = linear_speed_;
    twist_speed = twist_speed_;
  }

  void SetPose(double x, double y, double theta) {
    x_ = x;
    y_ = y;
    theta_ = theta;

    drift_x_ = x;
    drift_y_ = y;
    drift_theta_ = theta;
  }

  void GetRealPose(double& x, double& y, double& theta) const {
    x = x_;
    y = y_;
    theta = theta_;
  }

  void GetDriftPose(double& x, double& y, double& theta) const {
    x = drift_x_;
    y = drift_y_;
    theta = drift_theta_;
  }


  // 移动过程，仅含原地旋转和直线运动
  int Move(const std::vector<cv::Point>& path);  // 轨迹

 public:
  int robot_radius = 0.6 * kPixelPerMeter / 2;  // 方便显示设置0.6m大小， unit: pixel

  double linear_speed_ = 1;
  double twist_speed_ = 3;

  double pose_std_err_ = 0.;  // 机器人运动噪声

  std::atomic<double> x_, y_;   // 位置，单位 pixel
  std::atomic<double> theta_;  // 机器人角度，单位 degree

  std::atomic<double> drift_x_, drift_y_;   // 偏移位置，单位 pixel
  std::atomic<double> drift_theta_;  // 机器人偏移角度，单位 degree

};