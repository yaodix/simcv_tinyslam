#pragma once

#include <iostream>
#include <math.h>

#include "opencv2/opencv.hpp"


class Robot {
 public:
  // 初始化机器人
  void InitRobot();
  void Draw(cv::Mat& show_map);
  int SetPose(double x, double y, double theta);

  int GetPose(double& x, double& y, double& theta) const {
    x = x_;
    y = y_;
    theta = theta_;
  }

  // 移动过程，旋转和平移独立运动
  int Move(const std::vector<cv::Point>& path);  // 轨迹

 public:
  int robot_radius = 5;  // unit: pixel

  double speed_ = 0;  // 直线行驶角度
  double twist_ = 0;  // 旋转角度

  double x_, y_;   // 位置，单位 pixel
  double theta_;  // 机器人角度，单位 degree

};