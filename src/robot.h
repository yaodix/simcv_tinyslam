#pragma once

#include <iostream>
#include <math.h>

#include "opencv2/opencv.hpp"


class Robot {
 public:
  // 初始化机器人
  void InitRobot();

  void Draw(cv::Mat& show_map);

  int SetSpeed(double linear_speed, double twist_speed) {
    linear_speed_ = linear_speed_;
    twist_speed = twist_speed_;
  }

  int SetPose(double x, double y, double theta) {
    x_ = x;
    y_ = y;
    theta_ = theta;
  }

  int GetPose(double& x, double& y, double& theta) const {
    x = x_;
    y = y_;
    theta = theta_;
  }

  // 移动过程，仅含原地旋转和直线运动
  int Move(const std::vector<cv::Point>& path);  // 轨迹

 public:
  int robot_radius = 5;  // unit: pixel

  double linear_speed_ = 0;
  double twist_speed_ = 0;

  double x_, y_;   // 位置，单位 pixel
  double theta_;  // 机器人角度，单位 degree

};