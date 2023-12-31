
#include "robot.h"

#include <chrono>
#include <thread>

#include "common_config.h"

// 采用半圆半方结构

void Robot::Draw(cv::Mat& show_map) {
  cv::circle(show_map, cv::Point(drift_x_, drift_y_), robot_radius, cv::Scalar(255,0,0), 1);
  int x_robot_direct = std::round(robot_radius * 1.3 * cos(drift_theta_ * kDeg2rad));
  int y_robot_direct = std::round(robot_radius * 1.3 * sin(drift_theta_ * kDeg2rad));
  cv::arrowedLine(show_map, cv::Point(drift_x_, drift_y_), cv::Point(drift_x_+x_robot_direct, drift_y_+y_robot_direct),
    cv::Scalar(255,0,0), 1);
}

double getPathAngle(cv::Point pt1, cv::Point pt2) {
  // 求与水平方向夹角，0-360
  double rad = std::atan2(pt2.y-pt1.y, pt2.x-pt1.x);
  if (rad < 0) {
    rad = 2*M_PI + rad;
  }
  return rad * kRad2deg;
}

// 起点必须与机器人位置相同
int Robot::Move(const std::vector<cv::Point>& path) {
  if (path.front().x != x_ || path.front().y != y_) {
    std::cout << "plz check path start pt" << std::endl;
    return 1;
  }
  if (path.size() < 2) return 1;
  std::chrono::milliseconds t(50);
  std::cout << "stat positon " << x_ << " " << y_ << " " << theta_ << std::endl;


  for (int i = 0; i < path.size()-1; ++i) {
    // 先判断方向是否一致，否，则旋转方向
    double path_angle = getPathAngle(path[i], path[i+1]);
    double cur_twist_speed = twist_speed_;
    if (path_angle < theta_) {
      cur_twist_speed = -twist_speed_;
    }
    if (std::abs(theta_ - path_angle) > kAngleTolerance) {
      std::cout << "new destination " << x_ << " " << y_ << " " << path_angle << std::endl;
    }

    while (std::abs(theta_ - path_angle) > kAngleTolerance) {
      theta_ = theta_ + cur_twist_speed;
      // 旋转运动仅加载旋转噪声,且噪声累加
      double ra = ((double) rand() / (RAND_MAX));
      drift_theta_ = drift_theta_ + (cur_twist_speed + ra*pose_std_err_ );

      // delay
      std::this_thread::sleep_for(t);
      // std::cout << "cur pose " << x_ << " " << y_ << " " << theta_ << std::endl;
    }

    // 平移移动
    double len = cv::norm(path[i+1]- path[i]);
    int seg_cnt = len / linear_speed_;
    cv::Point2d direct_vec = cv::Point2d(path[i+1]- path[i])/(double)seg_cnt;
    std::cout << "new destination " << path[i+1].x << " " << path[i+1].y << " " << theta_ << std::endl;

    // 平移运动仅加载平移噪声
    for(int vec_inc = 1; vec_inc <= seg_cnt; vec_inc++) {
      x_ = x_ + direct_vec.x;
      y_ = y_ + direct_vec.y;

      double rx = ((double) rand() / (RAND_MAX));
      double ry = ((double) rand() / (RAND_MAX));

      drift_x_ = drift_x_ + direct_vec.x + (rx * pose_std_err_);
      drift_y_ = drift_y_ + direct_vec.y + (ry * pose_std_err_);
      // delay
      std::this_thread::sleep_for(t);
      // std::cout << "cur pose " << x_ << " " << y_ << " " << theta_ << std::endl;
    }
  }
}


