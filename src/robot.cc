
#include "robot.h"
#include "common_config.h"

// 采用半圆半方结构

void Robot::Draw(cv::Mat& show_map) {
  cv::circle(show_map, cv::Point(x_, y_),robot_radius, cv::Scalar(255,0,0), 1);
  int x_robot_direct = std::round(robot_radius * cos(theta_));
  int y_robot_direct = std::round(robot_radius * sin(theta_));
  cv::arrowedLine(show_map, cv::Point(x_, y_), cv::Point(x_+x_robot_direct, y_+y_robot_direct),
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
  for (int i =0; i < path.size()-1; ++i) {
    // 先判断方向是否一致，否，则旋转方向
    double path_angle = getPathAngle(path[i], path[i+1]);
    double cur_twist_speed = twist_speed_;
    if (path_angle < theta_) {
      cur_twist_speed = -twist_speed_;
    }

    while (std::abs(theta_ - path_angle) > kAngleTolerance) {
      theta_ += twist_speed_;
      // delay
    }

    // 平移移动
    double len = cv::norm(path[i+1]- path[i]);
    int seg_cnt = len / linear_speed_;
    cv::Point direct_vec = (path[i+1]- path[i])/len;

    for(int vec_inc = 1; vec_inc <= seg_cnt; vec_inc++) {
      x_ = x_ + direct_vec.x * vec_inc;
      y_ = y_ + direct_vec.y * vec_inc;
      // delay
      
    }
    
  }

}


