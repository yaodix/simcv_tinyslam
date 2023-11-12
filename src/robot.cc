

#include "robot.h"

// 采用半圆半方结构
void Robot::InitRobot() {

}

void Robot::Draw(cv::Mat& show_map) {
  cv::circle(show_map, cv::Point(x_, y_),robot_radius, cv::Scalar(255,0,0), 2);
  int x_robot_direct = std::round(robot_radius * cos(theta_));
  int y_robot_direct = std::round(robot_radius * sin(theta_));
  cv::arrowedLine(show_map, cv::Point(x_, y_), cv::Point(x_robot_direct, y_robot_direct),
    cv::Scalar(255,0,0), 2);
}

// 起点必须与机器人位置相同
int Robot::Move(const std::vector<cv::Point>& path) {
  
}

