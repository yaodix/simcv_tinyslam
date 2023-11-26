/*
机器人移动-平移、旋转
*/

#include <iostream>
#include <vector>
#include <thread>

#include "opencv2/opencv.hpp"

#include "src/common_config.h"
#include "src/robot.h"
#include "src/lidar.h"
#include "src/cost_map.h"


int main() {
  cv::Mat canvs = cv::Mat::zeros(300, 400, CV_8UC1);
  canvs.setTo(kUnknown);
  cv::rectangle(canvs, {50, 50}, {350, 250}, cv::Scalar::all(kObstacle),3);

  Robot robot(80, 100, 0);
  CostMap cost_map(canvs);

  std::vector<cv::Point> path;
  path.emplace_back(robot.x_, robot.y_);
  path.emplace_back(140, 200);
  path.emplace_back(300, 200);
  path.emplace_back(340, 60);

  std::thread robot_move(&Robot::Move, &robot, path);
  robot_move.detach();

  while(1) {
    double r_x, r_y, r_theta;
    robot.GetPose(r_x, r_y, r_theta);
    cv::Mat map_canvs = cost_map.GetGridMapCanvs().clone();
    cv::polylines(map_canvs, path, false, cv::Scalar(0, 123, 255), 1);
    robot.Draw(map_canvs);
    cv::imshow("robot_move", map_canvs);
    cv::waitKey(30);    
  }

  return 0;
}