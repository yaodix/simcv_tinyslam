/*
机器人移动-平移、旋转
*/

#include <iostream>
#include <vector>

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
  path.emplace_back(140, 200);
  path.emplace_back(300, 130);



  robot.Move();

  cv::Mat map_canvs = cost_map.GetGridMapCanvs().clone();
  robot.Draw(map_canvs);




  return 0;
}