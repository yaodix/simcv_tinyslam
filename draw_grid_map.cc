/*
栅格地图绘制
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
  Lidar lidar(0.2);
  lidar.SetLaserDetectionMax(100);
  CostMap cost_map(canvs);

  ScanData scan_data;
  lidar.Scan(robot, cost_map.GetGtMap());
  cost_map.DrawScanData(scan_data, 2);
  cv::Mat map_canvs = cost_map.GetGridMapCanvs().clone();
  robot.Draw(map_canvs);


  return 0;
}