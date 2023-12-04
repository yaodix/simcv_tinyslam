/*
栅格地图绘制
*/

#include <iostream>
#include <vector>
#include <string>

#include "opencv2/opencv.hpp"

#include "src/common_config.h"
#include "src/robot.h"
#include "src/lidar.h"
#include "src/cost_map.h"


int main(int argc, char** argv) {
  if (argc < 2) {
    std::cout << "input shoule include show method,0-1-2";
    return 1;
  }
  
  int method = std::stoi(std::string(argv[1]));
  cv::Mat canvs = cv::Mat::zeros(300, 400, CV_8UC1);
  canvs.setTo(kUnknown);
  cv::rectangle(canvs, {50, 50}, {350, 250}, cv::Scalar::all(kObstacle),3);

  Robot robot(80, 100, 0);
  Lidar lidar(0.2);
  lidar.SetLaserDetectionMax(180);
  CostMap cost_map(canvs);

  ScanData scan_data;
  lidar.Scan(robot, cost_map.GetGtMap());
  lidar.GetScanData(scan_data);

  cost_map.DrawScanData(scan_data, method);
  cv::Mat map_canvs = cost_map.GetGridMapCanvs().clone();
  robot.Draw(map_canvs);
  
  cv::imwrite("./workspace/grid_map.png", map_canvs);
  cv::imshow("show", map_canvs);
  cv::waitKey(30);
  return 0;
}