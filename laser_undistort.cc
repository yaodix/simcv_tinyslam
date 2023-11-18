#include <iostream>

#include <opencv2/opencv.hpp>

#include "src/common_config.h"
#include "src/lidar.h"

void test_CastStatic() {
  cv::Mat map = cv::Mat::zeros(800, 400, CV_8UC1);
  map.setTo(kUnknown);
  cv::line(map, cv::Point(350, 50), cv::Point(350, 750), cv::Scalar::all(kObstacle), 3);
  cv::Mat canvs = map.clone();
  cv::cvtColor(canvs, canvs, cv::COLOR_GRAY2BGR);

  ScanData scan_data;
  Lidar lidar(0.0);
  lidar.SetLaserDetectionMax(300);
  double robot_x, robot_y, robot_angle;
  // get robot pose
  robot_x = 200;
  robot_y = 400;
  robot_angle = 0;

  cv::circle(canvs, cv::Point(robot_x, robot_y), 8, cv::Scalar(0,120,255), 3);
  for (int i=0; i < kScanSize; i++) {

    double cur_angle = robot_angle + (360.0/kScanSize)*i;

    double laser_dist, laser_angle;
    lidar.Ray(robot_x, robot_y, cur_angle, laser_dist, laser_angle, map);
    scan_data.laser_distance[i] =  laser_dist;
    scan_data.laser_angle[i] =  laser_angle;

    if (laser_dist < 1e-4 ) {
      scan_data.values[i] = kFreeSpace;
    } else {
      scan_data.values[i] = kObstacle;
    }
  }

  // 变换到当前坐标系
  std::vector<cv::Point2d> trans_in_map;
  trans_in_map.reserve(500);
  scan_data.TransRaw({robot_x, robot_y}, robot_angle, trans_in_map);

  // 绘制激光雷达效果
  for (int i = 0; i < trans_in_map.size(); i++) {
    canvs.at<cv::Vec3b>(trans_in_map[i]) = cv::Vec3b(0,0,255);
  }
  cv::circle(canvs, trans_in_map[0], 1, cv::Scalar(0,255,0), 3);
  cv::circle(canvs, trans_in_map.back(), 1, cv::Scalar(255,0,0), 1);
  return;
}

int test_CastMove() {
    cv::Mat map = cv::Mat::zeros(800, 450, CV_8UC1);
  map.setTo(kUnknown);
  cv::line(map, cv::Point(350, 50), cv::Point(350, 750), cv::Scalar::all(kObstacle), 3);
  cv::Mat canvs = map.clone();
  cv::cvtColor(canvs, canvs, cv::COLOR_GRAY2BGR);

  ScanData scan_data;
  Lidar lidar(0.2);
  lidar.SetLaserDetectionMax(300);
  double robot_x, robot_y, robot_angle;
  // get robot pose
  double start_robot_x = 300;  // move: 300->200
  double x_diff = 100;
  robot_x = 300;  // move: 300->100
  robot_y = 400;
  robot_angle = 0;

  cv::circle(canvs, cv::Point(robot_x, robot_y), 8, cv::Scalar(0,120,255), 1);
  for (int i=0; i < kScanSize; i++) {
    double cur_angle = robot_angle + (360.0/kScanSize)*i;
    robot_x = start_robot_x -  i*x_diff / kScanSize;
    double laser_dist, laser_angle;
    lidar.Ray(robot_x, robot_y, cur_angle, laser_dist, laser_angle, map);
    scan_data.laser_distance[i] =  laser_dist;
    scan_data.laser_angle[i] =  laser_angle;

    if (laser_dist < 1e-4 ) {
      scan_data.values[i] = kFreeSpace;
    } else {
      scan_data.values[i] = kObstacle;
      // 记录
      scan_data.robot_base_pts[i].x = robot_x;
      scan_data.robot_base_pts[i].y = robot_y;
      scan_data.robot_base_angle[i] = 0;
    }
  }
  // 运动完成后机器人位置
  cv::circle(canvs, cv::Point(200, robot_y), 8, cv::Scalar(0,120,255), 1);

  // 变换到当前坐标系
  std::vector<cv::Point2d> trans_in_map;
  trans_in_map.reserve(500);
  scan_data.TransRaw({start_robot_x, robot_y}, robot_angle, trans_in_map);

  // 绘制原始激光雷达效果
  for (int i = 0; i < trans_in_map.size(); i++) {
    canvs.at<cv::Vec3b>(trans_in_map[i]) = cv::Vec3b(0,0,255);
  }
  cv::circle(canvs, trans_in_map[0], 1, cv::Scalar(0,255,0), 3);
  cv::circle(canvs, trans_in_map.back(), 1, cv::Scalar(255,0,0), 1);


  // 进行畸变矫正
  scan_data.TransUndistort(trans_in_map);
  // 绘制去畸变激光雷达效果
  for (int i = 0; i < trans_in_map.size(); i++) {
    canvs.at<cv::Vec3b>(trans_in_map[i]) = cv::Vec3b(0,255,0);
  }
  cv::circle(canvs, trans_in_map[0], 1, cv::Scalar(0,255,0), 3);
  cv::circle(canvs, trans_in_map.back(), 1, cv::Scalar(255,0,0), 1);

  return 0;
}

int main() {
  // test_CastStatic();
  test_CastMove();
  return 0;
}