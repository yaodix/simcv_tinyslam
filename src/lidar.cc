// 单线激光雷达类

#include "lidar.h"
#include <eigen3/Eigen/Dense>

int Lidar::Scan(ScanData& scan_data, const Robot& robot, const cv::Mat& map) {
  double x_end, y_end;
  for (int i=0; i < scan_size_; i++) {
    // get robot pose
    double robot_x, robot_y, robot_angle;
    robot.GetPose(robot_angle, robot_y, robot_angle);
    double cur_angle = robot_angle + (360.0/scan_size_)*i;

    int x_end, y_end;
    ray(robot_x, robot_y, cur_angle, x_end, y_end, map);
    scan_data.x[i] = x_end;
    scan_data.y[i] = y_end;
    if (x_end < 1e-4 && y_end < 1e-4) {
      scan_data.values[i] = kFreeSpace;
    } else {
      scan_data.values[i] = kObstacle;
    }
  }
}

// angle - [180°, 180°]
int Lidar::ray(double x_start, double y_start, double angle, int& x_end, int& y_end, const cv::Mat& map) {
  x_end = 0.0;
  y_end = 0.0;
  double cast_distance = 0;
  Eigen::Vector2d direct(cos(angle * kDeg2rad), sin(angle * kDeg2rad));
  Eigen::Vector2d origin(x_start, y_start);
  Eigen::Vector2d cast_positon = origin;
  bool collision = false;

  while (!collision && cast_distance < detection_max_) {
    cast_positon += direct;
    cast_distance += reseluton_;

    if (map.at<uchar>(std::round(cast_positon(1)), std::round(cast_positon(0))) == kObstacle) {
      collision = true;
      break;
    }
  }
  if(collision && (cast_distance >= detection_min_) && (cast_distance <= detection_max_)) {
        // add noise based on standard deviation error
        double rx = ((double) rand() / (RAND_MAX));
        double ry = ((double) rand() / (RAND_MAX));
        x_end = cast_positon(0) + rx*std_err_;
        y_end = cast_positon(1) + ry*std_err_;
  }
  return 0;  
}