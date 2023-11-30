// 单线激光雷达类
#include <thread>

#include "lidar.h"
#include <eigen3/Eigen/Dense>

int Lidar::LoopScan(const Robot& robot, const cv::Mat& map) {
  std::chrono::milliseconds t(10);
  while (true) {
    auto preprocess_start = std::chrono::high_resolution_clock::now();
    Scan(robot, map);
    auto preprocess_end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> preprocess_cost = preprocess_end - preprocess_start;
    std::cout << "lidar fps " << 1.0 / (preprocess_cost.count()*1) << "hz\n";

    std::this_thread::sleep_for(t);
  }  
}

int Lidar::Scan(const Robot& robot, const cv::Mat& map) {
  double x_end, y_end;
  std::chrono::nanoseconds t(30);

  sd_mtx.lock();
  for (int i=0; i < scan_size_; i++) {
    // get robot pose
    std::this_thread::sleep_for(t);
    double robot_x, robot_y, robot_angle;
    // robot.GetPose(robot_x, robot_y, robot_angle);
    robot.GetDriftPose(robot_x, robot_y, robot_angle);
    double cur_angle = robot_angle + (360.0/scan_size_)*i;

    double laser_dist, laser_angle;
    Ray(robot_x, robot_y, cur_angle, laser_dist, laser_angle, map);
    
    scan_data_.laser_distance[i] =  laser_dist;
    scan_data_.laser_angle[i] =  laser_angle - robot_angle;

    if (laser_dist < 1e-4 ) {
      scan_data_.values[i] = kFreeSpace;
    } else {
      scan_data_.values[i] = kObstacle;
    }
    // 记录
    scan_data_.robot_base_pts[i].x = robot_x;
    scan_data_.robot_base_pts[i].y = robot_y;
    scan_data_.robot_base_angle[i] = robot_angle;
  }
  sd_mtx.unlock();

}

// angle - [0°, 360°]
int Lidar::Ray(double x_start, double y_start, double angle, double& laser_distance, double& laser_angle, const cv::Mat& map) {
  laser_distance = 0.0;
  laser_angle = 0.0;
  double cast_distance = 0;
  Eigen::Vector2d direct(cos(angle * kDeg2rad), sin(angle * kDeg2rad));
  Eigen::Vector2d origin(x_start, y_start);
  Eigen::Vector2d cast_positon = origin;
  bool collision = false;

  while (!collision && cast_distance < range_max_) {
    cast_positon += direct*reselution_;
    cast_distance += reselution_;

    if (map.at<uchar>(std::round(cast_positon(1)), std::round(cast_positon(0))) == kObstacle) {
      collision = true;
      break;
    }
  }
  if(collision && (cast_distance >= range_min_) && (cast_distance <= range_max_)) {
        // add noise based on standard deviation error
        double rx = ((double) rand() / (RAND_MAX));
        double ry = ((double) rand() / (RAND_MAX));
        laser_distance = cast_distance + rx * ray_std_err_;
        laser_angle = angle + ry * ray_std_err_;
  }
  return 0;  
}