
/*
没有使用激光点匹配的直接建图
*/

#include <iostream>
#include <vector>
#include <thread>
#include <fstream>

#include "opencv2/opencv.hpp"
#include "eigen3/Eigen/Dense"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>    

#include "src/common_config.h"
#include "src/robot.h"
#include "src/lidar.h"
#include "src/cost_map.h"
#include "src/scan_matching_plicp.h"

int main() {
  cv::Mat canvs = cv::Mat::zeros(300, 500, CV_8UC1);
  canvs.setTo(kUnknown);
  cv::rectangle(canvs, {50, 50}, {400, 280}, cv::Scalar::all(kObstacle),3);

  Robot robot(80, 100, 0);
  robot.SetPoseStdErr(1);
  Lidar lidar;
  lidar.SetLaserDetectionMax(150);
  CostMap cost_map(canvs);

  ScanData cur_scan_data_1, cur_scan_data_2;

  double r_x, r_y, r_theta;
  robot.GetDriftPose(r_x, r_y, r_theta);
  lidar.Scan(robot, canvs);
  lidar.GetScanData(cur_scan_data_1);
  std::vector<cv::Point2d> trans_data_1;
  cur_scan_data_1.TransRaw({r_x, r_y}, r_theta, trans_data_1);


  robot.SetPose(85, 104, 5);
  lidar.Scan(robot, canvs);
  lidar.GetScanData(cur_scan_data_2);
  robot.GetDriftPose(r_x, r_y, r_theta);
  std::vector<cv::Point2d> trans_data_2;
  cur_scan_data_2.TransRaw({r_x+7, r_y+9}, r_theta+5, trans_data_2);

  cv::Mat map_canvs = cost_map.GetGridMapCanvs().clone();
  for (auto& pt : trans_data_1) {
    map_canvs.at<cv::Vec3b>(pt) = cv::Vec3b(0,234,0);
  }

  for (auto& pt : trans_data_2) {
    map_canvs.at<cv::Vec3b>(pt) = cv::Vec3b(0,0,234);
  }

  // 开始匹配
  ScanMatchingPLICP plicp;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ref(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr per(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr transform_pc(new pcl::PointCloud<pcl::PointXYZI>);
  for (auto& pt : trans_data_1) {
    pcl::PointXYZI pt_tmp;
    pt_tmp.x = pt.x;
    pt_tmp.y = pt.y;
    pt_tmp.z = 1.;
    pt_tmp.intensity = 255;
    ref->push_back(pt_tmp);
  }
  for (auto& pt : trans_data_2) {
    pcl::PointXYZI pt_tmp;
    pt_tmp.x = pt.x;
    pt_tmp.y = pt.y;
    pt_tmp.z = 1.;
    pt_tmp.intensity = 100;
    per->push_back(pt_tmp);
  }

  std::cout << ref->points.front() << std::endl;
  std::cout << per->points.front() << std::endl;
  plicp.ScanMatching(ref, per);
  Eigen::Matrix4d transfrom_mat = plicp.ReturnPose();

  pcl::transformPointCloud (*per, *transform_pc, transfrom_mat);
  for (int i = 0; i < transform_pc->points.size(); i++) {
    map_canvs.at<cv::Vec3b>(transform_pc->points[i].y, transform_pc->points[i].x) = cv::Vec3b(234,0,0);

  }

  std::cout << transfrom_mat << std::endl;
  return 0;
}