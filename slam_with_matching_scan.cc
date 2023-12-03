
/*
没有使用激光点匹配的直接建图
*/

#include <iostream>
#include <vector>
#include <thread>

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

int GetScanTransform(const std::vector<cv::Point2d>& pre_scan, const std::vector<cv::Point2d>& cur_scan,
    const cv::Point2d& robot_pt, const std::vector<double>& init_pose, std::vector<cv::Point2d>& trans_scan,
     cv::Point2d& trans_robot_pt) {
  ScanMatchingPLICP plicp;

  pcl::PointCloud<pcl::PointXYZI>::Ptr ref(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr per(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr transform_pc(new pcl::PointCloud<pcl::PointXYZI>);
  for (auto& pt : pre_scan) {
    pcl::PointXYZI pt_tmp;
    pt_tmp.x = pt.x;
    pt_tmp.y = pt.y;
    pt_tmp.z = 1.;
    pt_tmp.intensity = 255;
    ref->push_back(pt_tmp);
  }
  for (auto& pt : cur_scan) {
    pcl::PointXYZI pt_tmp;
    pt_tmp.x = pt.x;
    pt_tmp.y = pt.y;
    pt_tmp.z = 1.;
    pt_tmp.intensity = 100;
    per->push_back(pt_tmp);
  }

  plicp.ScanMatching(ref, per, init_pose);
  Eigen::Matrix4d transfrom_mat = plicp.ReturnPose();

  pcl::PointXYZI rob_pt;
  rob_pt.x = robot_pt.x;
  rob_pt.y = robot_pt.y;
  rob_pt.z = 1.;
  rob_pt.intensity = 1.;
  per->push_back(rob_pt);

  pcl::transformPointCloud (*per, *transform_pc, transfrom_mat);
  trans_scan.clear();
  trans_scan.reserve(pre_scan.size());
  for (int i = 0; i < transform_pc->points.size()-1; i++) {
    trans_scan.emplace_back(transform_pc->points[i].x, transform_pc->points[i].y);
  }
  trans_robot_pt.x = transform_pc->points.back().x;
  trans_robot_pt.y = transform_pc->points.back().y;
  return 0;
}


int main() {
  cv::Mat canvs = cv::Mat::zeros(300, 500, CV_8UC1);
  canvs.setTo(kUnknown);
  cv::rectangle(canvs, {50, 50}, {400, 280}, cv::Scalar::all(kObstacle),3);

  int start_robo_x = 80;
  int start_robo_y = 100;
  int start_robo_theta = 0;
  Robot robot(start_robo_x, start_robo_y, start_robo_theta);
  robot.SetPoseStdErr(0.2);
  Lidar lidar;
  lidar.SetLaserDetectionMax(220);
  CostMap cost_map(canvs);

  std::vector<cv::Point> path;
  path.emplace_back(robot.x_, robot.y_);
  path.emplace_back(130, 200);
  path.emplace_back(270, 200);
  path.emplace_back(300, 100);

  std::thread lidar_scan(&Lidar::LoopScan, &lidar, std::cref(robot), canvs);
  std::thread robot_move(&Robot::Move, &robot, path);

  lidar_scan.detach();
  robot_move.detach();
  
  ScanData cur_scan_data;
  std::vector<cv::Point> drift_path, trans_path;
  std::vector<cv::Point2d> pre_scan;
  std::vector<cv::Point2d> cur_scan;
  std::vector<cv::Point2d> trans_scan;
  bool init_scan = false;
  cv::Point2d trans_robo_pt;

  while(1) {
    double r_x, r_y, r_theta;
    robot.GetDriftPose(r_x, r_y, r_theta);
    drift_path.emplace_back(r_x, r_y);

    lidar.GetScanData(cur_scan_data);
    cur_scan_data.TransUndistort(cur_scan);
    if (!init_scan) {
      init_scan = true;
      trans_scan = cur_scan;
      trans_robo_pt = cur_scan_data.robot_base_pts.front();
    } else {
      if (cur_scan.empty())
        continue;

    // 更新scan数据和机器人位置 
      std::vector<double> init_pose{1, 1, 0};
      // init_pose[0] = r_x
      GetScanTransform(pre_scan, cur_scan, cur_scan_data.robot_base_pts.front(),init_pose, trans_scan, trans_robo_pt);

      cv::Mat map_show_icp = cost_map.GetGridMapCanvs().clone();
      for (auto& pt : pre_scan) {
        map_show_icp.at<cv::Vec3b>(pt) = cv::Vec3b(0,234,0);
      }
      for (int i =0; i < cur_scan.size(); i++) {
        map_show_icp.at<cv::Vec3b>(cur_scan[i]) = cv::Vec3b(234,0,0);
        map_show_icp.at<cv::Vec3b>(trans_scan[i]) = cv::Vec3b(0,0,234);
      }
      // robot.SetDriftPose()
      trans_path.emplace_back(trans_robo_pt);
    }

    cost_map.DrawScanData(trans_scan, trans_robo_pt);
    pre_scan = trans_scan;

    cv::Mat map_canvs = cost_map.GetGridMapCanvs().clone();
    cv::polylines(map_canvs, path, false, cv::Scalar(0, 123, 123), 2);
    cv::polylines(map_canvs, drift_path, false, cv::Scalar(0, 0, 255), 1);
    cv::polylines(map_canvs, trans_path, false, cv::Scalar(234, 0, 0), 1);

    robot.Draw(map_canvs);

    cv::imshow("robot_move", map_canvs);
    cv::waitKey(30);    
  }

  return 0;
}