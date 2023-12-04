
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
#include <pcl/registration/icp.h>

#include "src/common_config.h"
#include "src/robot.h"
#include "src/lidar.h"
#include "src/cost_map.h"
#include "src/scan_matching_plicp.h"

int PclIcp(const std::vector<cv::Point2d>& pre_scan, const std::vector<cv::Point2d>& cur_scan,
    const cv::Point2d& robot_pt, std::vector<cv::Point2d>& trans_scan,
     cv::Point2d& trans_robot_pt) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr ref(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr per(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr transform_pc(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto& pt : pre_scan) {
    pcl::PointXYZ pt_tmp;
    pt_tmp.x = pt.x;
    pt_tmp.y = pt.y;
    pt_tmp.z = 1.;
    ref->push_back(pt_tmp);
  }
  for (auto& pt : cur_scan) {
    pcl::PointXYZ pt_tmp;
    pt_tmp.x = pt.x;
    pt_tmp.y = pt.y;
    pt_tmp.z = 1.;
    per->push_back(pt_tmp);
  }
 //creates an instance of an IterativeClosestPoint and gives it some useful information
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setTransformationEpsilon(1e-5);
  icp.setMaximumIterations(1e3);
  icp.setMaxCorrespondenceDistance(20);  // 剔除离群点

  icp.setInputTarget(ref);
  icp.setInputCloud(per);

  //Creates a pcl::PointCloud<pcl::PointXYZ> to which the IterativeClosestPoint can save the resultant cloud after applying the algorithm
  pcl::PointCloud<pcl::PointXYZ> Final;

  //Call the registration algorithm which estimates the transformation and returns the transformed source (input) as output.
  icp.align(Final);
  trans_scan.clear();
  trans_scan.reserve(pre_scan.size());
  for (int i = 0; i < Final.points.size(); i++) {
    trans_scan.emplace_back(Final.points[i].x, Final.points[i].y);
  }

  //Return the state of convergence after the last align run. 
  //If the two PointClouds align correctly then icp.hasConverged() = 1 (true). 
  std::cout << "has converged: " << icp.hasConverged() <<std::endl;

  //Obtain the Euclidean fitness score (e.g., sum of squared distances from the source to the target) 
  std::cout << "score: " <<icp.getFitnessScore() << std::endl; 
  std::cout << "----------------------------------------------------------"<< std::endl;

  //Get the final transformation matrix estimated by the registration method. 
  std::cout << icp.getFinalTransformation() << std::endl;

  pcl::PointCloud<pcl::PointXYZ>::Ptr robot_pc(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointXYZ rob_pt;
  rob_pt.x = robot_pt.x;
  rob_pt.y = robot_pt.y;
  rob_pt.z = 1.;
  robot_pc->push_back(rob_pt);

  pcl::transformPointCloud (*robot_pc, *transform_pc, icp.getFinalTransformation());
  trans_robot_pt.x = transform_pc->points.front().x;
  trans_robot_pt.y = transform_pc->points.front().y;

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
      PclIcp(pre_scan, cur_scan, cur_scan_data.robot_base_pts.front(), trans_scan, trans_robo_pt);

      cv::Mat map_show_icp = cost_map.GetGridMapCanvs().clone();
      for (auto& pt : pre_scan) {
        map_show_icp.at<cv::Vec3b>(pt) = cv::Vec3b(0,234,0);
      }
      for (int i =0; i < cur_scan.size(); i++) {
        map_show_icp.at<cv::Vec3b>(cur_scan[i]) = cv::Vec3b(234,0,0);
        map_show_icp.at<cv::Vec3b>(trans_scan[i]) = cv::Vec3b(0,0,234);
      }
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