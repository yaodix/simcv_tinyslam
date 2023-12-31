
#ifndef SCAN_MATCHING_PLICP_H
#define SCAN_MATCHING_PLICP_H
#include <iostream>
#include <fstream>
#include <istream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/impl/point_cloud_geometry_handlers.hpp>

#include "pl_icp_cost_function.h"

using namespace std;

class ScanMatchingPLICP
{
  public:
    ScanMatchingPLICP();
    ~ScanMatchingPLICP();
    void ScanMatching(pcl::PointCloud<pcl::PointXYZI>::Ptr ref,pcl::PointCloud<pcl::PointXYZI>::Ptr per, const std::vector<double>& init_pose);

    Eigen::Matrix4d ReturnPose() {
      return pose_plicp_;
    }

    Eigen::Vector2d PCL2Eigen(pcl::PointXYZI& p);
    void ComputeCovariance(vector<Eigen::Vector2d>& data_pi, vector<Eigen::Vector2d>& model_qi, 
                          Eigen::Vector3d& transform, Eigen::MatrixXd& ICP_COV);
    void ComputeCovarianceWithPP(vector<Eigen::Vector2d>& data_pi, vector<Eigen::Vector2d>& model_qi, 
                                Eigen::Vector3d& transform, Eigen::MatrixXd& ICP_COV); 
    Eigen::Matrix3d Euler2Rotation(double yaw);
    
  public:
    vector<Eigen::Vector2d> points_l_;
    pcl::PointCloud<pcl::PointXYZI> lane_l_;
    vector<Eigen::Vector2d> points_map_;

    Eigen::Matrix3d rot_plicp_;
    Eigen::Vector3d trans_plicp_;
    Eigen::Matrix4d pose_plicp_;

    vector<Eigen::Vector2d> nearest_points_;
    vector<Eigen::Vector2d> nearest_points_2_;
    vector<Eigen::Vector2d> matched_map_points_;
    vector<Eigen::Vector2d> points_transformed_;
    vector<Eigen::Vector2d> normal_vector_;
};
#endif