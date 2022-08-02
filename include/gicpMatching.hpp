#ifndef _GICP_MATCHING_
#define _GICP_MATCHING_

#include <string>
#include <vector>
#include <ctime>
#include <cmath>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <pclomp/gicp_omp.h>

#include <opencv2/core/types.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>

namespace GicpMatching{
  class gicpMatching{
  private:
    double m_theta;
    double m_transform_x;
    double m_transform_y;
    double m_transform_z;
    Eigen::Matrix4f m_prev_pose;
    double m_map_resolution;
    bool m_submap_select;
    double m_search_radius;
    double m_near_points;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr m_kd_tree;
    pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr m_gicp;

  public:
    void setMapTransformInfo(const double &theta, const double &x, const double &y, const double &z);
    void setInitPosition(const double &x, const double &y, const double &z, const double &theta);
    void init(const double &map_resolution, const std::string &map_path, const std::string &map_name, const bool &submap_select, const double &search_radius, const double &near_points, const int &max_iter);
    void processGicp(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out, Eigen::Matrix4f &out_pose);
    void radiusSearch(const cv::Point3d &based_point, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out);
    void kNearestSearch(const cv::Point3d &based_point, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out);
    void pcdMapTransform(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in);
    pcl::PointCloud<pcl::PointXYZI>::Ptr mp_pcd_map;  
      gicpMatching(void);
  };
}

#endif _GICP_MATCHING_
