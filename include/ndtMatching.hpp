#ifndef _NDT_MATCHING_
#define _NDT_MATCHING_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/ndt.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/io/pcd_io.h>

#include <opencv2/core/types.hpp>

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <string>
#include <vector>
#include <ctime>
#include <cmath>

namespace NdtMatching{
  class ndtMatching{
  private:
  pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr m_kd_tree;
  double m_leaf_size;
  bool m_submap_select;
  double m_search_radius;
  int m_near_points;
  int m_max_iter;
  double m_theta;
  double m_x;
  double m_y;
  double m_z;
  Eigen::Matrix4f m_transform_matrix;
  pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr m_ndt;
  public:
  void init(const double &map_resolution, const std::string &map_path, const std::string &map_name, const double &leaf_size, const bool &submap_select,const double &search_radius, const int & near_points, const int &max_iter);
  void setTransformInfo(const double &theta, const double &x, const double &y, const double &z);
  void processNdt(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out,const cv::Point3d &odom_xyz);
  void radiusSearch(const cv::Point3d &based_point, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out);
  void kNearestSearch(const cv::Point3d &based_point, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out);
  void pcdMapTransform(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in);
  pcl::PointCloud<pcl::PointXYZI>::Ptr mp_pcd_map;
    ndtMatching(void);
  };
}


#endif _NDT_MATCHING_
