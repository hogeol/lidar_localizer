#ifndef _NDT_MATCHING_
#define _NDT_MATCHING_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <pclomp/ndt_omp.h>

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <string>
#include <vector>
#include <ctime>
#include <cmath>

namespace NdtMatching{
  class ndtMatching{
  private:
    int m_local_count;
    double m_map_resolution;
    bool m_submap_select;
    double m_search_radius;
    int m_near_points;
    double m_theta;
    double m_transform_x;
    double m_transform_y;
    double m_transform_z;
    double m_diff_x;
    double m_diff_y;
    double m_diff_z;
    pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr m_kd_tree; //map
    Eigen::Matrix4f m_last_pose;
    pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr m_ndt;
  public:
    void setMapTransformInfo(const double &theta, const double &x, const double &y, const double &z);
    void setInitPosition(const double &x, const double &y, const double &z, const double &theta);
    void setGpsLidarTF(const double &diff_x, const double &diff_y, const double &diff_z);
    void init(const double &map_resolution, const std::string &map_path, const std::string &map_name, const bool &submap_select,const double &search_radius, const int &near_points, const int &max_iter, const int &ndt_threads);
    void processNdt(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out, const Eigen::Isometry3d &pose_in, Eigen::Isometry3d &pose_out);
    void sensorTFCorrection(Eigen::Isometry3d &pose_out);
    void processNdtWithColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_out, const Eigen::Matrix4f &pose_in, Eigen::Matrix4f &pose_out);
    void radiusSearch(const Eigen::Vector3d &based_point, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out);
    void kNearestSearch(const Eigen::Vector3d &based_point, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out);
    void pcdMapTransform(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in);
    void relocalize(const Eigen::Matrix4f &last_gps_odom);
    double calDistance(const Eigen::Vector3f &gps_xyz, const Eigen::Vector3f &ndt_xyz);
    pcl::PointCloud<pcl::PointXYZI>::Ptr mp_pcd_map;
    bool mp_pose_inited;
      ndtMatching(void);
  };
}


#endif _NDT_MATCHING_
