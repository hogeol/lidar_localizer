#ifndef _LIDAR_PROCESSING_
#define _LIDAR_PROCESSING_

#include "lidar.hpp"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <vector>
#include <cmath>


namespace LidarProcessing{

  class robotDimension{
    public:   
      double m_robot_x_min;
      double m_robot_x_max;
      double m_robot_y_min;
      double m_robot_y_max;
      double m_robot_z_min;
      double m_robot_z_max;
      void setRobotX(const double &robot_x_min, const double &robot_x_max);
      void setRobotY(const double &robot_y_min, const double &robot_y_max);
      void setRobotZ(const double &robot_z_min, const double &robot_z_max);
        robotDimension(void);
  };
  
  class lidarProcessingClass{
  private:
    double m_leaf_size;
    Lidar::lidarClass m_lidar_info;
    robotDimension m_robot_info;
    pcl::PointCloud<pcl::PointXYZI>::Ptr m_point_xyzi;
  public:
    void setLidar(const int &scan_line, const double &max_distance, const double &min_distance, const double &vertical_angle, const double &downsampling_size);
    void setRobot (const double &x_min, const double &x_max, const double &y_min, const double &y_max, const double &z_min , const double &z_max);
    void featureExtraction(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out);
    void colorize(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_out, const std::vector<int> &color);
    void distanceFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out);
      lidarProcessingClass(void);
  };
}


#endif _LIDAR_PROCESSING_

