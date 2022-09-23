#ifndef _LIDAR_PROCESSING_
#define _LIDAR_PROCESSING_

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>

#include <vector>
#include <cmath>

#include "lidar.hpp"

namespace LidarProcessing{  
  class lidarProcessingClass{
  private:
    double m_leaf_size;
    Lidar::lidar m_lidar_info;
  public:
    void setLidar(const int &scan_line, const double &max_distance, const double &min_distance, const double &vertical_angle, const double &downsampling_size);
    void featureExtraction(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out);
    void distanceFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out);
    void colorize(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_out, const std::vector<int> &color);
      lidarProcessingClass(void);
  };
}


#endif _LIDAR_PROCESSING_

