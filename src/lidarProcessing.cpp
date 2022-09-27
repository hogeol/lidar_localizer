#define _USE_MATH_DEFINES_
#include "lidarProcessing.hpp"

namespace LidarProcessing{
  void lidarProcessingClass::setLidar(const int &scan_line, const double &max_distance, const double &min_distance,const double &vertical_angle, const double &downsampling_size)
  {
    m_lidar_info.setScanLine(scan_line);
    m_lidar_info.setMaxDis(max_distance);
    m_lidar_info.setMinDis(min_distance);
    m_lidar_info.setVerticalAngle(vertical_angle);
    m_leaf_size = downsampling_size;
  }

  void lidarProcessingClass::featureExtraction(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out)
  { 
    //remove NaN points
    std::vector<int> nan_idx;
    pcl::removeNaNFromPointCloud<pcl::PointXYZI>(*pc_in, *pc_in, nan_idx);

    //downsampling
    pcl::PointCloud<pcl::PointXYZI>::Ptr down_sampling_out(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::ApproximateVoxelGrid<pcl::PointXYZI>::Ptr avg_filter(new pcl::ApproximateVoxelGrid<pcl::PointXYZI>());
    avg_filter->setInputCloud(pc_in);
    avg_filter->setLeafSize(m_leaf_size, m_leaf_size, m_leaf_size);
    //avg_filter.filter(*down_sampling_out);
    avg_filter->filter(*down_sampling_out);
    
    //statistical outrier removal
    int num_neighbor_points = 20;
    double std_multiplier = 10.0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr sor_out(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI>::Ptr sor_removal(new pcl::StatisticalOutlierRemoval<pcl::PointXYZI>());
    sor_removal->setInputCloud(down_sampling_out);
    sor_removal->setMeanK(num_neighbor_points);
    sor_removal->setStddevMulThresh(std_multiplier);
    sor_removal->filter(*sor_out);
    //sor_limover.filter(*pc_out);

    //radius outlier removal
    pcl::PointCloud<pcl::PointXYZI>::Ptr rad_out(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::RadiusOutlierRemoval<pcl::PointXYZI>::Ptr rad_removal(new pcl::RadiusOutlierRemoval<pcl::PointXYZI>());
    rad_removal->setInputCloud(sor_out);
    rad_removal->setRadiusSearch(5.0);
    rad_removal->setMinNeighborsInRadius(20);
    rad_removal->setKeepOrganized(true);
    rad_removal->filter(*rad_out);
    
    //distance filter
    //pcl::PointCloud<pcl::PointXYZI>::Ptr dis_out(new pcl::PointCloud<pcl::PointXYZI>());
    distanceFilter(rad_out, pc_out);

    //colorize
    //colorize(dis_out, pc_out, {255, 0, 0});
    
    
    //colorize(down_pcd, pc_out, {255, 0, 0});
  }

  void lidarProcessingClass::distanceFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out)
  {
    pc_out->clear();
    pcl::PointXYZI pc_out_tmp;
    int input_point_size = pc_in->points.size();
    for(int point_iter=0; point_iter < input_point_size; point_iter++){
      const auto &pt_iter = pc_in->points[point_iter];
      if(sqrt(pow(pt_iter.x,2)+pow(pt_iter.y,2)+pow(pt_iter.z,2))>= m_lidar_info.m_min_dis && sqrt(pow(pt_iter.x,2)+pow(pt_iter.y,2)+pow(pt_iter.z,2))<=m_lidar_info.m_max_dis){
        pc_out_tmp.x = pt_iter.x;
        pc_out_tmp.y = pt_iter.y;
        pc_out_tmp.z = pt_iter.z;
        pc_out_tmp.intensity = pt_iter.intensity;
        pc_out->points.emplace_back(pc_out_tmp);
      }
    }
  }

  void lidarProcessingClass::colorize(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_out, const std::vector<int> &color)
  {
    pc_out->clear();
    int input_points_size = pc_in->points.size();
    pcl::PointXYZRGB pc_out_tmp;
    for(int point_iter=0; point_iter<input_points_size; point_iter++){
      const auto &pt_iter = pc_in->points[point_iter];
      pc_out_tmp.x = pt_iter.x;
      pc_out_tmp.y = pt_iter.y;
      pc_out_tmp.z = pt_iter.z;
      pc_out_tmp.r = color[0];
      pc_out_tmp.g = color[1];
      pc_out_tmp.b = color[2];
      pc_out->points.emplace_back(pc_out_tmp);
    }
  }
  

  lidarProcessingClass::lidarProcessingClass(void)
  {
  }
}