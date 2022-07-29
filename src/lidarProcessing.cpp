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

  void lidarProcessingClass::setRobot(const double &x_min, const double &x_max, const double &y_min, const double &y_max, const double &z_min , const double &z_max)
  {
    m_robot_info.setRobotX(x_min, x_max);
    m_robot_info.setRobotY(y_min, y_max);
    m_robot_info.setRobotZ(z_min, z_max);
  }

  void lidarProcessingClass::featureExtraction(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out)
  { 
    //remove NaN points
    std::vector<int> nan_idx;
    pcl::removeNaNFromPointCloud<pcl::PointXYZI>(*pc_in, *pc_in, nan_idx);
    //downsampling
    pcl::PointCloud<pcl::PointXYZI>::Ptr down_sampling_out(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::ApproximateVoxelGrid<pcl::PointXYZI> avg_filter;
    avg_filter.setInputCloud(pc_in);
    avg_filter.setLeafSize(m_leaf_size, m_leaf_size, m_leaf_size);
    //avg_filter.filter(*down_sampling_out);
    avg_filter.filter(*down_sampling_out);
   
    //indices filter remove noise
    // pcl::PointCloud<pcl::PointXYZI>::Ptr rm_noise_out(new pcl::PointCloud<pcl::PointXYZI>());
    // pcl::PointCloud<pcl::PointXYZI>::iterator pc_in_iter = pc_in->begin();
    // pcl::PointIndices::Ptr noise(new pcl::PointIndices());
    // pcl::ExtractIndices<pcl::PointXYZI> ext;
    // ext.setInputCloud(down_sampling_out);
    // for(int index_iter=0; pc_in_iter!=pc_in->end(); index_iter++, pc_in_iter++){
    //   if(pc_in_iter->intensity == 255.0){
    //     noise->indices.emplace_back(index_iter);
    //   }
    // }
    // ext.setIndices(noise);
    // ext.setNegative(true);
    // //ext.filter(*rm_noise_out);
    // ext.filter(*pc_out);

    // //passthrough filter
    pcl::PointCloud<pcl::PointXYZI>::Ptr ptf_out(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PassThrough<pcl::PointXYZI> pass_through;
    pass_through.setInputCloud(down_sampling_out);
    //pass_through.setInputCloud(rm_noise_out);
    pass_through.setFilterFieldName("y");
    pass_through.setFilterLimits(m_robot_info.m_robot_y_min, m_robot_info.m_robot_y_max);
    pass_through.setFilterLimitsNegative(true);
    pass_through.filter(*ptf_out);
    //pass_through.filter(*pc_out);
    
    //statistical outrier removal
    int num_neighbor_points = 20;
    double std_multiplier = 10.0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr sor_out(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor_limover;
    sor_limover.setInputCloud(ptf_out);
    sor_limover.setMeanK(num_neighbor_points);
    sor_limover.setStddevMulThresh(std_multiplier);
    sor_limover.filter(*sor_out);
    //sor_limover.filter(*pc_out);

    //distance filter
    distanceFilter(sor_out, pc_out);
    
    //colorize(down_pcd, pc_out, {255, 0, 0});
  }

  void lidarProcessingClass::distanceFilter(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out)
  {
    pc_out->clear();
    pcl::PointCloud<pcl::PointXYZI>::Ptr after_distance(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointXYZI pc_out_tmp;
    int input_point_size = pc_in->points.size();
    for(int point_iter=0; point_iter<input_point_size; point_iter++){
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
  
  void robotDimension::setRobotX(const double &robot_x_min, const double &robot_x_max)
  {
    m_robot_x_min = robot_x_min;
    m_robot_x_max = robot_x_max;
  }
  
  void robotDimension::setRobotY(const double &robot_y_min, const double &robot_y_max)
  {
    m_robot_y_min = robot_y_min;
    m_robot_y_max = robot_y_max;
  }

  void robotDimension::setRobotZ(const double &robot_z_min, const double &robot_z_max)
  {
    m_robot_z_min = robot_z_min;
    m_robot_z_max = robot_z_max;
  }

  lidarProcessingClass::lidarProcessingClass(void)
  {
  }

  robotDimension::robotDimension(void)
  {
  }
}