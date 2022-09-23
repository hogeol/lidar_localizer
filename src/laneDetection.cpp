#include "laneDetection.hpp"

namespace LaneDetection{
  void laneDetection::init(const double &x_min, const double &x_max, const double &y_min, const double &y_max, const double &z_min, const double &z_max)
  {
    m_range_info.setRangeFront(x_min);
    m_range_info.setRangeBack(x_max);
    m_range_info.setRangeLeft(y_min);
    m_range_info.setRangeRight(y_max);
    m_range_info.setRangeBottom(z_min);
    m_range_info.setRangeTop(z_max);
  }

  void laneDetection::extractDesiredDimension(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out)
  {
    pc_out->clear();
    pcl::PointCloud<pcl::PointXYZI>::Ptr after_ptf(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PassThrough<pcl::PointXYZI> pass_through_filter;
    pass_through_filter.setInputCloud(pc_in);
    pass_through_filter.setFilterFieldName("y");
    pass_through_filter.setFilterLimits(m_range_info.mp_range_left, m_range_info.mp_range_right);
    //printf("\nfiltered range: %.4f, %.4f\n", m_range_info.mp_range_left, m_range_info.mp_range_right);
    pass_through_filter.setFilterLimitsNegative(false);
    pass_through_filter.filter(*after_ptf);
    filteringIntensity(after_ptf, pc_out);
  }

  void laneDetection::filteringIntensity(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out)
  {
    pc_out->clear();
    int point_size = pc_in->points.size();
    pcl::PointXYZI pc_in_points_tmp;
    for(int point_iter = 0; point_iter < point_size; point_iter++){
      const auto &pc_in_tmp = pc_in->points[point_iter];
      pc_in_points_tmp.x = pc_in_tmp.x;
      pc_in_points_tmp.y = pc_in_tmp.y;
      pc_in_points_tmp.z = pc_in_tmp.z;
      pc_in_points_tmp.intensity = pc_in_tmp.intensity;
      if(pc_in_points_tmp.intensity < 50){
        pc_out->points.emplace_back(pc_in_points_tmp);
      }
    }
  }

  laneDetection::laneDetection(void)
  {
  }
}