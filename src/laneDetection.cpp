#define _USE_MATH_DEFINES_
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
    pass_through_filter.setInputCloud(after_ptf);
    pass_through_filter.setFilterFieldName("x");
    pass_through_filter.setFilterLimits(m_range_info.mp_range_front, m_range_info.mp_range_back);
    pass_through_filter.setFilterLimitsNegative(false);
    pass_through_filter.filter(*after_ptf);
    filteringIntensity(after_ptf, pc_out);
  }

  void laneDetection::filteringIntensity(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out)
  {
    pc_out->clear();
    int point_size = pc_in->points.size();
    double min_z = 0.0;
    double min_intensity = 255.0;
    pcl::PointXYZI pt_xyzi;
    for(int point_iter = 0; point_iter < point_size; point_iter++){
      const auto &pc_in_tmp = pc_in->points[point_iter];
      pt_xyzi.x = pc_in_tmp.x;
      pt_xyzi.y = pc_in_tmp.y;
      pt_xyzi.z = pc_in_tmp.z;
      pt_xyzi.intensity = pc_in_tmp.intensity;
      if(min_z > pt_xyzi.z){
        min_z = pt_xyzi.z;
      }
      if(min_intensity > pt_xyzi.intensity){
        min_intensity = pt_xyzi.intensity;
      }
    }
    for(int pt_iter; pt_iter < point_size; pt_iter++){
      const auto &pc_in_pt = pc_in->points[pt_iter];
      pt_xyzi.x = pc_in_pt.x;
      pt_xyzi.y = pc_in_pt.y;
      pt_xyzi.z = pc_in_pt.z;
      pt_xyzi.intensity = pc_in_pt.intensity;
      if(std::abs(pt_xyzi.z - min_z) < 1.0){
        pc_out->emplace_back(pt_xyzi);
      }
    }
  }

  laneDetection::laneDetection(void)
  {
  }
}