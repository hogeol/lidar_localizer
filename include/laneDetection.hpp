#ifndef _LANE_DETECTION_
#define _LANE_DETECTION_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

#include "range.hpp"

namespace LaneDetection{
  class laneDetection{
  private:
    Range::range m_range_info;
  public:
    void init(const double &x_min, const double &x_max, const double &y_min, const double &y_max, const double &z_min, const double &z_max);
    void extractDesiredDimension(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out);
    void filteringIntensity(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out);
      laneDetection(void);
  };
}

#endif _LANE_DETECTION_
