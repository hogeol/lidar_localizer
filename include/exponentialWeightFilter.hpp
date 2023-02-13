#ifndef _EXPONENTIAL_WEIGHT_FILTER_
#define _EXPONENTIAL_WEIGHT_FILTER_

#include <iostream>
#include <cmath>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace ExponentialWeightFilter{
  class exponentialWeightFilter{
  private:
    int m_orientation_window_size;
    int m_position_window_size;
    double m_orientation_exponential_weight;
    double m_position_exponential_weight;
    Eigen::Isometry3d m_last_pose;
  public:
    void correctionInit(const int &orientation_window_size, const int &position_window_size);
    void setInitPosition(const Eigen::Isometry3d &pres_pose);
    void processExpFilter(const Eigen::Isometry3d &pres_pose, Eigen::Isometry3d &pose_out);
    void exponentialWeight(const Eigen::Isometry3d &pres_pose);
      exponentialWeightFilter(void);
  };
}

#endif _EXPONENTIAL_WEIGHT_FILTER_
