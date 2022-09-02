#ifndef _EXTENDED_KALMAN_FILTER_
#define _EXTENDED_KALMAN_FILTER_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace ExtendedKalmanFilter{
  class extendedKalmanFilter{
  private:
    int m_orientation_window_size;
    int m_position_window_size;
    double m_orientation_exponential_weight;
    double m_position_exponential_weight;
    Eigen::Isometry3d m_last_pose;
  public:
    void correctionInit(const int &orientation_window_size, const int &position_window_size);
    void setInitPosition(const Eigen::Isometry3d &pres_pose);
    void processKalmanFilter(const Eigen::Isometry3d &pres_pose, Eigen::Isometry3d &pose_out);
    void exponentialWeight(const Eigen::Isometry3d &pres_pose);
      extendedKalmanFilter(void);
  };
}

#endif _EXTENDED_KALMAN_FILTER_
