#ifndef _EXTENDED_KALMAN_FILTER_
#define _EXTENDED_KALMAN_FILTER_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace ExtendedKalmanFilter{
  class extendedKalmanFilter{
  private:
    int m_window_size;
    double m_exponential_weight;
    Eigen::Matrix4d m_last_pose;
    Eigen::Quaterniond m_last_orientation;
  public:
    void setInitPosition(const Eigen::Matrix4d &pres_pose);
    void correctionInit(const int &window_size);
    void processKalmanFilter(const Eigen::Matrix4d &pres_pose, Eigen::Matrix4d &pose_out);
    void exponentialWeight(const Eigen::Matrix4d &pres_pose);
    bool angleThreshold(const Eigen::Quaterniond &last_quaternion);
      extendedKalmanFilter(void);
  };
}

#endif _EXTENDED_KALMAN_FILTER_
