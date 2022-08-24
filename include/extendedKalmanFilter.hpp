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
    Eigen::Vector4d m_sensor_diff;
    Eigen::Matrix4d m_last_pose;
    Eigen::Quaterniond m_last_orientation;
  public:
    void correctionInit(const int &window_size, const double &diff_x, const double &diff_y, const double &diff_z);
    void processKalmanFilter(const Eigen::Matrix4d &pres_pose, Eigen::Matrix4d &pose_out);
    void exponentialWeight(const Eigen::Matrix4d &pres_pose, Eigen::Matrix4d &pose_out);
    void sensorTFCorrection();
    bool calculateDifference(const Eigen::Quaterniond &pres_orientation);
      extendedKalmanFilter(void);
  };
}

#endif _EXTENDED_KALMAN_FILTER_
