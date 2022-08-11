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
    Eigen::Quaternionf m_prev_orientation;
    Eigen::Vector3f m_prev_position;
  public:
    void init(const int &window_size);
    void exponentialWeight(const Eigen::Matrix4f &pres_pose, Eigen::Matrix4f &pose_out);
    bool calculateDifference(const Eigen::Quaternionf &pres_orientation);
      extendedKalmanFilter(void);
  };
}

#endif _EXTENDED_KALMAN_FILTER_
