#include "localPoseProcessing.hpp"

namespace LocalPoseProcessing{
  void localPoseProcessing::init(const int &imu_window_size, const int &gps_window_size)
  {
    positionClear();
    m_imu_window_size = imu_window_size;
    m_imu_weight = 1.0 - (1.0 / (double)m_imu_window_size);
    m_gps_weight = 1.0 - (1.0 / (double)gps_window_size);
  }

  void localPoseProcessing::weightPrevPosition(Eigen::Vector3d &pres_position)
  {
    m_prev_position(0) = m_gps_weight * m_prev_position(0) + (1 - m_gps_weight) * pres_position(0);
    m_prev_position(1) = m_gps_weight * m_prev_position(1) + (1 - m_gps_weight) * pres_position(1);
    m_prev_position(2) = m_gps_weight * m_prev_position(2) + (1 - m_gps_weight) * pres_position(2);
    pres_position = m_prev_position;
  }

  void localPoseProcessing::weightPrevOrientation(Eigen::Quaterniond &pres_orientation)
  {
    m_prev_orientation.w() = m_imu_weight * m_prev_orientation.w() + (1 - m_imu_weight) * pres_orientation.w();
    m_prev_orientation.x() = m_imu_weight * m_prev_orientation.x() + (1 - m_imu_weight) * pres_orientation.x();
    m_prev_orientation.y() = m_imu_weight * m_prev_orientation.y() + (1 - m_imu_weight) * pres_orientation.y();
    m_prev_orientation.z() = m_imu_weight * m_prev_orientation.z() + (1 - m_imu_weight) * pres_orientation.z();
    pres_orientation = m_prev_orientation;
  }

  void localPoseProcessing::positionClear()
  {
    m_prev_position(0) = 0.0;
    m_prev_position(1) = 0.0;
    m_prev_position(2) = 0.0;
    m_prev_orientation.w() = 0.0;
    m_prev_orientation.x() = 0.0;
    m_prev_orientation.y() = 0.0;
    m_prev_orientation.z() = 0.0;
  }
  
  int localPoseProcessing::getImuWindowSize() const
  {
    return m_imu_window_size;
  }
  localPoseProcessing::localPoseProcessing(void)
  {
  }
}