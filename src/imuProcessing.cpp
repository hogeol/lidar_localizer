#include "imuProcessing.hpp"

namespace ImuProcessing{
  void imuProcessing::init(const int &imu_window_size)
  {
    m_window_size = imu_window_size;
  }

  void imuProcessing::weightPrevOrientation(const Eigen::Quaterniond &pres_orientation)
  {
    m_prev_stats.w() += pres_orientation.w()*(0.1 * (double)(mp_imu_count % m_window_size));
    m_prev_stats.x() += pres_orientation.x()*(0.1 * (double)(mp_imu_count % m_window_size));
    m_prev_stats.y() += pres_orientation.y()*(0.1 * (double)(mp_imu_count % m_window_size));
    m_prev_stats.z() += pres_orientation.z()*(0.1 * (double)(mp_imu_count % m_window_size));
    printf("\nimu_count: %d\n", mp_imu_count);
  }

  void imuProcessing::prevStatsClear()
  {
    m_prev_stats.w() = 0.0;
    m_prev_stats.x() = 0.0;
    m_prev_stats.y() = 0.0;
    m_prev_stats.z() = 0.0;
    printf("\nclear\n");
  }

  Eigen::Quaterniond imuProcessing::getWeightedOrientation() const
  {
    return m_prev_stats;
  }

  int imuProcessing::getWindowSize() const
  {
    return m_window_size;
  }

  imuProcessing::imuProcessing(void)
  {
  }
}