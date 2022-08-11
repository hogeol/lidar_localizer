#ifndef _IMU_PROCESSING_
#define _IMU_PROCESSING_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace ImuProcessing{
  class imuProcessing{
  private:
    int m_imu_window_size;
    double m_imu_weight;
    double m_gps_weight;
    Eigen::Vector3d m_prev_position;
    Eigen::Quaterniond m_prev_orientation;   
  public:
    void init(const int &imu_window_size, const int &gps_window_size);
    void weightPrevPosition(const Eigen::Vector3d &pres_position);
    void weightPrevOrientation(const Eigen::Quaterniond &pres_orientation);
    void positionClear();
    int getImuWindowSize() const;
      imuProcessing(void);
  };
}


#endif _IMU_PROCESSING_
