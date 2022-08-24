#ifndef _IMU_PROCESSING_
#define _IMU_PROCESSING_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace LocalPoseProcessing{
  class localPoseProcessing{
  private:
    int m_imu_window_size;
    double m_imu_weight;
    double m_gps_weight;
    Eigen::Vector3d m_prev_position;
    Eigen::Quaterniond m_prev_orientation;   
  public:
    void init(const int &imu_window_size, const int &gps_window_size);
    void weightPrevPosition(Eigen::Vector3d &pres_position);
    void weightPrevOrientation(Eigen::Quaterniond &pres_orientation);
    void positionClear();
    int getImuWindowSize() const;
      localPoseProcessing(void);
  };
}


#endif _IMU_PROCESSING_
