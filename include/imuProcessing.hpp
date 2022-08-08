#ifndef _IMU_PROCESSING_
#define _IMU_PROCESSING_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace ImuProcessing{
  class imuProcessing{
  private:
    int m_window_size;
    Eigen::Quaterniond m_prev_stats;   
  public:
    void init(const int &imu_window_size);
    void weightPrevOrientation(const Eigen::Quaterniond &pres_orientation);
    void prevStatsClear();
    Eigen::Quaterniond getWeightedOrientation() const;
    int getWindowSize() const;
    int mp_imu_count;
      imuProcessing(void);
  };
}


#endif _IMU_PROCESSING_
