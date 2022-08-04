#ifndef _GPS_IMU_
#define _GPS_IMU_

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

namespace GpsImu{
  class gpsImu{
  private:
  
  public:
    void setYaw(const Eigen::Quaterniond &eigen_quat_in, Eigen::Quaterniond &eigen_quat_out, const double &change_yaw);
  
  };
}

#endif _GPS_IMU_
