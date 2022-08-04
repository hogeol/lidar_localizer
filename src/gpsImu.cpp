#include "gpsImu.hpp"

namespace GpsImu{
  void gpsImu::setYaw(const Eigen::Quaterniond &eigen_quat_in, Eigen::Quaterniond &eigen_quat_out, const double &change_yaw)
  {
    Eigen::Matrix3d eigen_rot_matrix = eigen_quat_in.toRotationMatrix();
    tf::Matrix3x3 tf_rot_matrix;
    tf::matrixEigenToTF(eigen_rot_matrix, tf_rot_matrix);
    double roll, pitch, yaw;
    tf_rot_matrix.getRPY(roll, pitch, yaw);
    yaw -= change_yaw;
    tf_rot_matrix.setEulerZYX(yaw, pitch, roll);
    tf::matrixTFToEigen(tf_rot_matrix, eigen_rot_matrix);
    eigen_quat_out = eigen_rot_matrix;
  }
}