#include "extendedKalmanFilter.hpp"

namespace ExtendedKalmanFilter{
  void extendedKalmanFilter::correctionInit(const int &window_size, const double &diff_x, const double &diff_y, const double &diff_z)
  {
    m_window_size = window_size;
    m_exponential_weight = 1.0 - (1.0 / (double)m_window_size);
    m_sensor_diff(0) = diff_x;
    m_sensor_diff(1) = diff_y;
    m_sensor_diff(2) = diff_z;
    m_sensor_diff(3) = 1.0;
    m_last_pose = Eigen::Matrix4d::Identity();
    m_last_orientation = Eigen::Quaterniond::Identity();
  }

  void extendedKalmanFilter::processKalmanFilter(const Eigen::Matrix4d &pres_pose, Eigen::Matrix4d &pose_out)
  {
    printf("\n---\npose:\nx: %.4f\ny: %.4f\nz: %.4f\n---\n", pres_pose(0,3), pres_pose(1,3), pres_pose(2,3));
    //sensorTFCorrection();
    exponentialWeight(pres_pose, pose_out);
    m_last_pose.block<3,3>(0,0) = m_last_orientation.toRotationMatrix();
    pose_out = m_last_pose;
    //printf("\n---\npose:\nx: %.4f\ny: %.4f\nz: %.4f\n---\n", m_last_pose(0,3), m_last_pose(1,3), m_last_pose(2,3));
  }

  void extendedKalmanFilter::exponentialWeight(const Eigen::Matrix4d &pres_pose, Eigen::Matrix4d &pose_out)
  {
    Eigen::Quaterniond pres_orientation(pres_pose.block<3,3>(0,0));
    if(calculateDifference(pres_orientation)){
      m_last_orientation.w() = m_exponential_weight * m_last_orientation.w() + (1 - m_exponential_weight) * pres_orientation.w();
      m_last_orientation.x() = m_exponential_weight * m_last_orientation.x() + (1 - m_exponential_weight) * pres_orientation.x();
      m_last_orientation.y() = m_exponential_weight * m_last_orientation.y() + (1 - m_exponential_weight) * pres_orientation.y();
      m_last_orientation.z() = m_exponential_weight * m_last_orientation.z() + (1 - m_exponential_weight) * pres_orientation.z();
      m_last_pose(0,3) = m_exponential_weight * m_last_pose(0,3) + (1 - m_exponential_weight) * pres_pose(0,3);
      m_last_pose(1,3) = m_exponential_weight * m_last_pose(1,3) + (1 - m_exponential_weight) * pres_pose(1,3);
      m_last_pose(2,3) = m_exponential_weight * m_last_pose(2,3) + (1 - m_exponential_weight) * pres_pose(2,3);
      printf("\n---\npose_x: %.4f\npose_y: %.4f\npose_z: %.4f\n---\n", m_last_pose(0,3), m_last_pose(1,3), m_last_pose(2,3));
    }
    else{
      m_last_pose = m_last_pose;
    }
  }

  void extendedKalmanFilter::sensorTFCorrection()
  {
    Eigen::Vector4d correct_position = m_last_pose * m_sensor_diff;
    m_last_pose(0,3) = correct_position(0);
    m_last_pose(1,3) = correct_position(1);
    m_last_pose(2,3) = correct_position(2);
    m_last_pose(3,3) = 1.0;
  }

  bool extendedKalmanFilter::calculateDifference(const Eigen::Quaterniond &pres_orientation)
  {
    if(sqrt(pow(m_last_orientation.x() - pres_orientation.x(),2)) > 0.001 || sqrt(pow(m_last_orientation.y() - pres_orientation.y(),2)) > 0.001 || sqrt(pow(m_last_orientation.z() - pres_orientation.z(),2)) > 0.001 || sqrt(pow(m_last_orientation.w() - pres_orientation.w(),2)) > 0.001){
      return false;
    }
    return true;
  }

  extendedKalmanFilter::extendedKalmanFilter(void)
  {
  }
}