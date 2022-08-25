#include "extendedKalmanFilter.hpp"

namespace ExtendedKalmanFilter{
  void extendedKalmanFilter::setInitPosition(const Eigen::Matrix4d &pres_pose)
  {
    m_last_pose = pres_pose;
    m_last_orientation = Eigen::Quaterniond(m_last_pose.block<3,3>(0,0));
  }

  void extendedKalmanFilter::correctionInit(const int &window_size)
  {
    m_window_size = window_size;
    m_exponential_weight = 1.0 - (1.0 / (double)m_window_size);
    m_last_pose = Eigen::Matrix4d::Identity();
    m_last_orientation = Eigen::Quaterniond::Identity();
  }

  void extendedKalmanFilter::processKalmanFilter(const Eigen::Matrix4d &pres_pose, Eigen::Matrix4d &pose_out)
  {
    exponentialWeight(pres_pose);
    m_last_pose.block<3,3>(0,0) = m_last_orientation.toRotationMatrix();
    pose_out = m_last_pose;
  }

  void extendedKalmanFilter::exponentialWeight(const Eigen::Matrix4d &pres_pose)
  {
    Eigen::Quaterniond pres_orientation(pres_pose.block<3,3>(0,0));
    m_last_pose(0,3) = m_exponential_weight * m_last_pose(0,3) + (1 - m_exponential_weight) * pres_pose(0,3);
    m_last_pose(1,3) = m_exponential_weight * m_last_pose(1,3) + (1 - m_exponential_weight) * pres_pose(1,3);
    m_last_pose(2,3) = m_exponential_weight * m_last_pose(2,3) + (1 - m_exponential_weight) * pres_pose(2,3);
    if(angleThreshold(pres_orientation)){
      m_last_orientation.w() = m_exponential_weight * m_last_orientation.w() + (1 - m_exponential_weight) * pres_orientation.w();
      m_last_orientation.x() = m_exponential_weight * m_last_orientation.x() + (1 - m_exponential_weight) * pres_orientation.x();
      m_last_orientation.y() = m_exponential_weight * m_last_orientation.y() + (1 - m_exponential_weight) * pres_orientation.y();
      m_last_orientation.z() = m_exponential_weight * m_last_orientation.z() + (1 - m_exponential_weight) * pres_orientation.z();
    }
    else{
      m_last_orientation = pres_pose.block<3,3>(0,0);
    }
  }

  bool extendedKalmanFilter::angleThreshold(const Eigen::Quaterniond &last_quaternion)
  {
    if(sqrt(pow(last_quaternion.w() - m_last_orientation.w(),2)) > 0.01 || sqrt(pow(last_quaternion.x() - m_last_orientation.x(),2)) > 0.01 || sqrt(pow(last_quaternion.y() - m_last_orientation.y(),2)) > 0.01 || sqrt(pow(last_quaternion.z() - m_last_orientation.z(),2)) > 0.01){
      return false;
    }
    return true;
  }

  extendedKalmanFilter::extendedKalmanFilter(void)
  {
  }
}