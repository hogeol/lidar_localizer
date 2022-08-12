#include "extendedKalmanFilter.hpp"

namespace ExtendedKalmanFilter{
  void extendedKalmanFilter::init(const int &window_size, const double &diff_x, const double &diff_y, const double &diff_z)
  {
    m_window_size = window_size;
    m_sensor_diff(0) = diff_x;
    m_sensor_diff(1) = diff_y;
    m_sensor_diff(2) = diff_z;
    m_sensor_diff(3) = 1.0;
    m_exponential_weight = 1.0 - (1.0 / (double)m_window_size);
    printf("\nweight: %.4f\n", m_exponential_weight);
    m_prev_orientation.w() = 0.0;
    m_prev_orientation.x() = 0.0;
    m_prev_orientation.y() = 0.0;
    m_prev_orientation.z() = 0.0;
    m_prev_position(0) = 0.0;
    m_prev_position(0) = 0.0;
    m_prev_position(0) = 0.0;
  }

  void extendedKalmanFilter::exponentialWeight(const Eigen::Matrix4f &pres_pose, Eigen::Matrix4f &pose_out)
  {
    Eigen::Quaternionf pres_orientation(pres_pose.block<3,3>(0,0));
    if(calculateDifference(pres_orientation)){
      m_prev_orientation.w() = m_exponential_weight * m_prev_orientation.w() + (1 - m_exponential_weight) * pres_orientation.w();
      m_prev_orientation.x() = m_exponential_weight * m_prev_orientation.x() + (1 - m_exponential_weight) * pres_orientation.x();
      m_prev_orientation.y() = m_exponential_weight * m_prev_orientation.y() + (1 - m_exponential_weight) * pres_orientation.y();
      m_prev_orientation.z() = m_exponential_weight * m_prev_orientation.z() + (1 - m_exponential_weight) * pres_orientation.z();
      m_prev_position(0) = m_exponential_weight * m_prev_position(0) + (1 - m_exponential_weight) * pres_pose(0,3);
      m_prev_position(1) = m_exponential_weight * m_prev_position(1) + (1 - m_exponential_weight) * pres_pose(1,3);
      m_prev_position(2) = m_exponential_weight * m_prev_position(2) + (1 - m_exponential_weight) * pres_pose(2,3);
      sensorTFCorrection();
      pose_out.block<3,3>(0,0) = m_prev_orientation.toRotationMatrix();
      pose_out(0,3) = m_prev_position(0);
      pose_out(1,3) = m_prev_position(1);
      pose_out(2,3) = m_prev_position(2);
    }
    else{
      pose_out = pres_pose;
    }
    
  }

  void extendedKalmanFilter::sensorTFCorrection()
  {
    Eigen::Matrix4f bias;
    bias.block<3,3>(0,0) = m_prev_orientation.toRotationMatrix().cast<float>();
    bias(0,3) = m_prev_position(0);
    bias(1,3) = m_prev_position(1);
    bias(2,3) = m_prev_position(2);
    Eigen::Vector4f sensor_diff = bias * m_sensor_diff;
    m_prev_position(0) = sensor_diff(0);
    m_prev_position(1) = sensor_diff(1);
    m_prev_position(2) = sensor_diff(2);
  }

  bool extendedKalmanFilter::calculateDifference(const Eigen::Quaternionf &pres_orientation)
  {
    if(sqrt(pow(m_prev_orientation.x() - pres_orientation.x(),2)) > 0.001 || sqrt(pow(m_prev_orientation.y() - pres_orientation.y(),2)) > 0.001 || sqrt(pow(m_prev_orientation.z() - pres_orientation.z(),2)) > 0.001 || sqrt(pow(m_prev_orientation.w() - pres_orientation.w(),2)) > 0.001){
      return false;
    }
    return true;
  }

  extendedKalmanFilter::extendedKalmanFilter(void)
  {
  }

}