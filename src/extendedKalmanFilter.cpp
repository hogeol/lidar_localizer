#include "extendedKalmanFilter.hpp"

namespace ExtendedKalmanFilter{
  void extendedKalmanFilter::init(const int &window_size)
  {
    m_window_size = window_size;
    m_prev_state.resize(4);
  }

  extendedKalmanFilter::extendedKalmanFilter(void)
  {
  }

}