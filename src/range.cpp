#include "range.hpp"

namespace Range{
  void range::setRangeFront(const double &x_min)
  {
    mp_range_front = x_min;
  }

  void range::setRangeBack(const double &x_max)
  {
    mp_range_back = x_max;
  }

  void range::setRangeLeft(const double &y_min)
  {
    mp_range_left = y_min;
  }

  void range::setRangeRight(const double &y_max)
  {
    mp_range_right = y_max;
  }

  void range::setRangeBottom(const double &z_min)
  {
    mp_range_bottom = z_min;
  }

  void range::setRangeTop(const double &z_max)
  {
    mp_range_top = z_max;
  }
  range::range(void)
  {
  }
}
