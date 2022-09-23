#ifndef _RANGE_
#define _RANGE_

namespace Range{
  class range{
  private:

  public:
    void setRangeFront(const double &x_min);
    void setRangeBack(const double &x_max);
    void setRangeLeft(const double &y_min);
    void setRangeRight(const double &y_max);
    void setRangeBottom(const double &z_min);
    void setRangeTop(const double &z_max);
    double mp_range_front;
    double mp_range_back;
    double mp_range_left;
    double mp_range_right;
    double mp_range_bottom;
    double mp_range_top;
      range(void);
  };
}
#endif
