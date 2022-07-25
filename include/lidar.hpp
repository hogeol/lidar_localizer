#ifndef _LIDAR_
#define _LIDAR_
#include <string>

namespace Lidar{
  class lidarClass{
  private:

  public:
    int m_scan_line;
    double m_max_dis;
    double m_min_dis;
    double m_vertical_angle;

    void setScanLine(const int &scan_line);
    void setMaxDis(const double &max_dis);
    void setMinDis(const double &min_dis);
    void setVerticalAngle(const double &vertical_angle);
      lidarClass();
  };
}

#endif _LIDAR_
