#include "lidar.hpp"

namespace Lidar{
  void lidarClass::setScanLine(const int &scan_line)
  {
    m_scan_line = scan_line; 
  }

  void lidarClass::setMaxDis(const double &max_dis)
  {
    m_max_dis = max_dis;
  }

  void lidarClass::setMinDis(const double &min_dis)
  {
    m_min_dis = min_dis;
  }
  void lidarClass::setVerticalAngle(const double &vertical_angle)
  {
    m_vertical_angle = vertical_angle;
  }
  lidarClass::lidarClass()
  {
  }
}