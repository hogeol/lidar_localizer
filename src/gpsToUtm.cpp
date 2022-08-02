#include "gpsToUtm.hpp"

namespace GpsToUtm{
  
  void gpsToUtm::init(const std::string &hemi, const int &zone)
  {
    m_hemi = hemi;
    m_zone = zone;
  }
  
  void gpsToUtm::gpsConvertToUtm(const double &latitude, const double &longitude, const double &altitude, Eigen::Vector3d &pose_out)
  {
    int lon0  = m_zone*6-183;
    double east,north;

    GeographicLib::TransverseMercator tm(m_at, m_fla, m_k0);

    tm.Forward(lon0, latitude, longitude, east, north);

    east += m_kE0;
    north += (m_hemi == "North") ? m_kNN : m_kNS;
    double x = east + m_cardinal.bx - m_offset_x, y = north + m_cardinal.by - m_offset_y, z = altitude + m_cardinal.bz - m_offset_z;
    pose_out << x, y, z;
    //printf("\n--\nbefore offset: %.3f, %.3f, %.3f\n", east + m_cardinal.bx, north + m_cardinal.by, altitude + m_cardinal.bz);
    //printf("\n--\noffset: x: %.4f, y: %.4f, z: %.4f\n---\n", m_offset_x, m_offset_y, m_offset_z);
  }

  void gpsToUtm::setGpsOffset(const double &x, const double &y, const double &z)
  {
    m_offset_x = x;
    m_offset_y = y;
    m_offset_z = z;
    printf("\n--\noffset: x: %.4f, y: %.4f, z: %.4f\n---\n", m_offset_x, m_offset_y, m_offset_z);
  } 

  std::string gpsToUtm::getHemi()
  {
    return m_hemi;
  }

  double gpsToUtm::getZone()
  {
    return m_zone;
  }

  double gpsToUtm::getOffsetX()
  {
    return m_offset_x;
  }

  double gpsToUtm::getOffsetY()
  {
    return m_offset_y;
  }

  double gpsToUtm::getOffsetZ()
  {
    return m_offset_z;
  }

  gpsToUtm::gpsToUtm(void)
  {
  }
}