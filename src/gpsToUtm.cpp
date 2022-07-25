#include "gpsToUtm.hpp"

namespace GpsToUtm{
  
  void gpsToUtm::init(const std::string &hemi, const int &zone)
  {
    m_hemi = hemi;
    m_zone = zone;
  }
  
  void gpsToUtm::gpsConvertToUtm(const double &latitude, const double &longitude, const double &altitude, std::vector<double> &xyz_out)
  {
    int lon0  = m_zone*6-183;
    double east,north;

    GeographicLib::TransverseMercator tm(m_at, m_fla, m_k0);

    tm.Forward(lon0, latitude, longitude, east, north);

    east += m_kE0;
    north += (m_hemi == "North") ? m_kNN : m_kNS;

    xyz_out.push_back(east - m_offset_x + m_cardinal.bx);
    xyz_out.push_back(north - m_offset_y + m_cardinal.by);
    xyz_out.push_back(altitude - m_offset_z + m_cardinal.bz);
  }

  void gpsToUtm::setGpsOffset(const double &x, const double &y, const double &z)
  {
    m_offset_x = x;
    m_offset_y = y;
    m_offset_z = z;
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