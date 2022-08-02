#ifndef _GPS_TO_UTM_
#define _GPS_TO_UTM_

#include <cmath>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <string>
#include <GeographicLib/TransverseMercator.hpp>

namespace GpsToUtm{
  //resource from wiki  Universal Transverse Mercator coordinate system. WGS 84
  struct basic{
    double bx = 0.0;//-594929.9431329881;//east
    double by = 0.0;//-4139043.529676078;//north
    double bz = 0.0;//unit in m
  };//origin point
  
  class gpsToUtm{ 
  private:
    const double m_kNN = 0;
    const double m_kNS = 10000000.0;
    const double m_kE0 = 500000.0;
    double m_at = 6378137.0; //unit in m
    double m_fla = 1.0/298.257223563; 
    double m_k0 = 0.9996;

    std::string m_hemi;
    int m_zone;
    double m_offset_x;
    double m_offset_y;
    double m_offset_z;
    basic m_cardinal;
  
  public:
    void init(const std::string &hemi, const int &zone);
    void gpsConvertToUtm(const double &latitude, const double &longitude, const double &altitude, Eigen::Vector3d &pose_out);
    void setGpsOffset(const double &x, const double &y, const double &z);
    std::string getHemi();
    double getZone();
    double getOffsetX();
    double getOffsetY();
    double getOffsetZ();
      gpsToUtm(void);
  };
}

#endif


