//stl
#include <string>
#include <queue>
#include <thread>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/Dense>
//ros
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <std_msgs/msg/header.hpp>
//local lib
#include "gpsToUtm.hpp"

//publisher
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr utm_pub;

std::queue<sensor_msgs::msg::NavSatFix::ConstPtr> gps_buf;

std::mutex mutex_control;

GpsToUtm::gpsToUtm gpsToUtmClass;

void gpsCallback(const sensor_msgs::msg::NavSatFix::ConstPtr& gps_msg)
{
  mutex_control.lock();
  gps_buf.push(gps_msg);
  mutex_control.unlock();  
}

void gpsToUtm()
{
  while(1){
    if(!gps_buf.empty()){
      mutex_control.lock();
      std_msgs::msg::Header gps_in_header = gps_buf.front()->header;
      double latitude = gps_buf.front()->latitude;
      double longitude = gps_buf.front()->longitude;
      double altitude = gps_buf.front()->altitude;
      gps_buf.pop();
      mutex_control.unlock();
      Eigen::Vector3d utm_pose;
      gpsToUtmClass.gpsConvertToUtm(latitude, longitude, altitude, utm_pose);
        
      geometry_msgs::msg::PoseStamped local_pose;
      Eigen::Matrix3d local_rpy = Eigen::Matrix3d::Identity();
      Eigen::Quaterniond local_quat{local_rpy};
      local_pose.header = gps_in_header;
      local_pose.pose.position.x = utm_pose.x();
      local_pose.pose.position.y = utm_pose.y();
      local_pose.pose.position.z = utm_pose.z();
      local_pose.pose.orientation.w = local_quat.w();
      local_pose.pose.orientation.x = local_quat.x();
      local_pose.pose.orientation.y = local_quat.y();
      local_pose.pose.orientation.z = local_quat.z();
      utm_pub -> publish(local_pose);
    }
    std::chrono::milliseconds dura(3);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr nh;
  
  std::string hemisphere = "North";
  std::string gps_topic = "fix";
  double offset_x=0;
  double offset_y=0;
  double offset_z=0;
  int utm_zone=52;
  
  RCLCPP_INFO(nh->get_logger(), "You can find utm zone at https://mangomap.com/robertyoung/maps/69585/what-utm-zone-am-i-in-#");

  if(nh -> get_parameter("hemisphere",hemisphere)){
    
    RCLCPP_INFO(nh->get_logger(), "North hemisphere or south hemisphere? %s ",hemisphere.c_str());
    if(hemisphere != "North" && hemisphere != "South"){
      RCLCPP_INFO(nh->get_logger(), "hemisphere only can equal to North or South!");
    }
  }
  else{
    RCLCPP_WARN(nh->get_logger(), "You need to specify you are in the north or south hemisphere. Default north");
    hemisphere = "North";
  }
  nh -> declare_parameter("utm_zone", 52);
  nh -> declare_parameter("gps_topic", "fix");
  nh -> declare_parameter("gps_offset_x", 0);
  nh -> declare_parameter("gps_offset_y", 0);
  nh -> declare_parameter("gps_offset_z", 0);

  nh -> get_parameter("utm_zone", utm_zone);
  nh -> get_parameter("gps_topic", gps_topic);
  nh -> get_parameter("gps_offset_x", offset_x);
  nh -> get_parameter("gps_offset_y", offset_y);
  nh -> get_parameter("gps_offset_z", offset_z);
  
  gpsToUtmClass.init(hemisphere, utm_zone);
  gpsToUtmClass.setGpsOffset(offset_x, offset_y, offset_z);

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub{nh -> create_subscription<sensor_msgs::msg::NavSatFix>(gps_topic, 1, gpsCallback)};

  utm_pub = nh->create_publisher<geometry_msgs::msg::PoseStamped>("/utm", 1);

  std::thread gpsToUtmProcess{gpsToUtm};

  rclcpp::spin(nh);

  rclcpp::shutdown();

  return 0;  
}
