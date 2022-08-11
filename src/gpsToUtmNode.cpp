//stl
#include <string>
#include <queue>
#include <thread>
#include <mutex>
#include <Eigen/Core>
#include <Eigen/Dense>
//ros
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Header.h>
#include <tf/tf.h>
//local lib
#include "gpsToUtm.hpp"

//publisher
ros::Publisher utm_pub;

GpsToUtm::gpsToUtm gpsToUtmClass;
std::queue<sensor_msgs::NavSatFixConstPtr> gps_buf;
std::mutex mutex_control;

void gpsCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg)
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
      std_msgs::Header gps_in_header = gps_buf.front()->header;
      double latitude = gps_buf.front()->latitude;
      double longitude = gps_buf.front()->longitude;
      double altitude = gps_buf.front()->altitude;
      gps_buf.pop();
      mutex_control.unlock();
      Eigen::Vector3d utm_pose;
      gpsToUtmClass.gpsConvertToUtm(latitude, longitude, altitude, utm_pose);
        
      geometry_msgs::PoseStamped local_pose;
      tf::Quaternion q;
      q.setRPY(0.0, 0.0, 0.0);
      local_pose.header = gps_in_header;
      local_pose.pose.position.x = utm_pose(0);
      local_pose.pose.position.y = utm_pose(1);
      local_pose.pose.position.z = utm_pose(2);
      local_pose.pose.orientation.w = q.w();
      local_pose.pose.orientation.x = q.x();
      local_pose.pose.orientation.y = q.y();
      local_pose.pose.orientation.z = q.z();
      utm_pub.publish(local_pose);
    }
    std::chrono::milliseconds dura(3);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"gpsToUtm");

  ros::NodeHandle nh;
  
  std::string hemisphere = "North";
  std::string gps_topic = "fix";
  double offset_x=0;
  double offset_y=0;
  double offset_z=0;
  int utm_zone=52;
  ROS_INFO("You can find utm zone at https://mangomap.com/robertyoung/maps/69585/what-utm-zone-am-i-in-#");


  if(nh.getParam("hemisphere",hemisphere)){
    ROS_INFO("North hemisphere or south hemisphere? %s ",hemisphere.c_str());
    if(hemisphere != "North" && hemisphere != "South"){
      ROS_ERROR("hemisphere only can equal to North or South!");
    }
  }
  else{
    ROS_WARN("You need to specify you are in the north or south hemisphere. Default north");
    hemisphere = "North";
  }

  nh.getParam("utm_zone", utm_zone);
  nh.getParam("gps_topic", gps_topic);
  nh.getParam("gps_offset_x", offset_x);
  nh.getParam("gps_offset_y", offset_y);
  nh.getParam("gps_offset_z", offset_z);
  
  gpsToUtmClass.init(hemisphere, utm_zone);
  gpsToUtmClass.setGpsOffset(offset_x, offset_y, offset_z);

  ros::Subscriber gps_sub = nh.subscribe(gps_topic,1 ,gpsCallback);
  utm_pub = nh.advertise<geometry_msgs::PoseStamped>("/utm", 1);

  std::thread gpsToUtmProcess{gpsToUtm};

  ros::spin();
  return 0;  
}
