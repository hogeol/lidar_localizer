//STL
#include <string>
#include <queue>
#include <thread>
#include <mutex>
//ros
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Header.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
//local lib
#include "gpsToUtm.hpp"

//publisher
ros::Publisher utm_pub;

GpsToUtm::gpsToUtm gpsToUtmClass;
std::queue<sensor_msgs::NavSatFixConstPtr> gps_buf;
std::mutex mutex_lock;

void gpsCallback(const sensor_msgs::NavSatFixConstPtr& gps_msg)
{
  mutex_lock.lock();
  gps_buf.push(gps_msg);
  mutex_lock.unlock();  
}

void gpsToUtm()
{
  while(1){
    if(!gps_buf.empty()){
      mutex_lock.lock();
      // ROS_INFO("Get data lagitude from GPS: [%f]", gps_buf.front()->latitude);
      // ROS_INFO("Get data longitude from GPS: [%f]", gps_buf.front()->longitude);
      // ROS_INFO("Get data altitude from GPS: [%f]", gps_buf.front()->altitude);
      std_msgs::Header gps_in_header = gps_buf.front()->header;
      double latitude = gps_buf.front()->latitude;
      double longitude = gps_buf.front()->longitude;
      double altitude = gps_buf.front()->altitude;
      gps_buf.pop();
      mutex_lock.unlock();
      std::vector<double> utm_xyz;
      gpsToUtmClass.gpsConvertToUtm(latitude, longitude, altitude, utm_xyz);
        
      geometry_msgs::PoseStamped local_pose;
      tf::Quaternion q;
      q.setRPY(0.0, 0.0, 0.0);
      local_pose.header = gps_in_header;
      local_pose.pose.position.x = utm_xyz[0];
      local_pose.pose.position.y = utm_xyz[1];
      local_pose.pose.position.z = utm_xyz[2];
      local_pose.pose.orientation.w = q.w();
      local_pose.pose.orientation.x = q.x();
      local_pose.pose.orientation.y = q.y();
      local_pose.pose.orientation.z = q.z();
      // geometry_msgs::Pose local_pose;
      // tf::Quaternion q;
      // q.setRPY(0.0, 0.0, 0.0);
      // local_pose.position.x = utm_xyz[0];
      // local_pose.position.y = utm_xyz[1];
      // local_pose.position.z = utm_xyz[2];
      // local_pose.orientation.w = q.w();
      // local_pose.orientation.x = q.x();
      // local_pose.orientation.y = q.y();
      // local_pose.orientation.z = q.z();

      utm_pub.publish(local_pose);
      utm_xyz.clear();
        
      // ROS_INFO("Convert to x: [%f]", local_pose.pose.position.x);
      // ROS_INFO("Convert to y: [%f]", local_pose.pose.position.y);
      // ROS_INFO("Convert to z: [%f]", local_pose.pose.position.z);
      // ROS_INFO("Convert to ow: [%f]", local_pose.pose.orientation.w);
      // ROS_INFO("Convert to ox: [%f]", local_pose.pose.orientation.x);
      // ROS_INFO("Convert to oy: [%f]", local_pose.pose.orientation.y);
      // ROS_INFO("Convert to oz: [%f]", local_pose.pose.orientation.z);
    }
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc,argv,"gpsToUtm");

  ros::NodeHandle nh;
  
  std::string hemisphere = "North";
  double offset_x=0;
  double offset_y=0;
  double offset_z=0;
  int zone=52;

  if(nh.getParam("utm_zone",zone)){
    ROS_INFO("UTM coordinate in zone %d ",zone);
  }
  else{
    ROS_ERROR("Error! You must specify the utm zone. Easy to find it at https://mangomap.com/robertyoung/maps/69585/what-utm-zone-am-i-in-#");
    ros::shutdown();
  }

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
  nh.getParam("gps_offset_x", offset_x);
  nh.getParam("gps_offset_y", offset_y);
  nh.getParam("gps_offset_z", offset_z);
  
  gpsToUtmClass.init(hemisphere, zone);
  gpsToUtmClass.setGpsOffset(offset_x, offset_y, offset_z);

  ros::Subscriber gps_sub = nh.subscribe("/fix",1 ,gpsCallback);
  utm_pub = nh.advertise<geometry_msgs::PointStamped>("/utm", 1);
  //utm_pub = nh.advertise<geometry_msgs::Pose>("/utm", 100);

  std::thread gpsToUtmProcess{gpsToUtm};

  ros::spin();
  return 0;  
}
