//STL
#include <string>
#include <queue>
#include <mutex>
#include <thread>

//ros
#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

//Eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

//local lib
#include "gpsImu.hpp"

GpsImu::gpsImu gps_imu;

//publisher
ros::Publisher odom_pub;

std::mutex mutex_lock;
std::queue<geometry_msgs::PointStampedConstPtr> utm_buf;
std::queue<sensor_msgs::ImuConstPtr> imu_buf;

void imuHandler(const sensor_msgs::ImuConstPtr &imu_msg)
{
  mutex_lock.lock();
  imu_buf.push(imu_msg);
  mutex_lock.unlock();
}

void utmCallback(const geometry_msgs::PointStampedConstPtr &utm_msg)
{
  mutex_lock.lock();
  utm_buf.push(utm_msg);
  mutex_lock.unlock();
}

void gpsImu()
{
  while(1){
    if(!utm_buf.empty() && !imu_buf.empty()){
      mutex_lock.lock();
      ros::Time utm_in_time = utm_buf.back()->header.stamp;
      ros::Time imu_in_time = imu_buf.back()->header.stamp;
      Eigen::Vector3f pose_tmp(utm_buf.back()->point.x, utm_buf.back()->point.y, utm_buf.back()->point.z);
      Eigen::Quaterniond eigen_quat_in(imu_buf.back()->orientation.w, imu_buf.back()->orientation.x, imu_buf.back()->orientation.y, imu_buf.back()->orientation.z);
      imu_buf.pop();
      utm_buf.pop();
      mutex_lock.unlock();
      double set_yaw = -0.1;
      Eigen::Quaterniond eigen_quat_out;
      gps_imu.setYaw(eigen_quat_in, eigen_quat_out, set_yaw);
      eigen_quat_out.normalize();
      nav_msgs::Odometry odom_msg;
      odom_msg.header.frame_id = "map";
      odom_msg.child_frame_id = "base_link";
      odom_msg.header.stamp = utm_in_time;
      odom_msg.pose.pose.position.x = pose_tmp(0);
      odom_msg.pose.pose.position.y = pose_tmp(1);
      //odom_msg.pose.pose.position.z = pose_tmp(2);
      odom_msg.pose.pose.position.z = 0.0;
      odom_msg.pose.pose.orientation.w = eigen_quat_out.w();
      odom_msg.pose.pose.orientation.x = eigen_quat_out.x();
      odom_msg.pose.pose.orientation.y = eigen_quat_out.y();
      odom_msg.pose.pose.orientation.z = eigen_quat_out.z();

      odom_pub.publish(odom_msg);
    }
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc ,char **argv)
{
  ros::init(argc, argv, "gpsImu");
  ros::NodeHandle nh;

  std::string imu_topic="/vectornav/IMU";
  int filter_window_size = 2;

  nh.getParam("imu_topic", imu_topic);
  nh.getParam("window_size", filter_window_size);

  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic, 1, imuHandler);
  ros::Subscriber utm_sub = nh.subscribe<geometry_msgs::PointStamped>("/utm", 1, utmCallback);

  odom_pub = nh.advertise<nav_msgs::Odometry>("/gps_odom", 1);

  std::thread gpsImuProcess{gpsImu};

  ros::spin();

  return 0;
}