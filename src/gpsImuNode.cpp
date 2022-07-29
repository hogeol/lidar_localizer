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

GpsImu::gpsImu gpsImuClass;

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
      ros::Time utm_in_time = utm_buf.front()->header.stamp;
      ros::Time imu_in_time = imu_buf.front()->header.stamp;
      Eigen::Vector3f pose_tmp(utm_buf.front()->point.x, utm_buf.front()->point.y, utm_buf.front()->point.z);
      Eigen::Quaternionf eigen_quat(imu_buf.front()->orientation.x, imu_buf.front()->orientation.y, imu_buf.front()->orientation.z, imu_buf.front()->orientation.w);
      imu_buf.pop();
      utm_buf.pop();
      mutex_lock.unlock();

      Eigen::Matrix4f eigen_pose = Eigen::Matrix4f::Identity();
      eigen_pose.block<3,3>(0,0) = eigen_quat.toRotationMatrix();
      Eigen::AngleAxisf init_rot(0.1, Eigen::Vector3f::UnitZ());
      Eigen::Translation3f init_translation(pose_tmp(0), pose_tmp(1), pose_tmp(2));
      eigen_pose = (init_translation*init_rot).matrix()*eigen_pose;
      eigen_quat = eigen_pose.block<3,3>(0,0);
      eigen_quat.normalize();
      nav_msgs::Odometry odom_msg;
      odom_msg.header.frame_id = "map";
      odom_msg.child_frame_id = "base_link";
      odom_msg.header.stamp = utm_in_time;
      odom_msg.pose.pose.position.x = eigen_pose(0,3);
      odom_msg.pose.pose.position.y = eigen_pose(1,3);
      odom_msg.pose.pose.position.z = eigen_pose(2,3);
      odom_msg.pose.pose.orientation.w = eigen_quat.w();
      odom_msg.pose.pose.orientation.x = eigen_quat.x();
      odom_msg.pose.pose.orientation.y = eigen_quat.y();
      odom_msg.pose.pose.orientation.z = eigen_quat.z();

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