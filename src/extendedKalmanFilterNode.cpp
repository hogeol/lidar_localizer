//stl
#include <queue>
#include <mutex>
#include <thread>

//ros
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "extendedKalmanFilter.hpp"

std::queue<nav_msgs::OdometryConstPtr> lidar_pose_buf;

std::mutex mutex_control;

ExtendedKalmanFilter::extendedKalmanFilter extended_kalman_filter;

void ndtCallback(const nav_msgs::OdometryConstPtr &lidar_pose_msg)
{
  mutex_control.lock();
  lidar_pose_buf.push(lidar_pose_msg);
  mutex_control.unlock();
}

void ekfProcess()
{
  while(1){
    if(!lidar_pose_buf.empty()){
      
    }
    std::chrono::milliseconds dura(3);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "kalmanFilter");
  ros::NodeHandle nh;

  ros::Subscriber lidar_pose_sub = nh.subscribe<nav_msgs::Odometry>("ndt_pose", 1, ndtCallback);

  std::thread ekfProcessProcessing{ekfProcess};

  ros::spin();

  return 0;
}