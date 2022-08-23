//stl
#include <queue>
#include <mutex>
#include <thread>
//ros
#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
//eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
//tf
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

#include "extendedKalmanFilter.hpp"

ros::Publisher odom_pub;

std::queue<geometry_msgs::PoseWithCovarianceStampedConstPtr> ndt_pose_buf;

std::mutex mutex_control;

ExtendedKalmanFilter::extendedKalmanFilter extended_kamlan_filter;

void ndtCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &ndt_pose_msg)
{
  mutex_control.lock();
  ndt_pose_buf.push(ndt_pose_msg);
  mutex_control.unlock();
}

void ekfProcess()
{
  while(1){
    if(!ndt_pose_buf.empty()){
      ros::Time ndt_pose_in_time = ndt_pose_buf.front()->header.stamp;

    }
    std::chrono::milliseconds dura(3);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kalmanFilter");
  ros::NodeHandle nh;

  double sensor_diff_x = 0.0;
  double sensor_diff_y = 0.0;
  double sensor_diff_z = 0.0;
  int ekf_window_size = 1;

  nh.getParam("sensor_diff_x", sensor_diff_x);
  nh.getParam("sensor_diff_y", sensor_diff_y);
  nh.getParam("sensor_diff_z", sensor_diff_z);
  nh.getParam("ekf_window_size", ekf_window_size);

  ros::Subscriber ndt_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("ndt_odom", 1, ndtCallback);

  odom_pub = nh.advertise<nav_msgs::Odometry>("final_odom", 1);

  std::thread ekfProcessProcessing{ekfProcess};
  
  ros::spin();
  return 0;
}