//stl
#include <queue>
#include <mutex>
#include <thread>
//ros
#include <ros/ros.h>
#include <std_msgs/Header.h>
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

//publisher
ros::Publisher final_odom_pub;

std::queue<geometry_msgs::PoseWithCovarianceStampedConstPtr> ndt_pose_buf;

std::mutex mutex_control;

ExtendedKalmanFilter::extendedKalmanFilter extended_kalman_filter;

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
      mutex_control.lock();
      std_msgs::Header header_in_ndt = ndt_pose_buf.back()->header;
      Eigen::Quaterniond quaternion_in_ndt = Eigen::Quaterniond(ndt_pose_buf.back()->pose.pose.orientation.w, ndt_pose_buf.back()->pose.pose.orientation.x, ndt_pose_buf.back()->pose.pose.orientation.y, ndt_pose_buf.back()->pose.pose.orientation.z);
      Eigen::Vector3d position_in_ndt = Eigen::Vector3d(ndt_pose_buf.back()->pose.pose.position.x, ndt_pose_buf.back()->pose.pose.position.y, ndt_pose_buf.back()->pose.pose.position.z);
      mutex_control.unlock();
      Eigen::Matrix4d pose_in_ndt = Eigen::Matrix4d::Identity();
      Eigen::Matrix4d final_pose = Eigen::Matrix4d::Identity();
      pose_in_ndt.block<3,3>(0,0) = quaternion_in_ndt.toRotationMatrix();
      pose_in_ndt(0,3) = position_in_ndt(0);
      pose_in_ndt(1,3) = position_in_ndt(1);
      pose_in_ndt(2,3) = position_in_ndt(2);
      extended_kalman_filter.processKalmanFilter(pose_in_ndt, final_pose);
      
      quaternion_in_ndt = final_pose.block<3,3>(0,0);
      //final odometry
      nav_msgs::Odometry final_pose_msg;
      final_pose_msg.header = header_in_ndt;
      final_pose_msg.pose.pose.position.x = final_pose(0,3);
      final_pose_msg.pose.pose.position.y = final_pose(1,3);
      final_pose_msg.pose.pose.position.z = final_pose(2,3);
      final_pose_msg.pose.pose.orientation.w = quaternion_in_ndt.w();
      final_pose_msg.pose.pose.orientation.w = quaternion_in_ndt.x();
      final_pose_msg.pose.pose.orientation.w = quaternion_in_ndt.y();
      final_pose_msg.pose.pose.orientation.w = quaternion_in_ndt.z();
      final_odom_pub.publish(final_pose_msg);

      //ndt pose transform
      static tf2_ros::TransformBroadcaster tf2_ndt_br;
      geometry_msgs::TransformStamped transformStamped_ndt;
      transformStamped_ndt.header.stamp = ros::Time::now();
      transformStamped_ndt.header.frame_id = "map";
      transformStamped_ndt.child_frame_id = "ndt";
      transformStamped_ndt.transform.translation.x = final_pose(0,3);
      transformStamped_ndt.transform.translation.y = final_pose(1,3);
      transformStamped_ndt.transform.translation.z = 0.0;
      transformStamped_ndt.transform.rotation.w = quaternion_in_ndt.w();
      transformStamped_ndt.transform.rotation.x = quaternion_in_ndt.x();
      transformStamped_ndt.transform.rotation.y = quaternion_in_ndt.y();
      transformStamped_ndt.transform.rotation.z = quaternion_in_ndt.z();
      tf2_ndt_br.sendTransform(transformStamped_ndt);
    }
    std::chrono::milliseconds dura(3);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kalmanFilter");
  ros::NodeHandle nh;

  int ekf_window_size = 10;
  double sensor_diff_x = 0.0;
  double sensor_diff_y = 0.0;
  double sensor_diff_z = 0.0;

  nh.getParam("sensor_diff_x", sensor_diff_x);
  nh.getParam("sensor_diff_y", sensor_diff_y);
  nh.getParam("sensor_diff_z", sensor_diff_z);
  nh.getParam("ekf_window_size", ekf_window_size);

  ros::Subscriber ndt_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("ndt_odom", 1, ndtCallback);

  final_odom_pub = nh.advertise<nav_msgs::Odometry>("final_odom", 1);

  extended_kalman_filter.correctionInit(ekf_window_size, sensor_diff_x, sensor_diff_y, sensor_diff_z);

  std::thread ekfProcessProcessing{ekfProcess};

  ros::spin();
  return 0;
}