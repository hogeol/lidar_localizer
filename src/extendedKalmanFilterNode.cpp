//stl
#include <queue>
#include <mutex>
#include <thread>
//ros
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <nav_msgs/Odometry.h>
//eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
//tf
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include "extendedKalmanFilter.hpp"

//publisher
ros::Publisher final_odom_pub;

std::queue<nav_msgs::OdometryConstPtr> ndt_pose_buf;

std::mutex mutex_control;

ExtendedKalmanFilter::extendedKalmanFilter extended_kalman_filter;

void ndtCallback(const nav_msgs::OdometryConstPtr &ndt_pose_msg)
{
  mutex_control.lock();
  ndt_pose_buf.push(ndt_pose_msg);
  if(ndt_pose_buf.size()>1)
    ndt_pose_buf.pop();
  mutex_control.unlock();
}

bool position_init;
void ekfProcess()
{
  while(1){
    if(!ndt_pose_buf.empty()){
      mutex_control.lock();
      ros::Time time_in_ndt = ndt_pose_buf.front()->header.stamp;
      Eigen::Quaterniond orientation_in_ndt = Eigen::Quaterniond(ndt_pose_buf.front()->pose.pose.orientation.w, ndt_pose_buf.front()->pose.pose.orientation.x, ndt_pose_buf.front()->pose.pose.orientation.y, ndt_pose_buf.front()->pose.pose.orientation.z);
      Eigen::Vector3d position_in_ndt = Eigen::Vector3d(ndt_pose_buf.front()->pose.pose.position.x, ndt_pose_buf.front()->pose.pose.position.y, ndt_pose_buf.front()->pose.pose.position.z);
      ndt_pose_buf.pop();
      mutex_control.unlock();
      Eigen::Isometry3d pose_in_ndt = Eigen::Isometry3d::Identity();
      Eigen::Isometry3d final_pose = Eigen::Isometry3d::Identity();
      pose_in_ndt.linear() = orientation_in_ndt.toRotationMatrix();
      pose_in_ndt.translation() = position_in_ndt;
      if(position_init == false){
        extended_kalman_filter.setInitPosition(pose_in_ndt);
        position_init = true;
        continue;
      }
      extended_kalman_filter.processKalmanFilter(pose_in_ndt, final_pose);

      Eigen::Vector3d final_position = final_pose.translation();
      Eigen::Quaterniond final_orientation(final_pose.rotation());
      final_orientation.normalize();

      //after kalman filter tf
      static tf2_ros::TransformBroadcaster final_broadcaster;
      geometry_msgs::TransformStamped final_transform_stamped;

      final_transform_stamped.header.stamp = ros::Time::now();
      final_transform_stamped.header.frame_id = "map";
      final_transform_stamped.child_frame_id = "final";
      final_transform_stamped.transform.translation.x = final_position.x();
      final_transform_stamped.transform.translation.y = final_position.y();
      final_transform_stamped.transform.translation.z = final_position.z();
      final_transform_stamped.transform.rotation.w = final_orientation.w();
      final_transform_stamped.transform.rotation.x = final_orientation.x();
      final_transform_stamped.transform.rotation.y = final_orientation.y();
      final_transform_stamped.transform.rotation.z = final_orientation.z();

      final_broadcaster.sendTransform(final_transform_stamped);

      //final odometry
      nav_msgs::Odometry final_pose_msg;
      final_pose_msg.header.stamp = time_in_ndt;
      final_pose_msg.header.frame_id = "map";
      final_pose_msg.child_frame_id = "final";
      final_pose_msg.pose.pose.position.x = final_position.x();
      final_pose_msg.pose.pose.position.y = final_position.y();
      final_pose_msg.pose.pose.position.z = final_position.z();
      final_pose_msg.pose.pose.orientation.w = final_orientation.w();
      final_pose_msg.pose.pose.orientation.x = final_orientation.x();
      final_pose_msg.pose.pose.orientation.y = final_orientation.y();
      final_pose_msg.pose.pose.orientation.z = final_orientation.z();
      final_odom_pub.publish(final_pose_msg);
    }
    std::chrono::milliseconds dura(3);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kalmanFilter");
  ros::NodeHandle nh;

  position_init = false;

  int orientation_window_size = 10;
  int position_window_size = 10;

  nh.getParam("orientation_window_size", orientation_window_size);
  nh.getParam("position_window_size", position_window_size);

  extended_kalman_filter.correctionInit(orientation_window_size, position_window_size);
  
  ros::Subscriber ndt_sub = nh.subscribe<nav_msgs::Odometry>("/ndt_pose", 1, ndtCallback);

  final_odom_pub = nh.advertise<nav_msgs::Odometry>("/final_odom", 1);

  
  std::thread ekfProcessProcessing{ekfProcess};

  ros::spin();

  return 0;
}