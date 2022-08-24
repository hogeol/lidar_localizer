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
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

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

void ekfProcess()
{
  while(1){
    if(!ndt_pose_buf.empty()){
      mutex_control.lock();
      ros::Time time_in_ndt = ndt_pose_buf.front()->header.stamp;
      Eigen::Quaterniond quaternion_in_ndt = Eigen::Quaterniond(ndt_pose_buf.front()->pose.pose.orientation.w, ndt_pose_buf.front()->pose.pose.orientation.x, ndt_pose_buf.front()->pose.pose.orientation.y, ndt_pose_buf.front()->pose.pose.orientation.z);
      Eigen::Vector3d position_in_ndt = Eigen::Vector3d(ndt_pose_buf.front()->pose.pose.position.x, ndt_pose_buf.front()->pose.pose.position.y, ndt_pose_buf.front()->pose.pose.position.z);
      ndt_pose_buf.pop();
      mutex_control.unlock();
      Eigen::Matrix4d pose_in_ndt = Eigen::Matrix4d::Identity();
      Eigen::Matrix4d final_pose = Eigen::Matrix4d::Identity();
      pose_in_ndt.block<3,3>(0,0) = quaternion_in_ndt.toRotationMatrix();
      pose_in_ndt(0,3) = position_in_ndt(0);
      pose_in_ndt(1,3) = position_in_ndt(1);
      pose_in_ndt(2,3) = position_in_ndt(2);
      extended_kalman_filter.processKalmanFilter(pose_in_ndt, final_pose);
      //printf("\n---\npose:\nx: %.4f\ny: %.4f\nz: %.4f\n---\n", pose_in_ndt(0,3), pose_in_ndt(1,3), pose_in_ndt(2,3));

      //Eigen::Quaterniond final_quat(final_pose.block<3,3>(0,0));
      Eigen::Quaterniond final_quat;
      final_quat = quaternion_in_ndt;
      final_quat.normalize();

      //ndt pose transform
      static tf::TransformBroadcaster final_tf_br;
      tf::Transform tf_map_to_final;
      tf_map_to_final.setOrigin(tf::Vector3(final_pose(0,3), final_pose(1,3), final_pose(2,3)));
      tf_map_to_final.setRotation(tf::Quaternion(final_quat.x(), final_quat.y(), final_quat.z(), final_quat.w()));
      final_tf_br.sendTransform(tf::StampedTransform(tf_map_to_final, ros::Time::now(), "map", "final"));
      
      //final odometry
      nav_msgs::Odometry final_pose_msg;
      final_pose_msg.header.stamp = time_in_ndt;
      final_pose_msg.header.frame_id = "map";
      final_pose_msg.child_frame_id = "final";
      final_pose_msg.pose.pose.position.x = final_pose(0,3);
      final_pose_msg.pose.pose.position.y = final_pose(1,3);
      final_pose_msg.pose.pose.position.z = final_pose(2,3);
      final_pose_msg.pose.pose.orientation.w = final_quat.w();
      final_pose_msg.pose.pose.orientation.x = final_quat.x();
      final_pose_msg.pose.pose.orientation.y = final_quat.y();
      final_pose_msg.pose.pose.orientation.z = final_quat.z();
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

  int ekf_window_size = 10;
  double sensor_diff_x = 0.0;
  double sensor_diff_y = 0.0;
  double sensor_diff_z = 0.0;

  nh.getParam("sensor_diff_x", sensor_diff_x);
  nh.getParam("sensor_diff_y", sensor_diff_y);
  nh.getParam("sensor_diff_z", sensor_diff_z);
  nh.getParam("ekf_window_size", ekf_window_size);

  ros::Subscriber ndt_sub = nh.subscribe<nav_msgs::Odometry>("/ndt_pose", 1, ndtCallback);

  final_odom_pub = nh.advertise<nav_msgs::Odometry>("/final_odom", 1);

  extended_kalman_filter.correctionInit(ekf_window_size, sensor_diff_x, sensor_diff_y, sensor_diff_z);
  
  std::thread ekfProcessProcessing{ekfProcess};

  ros::spin();

  return 0;
}