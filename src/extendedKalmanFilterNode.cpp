//stl
#include <queue>
#include <mutex>
#include <thread>

//ros
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "extendedKalmanFilter.hpp"

std::queue<nav_msgs::msg::Odometry::ConstPtr> lidar_pose_buf;

std::mutex mutex_control;

ExtendedKalmanFilter::extendedKalmanFilter extended_kalman_filter;

void ndtCallback(const nav_msgs::msg::Odometry::ConstPtr &lidar_pose_msg)
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
  rclcpp::init(argc, argv);
  auto nh{std::make_shared<rclcpp::Node>("extended_kalman_filter_node")};

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr lidar_pose_sub;
  lidar_pose_sub = nh->create_subscription<nav_msgs::msg::Odometry>("ndt_pose", 1, ndtCallback);

  std::thread ekfProcessProcessing{ekfProcess};

  rclcpp::spin(nh);

  rclcpp::shutdown();
  
  return 0;
}