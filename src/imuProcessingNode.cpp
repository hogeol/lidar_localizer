//stl
#include <string>
#include <queue>
#include <mutex>
#include <thread>

//ros
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

//eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

//local
#include "imuProcessing.hpp"

ros::Publisher filtered_imu_pub;

ImuProcessing::imuProcessing imu_processing;

std::queue<sensor_msgs::ImuConstPtr> imu_buf;

std::mutex mutex_control;

void imuHandler(const sensor_msgs::ImuConstPtr &imu_msg)
{
  mutex_control.lock();
  imu_buf.push(imu_msg);
  mutex_control.unlock();
}

void imuProcessing()
{
  while(1){
    if(!imu_buf.empty()){
      mutex_control.lock();
      Eigen::Quaterniond imu_orientation;
      ros::Time imu_in_time = imu_buf.front()->header.stamp;
      imu_orientation = Eigen::Quaterniond(-imu_buf.front()->orientation.w, -imu_buf.front()->orientation.x, -imu_buf.front()->orientation.y, -imu_buf.front()->orientation.z);
      imu_processing.mp_imu_count++;
      imu_buf.pop();
      mutex_control.unlock();
      imu_orientation.normalize();
      imu_processing.weightPrevOrientation(imu_orientation);
      if(imu_processing.mp_imu_count % imu_processing.getWindowSize() == 0){
        Eigen::Quaterniond weighted_pose = imu_processing.getWeightedOrientation();
        imu_processing.prevStatsClear();
        weighted_pose.normalize();
        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.stamp = imu_in_time;
        pose_msg.pose.orientation.w = weighted_pose.w();
        pose_msg.pose.orientation.x = weighted_pose.x();
        pose_msg.pose.orientation.y = weighted_pose.y();
        pose_msg.pose.orientation.z = weighted_pose.z();
        filtered_imu_pub.publish(pose_msg);
        if(imu_processing.mp_imu_count == 100){
          imu_processing.mp_imu_count = 0;
        }
      }
    }
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imuProcessing");
  ros::NodeHandle nh;

  int imu_window_size = 4;
  std::string imu_topic = "/vectornav/IMU";

  nh.getParam("imu_window_size", imu_window_size);
  nh.getParam("imu_topic", imu_topic);

  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic, 1, imuHandler);

  filtered_imu_pub = nh.advertise<geometry_msgs::PoseStamped>("/filtered_imu", 1);  

  imu_processing.init(imu_window_size);

  std::thread imuProcessingProcess{imuProcessing};
  
  ros::spin();

  return 0;
}

