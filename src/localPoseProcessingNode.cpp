//stl
#include <string>
#include <queue>
#include <mutex>
#include <thread>
#include <cmath>

//ros
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

//eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

//tf
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

//local
#include "localPoseProcessing.hpp"

ros::Publisher filtered_pose_pub;

std::queue<sensor_msgs::ImuConstPtr> imu_buf;
std::queue<geometry_msgs::PoseStampedConstPtr> utm_buf;

std::mutex mutex_control;

LocalPoseProcessing::localPoseProcessing local_pose_processing;

void imuHandler(const sensor_msgs::ImuConstPtr &imu_msg)
{
  mutex_control.lock();
  imu_buf.push(imu_msg);
  if(imu_buf.size()>2){
    imu_buf.pop();
  }
  mutex_control.unlock();
}

void utmCallback(const geometry_msgs::PoseStampedConstPtr &utm_msg)
{
  mutex_control.lock();
  utm_buf.push(utm_msg);
  if(utm_buf.size()>1){
    utm_buf.pop();
  }
  mutex_control.unlock();
}

int imu_frame = 0;
int gps_frame = 0;
void localProcessing()
{
  while(1){
    if(!imu_buf.empty()){
      mutex_control.lock();
      ros::Time imu_in_time = imu_buf.front()->header.stamp;
      Eigen::Quaterniond pres_orientation{imu_buf.front()->orientation.w, imu_buf.front()->orientation.x, imu_buf.front()->orientation.y, imu_buf.front()->orientation.z};      
      Eigen::Matrix4d imu_heading{Eigen::Matrix4d::Identity()};
      imu_heading.block<3,3>(0,0) = pres_orientation.toRotationMatrix();
      Eigen::Matrix4d imu_filtering{Eigen::Matrix4d::Identity()};
      imu_filtering << std::cos(1.5708), -std::sin(1.5708), 0.0, 0.0,
                          std::sin(1.5708),  std::cos(1.5708), 0.0, 0.0,
                          0.0              ,  0.0              , 1.0, 0.0,
                          0.0              ,  0.0              , 0.0, 1.0;
      imu_heading = imu_filtering * imu_heading;                           
      pres_orientation = imu_heading.block<3,3>(0,0);

      Eigen::Vector3d pres_position;
      imu_buf.pop();
      if(!utm_buf.empty()){
        pres_position = Eigen::Vector3d(utm_buf.front()->pose.position.x, utm_buf.front()->pose.position.y, utm_buf.front()->pose.position.z);
        local_pose_processing.weightPrevPosition(pres_position);
        utm_buf.pop();
      }
      mutex_control.unlock();
      local_pose_processing.weightPrevOrientation(pres_orientation);
      imu_frame++;
      pres_orientation.normalize();
      //printf("\n--IMU\nw: %.3f\nx: %.3f\ny: %.3f\nz: %.3f\n---\n", pres_orientation.w(), pres_orientation.x(), pres_orientation.y(), pres_orientation.z());

      if(imu_frame % local_pose_processing.getImuWindowSize() == 0){
        geometry_msgs::PoseStamped local_pose_msg;
        local_pose_msg.header.stamp = imu_in_time;
        local_pose_msg.header.frame_id = "map";
        local_pose_msg.pose.position.x = pres_position.x();
        local_pose_msg.pose.position.y = pres_position.y();
        local_pose_msg.pose.position.z = pres_position.z();
        local_pose_msg.pose.orientation.w = pres_orientation.w();
        local_pose_msg.pose.orientation.x = pres_orientation.x();
        local_pose_msg.pose.orientation.y = pres_orientation.y();
        local_pose_msg.pose.orientation.z = pres_orientation.z();
        filtered_pose_pub.publish(local_pose_msg);
        imu_frame = 0;
      }
    }
    std::chrono::milliseconds dura(3);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "localProcessing");
  ros::NodeHandle nh;

  std::string imu_topic = "/vectornav/IMU";
  int imu_window_size = 4;
  int utm_window_size = 2;

  nh.getParam("imu_topic", imu_topic);
  nh.getParam("imu_window_size", imu_window_size);
  nh.getParam("utm_window_size", utm_window_size);

  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic, 1, imuHandler);
  ros::Subscriber utm_sub = nh.subscribe<geometry_msgs::PoseStamped>("/utm", 1, utmCallback);

  filtered_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/filtered_pose", 1);  

  local_pose_processing.init(imu_window_size, utm_window_size);

  std::thread localProcessingProcess{localProcessing};
  
  ros::spin();

  return 0;
}

