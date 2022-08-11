//stl
#include <string>
#include <queue>
#include <mutex>
#include <thread>

//ros
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>

//eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

//local
#include "localPoseProcessing.hpp"

ros::Publisher filtered_pose_pub;

ImuProcessing::imuProcessing imu_processing;

std::queue<sensor_msgs::ImuConstPtr> imu_buf;
std::queue<geometry_msgs::PoseStampedConstPtr> utm_buf;

std::mutex mutex_control;

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
void imuProcessing()
{
  while(1){
    if(!imu_buf.empty()){
      mutex_control.lock();
      ros::Time imu_in_time = imu_buf.front()->header.stamp;
      Eigen::Quaterniond pres_orientation(imu_buf.front()->orientation.w, imu_buf.front()->orientation.x, imu_buf.front()->orientation.y, imu_buf.front()->orientation.z);
      Eigen::Vector3d pres_position;
      imu_buf.pop();
      if(!utm_buf.empty()){
        pres_position = Eigen::Vector3d(utm_buf.front()->pose.position.x, utm_buf.front()->pose.position.y, utm_buf.front()->pose.position.z);
        utm_buf.pop();
      }
      mutex_control.unlock();
      imu_processing.weightPrevOrientation(pres_orientation);
      imu_frame++;
      if(imu_frame % imu_processing.getImuWindowSize() == 0){
        geometry_msgs::PoseStamped local_pose;
        local_pose.header.stamp = imu_in_time;
        local_pose.pose.position.x = pres_position(0);
        local_pose.pose.position.y = pres_position(1);
        local_pose.pose.position.z = pres_position(2);
        local_pose.pose.orientation.w = pres_orientation.w();
        local_pose.pose.orientation.x = pres_orientation.x();
        local_pose.pose.orientation.y = pres_orientation.y();
        local_pose.pose.orientation.z = pres_orientation.z();
        filtered_pose_pub.publish(local_pose);
        //imu_processing.positionClear();
        imu_frame = 0;
      }
    }
    std::chrono::milliseconds dura(3);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "imuProcessing");
  ros::NodeHandle nh;

  int imu_window_size = 4;
  int utm_window_size = 2;
  std::string imu_topic = "/vectornav/IMU";

  nh.getParam("imu_window_size", imu_window_size);
  nh.getParam("imu_topic", imu_topic);
  nh.getParam("utm_window_size", utm_window_size);

  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic, 1, imuHandler);
  ros::Subscriber utm_sub = nh.subscribe<geometry_msgs::PoseStamped>("/utm", 1, utmCallback);

  filtered_pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/filtered_pose", 1);  

  imu_processing.init(imu_window_size, utm_window_size);

  std::thread imuProcessingProcess{imuProcessing};
  
  ros::spin();

  return 0;
}

