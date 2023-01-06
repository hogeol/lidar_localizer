//stl
#include <string>
#include <queue>
#include <mutex>
#include <thread>

//ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

//eigen
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

//tf
#include <tf2_ros/transform_broadcaster.h>

//local
#include "localPoseProcessing.hpp"

rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr filtered_pose_pub;

std::queue<sensor_msgs::msg::Imu::ConstPtr> imu_buf;
std::queue<geometry_msgs::msg::PoseStamped::ConstPtr> utm_buf;

std::mutex mutex_control;

LocalPoseProcessing::localPoseProcessing local_pose_processing;

void imuHandler(const sensor_msgs::msg::Imu::ConstPtr &imu_msg)
{
  mutex_control.lock();
  imu_buf.push(imu_msg);
  if(imu_buf.size()>2){
    imu_buf.pop();
  }
  mutex_control.unlock();
}

void utmCallback(const geometry_msgs::msg::PoseStamped::ConstPtr &utm_msg)
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
      rclcpp::Time imu_in_time{imu_buf.front()->header.stamp};
      Eigen::Quaterniond pres_orientation(imu_buf.front()->orientation.w, imu_buf.front()->orientation.x, imu_buf.front()->orientation.y, -imu_buf.front()->orientation.z);
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

      if(imu_frame % local_pose_processing.getImuWindowSize() == 0){
        geometry_msgs::msg::PoseStamped local_pose_msg;
        local_pose_msg.header.stamp = imu_in_time;
        local_pose_msg.header.frame_id = "map";
        local_pose_msg.pose.position.x = pres_position.x();
        local_pose_msg.pose.position.y = pres_position.y();
        local_pose_msg.pose.position.z = pres_position.z();
        local_pose_msg.pose.orientation.w = pres_orientation.w();
        local_pose_msg.pose.orientation.x = pres_orientation.x();
        local_pose_msg.pose.orientation.y = pres_orientation.y();
        local_pose_msg.pose.orientation.z = pres_orientation.z();
        filtered_pose_pub -> publish(local_pose_msg);
        imu_frame = 0;
      }
    }
    std::chrono::milliseconds dura(3);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh(std::make_shared<rclcpp::Node>("local_pose_processing_node"));

  nh -> declare_parameter("imu_topic", "/vectornav/IMU");
  nh -> declare_parameter("imu_window_size", 4);
  nh -> declare_parameter("utm_window_size", 2);

  std::string imu_topic(nh->get_parameter("imu_topic").as_string());
  int imu_window_size(nh->get_parameter("imu_window_size").as_int());
  int utm_window_size(nh->get_parameter("utm_window_size").as_int());

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub{nh -> create_subscription<sensor_msgs::msg::Imu>(imu_topic, 1, imuHandler)};
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr utm_sub{nh -> create_subscription<geometry_msgs::msg::PoseStamped>("/utm", 1, utmCallback)};

  filtered_pose_pub = nh -> create_publisher<geometry_msgs::msg::PoseStamped>("/filtered_pose", 1);

  local_pose_processing.init(imu_window_size, utm_window_size);

  std::thread localProcessingProcess{localProcessing};
  
  rclcpp::spin(nh);

  rclcpp::shutdown();

  return 0;
}

