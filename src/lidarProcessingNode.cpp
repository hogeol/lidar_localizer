//stl
#include <string>
#include <queue>
#include <mutex>
#include <thread>
//pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//ros
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

//local lib
#include "lidarProcessing.hpp"

//publisher
rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_lidar_pub;

std::queue<sensor_msgs::msg::PointCloud2::ConstPtr> lidar_buf;

std::mutex mutex_control;

LidarProcessing::lidarProcessingClass lidar_processing;

void laserCallback(const sensor_msgs::msg::PointCloud2::ConstPtr &laser_msg)
{
  mutex_control.lock();
  lidar_buf.push(laser_msg);
  mutex_control.unlock();
}

void laserProcessing()
{
  while(1){
    if(!lidar_buf.empty()){
      mutex_control.lock();
      rclcpp::Time point_in_time{lidar_buf.front()->header.stamp};
      std::string point_header = lidar_buf.front()->header.frame_id;
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_in(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_out(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*lidar_buf.front(), *point_in);
      lidar_buf.pop();
      mutex_control.unlock();
      lidar_processing.featureExtraction(point_in, point_out);
      sensor_msgs::msg::PointCloud2 point_filtered_msg;
      pcl::toROSMsg(*point_out, point_filtered_msg);
      point_filtered_msg.header.frame_id = point_header;
      point_filtered_msg.header.stamp = point_in_time;
      filtered_lidar_pub -> publish(point_filtered_msg);
    }
    std::chrono::milliseconds dura(3);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto nh{std::make_shared<rclcpp::Node>("lidar_processing_node")};

  std::string laser_topic = "/velodyne_points";
  int scan_line=32;
  double max_distance=60.0;
  double min_distance=3.0;
  double vertical_angle=0.3f;
  double filtering_leaf_size=0.1;
  
  nh->declare_parameter("laser_topic", laser_topic);
  nh->declare_parameter("scan_line", scan_line);
  nh->declare_parameter("max_distance", max_distance);
  nh->declare_parameter("min_distance", min_distance);
  nh->declare_parameter("vertical_angle", vertical_angle);
  nh->declare_parameter("filtering_leaf_size", filtering_leaf_size);

  nh->get_parameter("laser_topic", laser_topic);
  nh->get_parameter("scan_line", scan_line);
  nh->get_parameter("max_distance", max_distance);
  nh->get_parameter("min_distance", min_distance);
  nh->get_parameter("vertical_angle", vertical_angle);
  nh->get_parameter("filtering_leaf_size", filtering_leaf_size);

  lidar_processing.setLidar(scan_line, max_distance, min_distance, vertical_angle, filtering_leaf_size);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub{nh -> create_subscription<sensor_msgs::msg::PointCloud2>(laser_topic, 1, laserCallback)};

  filtered_lidar_pub = nh -> create_publisher<sensor_msgs::msg::PointCloud2>("/filtered_point", 1);

  std::thread laserProcessingProcess{laserProcessing};

  rclcpp::spin(nh);
  
  rclcpp::shutdown();

  return 0;
}