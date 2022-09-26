//stl
#include <queue>
#include <mutex>
#include <thread>

//ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

//pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

//local lib
#include "laneDetection.hpp"

//publisher
ros::Publisher point_lane_pub;

std::queue<sensor_msgs::PointCloud2ConstPtr> filtered_lidar_buf;

std::mutex mutex_control;

LaneDetection::laneDetection lane_detection;

void lidarCallback(const sensor_msgs::PointCloud2ConstPtr &filtered_lidar_msg)
{
  mutex_control.lock();
  filtered_lidar_buf.push(filtered_lidar_msg);
  mutex_control.unlock();
}

void laneDetection()
{
  while(1){
    if(!filtered_lidar_buf.empty()){
      mutex_control.lock();
      ros::Time point_in_time = filtered_lidar_buf.front()->header.stamp;
      std::string point_header = filtered_lidar_buf.front()->header.frame_id;
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_in(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_out(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*(filtered_lidar_buf.front()), *point_in);
      filtered_lidar_buf.pop();
      mutex_control.unlock();
      lane_detection.extractDesiredDimension(point_in, point_out);
      sensor_msgs::PointCloud2 point_msg;
      pcl::toROSMsg(*point_out, point_msg);
      point_msg.header.stamp = point_in_time;
      point_msg.header.frame_id = point_header;
      point_lane_pub.publish(point_msg);
    }
    std::chrono::milliseconds dura(3);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "lane_detection");
  ros::NodeHandle nh;
  
  double range_x_min = 1.0;
  double range_x_max = 1.0;
  double range_y_min = 1.0;
  double range_y_max = 1.0;
  double range_z_min = 1.0;
  double range_z_max = 1.0;

  nh.getParam("range_x_min", range_x_min);
  nh.getParam("range_x_max",range_x_max);
  nh.getParam("range_y_min", range_y_min);
  nh.getParam("range_y_max", range_y_max);
  nh.getParam("range_z_min", range_z_min);
  nh.getParam("range_z_max", range_z_max);

  ros::Subscriber lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>("filtered_point", 1, lidarCallback);

  point_lane_pub = nh.advertise<sensor_msgs::PointCloud2>("laser_lane", 1);

  lane_detection.init(range_x_min, range_x_max, range_y_min, range_y_max, range_z_min, range_z_max);

  std::thread lanDetectionProcess{laneDetection};

  ros::spin();

  return 0;
}