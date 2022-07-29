//STL
#include <string>
#include <queue>
#include <mutex>
#include <thread>
//pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

//local lib
#include "lidarProcessing.hpp"

//subscriber

LidarProcessing::lidarProcessingClass lidar_processing;
std::queue<sensor_msgs::PointCloud2ConstPtr> lidar_buf;
std::mutex mutex_lock;

//publisher
ros::Publisher filtered_lidar_pub;

void laserCallback(const sensor_msgs::PointCloud2ConstPtr &laserMsg)
{
  mutex_lock.lock();
  lidar_buf.push(laserMsg);
  mutex_lock.unlock();
}

void laserProcessing()
{
  while(1){
    if(!lidar_buf.empty()){
      mutex_lock.lock();
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_in(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_out(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*lidar_buf.front(), *point_in);
      ros::Time point_in_time = lidar_buf.front()->header.stamp;
      lidar_buf.pop();
      mutex_lock.unlock();
      lidar_processing.featureExtraction(point_in, point_out);
      sensor_msgs::PointCloud2 point_filtered_msg;
      pcl::toROSMsg(*point_out, point_filtered_msg);
      point_filtered_msg.header.frame_id = "velodyne";
      point_filtered_msg.header.stamp = point_in_time;
      filtered_lidar_pub.publish(point_filtered_msg);
    }
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "lidarProcessing");
  ros::NodeHandle nh;

  std::string laser_topic = "/velodyne_points";
  int scan_line=32;
  double max_distance=60.0;
  double min_distance=3.0;
  double vertical_angle=0.3f;
  double filtering_leaf_size=0.1;
  double robot_x_min=1.0;
  double robot_x_max=1.0;
  double robot_y_min=1.0;
  double robot_y_max=1.0;
  double robot_z_min=1.0;
  double robot_z_max=1.0;

  nh.getParam("laser_topic", laser_topic);
  nh.getParam("scan_line", scan_line);
  nh.getParam("max_distance", max_distance);
  nh.getParam("min_distance", min_distance);
  nh.getParam("vertical_angle", vertical_angle);
  nh.getParam("robot_x_min", robot_x_min);
  nh.getParam("robot_x_max",robot_x_max);
  nh.getParam("robot_y_min", robot_y_min);
  nh.getParam("robot_y_max", robot_y_max);
  nh.getParam("robot_z_min", robot_z_min);
  nh.getParam("robot_z_max", robot_z_max);
  nh.getParam("filtering_leaf_size", filtering_leaf_size);

  lidar_processing.setLidar(scan_line, max_distance, min_distance, vertical_angle, filtering_leaf_size);
  lidar_processing.setRobot(robot_x_min, robot_x_max, robot_y_min, robot_y_max, robot_z_min, robot_z_max);

  ros::Subscriber lidarSub = nh.subscribe<sensor_msgs::PointCloud2>(laser_topic, 1, laserCallback);

  filtered_lidar_pub = nh.advertise<sensor_msgs::PointCloud2>("/filtered_point", 1);

  std::thread laserProcessingProcess{laserProcessing};

  ros::spin();
  
  return 0;
}