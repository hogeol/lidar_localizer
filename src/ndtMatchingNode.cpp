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
#include <opencv2/core/types.hpp>
#include "ndtMatching.hpp"

//subscriber
//publisher
ros::Publisher map_pub;
ros::Publisher ndt_pub;

NdtMatching::ndtMatching ndt_matching;
std::queue<sensor_msgs::PointCloud2ConstPtr> lidar_buf;

std::mutex mutex_lock;

cv::Point3d virtual_point3d(0.0, 0.0, 0.0);

void lidarHandler(const sensor_msgs::PointCloud2ConstPtr &filtered_msg)
{
  mutex_lock.lock();
  lidar_buf.push(filtered_msg);
  mutex_lock.unlock();
}

void ndtMatching()
{
  while(1){
    if(!lidar_buf.empty()){
      //point ros to pointcloud
      mutex_lock.lock();
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_in(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_out(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::fromROSMsg(*lidar_buf.front(), *point_in);
      ros::Time point_in_time = lidar_buf.front()->header.stamp;
      //ROS_INFO("\n---\npoint_time: %.3f\nimu_time: %.3f", point_in_time, imu_in_time);
      lidar_buf.pop();
      mutex_lock.unlock();
      
      //after ndt matching
      ndt_matching.processNdt(point_in, point_out, virtual_point3d);
      sensor_msgs::PointCloud2 ndt_msg;
      pcl::toROSMsg(*point_out, ndt_msg);
      ndt_msg.header.stamp = point_in_time;
      ndt_msg.header.frame_id = "velodyne";
      ndt_pub.publish(ndt_msg);
      
      //map publish
      sensor_msgs::PointCloud2 map_msg;
      pcl::toROSMsg(*(ndt_matching.mp_pcd_map), map_msg);
      map_msg.header.stamp = point_in_time;
      map_msg.header.frame_id = "map";
      map_pub.publish(map_msg);
    }
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ndtMatching");
  ros::NodeHandle nh;
  
  double map_resolution = 0.1;
  double downsampling_leaf_size = 0.1;
  double search_radius = 150.0;
  int near_points = 500000;
  int ndt_max_iteration = 50;
  bool submap_select=1;

  double rotation_theta=0.0;
  double translation_x=0.0;
  double translation_y=0.0;
  double translation_z=0.0;
  
  std::string map_path = "/home/a/ace_ws/src/velodyne_ndt/map/";
  std::string map_name = "map.pcd";

  nh.getParam("map_resolution", map_resolution);
  nh.getParam("leaf_size", downsampling_leaf_size);
  nh.getParam("map_path", map_path);
  nh.getParam("map_name", map_name);
  nh.getParam("submap_select", submap_select);
  nh.getParam("kdtree_search_radius", search_radius);
  nh.getParam("ndt_near_points", near_points);
  nh.getParam("ndt_max_iteration", ndt_max_iteration);
  nh.getParam("rotation_theta", rotation_theta);
  nh.getParam("translation_x", translation_x);
  nh.getParam("translation_y", translation_y);
  nh.getParam("translation_z", translation_z);
  
  ndt_matching.setTransformInfo(rotation_theta, translation_x, translation_y, translation_z);
  ndt_matching.init(map_resolution, map_path, map_name, downsampling_leaf_size, submap_select, search_radius, near_points, ndt_max_iteration);

  ros::Subscriber filtered_lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>("/filtered_point", 100, lidarHandler);

  map_pub = nh.advertise<sensor_msgs::PointCloud2>("/pcd_map", 100);
  ndt_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndt", 100);
  std::thread ndtMatchingProcess{ndtMatching};

  ros::spin();

  return 0;
}