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
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//local lib
#include "gicpMatching.hpp"

//subscriber
//publisher
ros::Publisher map_pub;
ros::Publisher gicp_pub;
ros::Publisher odom_pub;

GicpMatching::gicpMatching gicp_matching;
std::queue<sensor_msgs::PointCloud2ConstPtr> lidar_buf;
std::queue<nav_msgs::OdometryConstPtr> odom_buf;

std::mutex mutex_lock;

void lidarHandler(const sensor_msgs::PointCloud2ConstPtr &filtered_msg)
{
  mutex_lock.lock();
  lidar_buf.push(filtered_msg);
  mutex_lock.unlock();
}

void odomCallback(const nav_msgs::OdometryConstPtr &odom_msg)
{
  mutex_lock.lock();
  odom_buf.push(odom_msg);
  mutex_lock.unlock();
}

void gicpMatching()
{
  while(1){
    if(!lidar_buf.empty() && !odom_buf.empty()){
      //point ros to pointcloud
      mutex_lock.lock();
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_in(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_out(new pcl::PointCloud<pcl::PointXYZI>());
      
      pcl::fromROSMsg(*lidar_buf.front(), *point_in);
      ros::Time point_in_time = lidar_buf.front()->header.stamp;
      ros::Time odom_in_time = odom_buf.front()->header.stamp;
      //ROS_INFO("\n---\npoint_time: %.3f\nimu_time: %.3f", point_in_time, imu_in_time);
      odom_buf.pop();
      lidar_buf.pop();
      mutex_lock.unlock();
      
      //after ndt matching
      Eigen::Matrix4f result_pose = Eigen::Matrix4f::Identity();
      gicp_matching.processGicp(point_in, point_out, result_pose);
      Eigen::Quaternionf q(result_pose.block<3,3>(0,0));
      q.normalize();

      sensor_msgs::PointCloud2 gicp_msg;
      pcl::toROSMsg(*point_out, gicp_msg);
      gicp_msg.header.stamp = point_in_time;
      gicp_msg.header.frame_id = "map";
      gicp_pub.publish(gicp_msg);
      
      geometry_msgs::Pose pose_msg;
      pose_msg.position.x = result_pose(0,3);
      pose_msg.position.y = result_pose(1,3);
      pose_msg.position.z = result_pose(2,3);
      pose_msg.orientation.w = q.w();
      pose_msg.orientation.x = q.x();
      pose_msg.orientation.y = q.y();
      pose_msg.orientation.z = q.z();
      nav_msgs::Odometry odom_msg;
      odom_msg.header.frame_id = "map";
      odom_msg.child_frame_id = "base_link";
      odom_msg.header.stamp = point_in_time;
      odom_msg.pose.pose = pose_msg;
      
      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(result_pose(0,3), result_pose(1,3), result_pose(2,3)));
      tf::Quaternion q_tf(q.x(), q.y(), q.z(), q.w());
      transform.setRotation(q_tf);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
      odom_pub.publish(odom_msg);
      //map publish
      sensor_msgs::PointCloud2 map_msg;
      pcl::toROSMsg(*(gicp_matching.mp_pcd_map), map_msg);
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
  ros::init(argc, argv, "gicpMatching");
  ros::NodeHandle nh;
  
  double map_resolution = 2.0;
  double search_radius = 150.0;
  int gicp_near_points = 500000;
  int gicp_max_iteration = 50;
  bool submap_select=1;
  double init_x = 0.0;
  double init_y = 0.0;
  double init_z = 0.0;
  double init_rotation=0.96;

  double rotation_theta=0.0;
  double translation_x=0.0;
  double translation_y=0.0;
  double translation_z=0.0;

  std::string map_path = "/home/a/ace_ws/src/velodyne_ndt/map/";
  std::string map_name = "map.pcd";

  nh.getParam("map_resolution", map_resolution);
  nh.getParam("map_path", map_path);
  nh.getParam("map_name", map_name);
  nh.getParam("submap_select", submap_select);
  nh.getParam("kdtree_search_radius", search_radius);
  nh.getParam("gicp_near_points", gicp_near_points);
  nh.getParam("gicp_max_iteration", gicp_max_iteration);
  nh.getParam("rotation_theta", rotation_theta);
  nh.getParam("translation_x", translation_x);
  nh.getParam("translation_y", translation_y);
  nh.getParam("translation_z", translation_z);
  nh.getParam("init_x", init_x);
  nh.getParam("init_y", init_y);
  nh.getParam("init_z", init_z);
  nh.getParam("init_rotation", init_rotation);
  
  gicp_matching.setInitPosition(init_x, init_y, init_z, init_rotation);
  gicp_matching.setMapTransformInfo(rotation_theta, translation_x, translation_y, translation_z);
  gicp_matching.init(map_resolution, map_path, map_name, submap_select, search_radius, gicp_near_points, gicp_max_iteration);

  ros::Subscriber filtered_lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>("/filtered_point", 1, lidarHandler);
  ros::Subscriber gps_odom_sub = nh.subscribe<nav_msgs::Odometry>("/gps_odom", 1, odomCallback);
  
  map_pub = nh.advertise<sensor_msgs::PointCloud2>("/pcd_map", 1);
  gicp_pub = nh.advertise<sensor_msgs::PointCloud2>("/gicp", 1);
  odom_pub = nh.advertise<nav_msgs::Odometry>("/gicp_odom", 1);

  std::thread ndtMatchingProcess{gicpMatching};

  ros::spin();

  return 0;
}