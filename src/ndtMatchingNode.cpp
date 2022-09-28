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
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
//tf
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

//local lib
#include "ndtMatching.hpp"

//publisher
ros::Publisher map_pub;
ros::Publisher ndt_pc_pub;
ros::Publisher ndt_pose_pub;

std::queue<sensor_msgs::PointCloud2ConstPtr> lidar_buf;
std::queue<geometry_msgs::PoseStampedConstPtr> filtered_pose_buf;

std::mutex mutex_control;

NdtMatching::ndtMatching ndt_matching;

bool is_init;
int odom_frame = 0;

void lidarHandler(const sensor_msgs::PointCloud2ConstPtr &filtered_msg)
{
  mutex_control.lock();
  lidar_buf.push(filtered_msg);
  //printf("\nlidar: %d\n", lidar_buf.size());
  mutex_control.unlock();
}

void poseCallback(const geometry_msgs::PoseStampedConstPtr &pose_msg)
{
  mutex_control.lock();
  filtered_pose_buf.push(pose_msg);
  //printf("\nimu: %d\n", filtered_pose_buf.size());
  mutex_control.unlock();
}

int init_cnt = 0;
void ndtMatching()
{
  while(1){
    if(!lidar_buf.empty() && !filtered_pose_buf.empty()){
      //point ros to pointcloud
      mutex_control.lock();
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_in(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_out(new pcl::PointCloud<pcl::PointXYZI>());
      //lidar ros to pointcloud
      pcl::fromROSMsg(*lidar_buf.back(), *point_in);
      ros::Time point_in_time = lidar_buf.back()->header.stamp;
      Eigen::Isometry3d nav_pose = Eigen::Isometry3d::Identity();
      Eigen::Quaterniond local_orientation = Eigen::Quaterniond(filtered_pose_buf.back()->pose.orientation.w, filtered_pose_buf.back()->pose.orientation.x, filtered_pose_buf.back()->pose.orientation.y, filtered_pose_buf.back()->pose.orientation.z); 
      
      nav_pose.translation().x() = filtered_pose_buf.back()->pose.position.x;
      nav_pose.translation().y() = filtered_pose_buf.back()->pose.position.y;
      nav_pose.translation().z() = filtered_pose_buf.back()->pose.position.z;
      nav_pose.linear() = local_orientation.toRotationMatrix();
      
      if(is_init == false || init_cnt < 30){
        Eigen::Vector3d eigen_rpy = nav_pose.rotation().eulerAngles(0, 1, 2);
        ndt_matching.setInitPosition(nav_pose.translation().x(), nav_pose.translation().y(), nav_pose.translation().z(), eigen_rpy.z());
        is_init = true;
        init_cnt++;
        filtered_pose_buf.pop();
        lidar_buf.pop();
        mutex_control.unlock();
        continue;
      }
      else{
          filtered_pose_buf.pop();
          lidar_buf.pop();
          mutex_control.unlock();  
      }
      
      //pose after ndt
      Eigen::Isometry3d ndt_result_pose = Eigen::Isometry3d::Identity();
      ndt_matching.processNdt(point_in, point_out, nav_pose, ndt_result_pose);

      Eigen::Vector3d ndt_position = ndt_result_pose.translation();
      Eigen::Quaterniond ndt_orientation(ndt_result_pose.rotation());
      ndt_orientation.normalize();

      //pointcloud after ndt
      sensor_msgs::PointCloud2 ndt_pc_msg;
      pcl::toROSMsg(*point_out, ndt_pc_msg);
      ndt_pc_msg.header.stamp = point_in_time;
      ndt_pc_msg.header.frame_id = "map";
      ndt_pc_pub.publish(ndt_pc_msg);

      //ndt tf
      static tf2_ros::TransformBroadcaster ndt_broadcaster;
      geometry_msgs::TransformStamped ndt_transform_stamped;
      ndt_transform_stamped.header.stamp = ros::Time::now();
      ndt_transform_stamped.header.frame_id = "map";
      ndt_transform_stamped.child_frame_id = "base_link";
      ndt_transform_stamped.transform.translation.x = ndt_position.x();
      ndt_transform_stamped.transform.translation.y = ndt_position.y();
      ndt_transform_stamped.transform.translation.z = ndt_position.z();
      ndt_transform_stamped.transform.rotation.w = ndt_orientation.w();
      ndt_transform_stamped.transform.rotation.x = ndt_orientation.x();
      ndt_transform_stamped.transform.rotation.y = ndt_orientation.y();
      ndt_transform_stamped.transform.rotation.z = ndt_orientation.z();
      ndt_broadcaster.sendTransform(ndt_transform_stamped);

      //utm tf
      static tf2_ros::TransformBroadcaster utm_br;
      geometry_msgs::TransformStamped utm_transform_stamped;
      utm_transform_stamped.header.stamp = ros::Time::now();
      utm_transform_stamped.header.frame_id = "map";
      utm_transform_stamped.child_frame_id = "utm";
      utm_transform_stamped.transform.translation.x = nav_pose.translation().x();
      utm_transform_stamped.transform.translation.y = nav_pose.translation().y();
      utm_transform_stamped.transform.translation.z = nav_pose.translation().z();
      utm_transform_stamped.transform.rotation.w = local_orientation.w();
      utm_transform_stamped.transform.rotation.x = local_orientation.x();
      utm_transform_stamped.transform.rotation.y = local_orientation.y();
      utm_transform_stamped.transform.rotation.z = local_orientation.z();
      utm_br.sendTransform(utm_transform_stamped);

      //ndt odometry
      nav_msgs::Odometry ndt_pose_msg;
      ndt_pose_msg.header.stamp = point_in_time;
      ndt_pose_msg.header.frame_id = "map";
      ndt_pose_msg.child_frame_id = "base_link";
      ndt_pose_msg.pose.pose.position.x = ndt_position.x();
      ndt_pose_msg.pose.pose.position.y = ndt_position.y();
      ndt_pose_msg.pose.pose.position.z = ndt_position.z();
      ndt_pose_msg.pose.pose.orientation.w = ndt_orientation.w();
      ndt_pose_msg.pose.pose.orientation.x = ndt_orientation.x();
      ndt_pose_msg.pose.pose.orientation.y = ndt_orientation.y();
      ndt_pose_msg.pose.pose.orientation.z = ndt_orientation.z();
      ndt_pose_pub.publish(ndt_pose_msg);

      //map publish
      if(odom_frame%30 == 0){
        sensor_msgs::PointCloud2 map_msg;
        pcl::toROSMsg(*(ndt_matching.mp_pcd_map), map_msg);
        map_msg.header.stamp = point_in_time;
        map_msg.header.frame_id = "map";
        map_pub.publish(map_msg);
        odom_frame = 0;
      }
      odom_frame++;
      double diff = sqrt(pow(ndt_result_pose.translation().x() - nav_pose.translation().x(), 2) + pow(ndt_result_pose.translation().y() - nav_pose.translation().y(), 2));
      printf("\n---\ndiff: %.4f\n---\n", diff);
    }
    std::chrono::milliseconds dura(3);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ndtMatching");
  ros::NodeHandle nh;
  
  //NDT
  double pcd_map_resolution = 2.0;
  double search_radius = 150.0;
  int ndt_near_points = 500000;
  int ndt_max_iteration = 50;
  int ndt_max_threads = 10;
  bool submap_select=1;
  double odom_init_x = 0.0;
  double odom_init_y = 0.0;
  double odom_init_z = 0.0;
  double odom_init_rotation=0.0;
  double sensor_diff_x = 0.0;
  double sensor_diff_y = 0.0;
  double sensor_diff_z = 0.0;
  double map_rotation_theta=0.0;
  double map_translation_x=0.0;
  double map_translation_y=0.0;
  double map_translation_z=0.0;
  is_init = false;
  std::string pcd_map_path = "/home/a/ace_ws/src/velodyne_ndt/map/";
  std::string pcd_map_name = "map.pcd";
  std::string ndt_search_method = "pclomp::DIRECT7";

  nh.getParam("pcd_map_resolution", pcd_map_resolution);
  nh.getParam("pcd_map_path", pcd_map_path);
  nh.getParam("pcd_map_name", pcd_map_name);
  nh.getParam("map_translation_x", map_translation_x);
  nh.getParam("map_translation_y", map_translation_y);
  nh.getParam("map_translation_z", map_translation_z);
  nh.getParam("map_rotation_theta", map_rotation_theta);
  nh.getParam("user_init_pose", is_init);
  nh.getParam("odom_init_x", odom_init_x);
  nh.getParam("odom_init_y", odom_init_y);
  nh.getParam("odom_init_z", odom_init_z);
  nh.getParam("odom_init_rotation", odom_init_rotation);
  nh.getParam("sensor_diff_x", sensor_diff_x);
  nh.getParam("sensor_diff_y", sensor_diff_y);
  nh.getParam("sensor_diff_z", sensor_diff_z);
  nh.getParam("submap_select", submap_select);
  nh.getParam("kdtree_search_radius", search_radius);
  nh.getParam("ndt_near_points", ndt_near_points);
  nh.getParam("ndt_max_iteration", ndt_max_iteration);
  nh.getParam("ndt_max_threads", ndt_max_threads);
  nh.getParam("ndt_search_method", ndt_search_method);
  
  ndt_matching.setMapTransformInfo(map_rotation_theta, map_translation_x, map_translation_y, map_translation_z);
  if(is_init == true){
    ROS_INFO("\n-----User init pose-----\n");
    ndt_matching.setInitPosition(odom_init_x, odom_init_y, odom_init_z, odom_init_rotation);
  }
  ndt_matching.init(pcd_map_resolution, pcd_map_path, pcd_map_name, submap_select, search_radius, ndt_near_points, ndt_max_iteration, ndt_max_threads, ndt_search_method);
  ndt_matching.setGpsLidarTF(sensor_diff_x, sensor_diff_y, sensor_diff_z);

  ros::Subscriber filtered_lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>("/filtered_point", 1, lidarHandler);
  ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/filtered_pose", 1, poseCallback);

  map_pub = nh.advertise<sensor_msgs::PointCloud2>("/pcd_map", 1);
  ndt_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndt", 1);
  ndt_pose_pub = nh.advertise<nav_msgs::Odometry>("/ndt_pose", 1);

  std::thread ndtMatchingProcess{ndtMatching};

  ros::spin();

  return 0;
}