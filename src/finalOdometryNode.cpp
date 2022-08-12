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
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
//tf
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//local lib
#include "ndtMatching.hpp"
#include "extendedKalmanFilter.hpp"

//publisher
ros::Publisher map_pub;
ros::Publisher ndt_pub;
ros::Publisher odom_pub;
ros::Publisher local_pub;

NdtMatching::ndtMatching ndt_matching;
ExtendedKalmanFilter::extendedKalmanFilter extended_kalman_filter;

std::queue<sensor_msgs::PointCloud2ConstPtr> lidar_buf;
std::queue<geometry_msgs::PoseStampedConstPtr> filtered_pose_buf;

std::mutex mutex_control;

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

void finalOdometry()
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
      Eigen::Matrix4d nav_pose = Eigen::Matrix4d::Identity();
      Eigen::Quaterniond local_orientation = Eigen::Quaterniond(filtered_pose_buf.back()->pose.orientation.w, filtered_pose_buf.back()->pose.orientation.x, filtered_pose_buf.back()->pose.orientation.y, filtered_pose_buf.back()->pose.orientation.z);
      nav_pose.block<3,3>(0,0) = local_orientation.toRotationMatrix();
      nav_pose(0,3) = filtered_pose_buf.back()->pose.position.x;
      nav_pose(1,3) = filtered_pose_buf.back()->pose.position.y;
      nav_pose(2,3) = filtered_pose_buf.back()->pose.position.z;
      if(is_init == false){
        tf::Matrix3x3 tf_rot_matrix;
        tf::matrixEigenToTF(nav_pose.block<3,3>(0,0), tf_rot_matrix);
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        tf_rot_matrix.getRPY(roll, pitch, yaw);
        ndt_matching.setInitPosition(nav_pose(0,3), nav_pose(1,3), nav_pose(2,3), yaw);
        is_init = true;
        filtered_pose_buf.pop();
        lidar_buf.pop();
        mutex_control.unlock();
      }
      else{
          filtered_pose_buf.pop();
          lidar_buf.pop();
          mutex_control.unlock();  
      }
      
      //after ndt matching
      Eigen::Matrix4f ndt_pose = Eigen::Matrix4f::Identity();
      Eigen::Matrix4f result_pose = Eigen::Matrix4f::Identity();
      ndt_matching.processNdt(point_in, point_out, nav_pose.cast<float>(), ndt_pose);
      odom_frame++;
      extended_kalman_filter.exponentialWeight(ndt_pose, result_pose);
      Eigen::Quaterniond result_orientation(result_pose.block<3,3>(0,0).cast<double>());
      result_orientation.normalize();

      sensor_msgs::PointCloud2 ndt_msg;
      pcl::toROSMsg(*point_out, ndt_msg);
      ndt_msg.header.stamp = point_in_time;
      ndt_msg.header.frame_id = "map";
      ndt_pub.publish(ndt_msg);

      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(result_pose(0,3), result_pose(1,3), 0.0));
      tf::Quaternion q_tf(result_orientation.x(), result_orientation.y(), result_orientation.z(), result_orientation.w());
      transform.setRotation(q_tf);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

      static tf::TransformBroadcaster gps_br;
      tf::Transform gps_transform;
      gps_transform.setOrigin(tf::Vector3(nav_pose(0,3), nav_pose(1,3), 0.0));
      tf::Quaternion gps_q_tf(local_orientation.x(), local_orientation.y(), local_orientation.z(), local_orientation.w());
      gps_transform.setRotation(q_tf);
      gps_br.sendTransform(tf::StampedTransform(gps_transform, ros::Time::now(), "map", "local"));

      nav_msgs::Odometry odom_msg;
      odom_msg.header.frame_id = "map";
      odom_msg.child_frame_id = "base_link";
      odom_msg.header.stamp = point_in_time;
      odom_msg.pose.pose.position.x = result_pose(0,3);
      odom_msg.pose.pose.position.y = result_pose(1,3);
      odom_msg.pose.pose.position.z = result_pose(2,3);
      odom_msg.pose.pose.orientation.w = result_orientation.w();
      odom_msg.pose.pose.orientation.x = result_orientation.x();
      odom_msg.pose.pose.orientation.y = result_orientation.y();
      odom_msg.pose.pose.orientation.z = result_orientation.z();
      odom_pub.publish(odom_msg);
            
      nav_msgs::Odometry local_msg;
      local_msg.header.frame_id = "map";
      local_msg.child_frame_id = "local";
      local_msg.header.stamp = point_in_time;
      local_msg.pose.pose.position.x = nav_pose(0,3);
      local_msg.pose.pose.position.y = nav_pose(1,3);
      local_msg.pose.pose.position.z = nav_pose(2,3);
      local_msg.pose.pose.orientation.w = local_orientation.w();
      local_msg.pose.pose.orientation.x = local_orientation.x();
      local_msg.pose.pose.orientation.y = local_orientation.y();
      local_msg.pose.pose.orientation.z = local_orientation.z();
      local_pub.publish(local_msg);

      //map publish
      if(odom_frame%30 == 0){
        sensor_msgs::PointCloud2 map_msg;
        pcl::toROSMsg(*(ndt_matching.mp_pcd_map), map_msg);
        map_msg.header.stamp = point_in_time;
        map_msg.header.frame_id = "map";
        map_pub.publish(map_msg);
        odom_frame = 0;
      }
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
  double map_rotation_theta=0.0;
  double map_translation_x=0.0;
  double map_translation_y=0.0;
  double map_translation_z=0.0;
  is_init = false;
  std::string pcd_map_path = "/home/a/ace_ws/src/velodyne_ndt/map/";
  std::string pcd_map_name = "map.pcd";

  //Extended KalmanFilter
  int ekf_window_size=100;
  double sensor_diff_x = 0.0;
  double sensor_diff_y = 0.0;
  double sensor_diff_z = 0.0;

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
  nh.getParam("submap_select", submap_select);
  nh.getParam("kdtree_search_radius", search_radius);
  nh.getParam("ndt_near_points", ndt_near_points);
  nh.getParam("ndt_max_iteration", ndt_max_iteration);
  nh.getParam("ndt_max_threads", ndt_max_threads);

  nh.getParam("ekf_window_size", ekf_window_size);
  nh.getParam("sensor_diff_x", sensor_diff_x);
  nh.getParam("sensor_diff_y", sensor_diff_y);
  nh.getParam("sensor_diff_z", sensor_diff_z);
  
  ndt_matching.setMapTransformInfo(map_rotation_theta, map_translation_x, map_translation_y, map_translation_z);
  if(is_init == true){
    ROS_INFO("\n-----User init pose-----\n");
    ndt_matching.setInitPosition(odom_init_x, odom_init_y, odom_init_z, odom_init_rotation);
  }
  ndt_matching.init(pcd_map_resolution, pcd_map_path, pcd_map_name, submap_select, search_radius, ndt_near_points, ndt_max_iteration, ndt_max_threads);
  extended_kalman_filter.init(ekf_window_size, sensor_diff_x, sensor_diff_y, sensor_diff_z);


  ros::Subscriber filtered_lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>("/filtered_point", 1, lidarHandler);
  ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("/filtered_pose", 1, poseCallback);

  map_pub = nh.advertise<sensor_msgs::PointCloud2>("/pcd_map", 1);
  ndt_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndt", 1);
  odom_pub = nh.advertise<nav_msgs::Odometry>("/final_odom", 1);
  local_pub = nh.advertise<nav_msgs::Odometry>("/local_odom", 1);

  std::thread finalOdometryProcess{finalOdometry};

  ros::spin();

  return 0;
}