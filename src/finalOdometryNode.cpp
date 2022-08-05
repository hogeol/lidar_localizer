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
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
//tf
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <std_srvs/Trigger.h>

//local lib
#include "ndtMatching.hpp"
#include "extendedKalmanFilter.hpp"

//publisher
ros::Publisher map_pub;
ros::Publisher ndt_pub;
ros::Publisher odom_pub;

NdtMatching::ndtMatching ndt_matching;
ExtendedKalmanFilter::extendedKalmanFilter extended_kalman_filter;

std::queue<sensor_msgs::PointCloud2ConstPtr> lidar_buf;
std::queue<sensor_msgs::ImuConstPtr> imu_buf;
std::queue<geometry_msgs::PoseStampedConstPtr> utm_buf;

std::mutex mutex_lock;

void imuHandler(const sensor_msgs::ImuConstPtr &imu_msg)
{
  mutex_lock.lock();
  imu_buf.push(imu_msg);
  //printf("\nimu: %d", imu_buf.size());
  mutex_lock.unlock();
}

void lidarHandler(const sensor_msgs::PointCloud2ConstPtr &filtered_msg)
{
  mutex_lock.lock();
  lidar_buf.push(filtered_msg);
  //printf("\nlidar: %d", lidar_buf.size());
  mutex_lock.unlock();
}

void utmCallback(const geometry_msgs::PoseStampedConstPtr &utm_msg)
{
  mutex_lock.lock();
  utm_buf.push(utm_msg);
  //printf("\nutm: %d", utm_buf.size());
  mutex_lock.unlock();
}

bool is_init;
int odom_frame = 0;
int imu_frame = 0;

void finalOdometry()
{
  while(1){
    if(!lidar_buf.empty() && !utm_buf.empty() && !imu_buf.empty()){
      //point ros to pointcloud
      mutex_lock.lock();
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_in(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_out(new pcl::PointCloud<pcl::PointXYZI>());
      //lidar ros to pointcloud
      pcl::fromROSMsg(*lidar_buf.front(), *point_in);
      ros::Time point_in_time = lidar_buf.front()->header.stamp;
      Eigen::Matrix4d gps_in_pose = Eigen::Matrix4d::Identity();
      if(is_init == false){
        gps_in_pose.block<3,3>(0,0) = Eigen::Quaterniond(imu_buf.front()->orientation.w, imu_buf.front()->orientation.x, imu_buf.front()->orientation.y, imu_buf.front()->orientation.z).toRotationMatrix();
        gps_in_pose(0,3) = utm_buf.front()->pose.position.x;
        gps_in_pose(1,3) = utm_buf.front()->pose.position.y;
        gps_in_pose(2,3) = utm_buf.front()->pose.position.z;
        tf::Matrix3x3 tf_rot_matrix;
        tf::matrixEigenToTF(gps_in_pose.block<3,3>(0,0), tf_rot_matrix);
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        tf_rot_matrix.getRPY(roll, pitch, yaw);
        ndt_matching.setInitPosition(gps_in_pose(0,3), gps_in_pose(1,3), gps_in_pose(2,3), yaw);
        is_init = true;
        imu_buf.pop();
        utm_buf.pop();
        lidar_buf.pop();
        mutex_lock.unlock();  
      }
      else{
        gps_in_pose.block<3,3>(0,0) = Eigen::Quaterniond(imu_buf.back()->orientation.w, imu_buf.back()->orientation.x, imu_buf.back()->orientation.y, imu_buf.back()->orientation.z).toRotationMatrix();
        gps_in_pose(0,3) = utm_buf.back()->pose.position.x;
        gps_in_pose(1,3) = utm_buf.back()->pose.position.y;
        gps_in_pose(2,3) = utm_buf.back()->pose.position.z;
        imu_buf.pop();
        utm_buf.pop();
        lidar_buf.pop();
        mutex_lock.unlock();  
      }
      
      //after ndt matching
      Eigen::Matrix4f result_pose = Eigen::Matrix4f::Identity();
      ndt_matching.processNdt(point_in, point_out, gps_in_pose.cast<float>(), result_pose);
      odom_frame++;
      Eigen::Quaternionf q(result_pose.block<3,3>(0,0));
      q.normalize();

      sensor_msgs::PointCloud2 ndt_msg;
      pcl::toROSMsg(*point_out, ndt_msg);
      ndt_msg.header.stamp = point_in_time;
      ndt_msg.header.frame_id = "map";
      ndt_pub.publish(ndt_msg);
      
      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(result_pose(0,3), result_pose(1,3), result_pose(2,3)));
      tf::Quaternion q_tf(q.x(), q.y(), q.z(), q.w());
      transform.setRotation(q_tf);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));

      nav_msgs::Odometry odom_msg;
      odom_msg.header.frame_id = "map";
      odom_msg.child_frame_id = "base_link";
      odom_msg.header.stamp = point_in_time;
      odom_msg.pose.pose.position.x = result_pose(0,3);
      odom_msg.pose.pose.position.y = result_pose(1,3);
      odom_msg.pose.pose.position.z = result_pose(2,3);
      odom_msg.pose.pose.orientation.w = q.w();
      odom_msg.pose.pose.orientation.x = q.x();
      odom_msg.pose.pose.orientation.y = q.y();
      odom_msg.pose.pose.orientation.z = q.z();
      odom_pub.publish(odom_msg);

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
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ndtMatching");
  ros::NodeHandle nh;
  
  double pcd_map_resolution = 2.0;
  double search_radius = 150.0;
  int ndt_near_points = 500000;
  int ndt_max_iteration = 50;
  int ndt_max_threads = 10;
  bool submap_select=1;
  double odom_init_x = 0.0;
  double odom_init_y = 0.0;
  double odom_init_z = 0.0;
  double odom_init_rotation=0.96;

  double map_rotation_theta=0.0;
  double map_translation_x=0.0;
  double map_translation_y=0.0;
  double map_translation_z=0.0;
  is_init = false;

  std::string pcd_map_path = "/home/a/ace_ws/src/velodyne_ndt/map/";
  std::string pcd_map_name = "map.pcd";
  std::string imu_topic = "/vectornav/IMU";

  nh.getParam("pcd_map_resolution", pcd_map_resolution);
  nh.getParam("pcd_map_path", pcd_map_path);
  nh.getParam("pcd_map_name", pcd_map_name);
  nh.getParam("map_translation_x", map_translation_x);
  nh.getParam("map_translation_y", map_translation_y);
  nh.getParam("map_translation_z", map_translation_z);
  nh.getParam("map_rotation_theta", map_rotation_theta);
  nh.getParam("gps_init_pose", is_init);
  nh.getParam("odom_init_x", odom_init_x);
  nh.getParam("odom_init_y", odom_init_y);
  nh.getParam("odom_init_z", odom_init_z);
  nh.getParam("odom_init_rotation", odom_init_rotation);
  nh.getParam("submap_select", submap_select);
  nh.getParam("kdtree_search_radius", search_radius);
  nh.getParam("ndt_near_points", ndt_near_points);
  nh.getParam("ndt_max_iteration", ndt_max_iteration);
  nh.getParam("ndt_max_threads", ndt_max_threads);
  
  ndt_matching.setMapTransformInfo(map_rotation_theta, map_translation_x, map_translation_y, map_translation_z);
  if(is_init == true){
    ndt_matching.setInitPosition(odom_init_x, odom_init_y, odom_init_z, odom_init_rotation);
    ROS_INFO("\nset users initial pose\n");
  }
  ndt_matching.init(pcd_map_resolution, pcd_map_path, pcd_map_name, submap_select, search_radius, ndt_near_points, ndt_max_iteration, ndt_max_threads);

  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic, 1, imuHandler);
  ros::Subscriber filtered_lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>("/filtered_point", 1, lidarHandler);
  ros::Subscriber utm_sum = nh.subscribe<geometry_msgs::PoseStamped>("/utm", 1, utmCallback);

  map_pub = nh.advertise<sensor_msgs::PointCloud2>("/pcd_map", 1);
  ndt_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndt", 1);
  odom_pub = nh.advertise<nav_msgs::Odometry>("/final_odom", 1); 

  std::thread finalOdometryProcess{finalOdometry};

  ros::spin();

  return 0;
}