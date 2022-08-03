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
#include <geometry_msgs/PoseWithCovarianceStamped.h>
//tf
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

//local lib
#include "ndtMatching.hpp"

//subscriber
//publisher
ros::Publisher map_pub;
ros::Publisher ndt_pub;
ros::Publisher odom_pub;

NdtMatching::ndtMatching ndt_matching;
std::queue<sensor_msgs::PointCloud2ConstPtr> lidar_buf;
std::queue<nav_msgs::OdometryConstPtr> gps_odom_buf;

std::mutex mutex_lock;
bool is_init;


void lidarHandler(const sensor_msgs::PointCloud2ConstPtr &filtered_msg)
{
  mutex_lock.lock();
  lidar_buf.push(filtered_msg);
  mutex_lock.unlock();
}

void gpsCallback(const nav_msgs::OdometryConstPtr &gps_odom_msg)
{
  mutex_lock.lock();
  gps_odom_buf.push(gps_odom_msg);
  //printf("\n--\ngps\nx: %.4f\ny: %.4f\nz: %.4f\n---\n", gps_odom_buf.front()->pose.pose.position.x, gps_odom_buf.front()->pose.pose.position.y, gps_odom_buf.front()->pose.pose.position.z);
  mutex_lock.unlock();
}

void rvizCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &pose_msg)
{
  mutex_lock.lock();
  Eigen::Quaterniond eigen_orientation(pose_msg->pose.pose.orientation.w, pose_msg->pose.pose.orientation.x, pose_msg->pose.pose.orientation.y, pose_msg->pose.pose.orientation.z);
  Eigen::Matrix3d eigen_rot;
  eigen_rot.block<3,3>(0,0) = eigen_orientation.toRotationMatrix();
  tf::Matrix3x3 tf_rot_mat;
  tf::matrixEigenToTF(eigen_rot, tf_rot_mat);
  double r,p,y;
  tf_rot_mat.setRPY(r, p, y);
  ndt_matching.setInitPosition(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z, y);
  is_init = true;
  mutex_lock.unlock();
}

void ndtMatching()
{
  while(1){
    if(!lidar_buf.empty() && !gps_odom_buf.empty()){
      //point ros to pointcloud
      mutex_lock.lock();
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_in(new pcl::PointCloud<pcl::PointXYZI>());
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_out(new pcl::PointCloud<pcl::PointXYZI>());
      
      pcl::fromROSMsg(*lidar_buf.front(), *point_in);
      ros::Time point_in_time = lidar_buf.front()->header.stamp;
      ros::Time odom_in_time = gps_odom_buf.front()->header.stamp;
      Eigen::Vector3f gps_pose(gps_odom_buf.front()->pose.pose.position.x, gps_odom_buf.front()->pose.pose.position.y, gps_odom_buf.front()->pose.pose.position.z);
      Eigen::Quaterniond gps_orientation(gps_odom_buf.front()->pose.pose.orientation.w, gps_odom_buf.front()->pose.pose.orientation.x, gps_odom_buf.front()->pose.pose.orientation.y, gps_odom_buf.front()->pose.pose.orientation.z);
      if(is_init == false){
        Eigen::Matrix3d eigen_init_rot;
        eigen_init_rot.block<3,3>(0,0) = gps_orientation.toRotationMatrix();
        tf::Matrix3x3 tf_rot_matrix;
        tf::matrixEigenToTF(eigen_init_rot, tf_rot_matrix);
        double roll = 0.0, pitch = 0.0, yaw = 0.0;
        tf_rot_matrix.getRPY(roll, pitch, yaw);
        ndt_matching.setInitPosition(gps_pose(0), gps_pose(1), gps_pose(2), -0.8);
        is_init = true;
        ROS_INFO("\ngps inited\n");
        gps_odom_buf.pop();
        lidar_buf.pop();
        mutex_lock.unlock();  
      }
      else{
        gps_odom_buf.pop();
        lidar_buf.pop();
        mutex_lock.unlock();  
      }
      
      //after ndt matching
      Eigen::Matrix4f result_pose = Eigen::Matrix4f::Identity();
      ndt_matching.processNdt(point_in, point_out, gps_pose, result_pose);
      Eigen::Quaternionf q(result_pose.block<3,3>(0,0));
      q.normalize();

      sensor_msgs::PointCloud2 ndt_msg;
      pcl::toROSMsg(*point_out, ndt_msg);
      ndt_msg.header.stamp = point_in_time;
      ndt_msg.header.frame_id = "map";
      ndt_pub.publish(ndt_msg);
      
      geometry_msgs::Pose pose_msg;
      pose_msg.position.x = result_pose(0,3);
      pose_msg.position.y = result_pose(1,3);
      pose_msg.position.z = result_pose(2,3);
      pose_msg.orientation.w = q.w();
      pose_msg.orientation.x = q.x();
      pose_msg.orientation.y = q.y();
      pose_msg.orientation.z = q.z();
      nav_msgs::Odometry ndt_odom_msg;
      ndt_odom_msg.header.frame_id = "map";
      ndt_odom_msg.child_frame_id = "base_link";
      ndt_odom_msg.header.stamp = point_in_time;
      ndt_odom_msg.pose.pose = pose_msg;
      
      static tf::TransformBroadcaster br;
      tf::Transform transform;
      transform.setOrigin(tf::Vector3(result_pose(0,3), result_pose(1,3), result_pose(2,3)));
      tf::Quaternion q_tf(q.x(), q.y(), q.z(), q.w());
      transform.setRotation(q_tf);
      br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "base_link"));
      odom_pub.publish(ndt_odom_msg);
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
  
  double map_resolution = 2.0;
  double search_radius = 150.0;
  int ndt_near_points = 500000;
  int ndt_max_iteration = 50;
  int ndt_max_threads = 10;
  bool submap_select=1;
  double init_x = 0.0;
  double init_y = 0.0;
  double init_z = 0.0;
  double init_rotation=0.96;

  double rotation_theta=0.0;
  double translation_x=0.0;
  double translation_y=0.0;
  double translation_z=0.0;
  is_init = 0;

  std::string map_path = "/home/a/ace_ws/src/velodyne_ndt/map/";
  std::string map_name = "map.pcd";

  nh.getParam("map_resolution", map_resolution);
  nh.getParam("map_path", map_path);
  nh.getParam("map_name", map_name);
  nh.getParam("submap_select", submap_select);
  nh.getParam("kdtree_search_radius", search_radius);
  nh.getParam("ndt_near_points", ndt_near_points);
  nh.getParam("ndt_max_iteration", ndt_max_iteration);
  nh.getParam("gps_init_pose", is_init);
  nh.getParam("translation_x", translation_x);
  nh.getParam("translation_y", translation_y);
  nh.getParam("translation_z", translation_z);
  nh.getParam("rotation_theta", rotation_theta);
  nh.getParam("init_x", init_x);
  nh.getParam("init_y", init_y);
  nh.getParam("init_z", init_z);
  nh.getParam("init_rotation", init_rotation);
  nh.getParam("ndt_max_threads", ndt_max_threads);
  
  ndt_matching.setMapTransformInfo(rotation_theta, translation_x, translation_y, translation_z);
  if(is_init == true){
    ndt_matching.setInitPosition(init_x, init_y, init_z, init_rotation);
    ROS_INFO("\nset users initial pose\n");
  }
  ndt_matching.init(map_resolution, map_path, map_name, submap_select, search_radius, ndt_near_points, ndt_max_iteration, ndt_max_threads);

  ros::Subscriber filtered_lidar_sub = nh.subscribe<sensor_msgs::PointCloud2>("/filtered_point", 1, lidarHandler);
  ros::Subscriber gps_odom_sub = nh.subscribe<nav_msgs::Odometry>("/gps_odom", 1, gpsCallback);
  ros::Subscriber rviz_pose_sub = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1, rvizCallback);
  
  map_pub = nh.advertise<sensor_msgs::PointCloud2>("/pcd_map", 1);
  ndt_pub = nh.advertise<sensor_msgs::PointCloud2>("/ndt", 1);
  odom_pub = nh.advertise<nav_msgs::Odometry>("/ndt_odom", 1);

  std::thread ndtMatchingProcess{ndtMatching};

  ros::spin();

  return 0;
}