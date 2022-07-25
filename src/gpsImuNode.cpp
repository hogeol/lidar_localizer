//STL
#include <string>
#include <queue>
#include <mutex>
#include <thread>

//ros
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

std::mutex mutex_lock;
std::queue<geometry_msgs::PoseStampedConstPtr> utm_buf;
std::queue<sensor_msgs::ImuConstPtr> imu_buf;

void imuHandler(const sensor_msgs::ImuConstPtr &imu_msg)
{
  mutex_lock.lock();
  imu_buf.push(imu_msg);
  mutex_lock.unlock();
}

void utmCallback(const geometry_msgs::PoseStampedConstPtr &utm_msg)
{
  mutex_lock.lock();
  utm_buf.push(utm_msg);
  mutex_lock.unlock();
}

void gpsImu()
{
  while(1){
    if(!utm_buf.empty() && !imu_buf.empty()){
      mutex_lock.lock();
      ros::Time utm_in_time = utm_buf.front()->header.stamp;
      ros::Time imu_in_time = imu_buf.front()->header.stamp;

      mutex_lock.unlock();
    }
    std::chrono::milliseconds dura(2);
    std::this_thread::sleep_for(dura);
  }
}


int main(int argc ,char **argv)
{
  ros::init(argc, argv, "gpsImu");
  ros::NodeHandle nh;

  std::string imu_topic="/vectornav/IMU";
  nh.getParam("imu_topic", imu_topic);

  ros::Subscriber imu_sub = nh.subscribe<sensor_msgs::Imu>(imu_topic, 100, imuHandler);
  ros::Subscriber utm_sub = nh.subscribe<geometry_msgs::PoseStamped>("/utm", 100, utmCallback);

  std::thread gpsImuProcess{gpsImu};
  return 0;
}