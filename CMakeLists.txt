cmake_minimum_required(VERSION 3.0.2)
project(lidar_localizer)

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  rosbag
  std_msgs
  tf2
  tf2_ros
  eigen_conversions
  message_generation
  cv_bridge
  image_transport
)

find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()
find_package(OpenMP)
if(OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif() 
find_package(GeographicLib REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer thread)
find_package(PCL REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_library(pcl_omp
  src/pclomp/voxel_grid_covariance_omp.cpp
  src/pclomp/ndt_omp.cpp
  src/pclomp/gicp_omp.cpp
)

link_directories(
  include
  ${PCL_LIBRARY_DIRS}
  ${CERES_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL Ceres GTSAM pcl_omp
  INCLUDE_DIRS include
)
#GPS data to UTM coordinate node
add_executable(
gps_to_utm_node
src/gpsToUtm.cpp
src/gpsToUtmNode.cpp
)
target_link_libraries(gps_to_utm_node
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)

#lidar pre-filtering node
add_executable(
lidar_processing_node
src/lidar.cpp 
src/lidarProcessing.cpp 
src/lidarProcessingNode.cpp
)
target_link_libraries(lidar_processing_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#imu pre-filtering node
add_executable(
local_pose_processing_node
src/localPoseProcessing.cpp
src/localPoseProcessingNode.cpp
)
target_link_libraries(local_pose_processing_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)

#ndt matching node
add_executable(
ndt_matching_node
src/ndtMatching.cpp
src/ndtMatchingNode.cpp
)
target_link_libraries(ndt_matching_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  pcl_omp
)

#kalman filter node (final odometry)
add_executable(
  extended_kalman_filter_node
  src/extendedKalmanFilter.cpp
  src/extendedKalmanFilterNode.cpp
)
target_link_libraries(extended_kalman_filter_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
)
