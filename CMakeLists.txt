cmake_minimum_required(VERSION 3.5)
project(lidar_localizer)

set(CMAKE_CXX_STANDARD 14)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clagn")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(Boost REQUIRED COMPONENTS system filesystem timer thread)
find_package(GeographicLib REQUIRED)
find_package(PCL REQUIRED)

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

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(pcl_omp
  src/pclomp/voxel_grid_covariance_omp.cpp
  src/pclomp/ndt_omp.cpp
  src/pclomp/gicp_omp.cpp
)

link_directories(
  include
  ${PCL_LIBRARY_DIRS}
)

#GPS data to UTM coordinate node
add_executable(gps_to_utm_node
  src/gpsToUtm.cpp
  src/gpsToUtmNode.cpp
)
ament_target_dependencies(gps_to_utm_node
  rclcpp
  geometry_msgs
  std_msgs
  sensor_msgs
)
target_link_libraries(gps_to_utm_node
  ${GeographicLib_LIBRARIES}
)
install(TARGETS gps_to_utm_node
  DESTINATION lib/${PROJECT_NAME}
)

#lidar pre-filtering node
add_executable(lidar_processing_node
  src/lidar.cpp 
  src/lidarProcessing.cpp 
  src/lidarProcessingNode.cpp
)
ament_target_dependencies(lidar_processing_node
  rclcpp
  pcl_conversions
  sensor_msgs
)
target_link_libraries(lidar_processing_node
  ${PCL_LIBRARIES}
)
install(TARGETS lidar_processing_node
  DESTINATION lib/${PROJECT_NAME}
)

#lane detection node
add_executable(lane_detection_node
  src/range.cpp
  src/lidar.cpp
  src/laneDetection.cpp
  src/laneDetectionNode.cpp
)
ament_target_dependencies(lane_detection_node
  rclcpp
  pcl_conversions
  sensor_msgs
)
target_link_libraries(lane_detection_node
  ${PCL_LIBRARIES}
)
install(TARGETS lane_detection_node
  DESTINATION lib/${PROJECT_NAME}
)

#imu pre-filtering node
add_executable(local_pose_processing_node
  src/localPoseProcessing.cpp
  src/localPoseProcessingNode.cpp
)
ament_target_dependencies(local_pose_processing_node
  rclcpp
  sensor_msgs
  geometry_msgs
  tf2_ros
)
target_link_libraries(local_pose_processing_node
  ${EIGEN3_LIBRARIES}
)
install(TARGETS local_pose_processing_node
  DESTINATION lib/${PROJECT_NAME}
)

#ndt matching node
add_executable(ndt_matching_node
  src/ndtMatching.cpp
  src/ndtMatchingNode.cpp
)
ament_target_dependencies(ndt_matching_node
  rclcpp
  pcl_conversions
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf2_ros
)
target_link_libraries(ndt_matching_node
  ${PCL_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  pcl_omp
)
install(TARGETS ndt_matching_node
  DESTINATION lib/${PROJECT_NAME}
)

#exponential weight filter node
add_executable(exponential_weight_filter_node
  src/exponentialWeightFilter.cpp
  src/exponentialWeightFilterNode.cpp
)
ament_target_dependencies(exponential_weight_filter_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  tf2_ros
)
target_link_libraries(exponential_weight_filter_node
  ${EIGEN3_LIBRARIES}
)
install(TARGETS exponential_weight_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

#kalman filter node (final odometry)
add_executable(extended_kalman_filter_node
  src/extendedKalmanFilter.cpp
  src/extendedKalmanFilterNode.cpp
)
ament_target_dependencies(extended_kalman_filter_node
  rclcpp
  nav_msgs
)
target_link_libraries(extended_kalman_filter_node
  ${EIGEN3_LIBRARIES}
)
install(TARGETS extended_kalman_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)
ament_package()