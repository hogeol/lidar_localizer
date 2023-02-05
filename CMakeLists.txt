cmake_minimum_required(VERSION 3.10.0)
project(lidar_localizer)

add_definitions(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
set(CMAKE_C_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")
set(CMAKE_CXX_FLAGS "-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2")

set(CMAKE_BUILD_TYPE "Release")

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
)

### PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})

### EIGEN
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
include_directories(${EIGEN3_INCLUDE_DIR})

### OPENMP
find_package(OpenMP)
if(OPENMP_FOUND)
  set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
	set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

### GEOGRAPHIC AND BOOST 
find_package(GeographicLib REQUIRED)
find_package(Boost REQUIRED COMPONENTS timer thread)

### CUDA
find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})
link_directories(${CUDA_LIBRARY_DIRS})

### PCLOMP
add_library(pcl_omp
  src/pclomp/voxel_grid_covariance_omp.cpp
  src/pclomp/ndt_omp.cpp
  src/pclomp/gicp_omp.cpp
)

### FAST GICP
add_library(fastgicp SHARED
  src/fast_gicp/gicp/lsq_registration.cpp
  src/fast_gicp/gicp/fast_gicp.cpp
  src/fast_gicp/gicp/fast_gicp_st.cpp
  src/fast_gicp/gicp/fast_vgicp.cpp
)
target_link_libraries(fastgicp
  ${PCL_LIBRARIES}
)
target_include_directories(fastgicp PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

### FAST_VGICP
set(CUDA_NVCC_FLAGS "--expt-relaxed-constexpr")
add_definitions(-DUSE_VGICP_CUDA)

cuda_add_library(fast_vgicp_cuda SHARED
  src/fast_gicp/cuda/fast_vgicp_cuda.cu
  src/fast_gicp/cuda/brute_force_knn.cu
  src/fast_gicp/cuda/covariance_estimation.cu
  src/fast_gicp/cuda/covariance_estimation_rbf.cu
  src/fast_gicp/cuda/covariance_regularization.cu
  src/fast_gicp/cuda/gaussian_voxelmap.cu
  src/fast_gicp/cuda/find_voxel_correspondences.cu
  src/fast_gicp/cuda/compute_derivatives.cu
  src/fast_gicp/cuda/compute_mahalanobis.cu
  src/fast_gicp/cuda/ndt_cuda.cu
  src/fast_gicp/cuda/ndt_compute_derivatives.cu
)
target_include_directories(fast_vgicp_cuda PRIVATE
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  thirdparty/nvbio
)
target_link_libraries(fast_vgicp_cuda
  ${catkin_LIBRARIES}
)
cuda_add_cublas_to_target(fast_vgicp_cuda)

# add vgicp_cuda to libfast_gicp
target_sources(fastgicp PRIVATE
  src/fast_gicp/gicp/fast_vgicp_cuda.cpp
  src/fast_gicp/ndt/ndt_cuda.cpp
)
target_link_libraries(fastgicp
  fast_vgicp_cuda
)
add_dependencies(fastgicp fast_vgicp_cuda)
if(catkin_FOUND)
  install(TARGETS fast_vgicp_cuda  
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
endif()

add_service_files(
  DIRECTORY srv
  FILES
  relocalize.srv
)
generate_messages()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

link_directories(
  include
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL Ceres GTSAM pcl_omp
  INCLUDE_DIRS include
  LIBRARIES fast_gicp
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

#lane detection node
add_executable(lane_detection_node
src/range.cpp
src/lidar.cpp
src/laneDetection.cpp
src/laneDetectionNode.cpp
)
target_link_libraries(lane_detection_node
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
  fastgicp
	# nano_gicp
)

#exponential weight filter node
add_executable(
  exponential_weight_filter_node
  src/exponentialWeightFilter.cpp
  src/exponentialWeightFilterNode.cpp
)
target_link_libraries(exponential_weight_filter_node
  ${catkin_LIBRARIES}
  ${EIGEN3_LIBRARIES}
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