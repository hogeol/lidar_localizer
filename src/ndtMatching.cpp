#define _USE_MATH_DEFINES_
#include "ndtMatching.hpp"

namespace NdtMatching{
  void ndtMatching::setMapTransformInfo(const double &theta, const double &x, const double &y, const double &z)
  {
    m_theta = theta;
    m_transform_x = x;
    m_transform_y = y;
    m_transform_z = z;
  }

  void ndtMatching::setInitPosition(const double &x, const double &y, const double &z, const double &theta)
  {
    Eigen::AngleAxisf init_rotation(theta, Eigen::Vector3f::UnitZ());
    Eigen::Translation3f init_translation(x, y, z);
    m_last_pose = (init_translation * init_rotation).matrix();
    printf("\nx: %.5f\ny: %.5f\nz: %.5f\nposition inited\n", x, y, z);
  }

  void ndtMatching::setGpsLidarTF(const double &diff_x, const double &diff_y, const double &diff_z)
  {
    m_diff_x = diff_x;
    m_diff_y = diff_y;
    m_diff_z = diff_z;
  }

  void ndtMatching::init(const double &map_resolution, const std::string &map_path, const std::string &map_name, const bool &submap_select, const double &search_radius, const int & near_points, const int &max_iter, const int &ndt_threads, const bool &gps_diff_matching, const int &place_recognition_method)
  {
    m_local_count = 0;
    m_map_resolution = map_resolution;
    m_submap_select = submap_select;
    m_search_radius = search_radius;
    m_near_points = near_points;
    m_gps_diff_matching = gps_diff_matching;
    m_place_recognition_method = place_recognition_method;
    mp_pcd_map = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    m_kd_tree = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_load_pcd(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_after_downsampling_pcd(new pcl::PointCloud<pcl::PointXYZI>());
  
    if(pcl::io::loadPCDFile<pcl::PointXYZI>(map_path + map_name, *map_load_pcd) == -1){
      PCL_ERROR("Couldn't read source pcd file!\n");
    }

    pcl::ApproximateVoxelGrid<pcl::PointXYZI> app_voxel_grid;
    app_voxel_grid.setLeafSize(m_map_resolution, m_map_resolution, m_map_resolution);
    app_voxel_grid.setInputCloud(map_load_pcd);
    app_voxel_grid.filter(*map_after_downsampling_pcd);
    //transform if UTM coordinate is not provided in mapping (lotation is only z-axis based)
    pcdMapTransform(map_after_downsampling_pcd);
    m_kd_tree->setInputCloud(mp_pcd_map);

    if(place_recognition_method == 0){
      m_ndt = pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
      m_ndt->setTransformationEpsilon(0.001);
      m_ndt->setStepSize(0.1);
      m_ndt->setResolution(3.0);
      m_ndt->setNumThreads(ndt_threads);
      m_ndt->setMaximumIterations(max_iter);
      m_ndt->setInputTarget(mp_pcd_map);
    }
    else if(place_recognition_method == 1){
      m_vgicp = fast_gicp::FastVGICPCuda<pcl::PointXYZI, pcl::PointXYZI>::Ptr(new fast_gicp::FastVGICPCuda<pcl::PointXYZI, pcl::PointXYZI>());
      m_vgicp->setInputTarget(mp_pcd_map);
      m_vgicp->setMaximumIterations(max_iter);
    }
  }

  void ndtMatching::processPlaceRecognition(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out, const Eigen::Isometry3d &pose_in, Eigen::Isometry3d &pose_out)
  {
    clock_t start, end;
    //NDT
    start = clock();
    double matching_score = 0.0;
    pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_pcd(new pcl::PointCloud<pcl::PointXYZI>());
    if(m_place_recognition_method == 0){
      m_ndt->setInputSource(pc_in);
    
      m_ndt->align(*aligned_pcd, m_last_pose);
      //m_ndt->align(*aligned_pcd);
      matching_score =  m_ndt->getFitnessScore();
      if(m_ndt->hasConverged())
        printf("--\nscore: %.4f, iteration: %d\n---\n", matching_score, m_ndt->getFinalNumIteration());

      //In fitness score, lower is better
      m_last_pose = m_ndt->getFinalTransformation();
    }
    else if(m_place_recognition_method == 1){
      m_vgicp->setInputSource(pc_in);
      m_vgicp->align(*aligned_pcd, m_last_pose);
      matching_score = m_vgicp->getFitnessScore();
      if(m_vgicp->hasConverged())
        printf("--\nscore: %.4f\n---\n", matching_score);

      //In fitness score, lower is better
      m_last_pose = m_vgicp->getFinalTransformation();
    }
    
    Eigen::Vector3f measured_pose(m_last_pose(0,3), m_last_pose(1,3), m_last_pose(2,3));
    Eigen::Vector3d gps_in_pose(pose_in.translation().x(), pose_in.translation().y(), pose_in.translation().z());
    
    if(m_gps_diff_matching && calDistance(gps_in_pose.cast<float>(), measured_pose) > 0.4 && matching_score > 0.9){
        if(m_local_count % 20 == 0){
          m_last_pose(0,3) = gps_in_pose.x();
          m_last_pose(1,3) = gps_in_pose.y();
          m_last_pose(2,3) = gps_in_pose.z();
          m_last_pose.block<3,3>(0,0) = pose_in.linear().cast<float>();
          m_local_count = 0;
        }
        m_local_count++;
        //printf("\nlocal_count: %d\n", m_local_count);
    }
    
    pcl::transformPointCloud(*pc_in, *pc_out, m_last_pose);
    pose_out.translation().x() = m_last_pose(0,3);
    pose_out.translation().y() = m_last_pose(1,3);
    pose_out.translation().z() = m_last_pose(2,3);
    pose_out.linear() = m_last_pose.block<3,3>(0,0).cast<double>();
    sensorTFCorrection(pose_out);
    end = clock();

    // for(int i=0; i<4; i++){
    //   for(int j=0; j<4; j++){
    //     printf("matrix[%d, %d]: %.4f, ", i, j, out_pose(i,j));
    //   }
    //   printf("\n");
    // }
    double result_time = (double)(end - start)/CLOCKS_PER_SEC;
    //printf("\nndt_time: %f", result_time);
  }

  void ndtMatching::sensorTFCorrection(Eigen::Isometry3d &pose_out)
  {
    Eigen::Vector4f tf_bias(m_diff_x, m_diff_y, m_diff_z, 1.0);
    tf_bias = m_last_pose * tf_bias;
    pose_out.translation().x() = tf_bias.x();
    pose_out.translation().y() = tf_bias.y();
    pose_out.translation().z() = tf_bias.z();
  }
  void ndtMatching::radiusSearch(const Eigen::Vector3d &based_point, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out)
  {
    std::vector<int> indices;
    std::vector<float> sqr_dists;
    pcl::PointXYZI base_query;
    base_query.x = based_point.x();
    base_query.y = based_point.y();
    base_query.z = based_point.z();
    base_query.intensity = 0.0;
    m_kd_tree->radiusSearch(base_query, m_search_radius, indices, sqr_dists);
    for(const auto &idx: indices){
      pc_out->points.push_back(mp_pcd_map->points[idx]);
    }
  }

  void ndtMatching::kNearestSearch(const Eigen::Vector3d &based_point, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out)
  {
    std::vector<int> indices;
    std::vector<float> sqr_dists;
    pcl::PointXYZI base_query;
    base_query.x = based_point.x();
    base_query.y = based_point.y();
    base_query.z = based_point.z();
    base_query.intensity = 0.0;
    m_kd_tree->nearestKSearch(base_query, m_near_points, indices, sqr_dists);
    for(const auto &idx: indices){
      pc_out->points.push_back(mp_pcd_map->points[idx]);
    }
  }

  void ndtMatching::pcdMapTransform(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in)
  {
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    transform_matrix << std::cos(m_theta), -std::sin(m_theta), 0.0, m_transform_x,
                          std::sin(m_theta),  std::cos(m_theta), 0.0, m_transform_y,
                          0.0              ,  0.0              , 1.0, m_transform_z,
                          0.0              ,  0.0              , 0.0, 1.0; 
    pcl::transformPointCloud(*pc_in, *mp_pcd_map, transform_matrix);
  }
  
  void ndtMatching::relocalize(const Eigen::Matrix4f &last_gps_odom)
  {
    m_last_pose = last_gps_odom;
  }

  double ndtMatching::calDistance(const Eigen::Vector3f &gps_xyz, const Eigen::Vector3f &measured_pose)
  {
    //printf("\ndistance: %.4f\n", sqrt(pow(gps_xyz(0) - measured_pose(0), 2) + pow(gps_xyz(1) - measured_pose(1), 2)));
    return (sqrt(pow(gps_xyz.x() - measured_pose.x(), 2) + pow(gps_xyz.y() - measured_pose.y(), 2)));    
  }

  void ndtMatching::relocalizeService(const float &search_radius)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr last_place_points{new pcl::PointCloud<pcl::PointXYZI>()};
    Eigen::Vector3d last_place_translation{m_last_pose(0,3), m_last_pose(1,3), m_last_pose(2,3)};
    
    radiusSearch(last_place_translation, last_place_points);
    
  }

  ndtMatching::ndtMatching(void)
  {
  }
}