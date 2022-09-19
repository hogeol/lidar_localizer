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
    Eigen::AngleAxisf init_rotation(theta, Eigen::Vector3f::UnitZ ());
    Eigen::Translation3f init_translation(x, y, z);
    m_last_pose = (init_translation * init_rotation).matrix ();
    printf("\nx: %.5f\ny: %.5f\nz: %.5f\nposition inited\n", x, y, z);
  }

  void ndtMatching::setGpsLidarTF(const double &diff_x, const double &diff_y, const double &diff_z)
  {
    m_diff_x = diff_x;
    m_diff_y = diff_y;
    m_diff_z = diff_z;
  }

  void ndtMatching::init(const double &map_resolution, const std::string &map_path, const std::string &map_name, const bool &submap_select, const double &search_radius, const int & near_points, const int &max_iter, const int &ndt_threads)
  {
    mp_pose_inited = false;
    m_local_count = 0;
    m_map_resolution = map_resolution;
    m_submap_select = submap_select;
    m_search_radius = search_radius;
    m_near_points = near_points;
    mp_pcd_map = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    m_kd_tree = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    m_ndt = pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr(new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    
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
    m_ndt->setTransformationEpsilon(0.001);
    m_ndt->setStepSize(0.1);
    m_ndt->setResolution(2.8);
    //m_ndt->setNumThreads(omp_get_max_threads());
    m_ndt->setNumThreads(ndt_threads);
    m_ndt->setMaximumIterations(max_iter);
    m_ndt->setInputTarget(mp_pcd_map);
  }

  void ndtMatching::processNdt(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out, const Eigen::Isometry3d &pose_in, Eigen::Isometry3d &pose_out)
  {
    clock_t start, end;
    //NDT
    start = clock();
    m_ndt->setInputSource(pc_in);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_pcd(new pcl::PointCloud<pcl::PointXYZI>());
    m_ndt->align(*aligned_pcd, m_last_pose);
    //m_ndt->align(*aligned_pcd);
    double ndt_score =  m_ndt->getFitnessScore();
    if(m_ndt->hasConverged())
      //printf("--\nscore: %.4f, iteration: %d\n---\n", ndt_score, m_ndt->getFinalNumIteration());
    
    //In fitness score, lower is better
    m_last_pose = m_ndt->getFinalTransformation();
    
    Eigen::Vector3f ndt_xyz(m_last_pose(0,3), m_last_pose(1,3), m_last_pose(2,3));
    Eigen::Vector3d gps_in_pose(pose_in.translation().x(), pose_in.translation().y(), pose_in.translation().z());
    
    if(calDistance(gps_in_pose.cast<float>(), ndt_xyz) > 0.5 && ndt_score > 0.15){
        if(m_local_count % 50 == 0){
          m_last_pose(0,3) = gps_in_pose.x();
          m_last_pose(1,3) = gps_in_pose.y();
          m_last_pose(2,3) = gps_in_pose.z();
          m_local_count = 0;
          mp_pose_inited = false;
        }
        m_local_count++;
        //printf("\nlocal_count: %d\n", m_local_count);
    }
    else if(ndt_score < 0.1 && mp_pose_inited == false){
      mp_pose_inited = true;
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

  void ndtMatching::processNdtWithColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_out, const Eigen::Matrix4f &pose_in, Eigen::Matrix4f &pose_out)
  {
    clock_t start, end;
    //radius based map filtering
    // pcl::PointCloud<pcl::PointXYZI>::Ptr submap_pcd(new pcl::PointCloud<pcl::PointXYZI>());
    // Eigen::Vector3d based_point(m_last_pose(0,3), m_last_pose(1,3), m_last_pose(2,3));
    
    // if(m_submap_select == 1){
    //   printf("\n---\nknearest_search\nsearch_points: %d\n", m_near_points);
    //   kNearestSearch(based_point, submap_pcd);
    //   //kNearestSearch(xyz_point, pc_out);
    // }
    // else{
    //   printf("\n---\nradius_search\nsearch_radius: %.2f\n", m_search_radius);
    //   radiusSearch(based_point, submap_pcd);
    //   //radiusSearch(xyz_point, pc_out);
    // }
    //NDT
    start = clock();
    m_ndt->setInputSource(pc_in);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_pcd(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_pcd(new pcl::PointCloud<pcl::PointXYZI>());
    
    m_ndt->align(*aligned_pcd, m_last_pose);
    //m_ndt->align(*aligned_pcd);
    double ndt_score =  m_ndt->getFitnessScore();
    if(m_ndt->hasConverged())
      printf("--\nscore: %.4f, iteration: %d\n---\n", ndt_score, m_ndt->getFinalNumIteration());
    
    //In fitness score, lower is better
    m_last_pose = m_ndt->getFinalTransformation();

    // Eigen::Vector3f ndt_xyz(m_last_pose(0,3), m_last_pose(1,3), m_last_pose(2,3));
    // Eigen::Vector3f gps_in_pose(pose_in(0,3), pose_in(1,3), pose_in(2,3));
    // if(calDistance(gps_in_pose, ndt_xyz) > 2.5 && ndt_score > 0.15){
    //     if(m_local_count%25 == 0){
    //       m_last_pose(0,3) = pose_in(0,3);
    //       m_last_pose(1,3) = pose_in(1,3);
    //       m_last_pose(2,3) = pose_in(2,3);
    //       m_local_count = 0;
    //     }
    //     m_local_count++;
    //     printf("\nlocal_count: %d\n", m_local_count);
    // }
    pcl::transformPointCloud(*pc_in, *transformed_pcd, m_last_pose);
    pcl::PointXYZRGB pc_out_tmp;
    for(int point_iter=0; point_iter<transformed_pcd->points.size(); point_iter++){
      const auto &pt_iter = transformed_pcd->points[point_iter];
      pc_out_tmp.x=pt_iter.x;
      pc_out_tmp.y=pt_iter.y;
      pc_out_tmp.z=pt_iter.z;
      pc_out_tmp.r = 0;
      pc_out_tmp.g = 255;
      pc_out_tmp.b = 255;
      pc_out->points.emplace_back(pc_out_tmp);
    }
    pose_out = m_last_pose;    
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
  double ndtMatching::calDistance(const Eigen::Vector3f &gps_xyz, const Eigen::Vector3f &ndt_xyz)
  {
    //printf("\ndistance: %.4f\n", sqrt(pow(gps_xyz(0) - ndt_xyz(0), 2) + pow(gps_xyz(1) - ndt_xyz(1), 2)));
    return (sqrt(pow(gps_xyz.x() - ndt_xyz.x(), 2) + pow(gps_xyz.y() - ndt_xyz.y(), 2)));    
  }
  
  void ndtMatching::relocalize(const Eigen::Matrix4f &last_gps_odom)
  {
    m_last_pose = last_gps_odom;
  }

  ndtMatching::ndtMatching(void)
  {
  }
}