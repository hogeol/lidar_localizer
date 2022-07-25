#include "ndtMatching.hpp"

namespace NdtMatching{
  void ndtMatching::init(const double &map_resolution, const std::string &map_path, const std::string &map_name, const double &leaf_size, const bool &submap_select, const double &search_radius, const int & near_points, const int &max_iter)
  {
    printf("\nresolution: %.3f, map: %s %s", map_resolution, map_path, map_name);
    m_submap_select = submap_select;
    m_search_radius = search_radius;
    m_near_points = near_points;
    m_max_iter = max_iter;
    m_leaf_size = leaf_size;
    
    mp_pcd_map = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    m_kd_tree = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    m_ndt = pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr(new pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr map_tmp(new pcl::PointCloud<pcl::PointXYZI>());
  
    if(pcl::io::loadPCDFile<pcl::PointXYZI>(map_path + map_name, *map_tmp) == -1){
      PCL_ERROR("Couldn't read source pcd file!\n");
    }
    //transform if UTM coordinate is not provided in mapping (only z-axis based)
    pcdMapTransform(map_tmp);
    m_kd_tree->setInputCloud(mp_pcd_map);
    m_ndt->setTransformationEpsilon(0.1);
    m_ndt->setStepSize(0.1);
    m_ndt->setResolution(1.0);
    m_ndt->setMaximumIterations(m_max_iter);
  }

  void ndtMatching::setTransformInfo(const double &theta, const double &x, const double &y, const double &z)
  {
    m_theta = theta;
    m_x = x;
    m_y = y;
    m_z = z;
  }

  void ndtMatching::processNdt(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out, const cv::Point3d &odom_xyz)
  {
    clock_t start, end;
    //voxelization
    pcl::PointCloud<pcl::PointXYZI>::Ptr after_voxel_pc(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::ApproximateVoxelGrid<pcl::PointXYZI> app_voxel_grid;
    app_voxel_grid.setLeafSize(m_leaf_size, m_leaf_size, m_leaf_size);
    app_voxel_grid.setInputCloud(pc_in);
    app_voxel_grid.filter(*after_voxel_pc);

    //radius based map filtering
    pcl::PointCloud<pcl::PointXYZI>::Ptr kd_based_search_pcd(new pcl::PointCloud<pcl::PointXYZI>());
    if(m_submap_select == 1){
      printf("\n---\nknearest_search\nsearch_points: %d\n", m_near_points);
      kNearestSearch(odom_xyz, kd_based_search_pcd);
      //kNearestSearch(odom_xyz, pc_out);
    }
    else{
      printf("\n---\nradius_search\nsearch_radius: %.2f\n", m_search_radius);
      radiusSearch(odom_xyz, kd_based_search_pcd);
      //radiusSearch(odom_xyz, pc_out);
    }
    
    //NDT
    start = clock();
    m_ndt->setInputSource(after_voxel_pc);
    m_ndt->setInputTarget(kd_based_search_pcd);

    Eigen::AngleAxisf init_rotation(0.0, Eigen::Vector3f::UnitZ ());
    Eigen::Translation3f init_translation(0.0, 0.0, 0.0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();
    pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_pcd(new pcl::PointCloud<pcl::PointXYZI>());
    m_ndt->align(*aligned_pcd, init_guess);
    //In fitness score, lower is better
    printf("--\nscore: %.4f, max_iter: %d, iteration: %d\n---\n", m_ndt->getFitnessScore(), m_max_iter, m_ndt->getFinalNumIteration());
    pcl::transformPointCloud(*pc_in, *pc_out, m_ndt->getFinalTransformation());
    
    end = clock();

    double result = (double)(end - start)/CLOCKS_PER_SEC;
    printf("\nndt_time: %f", result);
  }

  void ndtMatching::radiusSearch(const cv::Point3d &based_point, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out)
  {
    std::vector<int> indices;
    std::vector<float> sqr_dists;
    pcl::PointXYZI base_query;
    base_query.x = based_point.x;
    base_query.y = based_point.y;
    base_query.z = based_point.z;
    base_query.intensity = 0.0;
    m_kd_tree->radiusSearch(base_query, m_search_radius, indices, sqr_dists);
    for(const auto &idx: indices){
      pc_out->points.push_back(mp_pcd_map->points[idx]);
    }
  }

  void ndtMatching::kNearestSearch(const cv::Point3d &based_point, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out)
  {
    std::vector<int> indices;
    std::vector<float> sqr_dists;
    pcl::PointXYZI base_query;
    base_query.x = based_point.x;
    base_query.y = based_point.y;
    base_query.z = based_point.z;
    base_query.intensity = 0.0;
    m_kd_tree->nearestKSearch(base_query, m_near_points, indices, sqr_dists);
    for(const auto &idx: indices){
      pc_out->points.push_back(mp_pcd_map->points[idx]);
    }
  }

  void ndtMatching::pcdMapTransform(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in)
  {
    m_transform_matrix << std::cos(m_theta), -std::sin(m_theta), 0, m_x,
                            std::sin(m_theta),  std::cos(m_theta), 0, m_y,
                            0                , 0                 , 1, m_z,
                            0                , 0                 , 0, 1; 
    pcl::transformPointCloud(*pc_in, *mp_pcd_map, m_transform_matrix);
  }

  ndtMatching::ndtMatching(void)
  {
  }
}