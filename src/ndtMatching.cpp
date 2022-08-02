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
    m_prev_pose = (init_translation * init_rotation).matrix ();
  }

  void ndtMatching::init(const double &map_resolution, const std::string &map_path, const std::string &map_name, const bool &submap_select, const double &search_radius, const int & near_points, const int &max_iter, const int &ndt_threads)
  {
    m_map_resolution = map_resolution;
    m_submap_select = submap_select;
    m_search_radius = search_radius;
    m_near_points = near_points;
    
    m_map_registor = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
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
    m_ndt->setResolution(1.0);
    //m_ndt->setNumThreads(omp_get_max_threads());
    m_ndt->setNumThreads(ndt_threads);
    m_ndt->setMaximumIterations(max_iter);
    m_ndt->setInputTarget(mp_pcd_map);
  }

  void ndtMatching::processNdt(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out, Eigen::Matrix4f &out_pose)
  {
    clock_t start, end;
 
    //radius based map filtering
    pcl::PointCloud<pcl::PointXYZI>::Ptr submap_pcd(new pcl::PointCloud<pcl::PointXYZI>());
    // cv::Point3d xyz_point = cv::Point3d(m_prev_pose(0,3), m_prev_pose(1,3), m_prev_pose(2,3));
    // if(m_submap_select == 1){
    //   printf("\n---\nknearest_search\nsearch_points: %d\n", m_near_points);
    //   kNearestSearch(xyz_point, submap_pcd);
    //   //kNearestSearch(xyz_point, pc_out);
    // }
    // else{
    //   printf("\n---\nradius_search\nsearch_radius: %.2f\n", m_search_radius);
    //   radiusSearch(xyz_point, submap_pcd);
    //   //radiusSearch(xyz_point, pc_out);
    // }
    //NDT
    start = clock();
    
    m_ndt->setInputSource(pc_in);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_pcd(new pcl::PointCloud<pcl::PointXYZI>());
    m_ndt->align(*aligned_pcd, m_prev_pose);
    if(m_ndt->hasConverged())
      printf("--\nscore: %.4f, iteration: %d\n---\n", m_ndt->getFitnessScore(), m_ndt->getFinalNumIteration());
    //In fitness score, lower is better
    pcl::transformPointCloud(*pc_in, *pc_out, m_ndt->getFinalTransformation());
    //*m_map_registor += *pc_out;
    //pc_out = m_map_registor;

    m_prev_pose = m_ndt->getFinalTransformation();
    out_pose = m_prev_pose;    
    end = clock();

    for(int i=0; i<4; i++){
      for(int j=0; j<4; j++){
        printf("matrix[%d, %d]: %.4f, ", i, j, out_pose(i,j));
      }
      printf("\n");
    }
    double result_time = (double)(end - start)/CLOCKS_PER_SEC;
    printf("\nndt_time: %f", result_time);
  }

  void ndtMatching::processNdtWithColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pc_out, Eigen::Matrix4f &out_pose)
  {
    clock_t start, end;
    //voxelization
    // pcl::PointCloud<pcl::PointXYZI>::Ptr after_voxel_pc(new pcl::PointCloud<pcl::PointXYZI>());
    // pcl::ApproximateVoxelGrid<pcl::PointXYZI> app_voxel_grid;
    // app_voxel_grid.setLeafSize(m_leaf_size, m_leaf_size, m_leaf_size);
    // app_voxel_grid.setInputCloud(pc_in);
    // app_voxel_grid.filter(*after_voxel_pc);

    //radius based map filtering
    pcl::PointCloud<pcl::PointXYZI>::Ptr submap_pcd(new pcl::PointCloud<pcl::PointXYZI>());
    cv::Point3d xyz_point = cv::Point3d(m_prev_pose(0,3), m_prev_pose(1,3), m_prev_pose(2,3));
    if(m_submap_select == 1){
      printf("\n---\nknearest_search\nsearch_points: %d\n", m_near_points);
      kNearestSearch(xyz_point, submap_pcd);
      //kNearestSearch(xyz_point, pc_out);
    }
    else{
      printf("\n---\nradius_search\nsearch_radius: %.2f\n", m_search_radius);
      radiusSearch(xyz_point, submap_pcd);
      //radiusSearch(xyz_point, pc_out);
    }
    //NDT
    start = clock();
    
    //m_ndt->setInputSource(after_voxel_pc);
    m_ndt->setInputSource(pc_in);
    //Eigen::AngleAxisf init_rotation(0.96, Eigen::Vector3f::UnitZ ());
    //Eigen::Translation3f init_translation(init_pose(0,3), init_pose(1,3), init_pose(2,3));
    //Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix ();
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_pcd(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr trans_pcd(new pcl::PointCloud<pcl::PointXYZI>());
    m_ndt->align(*aligned_pcd, m_prev_pose);
    if(m_ndt->hasConverged())
      printf("--\nscore: %.4f, max_iter: %d, iteration: %d\n---\n", m_ndt->getFitnessScore(), m_ndt->getFinalNumIteration());
    //In fitness score, lower is better
    pcl::transformPointCloud(*pc_in, *trans_pcd, m_ndt->getFinalTransformation());
    pcl::PointXYZRGB pc_out_tmp;
    for(int point_iter=0; point_iter<trans_pcd->points.size(); point_iter++){
      const auto &pt_iter = trans_pcd->points[point_iter];
      pc_out_tmp.x=pt_iter.x;
      pc_out_tmp.y=pt_iter.y;
      pc_out_tmp.z=pt_iter.z;
      pc_out_tmp.r = 0;
      pc_out_tmp.g = 255;
      pc_out_tmp.b = 255;
      pc_out->points.emplace_back(pc_out_tmp);
    }
    m_prev_pose = m_ndt->getFinalTransformation();
    out_pose = m_prev_pose;    
    end = clock();

    for(int i=0; i<4; i++){
      for(int j=0; j<4; j++){
        printf("matrix[%d, %d]: %.4f, ", i, j, out_pose(i,j));
      }
      printf("\n");
    }
    double result_time = (double)(end - start)/CLOCKS_PER_SEC;
    printf("\nndt_time: %f", result_time);
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
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    transform_matrix << std::cos(m_theta), -std::sin(m_theta), 0.0, m_transform_x,
                          std::sin(m_theta),  std::cos(m_theta), 0.0, m_transform_y,
                          0.0              ,  0.0              , 1.0, m_transform_z,
                          0.0              ,  0.0              , 0.0, 1.0; 
    pcl::transformPointCloud(*pc_in, *mp_pcd_map, transform_matrix);
  }

  ndtMatching::ndtMatching(void)
  {
  }
}