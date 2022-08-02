#include "gicpMatching.hpp"

namespace GicpMatching{
  void gicpMatching::setMapTransformInfo(const double &theta, const double &x, const double &y, const double &z)
  {
    m_theta = theta;
    m_transform_x = x;
    m_transform_y = y;
    m_transform_z = z;
  }

  void gicpMatching::setInitPosition(const double &x, const double &y, const double &z, const double &theta)
  {
    Eigen::AngleAxisf init_rotation(theta, Eigen::Vector3f::UnitZ ());
    Eigen::Translation3f init_translation(x, y, z);
    m_prev_pose = (init_translation * init_rotation).matrix ();
  }

  void gicpMatching::init(const double &map_resolution, const std::string &map_path, const std::string &map_name, const bool &submap_select, const double &search_radius, const double &near_points, const int &max_iter)
  {
    m_map_resolution = map_resolution;
    m_submap_select = submap_select;
    m_search_radius = search_radius;
    m_near_points = near_points;
    
    mp_pcd_map = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    m_kd_tree = pcl::KdTreeFLANN<pcl::PointXYZI>::Ptr(new pcl::KdTreeFLANN<pcl::PointXYZI>());
    m_gicp = pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr(new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
    
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
    m_gicp->setMaxCorrespondenceDistance(1.0);
    m_gicp->setTransformationEpsilon(0.001);
    m_gicp->setRotationEpsilon(0.001);
    m_gicp->setMaximumIterations(max_iter);
    m_gicp->setInputTarget(mp_pcd_map);
  }

  void gicpMatching::processGicp(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out, Eigen::Matrix4f &out_pose)
  { 
    clock_t start, end;

    //radius based map filtering
    // pcl::PointCloud<pcl::PointXYZI>::Ptr submap_pcd(new pcl::PointCloud<pcl::PointXYZI>());
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
    //GICP
    start = clock();
    
    m_gicp->setInputSource(pc_in);
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr aligned_pcd(new pcl::PointCloud<pcl::PointXYZI>());
    m_gicp->align(*aligned_pcd, m_prev_pose);
    if(m_gicp->hasConverged())
      printf("--\nscore: %.4f\n---\n", m_gicp->getFitnessScore());
    //In fitness score, lower is better
    pcl::transformPointCloud(*pc_in, *pc_out, m_gicp->getFinalTransformation());

    m_prev_pose = m_gicp->getFinalTransformation();
    out_pose = m_prev_pose;    
    end = clock();

    for(int i=0; i<4; i++){
      for(int j=0; j<4; j++){
        printf("matrix[%d, %d]: %.4f, ", i, j, out_pose(i,j));
      }
      printf("\n");
    }
    double result_time = (double)(end - start)/CLOCKS_PER_SEC;
    printf("\ngicp_time: %f", result_time);
  }

  void gicpMatching::radiusSearch(const cv::Point3d &based_point, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out)
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

  void gicpMatching::kNearestSearch(const cv::Point3d &based_point, pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_out)
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

  void gicpMatching::pcdMapTransform(const pcl::PointCloud<pcl::PointXYZI>::Ptr &pc_in)
  {
    Eigen::AngleAxisf z_rotation(m_theta, Eigen::Vector3f::UnitZ ());
    Eigen::Translation3f pcd_translation(m_transform_x, m_transform_y, m_transform_z);
    Eigen::Matrix4f transform_matrix = (pcd_translation * z_rotation).matrix();
    pcl::transformPointCloud(*pc_in, *mp_pcd_map, transform_matrix);
  }

  gicpMatching::gicpMatching(void)
  {
  }
}