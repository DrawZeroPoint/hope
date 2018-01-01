#include "plane_segment.h"

#include <tf2/LinearMath/Quaternion.h>

// Normal threshold, let (nx, ny, nz) be the unit normal vector of point p,
// |nz| > th_norm_ means p is from plane, notice p is transformed with camera orientation
float th_norm_ = 0.8;

// Region growing threshold
float th_smooth_ = 8;

// Voxel grid threshold
float th_leaf_ = 0.02;

// Points whose z value are within the range defined by th_deltaz_ are considered in plane
// th_deltaz_ must be 2 times bigger than th_leaf_
float th_deltaz_ = 4 * th_leaf_;

// Distance threshold for plane patch clustering
float th_cluster_ = 3 * th_leaf_;

// How much the patch need take in final segment
float th_rate_ = 0.5;

// Depth threshold for filtering source cloud, only used for real data
float th_max_depth_ = 8.0;

PlaneSegment::PlaneSegment(bool use_real_data, string base_frame, float th_area) :
  use_real_data_(use_real_data),
  fi_(new FetchRGBD),
  pub_it_(nh_),
  src_mono_cloud_(new PointCloudMono),
  src_rgb_cloud_(new PointCloud),
  src_rgbn_cloud_(new PointCloudRGBN),
  cloud_norm_fit_(new PointCloudRGBN),
  src_normals_(new NormalCloud),
  idx_norm_fit_(new pcl::PointIndices),
  src_z_inliers_(new pcl::PointIndices),
  m_tf_(new Transform),
  base_frame_(base_frame),
  th_area_(th_area),
  viewer(new pcl::visualization::PCLVisualizer("HOPE Result")),
  hst_("total")
{
  // For store max hull id and area
  global_area_temp_ = 0;
  
  // Regist the callback if using real point cloud data
  sub_pointcloud_ = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth_registered/points", 1, 
                                                            &PlaneSegment::cloudCallback, this);
  
  // Detect table obstacle
  pub_max_plane_ = nh_.advertise<sensor_msgs::PointCloud2>("/vision/max_plane", 1, true);
  pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/vision/points", 1, true);
  
  
  viewer->setBackgroundColor(0.2, 0.22, 0.24);
}

void PlaneSegment::setParams(int dataset_type, float roll, float pitch, 
                             float tx, float ty, float tz, float qx, float qy, float qz, float qw)
{
  if (dataset_type == 0) {
    roll_ = roll;
    pitch = pitch;
  }
  else if (dataset_type == 1) {
    tx_ = tx;
    ty_ = ty;
    tz_ = tz;
    qx_ = qx;
    qy_ = qy;
    qz_ = qz;
    qw_ = qw;
  }
  dataset_type_ = dataset_type;
}

void PlaneSegment::getHorizontalPlanes(PointCloud::Ptr cloud)
{
  // Notice that the point cloud is not transformed before this function
  if (use_real_data_) {
    // If using real data, the transform from camera frame to base frame
    // need to be provided
    getSourceCloud();
  }
  else {
    PointCloud::Ptr temp(new PointCloud);
    
    // To remove Nan and unreliable points with z value
    Utilities::getCloudByZ(cloud, src_z_inliers_, temp, 0.3, th_max_depth_);
    if (dataset_type_ == 1) {
      m_tf_->doTransform(temp, src_rgb_cloud_, tx_, ty_, tz_, qx_, qy_, qz_, qw_);
    }
    else {
      m_tf_->doTransform(temp, src_rgb_cloud_, roll_, pitch_);
    }
    Utilities::pointTypeTransfer(src_rgb_cloud_, src_mono_cloud_);
  }
  
  findAllPlanes();
}

bool PlaneSegment::getSourceCloud()
{
  while (ros::ok()) {
    if (!src_mono_cloud_->points.empty())
      return true;
    
    // Handle callbacks and sleep for a small amount of time
    // before looping again
    ros::spinOnce();
    ros::Duration(0.005).sleep();
  }
}

void PlaneSegment::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  if (msg->data.empty()) {
    ROS_WARN_THROTTLE(31, "PlaneSegment: PointCloud is empty.");
    return;
  }
  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *src_rgb_cloud_);
  
  PointCloudMono::Ptr temp_mono(new PointCloudMono);
  Utilities::pointTypeTransfer(src_rgb_cloud_, temp_mono);
  
  PointCloudMono::Ptr temp_filtered(new PointCloudMono);
  // Get cloud within range represented by z value
  Utilities::getCloudByZ(temp_mono, src_z_inliers_, temp_filtered, 0.3, th_max_depth_);
  
  m_tf_->getTransform(base_frame_, msg->header.frame_id);
  m_tf_->doTransform(temp_filtered, src_mono_cloud_);
}

void PlaneSegment::findAllPlanes()
{
  if (src_mono_cloud_->points.empty()) {
    ROS_WARN("PlaneSegment: Source cloud is empty.");
    return;
  }
  
  // Start timer
  hst_.start();
  
  // Clear temp
  plane_z_value_.clear();
  cloud_fit_parts_.clear();
  rg_cluster_indices_.clear();
  plane_coeff_.clear();
  plane_hull_.clear();
  global_area_temp_ = 0.0;
  
  // Calculate the normal of source mono cloud and down sampling
  // src_rgbn_cloud_ downsampled cloud with fake rgb value
  // src_normals_ downsampled normal cloud
  Utilities::estimateNorm(src_mono_cloud_, src_rgbn_cloud_, src_normals_, 
                          2 * th_leaf_, th_leaf_, true);
  
  // Extract all points with plane normal
  // idx_norm_fit_ indices with right normal
  Utilities::getCloudByNorm(src_rgbn_cloud_, idx_norm_fit_, th_norm_);
  
  if (idx_norm_fit_->indices.empty()) {
    ROS_DEBUG("PlaneSegment: No point normal fit horizontal plane.");
    return;
  }
  
  // Get plane cloud
  Utilities::getCloudByInliers(src_rgbn_cloud_, cloud_norm_fit_, idx_norm_fit_, false, false);
  
  cout<< "Points may from horizontal plane: " << cloud_norm_fit_->points.size() << endl;
  
  // Prepare normal data for clustering
  pcl::PointCloud<pcl::Normal>::Ptr norm_fit(new pcl::PointCloud<pcl::Normal>);
  norm_fit->resize(cloud_norm_fit_->size());
  
  size_t i = 0;
  for (PointCloudRGBN::const_iterator pit = cloud_norm_fit_->begin();
       pit != cloud_norm_fit_->end(); ++pit) {
    norm_fit->points[i].normal_x = pit->normal_x;
    norm_fit->points[i].normal_y = pit->normal_y;
    norm_fit->points[i].normal_z = pit->normal_z;
    ++i;
  }
  
  ///time used:35%
  
  // Perform clustering, cause the scene may contain multiple planes
  // generate rg_cluster_indices_
  calRegionGrowing(cloud_norm_fit_, norm_fit);
  
  // Region growing tires the whole cloud apart. Based on that we judge each part by mean z value
  // to determine whether some parts with similiar z value come from the same plane
  getMeanZofEachCluster(cloud_norm_fit_);
  
  ///time used:10%
  
  // Extract planes from the points with similar mean z,
  // the planes are stored in vector plane_hull_ with its coeff stored in plane_coeff_
  extractPlaneForEachZ(cloud_norm_fit_);
  
  ///time used:55%
  // Stop timer and get total processing time
  hst_.stop();
  hst_.print();
  
  
  // Visualize the result
  visualizeResult();
}

void PlaneSegment::calRegionGrowing(PointCloudRGBN::Ptr cloud_in, 
                                    pcl::PointCloud<pcl::Normal>::Ptr normals)
{
  pcl::RegionGrowing<pcl::PointXYZRGBNormal, pcl::Normal> reg;
  pcl::search::Search<pcl::PointXYZRGBNormal>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGBNormal> > 
      (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  
  reg.setMinClusterSize(30);
  reg.setMaxClusterSize(307200);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(20);
  reg.setInputCloud(cloud_in);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(th_smooth_ / 180.0 * M_PI);
  
  reg.extract(rg_cluster_indices_);
}

void PlaneSegment::getMeanZofEachCluster(PointCloudRGBN::Ptr cloud_norm_fit)
{
  if (rg_cluster_indices_.empty())
    ROS_DEBUG("PlaneSegment: Region growing get nothing.");
  
  else {
    size_t k = 0;
    // Traverse each part to determine its mean Z
    for (vector<pcl::PointIndices>::const_iterator it = rg_cluster_indices_.begin(); 
         it != rg_cluster_indices_.end(); ++it) {
      PointCloudRGBN::Ptr cloud_fit_part(new PointCloudRGBN);
      
      pcl::PointIndices::Ptr idx_rg(new pcl::PointIndices);
      idx_rg->indices = it->indices;
      Utilities::getCloudByInliers(cloud_norm_fit, cloud_fit_part, idx_rg, false, false);
      
      float part_mean_z = Utilities::getCloudMeanZ(cloud_fit_part);
      plane_z_value_.push_back(part_mean_z);
      k++;
    }
    
    ROS_DEBUG("Hypothetic plane number: %d", plane_z_value_.size());
    // Z is ordered from small to large, i.e., low to high
    //sort(planeZVector_.begin(), planeZVector_.end());
  }
}

void PlaneSegment::extractPlaneForEachZ(PointCloudRGBN::Ptr cloud_norm_fit)
{
  size_t id = 0;
  for (vector<float>::iterator cit = plane_z_value_.begin(); 
       cit != plane_z_value_.end(); cit++) {
    extractPlane(id, *cit, cloud_norm_fit);
    id++;
  }
}

void PlaneSegment::extractPlane(size_t id, float z_in, PointCloudRGBN::Ptr &cloud_norm_fit)
{
  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
  // Plane function: ax + by + cz + d = 0, here coeff[3] = d = -cz
  coeff->values.push_back(0.0);
  coeff->values.push_back(0.0);
  coeff->values.push_back(1.0);
  coeff->values.push_back(-z_in);
  
  // Get projected cloud for clustering
  PointCloudMono::Ptr cloud_fit_proj(new PointCloudMono);
  
  // Points within certain distance (th_deltaz_) to the plane model defined by the coeff 
  // is projected onto the hypothesis plane, which is horizontal by its nature
  vector<int> fit_proj_inliers;
  Utilities::cutCloud(coeff, th_deltaz_, cloud_norm_fit, fit_proj_inliers, cloud_fit_proj);
  
  // Since there may be multiple planes having similar mean Z, we need do clustering
  // to merge adjacent clusters and divide distant clusters
  vector<pcl::PointIndices> fit_cluster_inliers;
  Utilities::clusterExtract(cloud_fit_proj, fit_cluster_inliers, th_cluster_, 4, 307200);
  
  cout << "Plane cluster number: " << fit_cluster_inliers.size() << " at z: " << z_in << endl;
  
  // Travese each merged clusters of indices to extract corresponding plane patch
  for (vector<pcl::PointIndices>::const_iterator it = fit_cluster_inliers.begin(); 
       it != fit_cluster_inliers.end (); ++it) {
    // Get the indices for the plane patch according to cloud_norm_fit
    pcl::PointIndices::Ptr idx_expand (new pcl::PointIndices);
    for (size_t e = 0; e < it->indices.size(); ++e) {
      idx_expand->indices.push_back(fit_proj_inliers[it->indices[e]]);
    }
    
    pcl::PointIndices::Ptr idx_rg (new pcl::PointIndices);
    idx_rg->indices = rg_cluster_indices_[id].indices;
    
    // The plane patch extracted with z should contain the original region
    // by which the z was calculated and the original region should major the whole region
    if (!Utilities::checkWithIn(idx_expand, idx_rg))
      continue;
    
    // Extract one plane patch
    PointCloudMono::Ptr cluster_near_z(new PointCloudMono);
    pcl::PointIndices::Ptr idx_fit(new pcl::PointIndices);
    idx_fit->indices = it->indices;
    Utilities::getCloudByInliers(cloud_fit_proj, cluster_near_z, idx_fit, false, false);
    //coeff->values[3] = - Utilities::getCloudMeanZ(cluster_near_z);
    
    // Filter out used points if they have been merged into bigger planes
    // The cloud_norm_fit still has same size since using keep organized
    PointCloudRGBN::Ptr temp(new PointCloudRGBN);
    Utilities::getCloudByInliers(cloud_norm_fit, temp, idx_expand, true, true);
    cloud_norm_fit = temp;
    
    // Use convex hull to represent the plane patch
    PointCloudMono::Ptr cloud_hull(new PointCloudMono);
    pcl::ConvexHull<pcl::PointXYZ> hull;
    pcl::PolygonMesh mesh;
    hull.setInputCloud(cluster_near_z);
    hull.setComputeAreaVolume(true);
    hull.reconstruct(*cloud_hull);
    hull.reconstruct(mesh);
    
    float area_hull = hull.getTotalArea();
    
    // Small plane is filtered here with threshold th_area_
    if (cloud_hull->points.size() > 2 && area_hull > th_area_) {
      /* Select the plane which has similar th_height and
       * largest plane to be the output, notice that z_in is in base_link frame
       * th_height_ is the height of table, base_link is 0.4m above the ground */
      if (area_hull > global_area_temp_) {
        plane_max_hull_ = cloud_hull;
        plane_max_mesh_ = mesh;
        plane_max_coeff_ = coeff;
        // Update temp
        global_area_temp_ = area_hull;
      }
      
      ROS_DEBUG("Found plane with area %f.", area_hull);
      plane_coeff_.push_back(coeff);
      plane_points_.push_back(cluster_near_z);
      plane_hull_.push_back(cloud_hull);
      plane_mesh_.push_back(mesh);
    }
  }
}

void PlaneSegment::visualizeResult()
{
  // For visualizing in RViz
  publishCloud(src_rgb_cloud_, pub_cloud_);
  publishCloud(plane_max_hull_, pub_max_plane_);
  
  // Clear temps
  viewer->removeAllPointClouds();
  viewer->removeAllShapes();
  viewer->initCameraParameters();
  
  string name;
  /// Point size must be set after adding point cloud
  // Add source colored cloud for reference
  Utilities::generateName(0, "source_", "", name);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> src_rgb(src_rgb_cloud_);
  viewer->addPointCloud<pcl::PointXYZRGB>(src_rgb_cloud_, src_rgb, name);
  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4.0, name);
  //viewer->addPointCloudNormals<pcl::PointXYZRGBNormal, pcl::Normal>(src_rgbn_cloud_, src_normals_, 10, 0.05, "src_normals");
  //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.9, name);
  
  // Add normal filtered cloud as reference
  //  Utilities::generateName(0, "norm_", "", name);
  //  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBNormal> src_rgbn(cloud_norm_fit_);
  //  viewer->addPointCloud<pcl::PointXYZRGBNormal>(cloud_norm_fit_, src_rgbn, name);
  //  viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4.0, name);
  
  for (size_t i = 0; i < plane_hull_.size(); i++) {
    // Add hull points
    //Utilities::generateName(i, "hull_", "", name);
    //pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> rgb(plane_hull_[i]);
    //viewer->addPointCloud<pcl::PointXYZ>(plane_hull_[i], rgb, name);
    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10.0, name);
    
    // Add plane points
    //Utilities::generateName(i, "plane_", "", name);
    //pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> rgb(plane_points_[i]);
    //viewer->addPointCloud<pcl::PointXYZ>(plane_points_[i], rgb, name);
    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6.0, name);
    
    // Add hull polygons
    Utilities::generateName(i, "poly_", "", name);    
    viewer->addPolygonMesh(plane_mesh_[i], name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, name);
    double red = 0;
    double green = 0;
    double blue = 0;;
    pcl::visualization::getRandomColors(red, green, blue);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, red, green, blue, name);
    
    // Add model
    //NormalCloud::Ptr normals(new NormalCloud);
    //normals->height = plane_points_[i]->height;
    //normals->width  = plane_points_[i]->width;
    //normals->is_dense = true;
    //normals->resize(normals->height * normals->width);
    //for (size_t j = 0; j < normals->size(); ++j) {
    // normals->points[j].normal_x = 0;
    //  normals->points[j].normal_y = 0;
    //  normals->points[j].normal_z = 1;
    //}
    //pcl::PolygonMesh m = Utilities::generateMesh(plane_points_[i], normals);
    //Utilities::generateName(i, "model_", "", name);
    //const double kTableThickness = 0.02;
    //viewer->addCube(input.x_min, input.x_max, input.y_min, input.y_max,
    //                input.table_height - kTableThickness, input.table_height, 1.0, 0.0, 0.0,
    //                "support_surface");
    //viewer->addPolygonMesh(m, name);
    //viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY,
    //                                    0.6, name);
    //viewer->setShapeRenderingProperties(
    //      pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
    //      pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
    //      name);
  }
  
  viewer->addCoordinateSystem(0.5);
  
  while (!viewer->wasStopped())
    viewer->spinOnce();
}

template <typename PointTPtr>
void PlaneSegment::publishCloud(PointTPtr cloud, ros::Publisher pub)
{
  sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(*cloud, ros_cloud);
  ros_cloud.header.frame_id = base_frame_;
  ros_cloud.header.stamp = ros::Time(0);
  pub.publish(ros_cloud);
}

void PlaneSegment::poisson_reconstruction(NormalPointCloud::Ptr point_cloud, 
                                          pcl::PolygonMesh& mesh)
{
  // Initialize poisson reconstruction
  pcl::Poisson<pcl::PointNormal> poisson;
  
  /*
     * Set the maximum depth of the tree used in Poisson surface reconstruction.
     * A higher value means more iterations which could lead to better results but
     * it is also more computationally heavy.
     */
  poisson.setDepth(10);
  poisson.setInputCloud(point_cloud);
  
  // Perform the Poisson surface reconstruction algorithm
  poisson.reconstruct(mesh);
}

/**
 * Reconstruct a point cloud to a mesh by estimate the normals of the point cloud
 *
 * @param point_cloud The input point cloud that will be reconstructed
 * @return Returns a reconstructed mesh
 */
pcl::PolygonMesh PlaneSegment::mesh(const PointCloudMono::Ptr point_cloud, 
                                    NormalCloud::Ptr normals)
{
  // Add the normals to the point cloud
  NormalPointCloud::Ptr cloud_with_normals(new NormalPointCloud);
  pcl::concatenateFields(*point_cloud, *normals, *cloud_with_normals);
  
  // Point cloud to mesh reconstruction
  pcl::PolygonMesh mesh;
  poisson_reconstruction(cloud_with_normals, mesh);
  
  return mesh;
}
