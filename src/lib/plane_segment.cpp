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
float th_cluster_ = 2 * th_leaf_;

// Depth threshold for filtering source cloud, only used for real data
float th_max_depth_ = 8.0;

PlaneSegment::PlaneSegment(bool use_real_data, string base_frame, float th_area) :
  use_real_data_(use_real_data),
  fi_(new FetchRGBD),
  pub_it_(nh_),
  src_mono_cloud_(new PointCloudMono),
  src_rgb_cloud_(new PointCloud),
  src_filtered_(new PointCloudRGBN),
  src_normals_(new NormalCloud),
  src_z_inliers_(new pcl::PointIndices),
  m_tf_(new Transform),
  base_frame_(base_frame),
  th_area_(th_area),
  cloud_viewer(new pcl::visualization::PCLVisualizer("HOPE Result")),
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
  
  
  cloud_viewer->setBackgroundColor(0.2, 0.22, 0.24);
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
    if (dataset_type_ == 1) {
      m_tf_->doTransform(cloud, src_rgb_cloud_, tx_, ty_, tz_, qx_, qy_, qz_, qw_);
    }
    else {
      m_tf_->doTransform(cloud, src_rgb_cloud_, roll_, pitch_);
    }
    PointCloudMono::Ptr temp(new PointCloudMono);
    Utilities::pointTypeTransfer(src_rgb_cloud_, temp);
    // To remove Nan with z value
    Utilities::getCloudByZ(temp, src_z_inliers_, src_mono_cloud_, 0.0, th_max_depth_);
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
  PointCloud::Ptr temp(new PointCloud);
  pcl::fromPCLPointCloud2(pcl_pc2, *temp);
  
  PointCloudMono::Ptr temp_mono(new PointCloudMono);
  Utilities::pointTypeTransfer(temp, temp_mono);
  
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
  planeZVector_.clear();
  cloud_fit_parts_.clear();
  rg_cluster_indices_.clear();
  plane_coeff_.clear();
  plane_hull_.clear();
  global_area_temp_ = 0.0;
  global_height_temp_ = 0.0;
  
  // Calculate the normal of source cloud
  Utilities::estimateNorm(src_mono_cloud_, src_filtered_, src_normals_, 3*th_leaf_, th_leaf_, true);
  
  // Extract all points whose norm indicates that the point belongs to plane
  pcl::PointIndices::Ptr idx_norm_fit(new pcl::PointIndices);
  Utilities::getCloudByNorm(src_filtered_, idx_norm_fit, th_norm_);
  
  if (idx_norm_fit->indices.empty()) {
    ROS_DEBUG("PlaneSegment: No point normal fit horizontal plane.");
    return;
  }
  
  PointCloudRGBN::Ptr cloud_norm_fit(new PointCloudRGBN);
  Utilities::getCloudByInliers(src_filtered_, cloud_norm_fit, idx_norm_fit, false, false);
  
  //PointCloudMono::Ptr cloud_mono_fit(new PointCloudMono);
  //Utilities::pointTypeTransfer(cloud_norm_fit, cloud_mono_fit);
//  publishCloud(cloud_norm_fit, pub_max_plane_);
  
  if (cloud_norm_fit->points.empty()) {
    ROS_DEBUG("PlaneSegment: No point from horizontal plane detected.");
    return;
  }
  ROS_DEBUG("Points may from horizontal plane: %d", cloud_norm_fit->points.size());
  
  // Prepare normal data for clustering
  pcl::PointCloud<pcl::Normal>::Ptr norm_fit(new pcl::PointCloud<pcl::Normal>);
  norm_fit->resize(cloud_norm_fit->size());
  
  size_t i = 0;
  for (PointCloudRGBN::const_iterator pit = cloud_norm_fit->begin();
       pit != cloud_norm_fit->end(); ++pit) {
    norm_fit->points[i].normal_x = pit->normal_x;
    norm_fit->points[i].normal_y = pit->normal_y;
    norm_fit->points[i].normal_z = pit->normal_z;
    ++i;
  }
  // Perform clustering, cause the scene may contain multiple planes
  calRegionGrowing(cloud_norm_fit, norm_fit);
  
  // Region growing tires the whole cloud apart. Based on that we judge each part by mean z value
  // to determine whether some parts with similiar z value come from the same plane
  getMeanZofEachCluster(cloud_norm_fit);
  
  // Extract planes from the points with similar mean z,
  // the planes are stored in vector plane_hull_ with its coeff stored in plane_coeff_
  extractPlaneForEachZ(cloud_norm_fit);
  
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

void PlaneSegment::getMeanZofEachCluster(PointCloudRGBN::Ptr cloud_in)
{
  if (rg_cluster_indices_.empty())
    ROS_DEBUG("PlaneSegment: Region growing get nothing.");
  
  else {
    size_t k = 0;
    // Traverse each part to determine its mean Z
    for (vector<pcl::PointIndices>::const_iterator it = rg_cluster_indices_.begin(); 
         it != rg_cluster_indices_.end(); ++it) {
      PointCloudRGBN::Ptr cloud_fit_part(new PointCloudRGBN);
      int count = 0;
      
      // Traverse point id of each part to reorganize the part
      for (vector<int>::const_iterator pit = it->indices.begin(); 
           pit != it->indices.end(); ++pit) {
        cloud_fit_part->points.push_back(cloud_in->points[*pit]);
        count++;
      }
      // Reassemble the cloud part
      cloud_fit_part->width = cloud_fit_part->points.size();
      cloud_fit_part->height = 1;
      cloud_fit_part->is_dense = true;
      
      // Store initial segments of plane patches from region growing
      cloud_fit_parts_.push_back(cloud_fit_part);
      
      // Point type convention
      PointCloudMono::Ptr cloud_fit_part_t(new PointCloudMono);
      Utilities::pointTypeTransfer(cloud_fit_part, cloud_fit_part_t);
      
      // TODO: refine the mean z algorithm
      float part_mean_z = getCloudMeanZ(cloud_fit_part_t);
      planeZVector_.push_back(part_mean_z);
      k++;
    }
    
    ROS_DEBUG("Hypothetic plane number: %d", planeZVector_.size());
    // Z is ordered from small to large, i.e., low to high
    //sort(planeZVector_.begin(), planeZVector_.end());
  }
}

float PlaneSegment::getCloudMeanZ(PointCloudMono::Ptr cloud_in)
{
  // A simple yet easy to understand method
  float mid = 0.0;
  size_t ct = 0;
  for (PointCloudMono::const_iterator pit = cloud_in->begin();
       pit != cloud_in->end(); ++pit) {
    mid += pit->z;
    ct++;
  }
  return mid/ct;
}

void PlaneSegment::extractPlaneForEachZ(PointCloudRGBN::Ptr cloud_norm_fit)
{
  size_t id = 0;
  for (vector<float>::iterator cit = planeZVector_.begin(); 
       cit != planeZVector_.end(); cit++) {
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
  
  // Use source cloud as input, get projected cloud for clustering
  PointCloudMono::Ptr cloud_fit_proj(new PointCloudMono);
  PointCloudMono::Ptr cloud_src_proj(new PointCloudMono);
  // Points within certain distance (th_deltaz_) to the plane model defined by the coeff 
  // is projected onto the hypothesis plane, which is horizontal by its nature
  // TODO:consider using source_cloud instead of norm fit cloud here
  vector<int> proj_inliers;
  Utilities::cutCloud(coeff, th_deltaz_, cloud_norm_fit, proj_inliers, cloud_fit_proj);
  //Utilities::cutCloud(coeff, th_deltaz_, src_mono_cloud_, cloud_src_proj);
  
  // Since there may be multiple planes having similar mean Z, we need do clustering
  // to merge adjacent clusters and divide distant clusters
  vector<pcl::PointIndices> fit_proj_inliers;
  Utilities::clusterExtract(cloud_fit_proj, fit_proj_inliers, th_cluster_, 4, 307200);
  
  //vector<pcl::PointIndices> src_proj_inliers;
  //Utilities::clusterExtract(cloud_src_proj, src_proj_inliers, th_cluster_, 3, 307200);
  
  cout << "Plane cluster number: " << fit_proj_inliers.size() << " at z: " << z_in << endl;
  
  // Travese each merged clusters of indices to extract corresponding plane patch
  size_t ct = 0;
  for (vector<pcl::PointIndices>::const_iterator it = fit_proj_inliers.begin(); 
       it != fit_proj_inliers.end (); ++it) {
    // Get the indices for the plane patch
    pcl::PointIndices::Ptr idx_expand (new pcl::PointIndices);
    for (size_t e = 0; e < it->indices.size(); ++e) {
      idx_expand->indices.push_back(proj_inliers[it->indices[e]]);
    }
    
    pcl::PointIndices::Ptr idx_rg (new pcl::PointIndices);
    idx_rg->indices = rg_cluster_indices_[id].indices;
    
    // The plane patch extracted with z should contain the original region
    float rate = Utilities::checkWithIn(idx_expand, idx_rg);
    if (rate < 0.1)
      continue;
    
    // Extract one plane patch
    PointCloudMono::Ptr cluster_near_z(new PointCloudMono);
    for (vector<int>::const_iterator pit = it->indices.begin(); 
         pit != it->indices.end(); ++pit)
      cluster_near_z->points.push_back(cloud_fit_proj->points[*pit]);
    
    
    PointCloudRGBN::Ptr temp(new PointCloudRGBN);
    Utilities::getCloudByInliers(cloud_norm_fit, temp, idx_expand, true, true);
    cloud_norm_fit = temp;
    
    cluster_near_z->width = cluster_near_z->points.size();
    cluster_near_z->height = 1;
    cluster_near_z->is_dense = true;
    
    // Use convex hull to represent the plane patch
    PointCloudMono::Ptr cloud_hull(new PointCloudMono);
    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(cluster_near_z);
    hull.setComputeAreaVolume(true);
    hull.reconstruct(*cloud_hull);
    float area_hull = hull.getTotalArea();
    
    // Small plane is filtered here with threshold th_area_
    if (cloud_hull->points.size() > 2 && area_hull > th_area_) {
      /* Select the plane which has similar th_height and
       * largest plane to be the output, notice that z_in is in base_link frame
       * th_height_ is the height of table, base_link is 0.4m above the ground */
      if (area_hull > global_area_temp_) {
        plane_max_hull_ = cloud_hull;
        plane_max_coeff_ = coeff;
        // Update temp
        global_area_temp_ = area_hull;
      }
      
      ROS_DEBUG("Found plane with area %f.", area_hull);
      plane_coeff_.push_back(coeff);
      plane_hull_.push_back(cloud_hull);
    }
    
    // TODO:merge distant hulls if obstacle is presented
  }
}

void PlaneSegment::visualizeResult()
{
  // For visualizing in RViz
  publishCloud(src_rgb_cloud_, pub_cloud_);
  publishCloud(plane_max_hull_, pub_max_plane_);
  
  cloud_viewer->initCameraParameters();
  
  // Add source colored cloud for reference
  string name;
  Utilities::generateName(0, "source_", "", name);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> src_rgb(src_rgb_cloud_);
  
  // Point size must be set after adding point cloud
  cloud_viewer->addPointCloud<pcl::PointXYZRGB>(src_rgb_cloud_, src_rgb, name);
  //cloud_viewer->addPointCloudNormals<pcl::PointXYZRGBNormal, pcl::Normal>(src_filtered_, src_normals_, 10, 0.05, "src_normals");
  cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, name);
  
  for (size_t i = 0; i < plane_hull_.size(); i++) {
    Utilities::generateName(i, "plane_", "", name);
    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> rgb(plane_hull_[i]);
    cloud_viewer->addPointCloud<pcl::PointXYZ>(plane_hull_[i], rgb, name);
    cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10.0, name);
  }
  
  cloud_viewer->addCoordinateSystem(0.5);
  
  while (!cloud_viewer->wasStopped())
    cloud_viewer->spinOnce();
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
pcl::PolygonMesh PlaneSegment::mesh(const PointCloudMono::Ptr point_cloud, NormalCloud::Ptr normals)
{
  // Add the normals to the point cloud
  NormalPointCloud::Ptr cloud_with_normals(new NormalPointCloud);
  pcl::concatenateFields(*point_cloud, *normals, *cloud_with_normals);
  
  // Point cloud to mesh reconstruction
  pcl::PolygonMesh mesh;
  poisson_reconstruction(cloud_with_normals, mesh);
  
  return mesh;
}

NormalCloud::Ptr PlaneSegment::estimateNorm(PointCloudMono::Ptr cloud_in, float norm_r)
{
  NormalCloud::Ptr cloud_norm(new NormalCloud);
  
  /// Basic method
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud_in);
  
  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset 
  // (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(norm_r); // in meter
  
  // Compute the features
  ne.compute(*cloud_norm);
  
  /// Do it in parallel
  //  // Declare PCL objects needed to perform normal estimation
  //  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimation;
  //  pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZ>);
  
  //  // Set input parameters for normal estimation
  //  search_tree->setInputCloud(cloud_fit);
  //  normal_estimation.setInputCloud(cloud_fit);
  //  normal_estimation.setSearchMethod(search_tree);
  
  //  /*
  //   * When estimating normals, the algorithm looks at the nearest neighbors of every point
  //   * and fits a plane to these points as close as it can. The normal of this plane is
  //   * the estimated normal of the point.
  //   * This sets how many of the nearest neighbors to look at when estimating normals.
  //   * Is a rough setting for accuracy that can be adjusted.
  //   * A lower number here means that corners in the point cloud will be more accurate,
  //   * too low a number will cause problems.
  //   */
  //  normal_estimation.setKSearch(10);
  
  //  // Perform normal estimation algorithm
  //  normal_estimation.compute(*cloud_norm);
  
  // Reverse the direction of all normals so that the face of the object points outwards.
  // Should not be necessary but it is easier when visualising the object in MeshLab etc.
  for (size_t i = 0; i < cloud_norm->size(); ++i) {
    cloud_norm->points[i].normal_x *= -1;
    cloud_norm->points[i].normal_y *= -1;
    cloud_norm->points[i].normal_z *= -1;
  }
  return cloud_norm;
}
