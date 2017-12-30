#include "plane_segment.h"

#include <tf2/LinearMath/Quaternion.h>

// Normal threshold, let (nx, ny, nz) be the unit normal vector of point p,
// |nz| > th_norm_ means p is from plane, notice p is transformed with camera orientation
float th_norm_ = 0.7;

// Region growing threshold
float th_smooth_ = 8;

// Voxel grid threshold
float th_leaf_ = 0.01;

// Points whose z value are within the range defined by th_deltaz_ are considered in plane
// th_deltaz_ must be 2 times bigger than th_leaf_
float th_deltaz_ = 2 * th_leaf_;

// Distance threshold for plane patch clustering
float th_cluster_ = 2 * th_leaf_;

// Depth threshold for filtering source cloud, only used for real data
float th_max_depth_ = 8.0;

PlaneSegment::PlaneSegment(bool use_real_data, string base_frame, float base_to_ground) :
  use_real_data_(use_real_data),
  fi_(new FetchRGBD),
  pub_it_(nh_),
  src_mono_cloud_(new PointCloudMono),
  src_rgb_cloud_(new PointCloud),
  src_z_inliers_(new pcl::PointIndices),
  m_tf_(new Transform),
  base_frame_(base_frame),
  base_link_above_ground_(base_to_ground),
  th_height_(0.2),
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
  
  
  cloud_viewer->setBackgroundColor(0, 0, 0);
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
    Utilities::getCloudByZ(temp, src_z_inliers_, src_mono_cloud_, 0.3, th_max_depth_);
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
  plane_coeff_.clear();
  plane_hull_.clear();
  global_area_temp_ = 0.0;
  global_height_temp_ = 0.0;
  
  // Calculate the normal of source cloud
  PointCloudRGBN::Ptr src_norm(new PointCloudRGBN);
  Utilities::estimateNorm(src_mono_cloud_, src_norm, 2*th_leaf_, th_leaf_, true);
  
  // Extract all points whose norm indicates that the point belongs to plane
  pcl::PointIndices::Ptr idx_norm_fit(new pcl::PointIndices);
  Utilities::getCloudByNorm(src_norm, idx_norm_fit, th_norm_);
  
  if (idx_norm_fit->indices.empty()) {
    ROS_DEBUG("PlaneSegment: No point normal fit horizontal plane.");
    return;
  }
  
  PointCloudRGBN::Ptr cloud_norm_fit(new PointCloudRGBN);
  Utilities::getCloudByInliers(src_norm, cloud_norm_fit, idx_norm_fit, false, false);
  
  //PointCloudMono::Ptr cloud_mono_fit(new PointCloudMono);
  //Utilities::pointTypeTransfer(cloud_norm_fit, cloud_mono_fit);
  
  
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
  vector<pcl::PointIndices> clusters;
  calRegionGrowing(cloud_norm_fit, norm_fit, clusters);
  
  // Region growing tires the whole cloud apart. Based on that we judge each part by mean z value
  // to determine whether some parts with similiar z value come from the same plane
  getMeanZofEachCluster(clusters, cloud_norm_fit);
  
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
                                    pcl::PointCloud<pcl::Normal>::Ptr normals,
                                    vector<pcl::PointIndices> &clusters)
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
  
  reg.extract(clusters);
}

void PlaneSegment::getMeanZofEachCluster(vector<pcl::PointIndices> indices_in, 
                                         PointCloudRGBN::Ptr cloud_in)
{
  if (indices_in.empty())
    ROS_DEBUG("PlaneSegment: Region growing get nothing.");
  
  else {
    size_t k = 0;
    // Traverse each part to determine its mean Z
    for (vector<pcl::PointIndices>::const_iterator it = indices_in.begin(); 
         it != indices_in.end(); ++it) {
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
    sort(planeZVector_.begin(), planeZVector_.end());
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
  
//  // Remove the farest point in each loop
//  PointCloudMono::Ptr cloud_local_temp (new PointCloudMono);
//  PointCloudMono::Ptr cloud_temp (new PointCloudMono);
//  cloud_temp = cloud_in;
  
//  float mid, zrange;
//  while (cloud_temp->points.size() > 2) {
//    float dis_high = 0.0;
//    float dis_low = 0.0;
//    int max_high = -1;
//    int max_low = -1;
//    pcl::PointIndices::Ptr pointToRemove(new pcl::PointIndices);
//    Utilities::getAverage(cloud_temp, mid, zrange);
//    if (zrange <= th_deltaz_)
//      break;
    
//    // Remove both upper and bottom points
//    size_t ct = 0;
//    for (PointCloudMono::const_iterator pit = cloud_temp->begin();
//         pit != cloud_temp->end();++pit) {
//      float dis = pit->z - mid;
//      if (dis - dis_high >= th_leaf_) {
//        dis_high = dis;
//        max_high = ct;
//      }
//      if (dis - dis_low <= - th_leaf_) {
//        dis_low = dis;
//        max_low = ct;
//      }
//      ct++;
//    }
//    if (max_low < 0 && max_high < 0)
//      break;
//    if (max_high >= 0)
//      pointToRemove->indices.push_back(max_high);
//    if (max_low >= 0)
//      pointToRemove->indices.push_back(max_low);
    
//    Utilities::getCloudByInliers(cloud_temp, cloud_local_temp, 
//                                 pointToRemove, true, false);
//    cloud_temp = cloud_local_temp;
//  }
//  return mid;
}

void PlaneSegment::extractPlaneForEachZ(PointCloudRGBN::Ptr cloud_norm_fit)
{
  for (vector<float>::iterator cit = planeZVector_.begin(); 
       cit != planeZVector_.end(); cit++) {
    extractPlane(*cit, cloud_norm_fit);
  }
}

void PlaneSegment::extractPlane(float z_in, PointCloudRGBN::Ptr cloud_norm_fit)
{
  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
  // Plane function: ax + by + cz + d = 0, here coeff[3] = d = -cz
  coeff->values.push_back(0.0);
  coeff->values.push_back(0.0);
  coeff->values.push_back(1.0);
  coeff->values.push_back(-z_in);
  
  // Use source cloud as input, get projected cloud for clustering
  PointCloudMono::Ptr cloud_projected(new PointCloudMono);
  // Points within certain distance (th_deltaz_) to the plane model defined by the coeff 
  // is projected onto the hypothesis plane, which is horizontal by its nature
  // TODO:consider using source_cloud instead of norm fit cloud here
  //Utilities::cutCloud(coeff, th_deltaz_, cloud_norm_fit, cloud_projected);
  Utilities::cutCloud(coeff, th_deltaz_, src_mono_cloud_, cloud_projected);
  
  // Since there may be multiple planes having similar mean Z, we need do clustering
  // to merge adjacent clusters and divide distant clusters
  vector<pcl::PointIndices> cluster_indices;
  Utilities::clusterExtract(cloud_projected, cluster_indices, th_cluster_, 3, 307200);
  
  cout << "Plane cluster number: " << cluster_indices.size() << " at z: " << z_in << endl;
  
  // Travese each merged clusters of indices to extract corresponding plane patch
  for (vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); 
       it != cluster_indices.end (); ++it) {
    PointCloudMono::Ptr cloud_near_z(new PointCloudMono);
    for (vector<int>::const_iterator pit = it->indices.begin(); 
         pit != it->indices.end(); ++pit)
      cloud_near_z->points.push_back(cloud_projected->points[*pit]);
    
    cloud_near_z->width = cloud_near_z->points.size();
    cloud_near_z->height = 1;
    cloud_near_z->is_dense = true;
    
    // Use convex hull to represent the plane patch
    PointCloudMono::Ptr cloud_hull(new PointCloudMono);
    pcl::ConvexHull<pcl::PointXYZ> hull;
    hull.setInputCloud(cloud_near_z);
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
  publishCloud(plane_hull_[10], pub_max_plane_);
  
  cloud_viewer->removeAllPointClouds();
  
  // Add source colored cloud for reference
  string name;
//  Utilities::generateName(0, "source_", "", name);
//  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> src_rgb(src_rgb_cloud_);
//  cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 100, name);
//  if (!cloud_viewer->updatePointCloud(src_rgb_cloud_, src_rgb, name)) {
//    cloud_viewer->addPointCloud(src_rgb_cloud_, src_rgb, name);
//  }
  
  for (size_t i = 0; i < plane_hull_.size(); i++) {
    pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ> rgb(plane_hull_[i]);
    
    Utilities::generateName(i, "plane_", "", name);
    cloud_viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, name);
    
    if (!cloud_viewer->updatePointCloud(plane_hull_[i], rgb, name)) {
      cloud_viewer->addPointCloud(plane_hull_[i], rgb, name);
    }
  }
  
  cloud_viewer->addCoordinateSystem(0.5);
  cloud_viewer->initCameraParameters();
  
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
