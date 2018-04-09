#include "plane_segment.h"

#include <tf2/LinearMath/Quaternion.h>

/// Parameters of HOPE 
float th_grid_rsl_ = 0.015; // Resolution in XY direction
float th_z_rsl_ = 0.002; // Resolution in Z direction

/// Calculated parameters
float th_theta_;
float th_angle_;
float th_norm_;

// Depth threshold for filtering source cloud, only used for real data
float th_min_depth_ = 0.3;
float th_max_depth_ = 8.0;

bool cal_hull_ = false;
bool show_cluster_ = false;
bool show_egi_ = false;

HighResTimer hst_1_("pca");
HighResTimer hst_2_("ransac");

PlaneSegment::PlaneSegment(string base_frame, float th_xy, float th_z) :
  fi_(new FetchRGBD),
  type_(REAL),
  pub_it_(nh_),
  src_mono_cloud_(new PointCloudMono),
  src_rgb_cloud_(new PointCloud),
  cloud_norm_fit_mono_(new PointCloudMono),
  cloud_norm_fit_(new NormalCloud),
  src_sp_mono_(new PointCloudMono),
  src_sp_rgb_(new PointCloud),
  src_normals_(new NormalCloud),
  idx_norm_fit_(new pcl::PointIndices),
  src_z_inliers_(new pcl::PointIndices),
  tf_(new Transform),
  utl_(new Utilities),
  base_frame_(base_frame),
  viewer(new pcl::visualization::PCLVisualizer("HOPE Result")),
  hst_("total")
{
  th_grid_rsl_ = th_xy;
  th_z_rsl_ = th_z;
  th_theta_ = th_z_rsl_ / th_grid_rsl_;
  th_angle_ = atan(th_theta_);
  th_norm_ = sqrt(1 / (1 + 2 * pow(th_theta_, 2)));
  
  // For store max hull id and area
  global_size_temp_ = 0;
  
  // Regist the callback if using real point cloud data
  sub_pointcloud_ = nh_.subscribe<sensor_msgs::PointCloud2>("/camera/depth/points", 1,
                                                            &PlaneSegment::cloudCallback, this);
  
  // Detect table obstacle
  pub_max_plane_ = nh_.advertise<sensor_msgs::PointCloud2>("/vision/max_plane", 1, true);
  pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("/vision/points", 1, true);
  pub_max_mesh_ = nh_.advertise<geometry_msgs::PolygonStamped>("/vision/max_mesh",1, true);
  
  viewer->setBackgroundColor(0.8, 0.83, 0.86);
  viewer->initCameraParameters();
  viewer->setCameraPosition(1,0,2,0,0,1);
  viewer->addCoordinateSystem(0.1);
}

void PlaneSegment::visualizeProcess(PointCloud::Ptr cloud)
{
  // Clear temps
  viewer->removeAllPointClouds();
  viewer->removeAllShapes();
  string name;

  /// Point size must be set AFTER adding point cloud
  // Add source colored cloud for reference
  name = utl_->getName(0, "source_", -1);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> src_rgb(cloud);
  if (!viewer->updatePointCloud(cloud, src_rgb, name)){
    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, src_rgb, name);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, name);
  }

  while (!viewer->wasStopped()) {
    viewer->spinOnce(1);
  }
}

void PlaneSegment::setRPY(float roll = 0.0, float pitch = 0.0, float yaw = 0.0)
{
  roll_ = roll;
  pitch_ = pitch;
  yaw_ = yaw;
}

void PlaneSegment::setQ(float qx = 0.0, float qy = 0.0,
                        float qz = 0.0, float qw = 1.0)
{
  qx_ = qx;
  qy_ = qy;
  qz_ = qz;
  qw_ = qw;
}

void PlaneSegment::setT(float tx = 0.0, float ty = 0.0, float tz = 0.0)
{
  tx_ = tx;
  ty_ = ty;
  tz_ = tz;
}

// Notice that the point cloud may not transformed before this function
void PlaneSegment::getHorizontalPlanes(PointCloud::Ptr cloud)
{
  PointCloud::Ptr temp(new PointCloud);
  if (type_ == REAL || type_ == SYN) {
    // If using real data, the transform from camera frame to base frame
    // need to be provided
    getSourceCloud();
  }
  else if (type_ == POINT_CLOUD) {
    src_rgb_cloud_ = cloud;
  }
  else if (type_ >= TUM_SINGLE) {
    // To remove Nan and unreliable points with z value
    utl_->getCloudByZ(cloud, src_z_inliers_, temp, th_min_depth_, th_max_depth_);
    //visualizeProcess(temp);

    if (type_ <= TUM_LIST) {
      tf_->doTransform(temp, src_rgb_cloud_, tx_, ty_, tz_, qx_, qy_, qz_, qw_);
      //tf_->doTransform(temp, src_rgb_cloud_, 0, 0, 0, qx_, qy_, qz_, qw_);
    }
    else {
      tf_->doTransform(temp, src_rgb_cloud_, roll_, pitch_, yaw_);
    }
  }

  utl_->pointTypeTransfer(src_rgb_cloud_, src_mono_cloud_);

  //visualizeProcess(src_rgb_cloud_);
  //pcl::io::savePCDFile("/home/omnisky/src.pcd", *src_rgb_cloud_);
  
  // Down sampling
  if (th_grid_rsl_ > 0 && th_z_rsl_ > 0) {
    utl_->downSampling(src_mono_cloud_, src_sp_mono_, th_grid_rsl_, th_z_rsl_);
    utl_->downSampling(src_rgb_cloud_, src_sp_rgb_, th_grid_rsl_, th_z_rsl_);
  }
  else {
    src_sp_mono_ = src_mono_cloud_;
    src_sp_rgb_ = src_rgb_cloud_;
  }
  //visualizeProcess(src_sp_rgb_);
  cout << "Point number after down sampling: #" << src_sp_rgb_->points.size() << endl;

  if (src_sp_mono_->points.empty()) {
    ROS_WARN("PlaneSegment: Source cloud is empty.");
    return;
  }

  // Clear temp and get the candidates of horizontal plane points using normal
  reset();
  computeNormalAndFilter();

  //pcl::io::savePCDFile("/home/omnisky/normal_filter.pcd", *cloud_norm_fit_mono_);
  
  // Start timer
  hst_.start();
  
  findAllPlanes();
  //findAllPlanesRANSAC(true, 500, 1.01*th_grid_rsl_, 0.001);
  //findAllPlanesRG(20, 20, 8.0, 1.0);
  
  // Stop timer and get total processing time
  hst_.stop();
  hst_.print();

  setID();
  visualizeResult(true, true, false, cal_hull_);
}

void PlaneSegment::findAllPlanesRG(int norm_k, int num_n, float s_th, float c_th)
{
  cout << "Threshold for RG:" << endl;
  cout << "K search for normal computation: " << norm_k << endl;
  cout << "RG number of neighbours: " << num_n << endl;
  cout << "smooth threshold: " << s_th << endl;
  cout << "curvature threshold: " << c_th << endl;

  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >
      (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
  normal_estimator.setSearchMethod(tree);
  normal_estimator.setInputCloud(cloud_norm_fit_mono_);
  normal_estimator.setKSearch(norm_k);
  normal_estimator.compute(*normals);

  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
  reg.setMinClusterSize(3);
  reg.setMaxClusterSize(INT_MAX);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(num_n);
  reg.setInputCloud(cloud_norm_fit_mono_);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(s_th / 180.0 * M_PI);
  reg.setCurvatureThreshold(c_th);
  vector<pcl::PointIndices> clusters;
  reg.extract(clusters);
  //cout << "Number of clusters: " << clusters.size () << endl;

  for (size_t i = 0; i < clusters.size(); ++i) {
    PointCloudMono::Ptr cloud_p (new PointCloudMono);
    pcl::PointIndices::Ptr idx_seed(new pcl::PointIndices);
    idx_seed->indices = clusters[i].indices;
    utl_->getCloudByInliers(cloud_norm_fit_mono_, cloud_p, idx_seed, false, false);
    plane_points_.push_back(cloud_p);

    float avr_z, delta_z;
    utl_->getAverage(cloud_p, avr_z, delta_z);
    setFeatures(avr_z, cloud_p);
  }
}

void PlaneSegment::findAllPlanes()
{
  zClustering(cloud_norm_fit_mono_); // -> seed_clusters_indices_
  getMeanZofEachCluster(cloud_norm_fit_mono_); // -> plane_z_values_
  extractPlaneForEachZ(cloud_norm_fit_mono_);
}

void PlaneSegment::findAllPlanesRANSAC(bool isOptimize, int maxIter,
                                       float disThresh, float omit)
{
  cout << "Threshold for RANSAC:" << endl;
  cout << "Is optimize: " << isOptimize << endl;
  cout << "Max iteration: " << maxIter << endl;
  cout << "Distance threshold: " << disThresh << endl;
  cout << "Omit rate: " << omit << endl;

  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients(isOptimize);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(maxIter);
  seg.setDistanceThreshold(disThresh);

  int i = 0;
  int n_points = (int)cloud_norm_fit_mono_->points.size();

  while (cloud_norm_fit_mono_->points.size() > omit * n_points) {
    // Segment the largest planar component from the remaining cloud
    PointCloudMono::Ptr  cloud_p(new PointCloudMono);
    PointCloudMono::Ptr  cloud_f(new PointCloudMono);
    seg.setInputCloud(cloud_norm_fit_mono_);
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size() == 0) {
      cerr << "Could not estimate a planar model for the given dataset." << endl;
      break;
    }
    // Extract the inliers
    utl_->getCloudByInliers(cloud_norm_fit_mono_, cloud_p, inliers, false, false);
    cout << "PointCloud representing the planar component: " << cloud_p->points.size() << " data points." << endl;
    plane_points_.push_back(cloud_p);

    if (cal_hull_) {
      PointCloud::Ptr cloud_2d_rgb(new PointCloud);
      PointCloudMono::Ptr cloud_proj(new PointCloudMono);

      utl_->projectCloud(coefficients, cloud_p, cloud_proj);
      utl_->pointTypeTransfer(cloud_proj, cloud_2d_rgb, 100, 100, 100);
      computeHull(cloud_2d_rgb);
    }

    setFeatures(coefficients->values[3], cloud_p);

    //std::stringstream ss;
    //ss << "plane_" << i << ".pcd";
    //writer.write<pcl::PointXYZ>(ss.str(), *cloud_p, false);

    // Create the filtering object
    utl_->getCloudByInliers(cloud_norm_fit_mono_, cloud_f, inliers, true, false);
    cloud_norm_fit_mono_.swap(cloud_f);
    i++;
  }
}

void PlaneSegment::getMeanZofEachCluster(PointCloudMono::Ptr cloud_norm_fit_mono)
{
  if (seed_clusters_indices_.empty())
    ROS_DEBUG("PlaneSegment: Region growing get nothing.");
  
  else {
    size_t k = 0;
    // Traverse each part to determine its mean Z value
    for (vector<pcl::PointIndices>::const_iterator it = seed_clusters_indices_.begin();
         it != seed_clusters_indices_.end(); ++it) {
      PointCloudMono::Ptr cloud_fit_part(new PointCloudMono);
      
      pcl::PointIndices::Ptr idx_seed(new pcl::PointIndices);
      idx_seed->indices = it->indices;
      utl_->getCloudByInliers(cloud_norm_fit_mono, cloud_fit_part, idx_seed, false, false);
      
      if (show_cluster_) {
        string name = utl_->getName(k, "part_", -1);
        Vec3f c = utl_->getColorWithID(k);

        viewer->addPointCloud<pcl::PointXYZ>(cloud_fit_part, name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10.0, name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0.7, 0, name);
      }
      
      float part_mean_z = utl_->getCloudMeanZ(cloud_fit_part);
      //cout << "Cluster has " << idx_seed->indices.size() << " points at z: " << part_mean_z << endl;
      plane_z_values_.push_back(part_mean_z);
      k++;
    }
    
    ROS_DEBUG("Hypothetic plane number: %d", int(plane_z_values_.size()));
    // Z is ordered from small to large, i.e., low to high
    //sort(planeZVector_.begin(), planeZVector_.end());
  }
}

void PlaneSegment::zClustering(PointCloudMono::Ptr cloud_norm_fit_mono)
{
  ZGrowing zg;
  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >
      (new pcl::search::KdTree<pcl::PointXYZ>);
  
  zg.setMinClusterSize(3);
  zg.setMaxClusterSize(INT_MAX);
  zg.setSearchMethod(tree);
  zg.setNumberOfNeighbours(8);
  zg.setInputCloud(cloud_norm_fit_mono);
  zg.setZThreshold(th_z_rsl_);
  
  zg.extract(seed_clusters_indices_);
}

void PlaneSegment::extractPlaneForEachZ(PointCloudMono::Ptr cloud_norm_fit)
{
  size_t id = 0;
  for (vector<float>::iterator cit = plane_z_values_.begin();
       cit != plane_z_values_.end(); cit++) {
    getPlane(id, *cit, cloud_norm_fit);
    id++;
  }
}

void PlaneSegment::getPlane(size_t id, float z_in, PointCloudMono::Ptr &cloud_norm_fit_mono)
{
  pcl::ModelCoefficients::Ptr cluster_coeff(new pcl::ModelCoefficients);
  // Plane function: ax + by + cz + d = 0, here coeff[3] = d = -cz
  cluster_coeff->values.push_back(0.0);
  cluster_coeff->values.push_back(0.0);
  cluster_coeff->values.push_back(1.0);
  cluster_coeff->values.push_back(-z_in);
  
  pcl::PointIndices::Ptr idx_seed (new pcl::PointIndices);
  idx_seed->indices = seed_clusters_indices_[id].indices;
  
  // Extract the plane points indexed by idx_seed
  PointCloudMono::Ptr cluster_near_z(new PointCloudMono);
  utl_->getCloudByInliers(cloud_norm_fit_mono, cluster_near_z, idx_seed, false, false);
  
  // If the points do not pass the error test, return
  PointCloud::Ptr cluster_2d_rgb(new PointCloud);
  if (!gaussianImageAnalysis(id)) return;
  
  // If the cluster of points pass the check,
  // push it and corresponding projected points into resulting vectors
  plane_results_.push_back(cluster_2d_rgb);
  plane_points_.push_back(cluster_near_z);
  
  // Use convex hull to represent the plane patch
  if (cal_hull_) {
    computeHull(cluster_2d_rgb);
  }

  setFeatures(z_in, cluster_near_z);
  
  // Update the data of the max plane detected
  //  if (cluster_2d_rgb->points.size() > global_size_temp_) {
  //    plane_max_result_ = cluster_2d_rgb;
  //    plane_max_points_ = cluster_near_z;
  //    plane_max_coeff_ = feature;
  //    global_size_temp_ = cluster_2d_rgb->points.size();
  //  }
}

void PlaneSegment::computeHull(PointCloud::Ptr cluster_2d_rgb)
{
  PointCloud::Ptr cluster_hull(new PointCloud);
  pcl::ConvexHull<pcl::PointXYZRGB> hull;
  pcl::PolygonMesh cluster_mesh;

  hull.setInputCloud(cluster_2d_rgb);
  hull.setComputeAreaVolume(true);
  hull.reconstruct(*cluster_hull);
  hull.reconstruct(cluster_mesh);

  plane_hull_.push_back(cluster_hull);
  plane_mesh_.push_back(cluster_mesh);
  plane_max_hull_ = cluster_hull;
  plane_max_mesh_ = cluster_mesh;
}

void PlaneSegment::setFeatures(float z_in, PointCloudMono::Ptr cluster)
{
  // Prepare the feature vector for each plane to identify its id
  vector<float> feature;
  feature.push_back(z_in); // z value
  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D(*cluster, minPt, maxPt);
  feature.push_back(minPt.x); // cluster min x
  feature.push_back(minPt.y); // cluster min y
  feature.push_back(maxPt.x); // cluster max x
  feature.push_back(maxPt.y); // cluster max y
  plane_coeff_.push_back(feature);
}

bool PlaneSegment::gaussianImageAnalysis(size_t id)
{
  /// Get normal cloud for current cluster
  pcl::PointIndices::Ptr idx_seed (new pcl::PointIndices);
  idx_seed->indices = seed_clusters_indices_[id].indices;

  // Extract the plane points indexed by idx_seed
  NormalCloud::Ptr cluster_normal(new NormalCloud);
  utl_->getCloudByInliers(cloud_norm_fit_, cluster_normal, idx_seed, false, false);

  /// Construct a Pointcloud to store normal points
  if (show_egi_) {
    PointCloudMono::Ptr cloud(new PointCloudMono);
    cloud->width = cluster_normal->width;
    cloud->height = cluster_normal->height;
    cloud->resize(cluster_normal->width * cluster_normal->height);

    size_t k = 0;
    for (PointCloudMono::const_iterator pit = cloud->begin();
         pit != cloud->end(); ++pit) {
      cloud->points[k].x = cluster_normal->points[k].normal_x;
      cloud->points[k].y = cluster_normal->points[k].normal_y;
      cloud->points[k].z = fabs(cluster_normal->points[k].normal_z);
      k++;
    }
    pcl::visualization::PCLVisualizer viewer ("EGI and normals distribution");
    viewer.setBackgroundColor(0.8, 0.83, 0.86);
    viewer.addPointCloud(cloud, "normals");
    viewer.addCoordinateSystem(0.5);

    /// Use wire frame sphere
    //pcl::PointXYZ p;
    //p.x = 0; p.y = 0; p.z = 0;
    //viewer.addSphere(p, 1.0, 0, 0.6, 0.8, "EGI");
    //viewer.setRepresentationToWireframeForAllActors();

    /// Use point cloud sphere
    int bandwidth = 128;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere (new pcl::PointCloud<pcl::PointXYZRGB>);
    sphere->resize (4 * bandwidth * bandwidth);
    uint8_t r = 0, g = 0, b = 0;
    uint32_t rgb = 0;
    float tmp_theta = 0.0;
    float tmp_phi = 0.0;
    for (size_t i = 0; i < 2 * bandwidth; i++)
    {
      tmp_theta = (2 * i + 1) * M_PI / 4 / bandwidth;
      for (size_t j = 0; j < 2 * bandwidth; j++)
      {
        tmp_phi = M_PI * j / bandwidth;
        sphere->points[i * 2 * bandwidth + j].x = cos (tmp_phi) * sin (tmp_theta);
        sphere->points[i * 2 * bandwidth + j].y = sin (tmp_phi) * sin (tmp_theta);
        sphere->points[i * 2 * bandwidth + j].z = cos (tmp_theta);
        utl_->heatmapRGB(float(i)/bandwidth/4, r, g, b);
        rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
        sphere->points[i * 2 * bandwidth + j].rgb = *reinterpret_cast<float*> (&rgb);
      }
    }
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> sp_rgb(sphere);
    viewer.addPointCloud<pcl::PointXYZRGB>(sphere, sp_rgb, "egi");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "egi");

    // Show normal distribution on the sphere
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "normals");
    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0.4, 0, "normals");

    while (!viewer.wasStopped()) {
      viewer.spinOnce(1); // ms
    }
  }

  if (utl_->normalAnalysis(cluster_normal, th_angle_)) return true;
  else return false;
}

void PlaneSegment::setID()
{
  if (global_id_temp_.empty()) {
    // Initialize the global id temp with the first detection
    for (size_t i = 0; i < plane_coeff_.size(); ++i) {
      global_id_temp_.push_back(i);
      global_coeff_temp_.push_back(plane_coeff_[i]);
    }
  }
  else {
    vector<int> local_id_temp;
    utl_->matchID(global_coeff_temp_, plane_coeff_, global_id_temp_, local_id_temp, 5);

    // Update global result temp
    global_coeff_temp_.clear();
    global_id_temp_.clear();
    for (size_t i = 0; i < plane_coeff_.size(); ++i) {
      global_id_temp_.push_back(local_id_temp[i]);
      global_coeff_temp_.push_back(plane_coeff_[i]);
    }
  }
}

int PlaneSegment::checkSimiliar(vector<float> coeff)
{
  int id = -1;
  float distemp = FLT_MAX;
  for (size_t i = 0; i < global_coeff_temp_.size(); ++i) {
    vector<float> coeff_prev = global_coeff_temp_[i];
    float dis = utl_->getDistance(coeff_prev, coeff);
    if (dis < distemp) {
      distemp = dis;
      id = global_id_temp_[i];
    }
  }
  return id;
}

void PlaneSegment::visualizeResult(bool display_source, bool display_raw,
                                   bool display_err, bool display_hull)
{
  // For visualizing in RViz
  //publishCloud(src_rgb_cloud_, pub_cloud_);
  //publishCloud(plane_max_points_, pub_max_plane_);
  
  // Clear temps
  viewer->removeAllPointClouds();
  viewer->removeAllShapes();
  string name;
  
  /// Point size must be set AFTER adding point cloud
  if (display_source) {
    // Add source colored cloud for reference
    name = utl_->getName(0, "source_", -1);

    //viewer->addPointCloud<pcl::PointXYZRGB>(src_sp_rgb_, name);
    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, name);
    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, name);
    //viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (src_sp_rgb_, src_normals_, 1, 0.05, "normals");

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> src_rgb(src_sp_rgb_);
    if (!viewer->updatePointCloud(src_sp_rgb_, src_rgb, name)){
      viewer->addPointCloud<pcl::PointXYZRGB>(src_sp_rgb_, src_rgb, name);
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, name);
    }
  }
  
  for (size_t i = 0; i < plane_points_.size(); i++) {
    int id = global_id_temp_[i];
    Vec3f c = utl_->getColorWithID(id);
    if (display_raw) {
      // Add raw plane points
      name = utl_->getName(i, "plane_", -1);
      viewer->addPointCloud<pcl::PointXYZ>(plane_points_[i], name);
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10.0, name);
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, c[0], c[1], c[2], name);
    }
    if (display_err) {
      // Add results with error display
      name = utl_->getName(i, "error_", -1);
      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> err_rgb(plane_results_[i]);
      if (!viewer->updatePointCloud(plane_results_[i], err_rgb, name)){
        viewer->addPointCloud<pcl::PointXYZRGB>(plane_results_[i], err_rgb, name);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10.0, name);
      }
    }
    if (cal_hull_ && display_hull) {
      // Add hull points
      //name = utl_->getName(i, "hull_", -1);
      //viewer->addPointCloud<pcl::PointXYZRGB>(plane_hull_[i], name);
      //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10.0, name);
      //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, c[0], c[1], c[2], name);
      // Add hull mesh
      name = utl_->getName(i, "mesh_", -1);
      viewer->addPolygonMesh(plane_mesh_[i], name);
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.9, name);
      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, c[0], c[1], c[2], name);
    }
  }
  
  while (!viewer->wasStopped()) {
    viewer->spinOnce(1); // ms
    if (type_ == TUM_LIST || type_ == REAL || type_ == SYN)
      break;
  }
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

void PlaneSegment::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
{
  if (msg->data.empty()) {
    ROS_WARN_THROTTLE(31, "PlaneSegment: PointCloud is empty.");
    return;
  }
  PointCloud::Ptr src_temp(new PointCloud);
  PointCloud::Ptr temp(new PointCloud);

  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg, pcl_pc2);
  pcl::fromPCLPointCloud2(pcl_pc2, *src_temp);

  utl_->getCloudByZ(src_temp, src_z_inliers_, temp,
                    th_min_depth_, th_max_depth_);

  if (type_ == REAL) {
    tf_->getTransform(base_frame_, msg->header.frame_id);
    tf_->doTransform(temp, src_rgb_cloud_);
  }
  else {
    tf_->doTransform(temp, src_rgb_cloud_, roll_, pitch_, yaw_);
  }

  utl_->pointTypeTransfer(src_rgb_cloud_, src_mono_cloud_);
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

void PlaneSegment::reset()
{
  // Clear temp
  plane_results_.clear();
  plane_points_.clear();
  plane_coeff_.clear();
  plane_hull_.clear();
  plane_mesh_.clear();

  plane_z_values_.clear();
  cloud_fit_parts_.clear();
  seed_clusters_indices_.clear();

  global_size_temp_ = 0;
  // Reset timer
  hst_.reset();
}

bool PlaneSegment::getSourceCloud()
{
  while (ros::ok()) {
    if (!src_rgb_cloud_->points.empty())
      return true;

    // Handle callbacks and sleep for a small amount of time
    // before looping again
    ros::spinOnce();
    ros::Duration(0.005).sleep();
  }
}

void PlaneSegment::computeNormalAndFilter()
{
  utl_->estimateNorm(src_sp_mono_, src_normals_, 1.01 * th_grid_rsl_);
  utl_->getCloudByNorm(src_normals_, idx_norm_fit_, th_norm_);

  if (idx_norm_fit_->indices.empty()) return;

  utl_->getCloudByInliers(src_sp_mono_, cloud_norm_fit_mono_, idx_norm_fit_, false, false);
  utl_->getCloudByInliers(src_normals_, cloud_norm_fit_, idx_norm_fit_, false, false);
}
