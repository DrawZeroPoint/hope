#include <cmath>
#include "plane_segment.h"

#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/Polygon.h>
#include <hope/Subsections.h>

using namespace std;
using namespace cv;

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

PlaneSegment::PlaneSegment(Params params, ros::NodeHandle nh) :
  fi_(new FetchRGBD),
  type_(REAL),
  nh_(nh),
  pub_it_(nh_),
  x_dim_(params.x_dim),
  y_dim_(params.y_dim),
  z_min_(params.z_min),
  z_max_(params.z_max),
  viz(params.viz),
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
  base_frame_(params.base_frame),
  hst_("total"),
  min_area_(params.area_min),
  max_area_(params.area_max)
{
  th_grid_rsl_ = params.xy_resolution;
  th_z_rsl_ = params.z_resolution;
  th_theta_ = th_z_rsl_ / th_grid_rsl_;
  th_angle_ = atan(th_theta_);
  th_norm_ = sqrt(1 / (1 + 2 * pow(th_theta_, 2)));
  
  // For store max hull id and area
  global_size_temp_ = 0;

  polygon_pub_ = nh_.advertise<geometry_msgs::PolygonStamped>("polygon_array", 100);

  subsections_pub_ = nh_.advertise<hope::Subsections>("subsections_array", 100);
  
  // Register the callback if using real point cloud data
  sub_pointcloud_ = nh_.subscribe<sensor_msgs::PointCloud2>(params.cloud_topic, 1,
                                                            &PlaneSegment::cloudCallback, this);
  
  // Detect table obstacle
  pub_max_plane_ = nh_.advertise<sensor_msgs::PointCloud2>("vision/max_plane", 1, true);
  pub_cloud_ = nh_.advertise<sensor_msgs::PointCloud2>("vision/points", 1, true);
  pub_max_mesh_ = nh_.advertise<geometry_msgs::PolygonStamped>("vision/max_mesh",1, true);
  
  // Visualizer
  if(viz){    
    viewer = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer("HOPE Result"));
    viewer->setBackgroundColor(0.8, 0.83, 0.86);
    viewer->initCameraParameters();
    viewer->setCameraPosition(1,0,2,0,0,1);
    viewer->addCoordinateSystem(0.1);
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
void PlaneSegment::getHorizontalPlanes()
{
  PointCloud::Ptr temp(new PointCloud);
  if (type_ == REAL) {
    // If using real data, the transform from camera frame to base frame
    // need to be provided
    if (!getSourceCloud())
      return;
  } else {
    ROS_WARN("Not supported");
    return;
  }

  //ROS_INFO("If statement passed");
 // utl_->pointTypeTransfer(src_rgb_cloud_, src_mono_cloud_);

  //visualizeProcess(src_rgb_cloud_);
  //pcl::io::savePCDFile("~/src.pcd", *src_rgb_cloud_);
  
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
  ROS_DEBUG_STREAM("Point number after down sampling: #" << src_sp_rgb_->points.size());

  if (src_sp_mono_->points.empty()) {
    ROS_WARN("PlaneSegment: Source cloud is empty.");
    return;
  }

  // Clear temp and get the candidates of horizontal plane points using normal
  reset();
  computeNormalAndFilter();

  //pcl::io::savePCDFile("~/normal_filter.pcd", *cloud_norm_fit_mono_);
  
  // Start timer
  hst_.start();
  
  findAllPlanes();

  /* You can alternatively use RANSAC or Region Growing instead of HoPE
   * to carry out the comparision experiments in the paper
   */
  //findAllPlanesRANSAC(true, 500, 1.01*th_grid_rsl_, 0.001);
  //findAllPlanesRG(20, 20, 8.0, 1.0);
  
  // Stop timer and get total processing time
  // hst_.stop();
  // hst_.print();

  setID();
  if(viz){
    visualizeResult(true, true, false, cal_hull_);
  }
}

void PlaneSegment::findAllPlanes()
{
  zClustering(cloud_norm_fit_mono_); // -> seed_clusters_indices_
  getMeanZofEachCluster(cloud_norm_fit_mono_); // -> plane_z_values_
  extractPlaneForEachZ(cloud_norm_fit_mono_);
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

        if(viz){
          viewer->addPointCloud<pcl::PointXYZ>(cloud_fit_part, name);
          viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10.0, name);
          viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0.7, 0, name);
        }
      }
      
      float part_mean_z = utl_->getCloudMeanZ(cloud_fit_part);
      //cout << "Cluster has " << idx_seed->indices.size() << " points at z: " << part_mean_z << endl;
      plane_z_values_.push_back(part_mean_z);
      k++;
    }
    
    ROS_DEBUG("Hypothesis plane number: %d", int(plane_z_values_.size()));
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
  ROS_DEBUG("Polygon found");
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
  geometry_msgs::Polygon polygon_points;
  double area = (maxPt.y - minPt.y) * (maxPt.x - minPt.x);
  if (z_in < z_min_ || z_in > z_max_) {
    ROS_DEBUG("Plane ignored because of not within required height range");
    return;
  }
  else if (area < min_area_ || area > max_area_) {
  	ROS_DEBUG("Plane ignored because of not within required area range");
    return;
  }
  subsections_.subsection_array.clear();
  geometry_msgs::Point32 min_pts_bottom, min_pts_top, max_pts_bottom, max_pts_top;
  geometry_msgs::Point32 min_new, max_new;
  min_pts_bottom.x = minPt.x;
  min_pts_bottom.y = minPt.y;
  min_pts_bottom.z = z_in;
  min_pts_top.x = minPt.x;
  min_pts_top.y = maxPt.y;
  min_pts_top.z = z_in;
  max_pts_bottom.x = maxPt.x;
  max_pts_bottom.y = minPt.y;
  max_pts_bottom.z = z_in;
  max_pts_top.x = maxPt.x;
  max_pts_top.y = maxPt.y;
  max_pts_top.z = z_in;
  
  polygon_points.points.push_back(min_pts_bottom);
  polygon_points.points.push_back(min_pts_top);
  polygon_points.points.push_back(max_pts_top);
  polygon_points.points.push_back(max_pts_bottom);
  polygon_array_.header.frame_id = base_frame_;
  polygon_array_.polygon = polygon_points;
  polygon_array_.header.stamp = ros::Time::now();
  polygon_pub_.publish(polygon_array_);

  double length_x = std::abs(maxPt.x - minPt.x);
  double length_y = std::abs(maxPt.y - minPt.y);
  double x_mul_big = length_x / x_dim_; //Always > 0
  double y_mul_big = length_y / y_dim_; //Always > 0
  unsigned int x_mul_small = std::floor(x_mul_big);
  unsigned int y_mul_small = std::floor(y_mul_big);

  /*Now we break down the original polygon into subsections based on, x_dim_ and y_dim_.
    1.) length_x is perfectly divisible by x_mul_small; and 
        length_y is perfectly divisible by y_mul_small;
    2.) length_y > y_mul_small * y_dim_;
    3.) length_x > x_mul_small * x_dim_;
    4.) x_dim_ > length_x; and length_y >= y_mul_small * y_dim_;
    5.) y_dim_ > length_y; and length_x >= x_mul_small * x_dim_;
    6.) Both x_dim_ < length_x; and y_dim_ < length_y
  */

  // Cases 1, 2 and 3
  if (x_mul_big >= x_mul_small && y_mul_big >= y_mul_small && 
  	  x_mul_small >= 2 && y_mul_small >= 2) {
  	for (size_t i = 2; i <= x_mul_small; i++) {
  		for (size_t j = 2; j <= y_mul_small; j++) {
  			min_pts_bottom.x = minPt.x + x_dim_ * (i - 1);
  			min_pts_bottom.y = minPt.y + y_dim_ * (j - 1);
  			min_pts_top.x = minPt.x + x_dim_ * (i - 1);
  			min_pts_top.y = minPt.y + y_dim_ * j;
  			max_pts_bottom.x = minPt.x + x_dim_ * i;
  			max_pts_bottom.y = minPt.y + y_dim_ * (j - 1);
  			max_pts_top.x = minPt.x + x_dim_ * i;
  			max_pts_top.y = minPt.y + y_dim_ * j;
  			polygon_points.points.clear();
  			polygon_points.points.push_back(min_pts_bottom);
  			polygon_points.points.push_back(min_pts_top);
  			polygon_points.points.push_back(max_pts_bottom);
  			polygon_points.points.push_back(max_pts_top);
  			subsections_.subsection_array.push_back(polygon_points);
  		}
  		if (length_y > y_mul_small * y_dim_) {
  			min_pts_bottom.x = minPt.x + x_dim_ * (i - 1);
  			min_pts_bottom.y = minPt.y + y_dim_ * y_mul_small;
  			min_pts_top.x = minPt.x + x_dim_ * (i - 1);
  			min_pts_top.y = maxPt.y;
  			max_pts_bottom.x = minPt.x + x_dim_ * i;
  			max_pts_bottom.y = minPt.y + y_dim_ * y_mul_small;
  			max_pts_top.x = minPt.x + x_dim_ * i;
  			max_pts_top.y = maxPt.y;
  			polygon_points.points.clear();
  			polygon_points.points.push_back(min_pts_bottom);
  			polygon_points.points.push_back(min_pts_top);
  			polygon_points.points.push_back(max_pts_bottom);
  			polygon_points.points.push_back(max_pts_top);
  			subsections_.subsection_array.push_back(polygon_points);
  			}
  	}
  	if (length_x > x_mul_small * x_dim_) {
  		for (size_t j = 2; j <= y_mul_small; j++) {
  			min_pts_bottom.x = minPt.x + x_dim_ * x_mul_small;
  			min_pts_bottom.y = minPt.y + y_dim_ * (j - 1);
  			min_pts_top.x = minPt.x + x_dim_ * x_mul_small;
  			min_pts_top.y = minPt.y + y_dim_ * j;
  			max_pts_bottom.x = maxPt.x;
  			max_pts_bottom.y = minPt.y + y_dim_ * (j - 1);
  			max_pts_top.x = maxPt.x;
  			max_pts_top.y = minPt.y + y_dim_ * j;
  			polygon_points.points.clear();
  			polygon_points.points.push_back(min_pts_bottom);
  			polygon_points.points.push_back(min_pts_top);
  			polygon_points.points.push_back(max_pts_bottom);
  			polygon_points.points.push_back(max_pts_top);
  			subsections_.subsection_array.push_back(polygon_points);
  		}
  		if (length_y > y_mul_small * y_dim_) {
  			min_pts_bottom.x = minPt.x + x_dim_ * x_mul_small;
  			min_pts_bottom.y = minPt.y + y_dim_ * y_mul_small;
  			min_pts_top.x = minPt.x + x_dim_ * x_mul_small;
  			min_pts_top.y = maxPt.y;
  			max_pts_bottom.x = maxPt.x;
  			max_pts_bottom.y = minPt.y + y_dim_ * y_mul_small;
  			max_pts_top.x = maxPt.x;
  			max_pts_top.y = maxPt.y;
  			polygon_points.points.clear();
  			polygon_points.points.push_back(min_pts_bottom);
  			polygon_points.points.push_back(min_pts_top);
  			polygon_points.points.push_back(max_pts_bottom);
  			polygon_points.points.push_back(max_pts_top);
  			subsections_.subsection_array.push_back(polygon_points);
  			}
  		}
  }
  // Case 4
  else if (x_dim_ > length_x && y_mul_big >= y_mul_small && y_mul_small >= 2) {
  	for (size_t j = 2; j <= y_mul_small; j++) {
  		min_pts_bottom.x = minPt.x;
  		min_pts_bottom.y = minPt.y + y_dim_ * (j - 1);
  		min_pts_top.x = minPt.x;
  		min_pts_top.y = minPt.y + y_dim_ * j;
  		max_pts_bottom.x = maxPt.x;
  		max_pts_bottom.y = minPt.y + y_dim_ * (j - 1);
  		max_pts_top.x = maxPt.x;
  		max_pts_top.y = minPt.y + y_dim_ * j;
  		polygon_points.points.clear();
  		polygon_points.points.push_back(min_pts_bottom);
  		polygon_points.points.push_back(min_pts_top);
  		polygon_points.points.push_back(max_pts_bottom);
  		polygon_points.points.push_back(max_pts_top);
  		subsections_.subsection_array.push_back(polygon_points);
  	}
  	if (y_mul_big > y_mul_small) {
  		min_pts_bottom.x = minPt.x;
  		min_pts_bottom.y = minPt.y + y_dim_ * y_mul_small;
  		min_pts_top.x = minPt.x;
  		min_pts_top.y = maxPt.y;
  		max_pts_bottom.x = maxPt.x;
  		max_pts_bottom.y = minPt.y + y_dim_ * y_mul_small;
  		max_pts_top.x = maxPt.x;
  		max_pts_top.y = maxPt.y;
  		polygon_points.points.clear();
  		polygon_points.points.push_back(min_pts_bottom);
  		polygon_points.points.push_back(min_pts_top);
  		polygon_points.points.push_back(max_pts_bottom);
  		polygon_points.points.push_back(max_pts_top);
  		subsections_.subsection_array.push_back(polygon_points);
  	}
  }
  // Case 5
  else if (x_mul_big >= x_mul_small &&  x_mul_small >= 2 && y_dim_ > length_y) {
  	for (size_t i = 2; i <= x_mul_small; i++) {
  		min_pts_bottom.x = minPt.x + x_dim_ * (i - 1);
  		min_pts_bottom.y = minPt.y;
  		min_pts_top.x = minPt.x + x_dim_ * (i - 1);
  		min_pts_top.y = maxPt.y;
  		max_pts_bottom.x = minPt.x + x_dim_ * i;
  		max_pts_bottom.y = minPt.y;
  		max_pts_top.x = minPt.x + x_dim_ * i;
  		max_pts_top.y = maxPt.y;
  		polygon_points.points.clear();
  		polygon_points.points.push_back(min_pts_bottom);
  		polygon_points.points.push_back(min_pts_top);
  		polygon_points.points.push_back(max_pts_bottom);
  		polygon_points.points.push_back(max_pts_top);
  		subsections_.subsection_array.push_back(polygon_points);
  	}
  	if (x_mul_big > x_mul_small) {
  		min_pts_bottom.x = minPt.x + x_dim_ * x_mul_small;
  		min_pts_bottom.y = minPt.y;
  		min_pts_top.x = minPt.x + x_dim_ * x_mul_small;
  		min_pts_top.y = maxPt.y;
  		max_pts_bottom.x = maxPt.x;
  		max_pts_bottom.y = minPt.y;
  		max_pts_top.x = maxPt.x;
  		max_pts_top.y = maxPt.y;
  		polygon_points.points.clear();
  		polygon_points.points.push_back(min_pts_bottom);
  		polygon_points.points.push_back(min_pts_top);
  		polygon_points.points.push_back(max_pts_bottom);
  		polygon_points.points.push_back(max_pts_top);
  		subsections_.subsection_array.push_back(polygon_points);
  	}
  } 
  // Case 6
  else {
  	min_pts_bottom.x = minPt.x;
  	min_pts_bottom.y = minPt.y;
  	min_pts_top.x = minPt.x;
  	min_pts_top.y = maxPt.y;
  	max_pts_bottom.x = maxPt.x;
  	max_pts_bottom.y = minPt.y;
  	max_pts_top.x = maxPt.x;
  	max_pts_top.y = maxPt.y;
  	polygon_points.points.clear();
  	polygon_points.points.push_back(min_pts_bottom);
  	polygon_points.points.push_back(min_pts_top);
  	polygon_points.points.push_back(max_pts_bottom);
  	polygon_points.points.push_back(max_pts_top);
  	subsections_.subsection_array.push_back(polygon_points);
  }
  subsections_.header.frame_id = base_frame_;
  subsections_.header.stamp = ros::Time::now();
  subsections_pub_.publish(subsections_);
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
    
    if(viz){
      pcl::visualization::PCLVisualizer viewer_egi ("EGI and normals distribution");
      viewer_egi.setBackgroundColor(0.8, 0.83, 0.86);
      viewer_egi.addPointCloud(cloud, "normals");
      viewer_egi.addCoordinateSystem(0.5);

      viewer_egi.addPointCloud<pcl::PointXYZRGB>(sphere, sp_rgb, "egi");
      viewer_egi.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "egi");

      // Show normal distribution on the sphere
      viewer_egi.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "normals");
      viewer_egi.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0.4, 0, "normals");

      while (!viewer_egi.wasStopped()) {
        viewer_egi.spinOnce(1); // ms
      }
    }  
  }

  return utl_->normalAnalysis(cluster_normal, th_angle_);
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

  ROS_DEBUG_STREAM("Total plane patches #: " << plane_points_.size());

  while (!viewer->wasStopped()) {
    viewer->spinOnce(1); // ms
    if (type_ == TUM_LIST || type_ == REAL || type_ == SYN)
      break;
  }
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
{ return !src_rgb_cloud_->points.empty();
}

void PlaneSegment::computeNormalAndFilter()
{
  utl_->estimateNorm(src_sp_mono_, src_normals_, 1.01 * th_grid_rsl_);
  utl_->getCloudByNorm(src_normals_, idx_norm_fit_, th_norm_);

  if (idx_norm_fit_->indices.empty()) return;

  utl_->getCloudByInliers(src_sp_mono_, cloud_norm_fit_mono_, idx_norm_fit_, false, false);
  utl_->getCloudByInliers(src_normals_, cloud_norm_fit_, idx_norm_fit_, false, false);
}
