#include "hope/plane_segment.h"
#include <tf2/LinearMath/Quaternion.h>

#include <utility>

using namespace std;
using namespace cv;

//HighResTimer hst_1_("pca");
//HighResTimer hst_2_("ransac");


namespace hope {

  PlaneSegment::PlaneSegment(
      float th_xy,
      float th_z,
      data_type type,
      bool verbose
  ) :
      th_xy_(th_xy),
      th_z_(th_z),
      type_(type),
      src_cloud_xyz_(new Cloud_XYZ),
      src_cloud_xyzrgb_(new Cloud_XYZRGB),
      normal_fitted_cloud_xyz_(new Cloud_XYZ),
      normal_fitted_cloud_n_(new Cloud_N),
      src_cloud_n_(new Cloud_N),
      normal_fitted_indices_(new pcl::PointIndices),
      hst_("total"),
      verbose_(verbose)
  {
    th_theta_ = th_z_ / th_xy_;
    th_angle_ = atan(th_theta_);
    th_norm_ = sqrt(1. / (1. + 2. * pow(th_theta_, 2.)));

    if (verbose_) {
      viewer_ = std::make_shared<pcl::visualization::PCLVisualizer>("HoPE Result");
      viewer_->setBackgroundColor(0.8, 0.83, 0.86);
      viewer_->initCameraParameters();
      viewer_->setCameraPosition(1,0,2,0,0,1);
      viewer_->addCoordinateSystem(0.1);
    }
  }

  void PlaneSegment::getHorizontalPlanes(const Cloud_XYZRGB::Ptr &cloud) {
    src_cloud_xyzrgb_ = cloud;
    Cloud_XYZ::Ptr cloud_temp(new Cloud_XYZ);
    Utilities::convertCloudType(cloud, cloud_temp);
    getHorizontalPlanes(cloud_temp);
  }

  void PlaneSegment::getHorizontalPlanes(const Cloud_XYZ::Ptr& cloud)
  {
    // Start timer
    if (verbose_) hst_.start();

    PlaneSegmentResults results{};

    Utilities::downSampling(cloud, src_cloud_xyz_, th_xy_, th_z_);
    unsigned long n_point = src_cloud_xyz_->points.size();
    if (n_point == 0) {
      ROS_WARN("PlaneSegment: Cloud is empty after down sampling.");
      return;
    }
    if (verbose_) {
      cout << "Point number before # " << cloud->points.size()
           << " / after down sampling: # " << n_point << endl;
    }

    // Clear temp and get the candidates of horizontal plane points using normal
    reset();
    if (!computeNormalAndFilter()) {
      ROS_WARN("PlaneSegment: Cloud is empty after applying normal filter.");
      return;
    }

    if (!zClustering()) {
      ROS_WARN("PlaneSegment: Cloud is empty after applying z clustering.");
      return;
    }
    getClustersMeanZ();
    extractPlaneForEachZ(results);
    results_ = results;

    // Stop timer and get total processing time
    if (verbose_) {
      hst_.stop();
      hst_.print();
      visualizeResult();
    }
  }

  bool PlaneSegment::computeNormalAndFilter()
  {
    Utilities::estimateNorm(src_cloud_xyz_, src_cloud_n_, 1.01 * th_xy_);
    Utilities::getCloudByNorm(src_cloud_n_, normal_fitted_indices_, th_norm_);

    if (normal_fitted_indices_->indices.empty()) return false;

    Utilities::getCloudByInliers(src_cloud_xyz_, normal_fitted_cloud_xyz_, normal_fitted_indices_, false, false);
    Utilities::getCloudByInliers(src_cloud_n_, normal_fitted_cloud_n_, normal_fitted_indices_, false, false);
    return true;
  }

//  void PlaneSegment::findAllPlanesRG(int norm_k, int num_n, float s_th, float c_th)
//  {
//    cout << "Threshold for RG:" << endl;
//    cout << "K search for normal computation: " << norm_k << endl;
//    cout << "RG number of neighbours: " << num_n << endl;
//    cout << "smooth threshold: " << s_th << endl;
//    cout << "curvature threshold: " << c_th << endl;
//
//    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >
//        (new pcl::search::KdTree<pcl::PointXYZ>);
//    pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
//    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
//    normal_estimator.setSearchMethod(tree);
//    normal_estimator.setInputCloud(normal_fitted_cloud_xyz_);
//    normal_estimator.setKSearch(norm_k);
//    normal_estimator.compute(*normals);
//
//    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
//    reg.setMinClusterSize(3);
//    reg.setMaxClusterSize(INT_MAX);
//    reg.setSearchMethod(tree);
//    reg.setNumberOfNeighbours(num_n);
//    reg.setInputCloud(normal_fitted_cloud_xyz_);
//    reg.setInputNormals(normals);
//    reg.setSmoothnessThreshold(s_th / 180.0 * M_PI);
//    reg.setCurvatureThreshold(c_th);
//    vector<pcl::PointIndices> clusters;
//    reg.extract(clusters);
//    //cout << "Number of clusters: " << clusters.size () << endl;
//
//    for (size_t i = 0; i < clusters.size(); ++i) {
//      Cloud_XYZ::Ptr cloud_p (new Cloud_XYZ);
//      pcl::PointIndices::Ptr idx_seed(new pcl::PointIndices);
//      idx_seed->indices = clusters[i].indices;
//      Utilities::getCloudByInliers(normal_fitted_cloud_xyz_, cloud_p, idx_seed, false, false);
//      plane_points_.push_back(cloud_p);
//
//      float z_mean, z_max, z_min, z_mid;
//      Utilities::getCloudZInfo(cloud_p, z_mean, z_max, z_min, z_mid);
//      setFeatures(z_mean, cloud_p);
//    }
//  }

//  void PlaneSegment::findAllPlanesRANSAC(bool isOptimize, int maxIter,
//                                         float disThresh, float omit)
//  {
//    cout << "Threshold for RANSAC:" << endl;
//    cout << "Is optimize: " << isOptimize << endl;
//    cout << "Max iteration: " << maxIter << endl;
//    cout << "Distance threshold: " << disThresh << endl;
//    cout << "Omit rate: " << omit << endl;
//
//    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
//    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
//    // Create the segmentation object
//    pcl::SACSegmentation<pcl::PointXYZ> seg;
//    // Optional
//    seg.setOptimizeCoefficients(isOptimize);
//    // Mandatory
//    seg.setModelType(pcl::SACMODEL_PLANE);
//    seg.setMethodType(pcl::SAC_RANSAC);
//    seg.setMaxIterations(maxIter);
//    seg.setDistanceThreshold(disThresh);
//
//    int i = 0;
//    int n_points = (int)normal_fitted_cloud_xyz_->points.size();
//
//    while (normal_fitted_cloud_xyz_->points.size() > omit * n_points) {
//      // Segment the largest planar component from the remaining cloud
//      Cloud_XYZ::Ptr  cloud_p(new Cloud_XYZ);
//      Cloud_XYZ::Ptr  cloud_f(new Cloud_XYZ);
//      seg.setInputCloud(normal_fitted_cloud_xyz_);
//      seg.segment(*inliers, *coefficients);
//      if (inliers->indices.size() == 0) {
//        cerr << "Could not estimate a planar model for the given dataset." << endl;
//        break;
//      }
//      // Extract the inliers
//      Utilities::getCloudByInliers(normal_fitted_cloud_xyz_, cloud_p, inliers, false, false);
//      cout << "Cloud_XYZRGB representing the planar component: " << cloud_p->points.size() << " data points." << endl;
//      plane_points_.push_back(cloud_p);
//
//      if (cal_hull_) {
//        Cloud_XYZRGB::Ptr cloud_2d_rgb(new Cloud_XYZRGB);
//        Cloud_XYZ::Ptr cloud_proj(new Cloud_XYZ);
//
//        Utilities::projectCloudTo2D(coefficients, cloud_p, cloud_proj);
//        Utilities::convertToColorCloud(cloud_proj, cloud_2d_rgb, 100, 100, 100);
//        getHull(cloud_2d_rgb);
//      }
//
//      setFeatures(coefficients->values[3], cloud_p);
//
//      //std::stringstream ss;
//      //ss << "plane_" << i << ".pcd";
//      //writer.write<pcl::PointXYZ>(ss.str(), *cloud_p, false);
//
//      // Create the filtering object
//      Utilities::getCloudByInliers(normal_fitted_cloud_xyz_, cloud_f, inliers, true, false);
//      normal_fitted_cloud_xyz_.swap(cloud_f);
//      i++;
//    }
//  }

  bool PlaneSegment::zClustering()
  {
    ZGrowing zg;
    pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >
        (new pcl::search::KdTree<pcl::PointXYZ>);

    zg.setMinClusterSize(3);
    zg.setMaxClusterSize(INT_MAX);
    zg.setSearchMethod(tree);
    zg.setNumberOfNeighbours(8);
    zg.setInputCloud(normal_fitted_cloud_xyz_);  // <--
    zg.setZThreshold(th_z_);  // <--

    zg.extract(z_clustered_indices_list_);  // -->

    return !z_clustered_indices_list_.empty();
  }

  bool PlaneSegment::getClustersMeanZ()
  {
    int k = 0;
    // Traverse each part to determine its mean Z value
    for (const auto & it : z_clustered_indices_list_) {
      Cloud_XYZ::Ptr part_cloud_xyz(new Cloud_XYZ);

      pcl::PointIndices::Ptr part_indices(new pcl::PointIndices);
      part_indices->indices = it.indices;
      Utilities::getCloudByInliers(normal_fitted_cloud_xyz_, part_cloud_xyz, part_indices, false, false);

      if (verbose_) {
        string name = Utilities::getName(k, "part_", -1);
        Vec3f c = Utilities::getColorWithID(k);
      }

      float z_mean, z_max, z_min, z_mid;
      Utilities::getCloudZInfo<Cloud_XYZ::Ptr>(part_cloud_xyz, z_mean, z_max, z_min, z_mid);
      //cout << "Cluster has " << part_indices->indices.size() << " points at z: " << z_mean << endl;
      plane_z_values_.push_back(z_mean);
      k++;
    }

    // Z is ordered from small to large, i.e., low to high
    //sort(planeZVector_.begin(), planeZVector_.end());
  }

  void PlaneSegment::extractPlaneForEachZ(PlaneSegmentResults &results)
  {
    size_t id = 0;
    unsigned long max_n = 0;
    for (float & z : plane_z_values_) {
      Cloud_XYZ::Ptr plane_cloud_xyz(new Cloud_XYZ);
      Cloud_XYZ::Ptr hull_cloud_xyz(new Cloud_XYZ);
      if (getPlane(id, z, plane_cloud_xyz)) {
        results.planes.push_back(plane_cloud_xyz);
      }
      if (getHull(plane_cloud_xyz, hull_cloud_xyz)) {
        results.hulls.push_back(hull_cloud_xyz);
      }
      if (plane_cloud_xyz->points.size() > max_n) {
        results.max_plane = plane_cloud_xyz;
        results.max_plane_hull = hull_cloud_xyz;
        results.max_plane_z = z;
        max_n = plane_cloud_xyz->points.size();
      }
      id++;
    }
    results.n_plane = results.planes.size();
  }

  bool PlaneSegment::getPlane(size_t id, float z_in, Cloud_XYZ::Ptr &plane_cloud)
  {
    pcl::ModelCoefficients::Ptr cluster_coeff(new pcl::ModelCoefficients);
    // Plane function: ax + by + cz + d = 0, hence coeff[3] = d = -cz
    cluster_coeff->values.push_back(0.0);
    cluster_coeff->values.push_back(0.0);
    cluster_coeff->values.push_back(1.0);
    cluster_coeff->values.push_back(-z_in);

    pcl::PointIndices::Ptr part_indices (new pcl::PointIndices);
    part_indices->indices = z_clustered_indices_list_[id].indices;

    // Extract the plane points indexed by part_indices
    Utilities::getCloudByInliers(normal_fitted_cloud_xyz_, plane_cloud, part_indices, false, false);

    // If the points do not pass the error test, return
    return gaussianImageAnalysis(id);

    // Update the data of the max plane detected
    //  if (cluster_2d_rgb->points.size() > max_plane_points_num_) {
    //    plane_max_result_ = cluster_2d_rgb;
    //    max_plane_cloud_ = in_cloud_xyz;
    //    max_coeff_ = feature;
    //    max_plane_points_num_ = cluster_2d_rgb->points.size();
    //  }
  }

  bool PlaneSegment::getHull(Cloud_XYZ::Ptr cloud_xyz, Cloud_XYZ::Ptr &hull_xyz)
  {
    if (cloud_xyz->points.size() < 4) {
      return false;
    }

    Cloud_XYZRGB::Ptr cluster_hull(new Cloud_XYZRGB);
    pcl::ConvexHull<pcl::PointXYZRGB> hull;
    pcl::PolygonMesh cluster_mesh;

    Cloud_XYZRGB::Ptr cloud_xyzrgb(new Cloud_XYZRGB);
    Utilities::convertCloudType(std::move(cloud_xyz), cloud_xyzrgb);
    hull.setInputCloud(cloud_xyzrgb);
    hull.setComputeAreaVolume(true);
    hull.reconstruct(*cluster_hull);
    hull.reconstruct(cluster_mesh);
    if (hull.getTotalArea() <= 0) return false;
    Utilities::convertCloudType(cluster_hull, hull_xyz);
    return true;
  }

//  void PlaneSegment::setFeatures(float z_in, Cloud_XYZ::Ptr cluster)
//  {
//    // Prepare the feature vector for each plane to identify its id
//    vector<float> feature;
//    feature.push_back(z_in); // z value
//    pcl::PointXYZ minPt, maxPt;
//    pcl::getMinMax3D(*cluster, minPt, maxPt);
//    feature.push_back(minPt.x); // cluster min x
//    feature.push_back(minPt.y); // cluster min y
//    feature.push_back(maxPt.x); // cluster max x
//    feature.push_back(maxPt.y); // cluster max y
//    plane_coeff_.push_back(feature);
//  }

  bool PlaneSegment::gaussianImageAnalysis(size_t id)
  {
    /// Get normal cloud for current cluster
    pcl::PointIndices::Ptr idx_seed (new pcl::PointIndices);
    idx_seed->indices = z_clustered_indices_list_[id].indices;

    // Extract the plane points indexed by idx_seed
    Cloud_N::Ptr cluster_normal(new Cloud_N);
    Utilities::getCloudByInliers(normal_fitted_cloud_n_, cluster_normal, idx_seed, false, false);

    /// Construct a Pointcloud to store normal points
//    if (show_egi_) {
//      Cloud_XYZ::Ptr cloud(new Cloud_XYZ);
//      cloud->width = cluster_normal->width;
//      cloud->height = cluster_normal->height;
//      cloud->resize(cluster_normal->width * cluster_normal->height);
//
//      size_t k = 0;
//      for (Cloud_XYZ::const_iterator pit = cloud->begin();
//           pit != cloud->end(); ++pit) {
//        cloud->points[k].x = cluster_normal->points[k].normal_x;
//        cloud->points[k].y = cluster_normal->points[k].normal_y;
//        cloud->points[k].z = fabs(cluster_normal->points[k].normal_z);
//        k++;
//      }
//      pcl::visualization::PCLVisualizer viewer ("EGI and normals distribution");
//      viewer.setBackgroundColor(0.8, 0.83, 0.86);
//      viewer.addPointCloud(cloud, "normals");
//      viewer.addCoordinateSystem(0.5);
//
//      /// Use wire frame sphere
//      //pcl::PointXYZ p;
//      //p.x = 0; p.y = 0; p.z = 0;
//      //viewer.addSphere(p, 1.0, 0, 0.6, 0.8, "EGI");
//      //viewer.setRepresentationToWireframeForAllActors();
//
//      /// Use point cloud sphere
//      int bandwidth = 128;
//      pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere (new pcl::PointCloud<pcl::PointXYZRGB>);
//      sphere->resize (4 * bandwidth * bandwidth);
//      uint8_t r = 0, g = 0, b = 0;
//      uint32_t rgb = 0;
//      double tmp_theta;
//      float tmp_phi;
//      for (size_t i = 0; i < 2 * bandwidth; i++) {
//        tmp_theta = (2 * i + 1) * M_PI / 4 / bandwidth;
//        for (size_t j = 0; j < 2 * bandwidth; j++) {
//          tmp_phi = M_PI * j / bandwidth;
//          sphere->points[i * 2 * bandwidth + j].x = cos (tmp_phi) * sin (tmp_theta);
//          sphere->points[i * 2 * bandwidth + j].y = sin (tmp_phi) * sin (tmp_theta);
//          sphere->points[i * 2 * bandwidth + j].z = cos (tmp_theta);
//          Utilities::heatmapRGB(float(i) / bandwidth / 4, r, g, b);
//          rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
//          sphere->points[i * 2 * bandwidth + j].rgb = *reinterpret_cast<float*> (&rgb);
//        }
//      }
//      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> sp_rgb(sphere);
//      viewer.addPointCloud<pcl::PointXYZRGB>(sphere, sp_rgb, "egi");
//      viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "egi");
//
//      // Show normal distribution on the sphere
//      viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "normals");
//      viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0.4, 0, "normals");
//
//      while (!viewer.wasStopped()) {
//        viewer.spinOnce(1); // ms
//      }
//    }

    return Utilities::normalAnalysis(cluster_normal, th_angle_);
  }
//
//  void PlaneSegment::setID()
//  {
//    if (global_id_temp_.empty()) {
//      // Initialize the global id temp with the first detection
//      for (size_t i = 0; i < plane_coeff_.size(); ++i) {
//        global_id_temp_.push_back(i);
//        global_coeff_temp_.push_back(plane_coeff_[i]);
//      }
//    }
//    else {
//      vector<int> local_id_temp;
//      Utilities::matchID(global_coeff_temp_, plane_coeff_, global_id_temp_, local_id_temp, 5);
//
//      // Update global result temp
//      global_coeff_temp_.clear();
//      global_id_temp_.clear();
//      for (size_t i = 0; i < plane_coeff_.size(); ++i) {
//        global_id_temp_.push_back(local_id_temp[i]);
//        global_coeff_temp_.push_back(plane_coeff_[i]);
//      }
//    }
//  }

//  int PlaneSegment::checkSimilar(vector<float> coeff)
//  {
//    int id = -1;
//    float distemp = FLT_MAX;
//    for (size_t i = 0; i < global_coeff_temp_.size(); ++i) {
//      vector<float> coeff_prev = global_coeff_temp_[i];
//      float dis = Utilities::getDistance(coeff_prev, coeff);
//      if (dis < distemp) {
//        distemp = dis;
//        id = global_id_temp_[i];
//      }
//    }
//    return id;
//  }

  void PlaneSegment::visualizeResult()
  {
    // Clear temps
    viewer_->removeAllPointClouds();
    viewer_->removeAllShapes();
    string name;

    /// Point size must be set AFTER adding point cloud
    // Add source colored cloud for reference
    name = Utilities::getName(0, "source_", -1);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> src_rgb(src_cloud_xyzrgb_);
    if (!viewer_->updatePointCloud(src_cloud_xyzrgb_, src_rgb, name)){
      viewer_->addPointCloud<pcl::PointXYZRGB>(src_cloud_xyzrgb_, src_rgb, name);
      viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, name);
    }

    for (int i = 0; i < results_.n_plane; i++) {
      Vec3f c = Utilities::getColorWithID(i);
      // Add detected plane points
      name = Utilities::getName(i, "plane_", -1);
      auto plane_i = results_.planes[i];
      Cloud_XYZRGB::Ptr plane_i_rgb(new Cloud_XYZRGB);
      Utilities::convertCloudType(plane_i, plane_i_rgb);
      viewer_->addPointCloud<pcl::PointXYZRGB>(plane_i_rgb, name);
      viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10.0, name);
      viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, c[0], c[1], c[2], name);
    }
    for (int j = 0; j < results_.hulls.size(); ++j) {
      Vec3f c = Utilities::getColorWithID(j);
      // Add hull points
      name = Utilities::getName(j, "mesh_", -1);
      auto hull_j = results_.hulls[j];
      Cloud_XYZRGB::Ptr hull_j_rgb(new Cloud_XYZRGB);
      Utilities::convertCloudType(hull_j, hull_j_rgb);
      viewer_->addPointCloud<pcl::PointXYZRGB>(hull_j_rgb, name);
      viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 20.0, name);
      viewer_->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, c[0], c[1], c[2], name);
    }
    cout << "Total plane patches #: " << results_.n_plane << endl;

    while (!viewer_->wasStopped()) {
      viewer_->spinOnce(1); // ms
      if (type_ == TUM_LIST)
        break;
    }
  }

  void PlaneSegment::poisson_reconstruction(const Cloud_XYZN::Ptr& point_cloud,
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
  pcl::PolygonMesh PlaneSegment::mesh(const Cloud_XYZ::Ptr& point_cloud,
                                      const Cloud_N::Ptr& normals)
  {
    // Add the normals to the point cloud
    Cloud_XYZN::Ptr cloud_with_normals(new Cloud_XYZN);
    pcl::concatenateFields(*point_cloud, *normals, *cloud_with_normals);

    // Point cloud to mesh reconstruction
    pcl::PolygonMesh mesh;
    poisson_reconstruction(cloud_with_normals, mesh);

    return mesh;
  }

  void PlaneSegment::reset()
  {
    plane_z_values_.clear();
    cloud_fit_parts_.clear();
    z_clustered_indices_list_.clear();

    // Reset timer
    hst_.reset();
  }
}
