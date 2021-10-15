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
      float th_z
  ) :
      th_xy_(th_xy),
      th_z_(th_z),
      src_cloud_xyz_(new Cloud_XYZ),
      normal_fitted_cloud_xyz_(new Cloud_XYZ),
      normal_fitted_cloud_n_(new Cloud_N),
      src_cloud_n_(new Cloud_N),
      normal_fitted_indices_(new pcl::PointIndices),
      hst_("total")
  {
    th_theta_ = th_z_ / th_xy_;
    th_angle_ = atan(th_theta_);
    th_norm_ = sqrt(1. / (1. + 2. * pow(th_theta_, 2.)));
  }

  PlaneSegmentResults PlaneSegment::getHorizontalPlanes(const Cloud_XYZ::Ptr& cloud, bool verbose)
  {
    // Start timer
    if (verbose) hst_.start();

    PlaneSegmentResults results{};

    Utilities::downSampling(cloud, src_cloud_xyz_, th_xy_, th_z_);
    unsigned long n_point = src_cloud_xyz_->points.size();
    if (n_point == 0) {
      ROS_WARN("PlaneSegment: Cloud is empty after down sampling.");
      return results;
    }
    if (verbose) {
      cout << "Point number after down sampling: #" << n_point << endl;
    }

    // Clear temp and get the candidates of horizontal plane points using normal
    reset();
    if (!computeNormalAndFilter()) {
      ROS_WARN("PlaneSegment: Cloud is empty after applying normal filter.");
      return results;
    }

    if (!zClustering()) {
      ROS_WARN("PlaneSegment: Cloud is empty after applying z clustering.");
      return results;
    }
    getClustersMeanZ(verbose);
    extractPlaneForEachZ(results);

    // Stop timer and get total processing time
    if (verbose) {
      hst_.stop();
      hst_.print();
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

  bool PlaneSegment::getClustersMeanZ(bool verbose)
  {
    int k = 0;
    // Traverse each part to determine its mean Z value
    for (const auto & it : z_clustered_indices_list_) {
      Cloud_XYZ::Ptr part_cloud_xyz(new Cloud_XYZ);

      pcl::PointIndices::Ptr part_indices(new pcl::PointIndices);
      part_indices->indices = it.indices;
      Utilities::getCloudByInliers(normal_fitted_cloud_xyz_, part_cloud_xyz, part_indices, false, false);

      if (verbose) {
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
        results.point_results.push_back(plane_cloud_xyz);
      }
      if (getHull(plane_cloud_xyz, hull_cloud_xyz)) {
        results.hull_results.push_back(hull_cloud_xyz);
      }
      if (plane_cloud_xyz->points.size() > max_n) {
        results.max_points = plane_cloud_xyz;
        results.max_hull = hull_cloud_xyz;
        results.max_z = z;
        max_n = plane_cloud_xyz->points.size();
      }
      id++;
    }
    results.n_plane = results.point_results.size();
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
    if (cloud_xyz->points.size() < 3) {
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

//  void PlaneSegment::visualizeResult(bool display_source, bool display_raw,
//                                     bool display_err, bool display_hull)
//  {
//    // For visualizing in RViz
//    //publishCloud(src_cloud_xyzrgb_, plane_cloud_publisher_);
//    //publishCloud(max_plane_cloud_, max_plane_cloud_publisher_);
//
//    // Clear temps
//    viewer->removeAllPointClouds();
//    viewer->removeAllShapes();
//    string name;
//
//    /// Point size must be set AFTER adding point cloud
//    if (display_source) {
//      // Add source colored cloud for reference
//      name = Utilities::getName(0, "source_", -1);
//
//      //viewer->addPointCloud<pcl::PointXYZRGB>(dsp_cloud_xyzrgb_, name);
//      //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, name);
//      //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, name);
//      //viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (dsp_cloud_xyzrgb_, src_cloud_n_, 1, 0.05, "normals");
//
//      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> src_rgb(dsp_cloud_xyzrgb_);
//      if (!viewer->updatePointCloud(dsp_cloud_xyzrgb_, src_rgb, name)){
//        viewer->addPointCloud<pcl::PointXYZRGB>(dsp_cloud_xyzrgb_, src_rgb, name);
//        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, name);
//      }
//    }
//
//    for (size_t i = 0; i < plane_points_.size(); i++) {
//      int id = global_id_temp_[i];
//      Vec3f c = Utilities::getColorWithID(id);
//      if (display_raw) {
//        // Add raw plane points
//        name = Utilities::getName(i, "plane_", -1);
//        viewer->addPointCloud<pcl::PointXYZ>(plane_points_[i], name);
//        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10.0, name);
//        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, c[0], c[1], c[2], name);
//      }
//      if (display_err) {
//        // Add results with error display
//        name = Utilities::getName(i, "error_", -1);
//        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> err_rgb(plane_results_[i]);
//        if (!viewer->updatePointCloud(plane_results_[i], err_rgb, name)){
//          viewer->addPointCloud<pcl::PointXYZRGB>(plane_results_[i], err_rgb, name);
//          viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10.0, name);
//        }
//      }
//      if (cal_hull_ && display_hull) {
//        // Add hull points
//        //name = utl_->getName(i, "hull_", -1);
//        //viewer->addPointCloud<pcl::PointXYZRGB>(plane_contour_list_[i], name);
//        //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10.0, name);
//        //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, c[0], c[1], c[2], name);
//        // Add hull mesh
//        name = Utilities::getName(i, "mesh_", -1);
//        viewer->addPolygonMesh(plane_mesh_[i], name);
//        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.9, name);
//        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, c[0], c[1], c[2], name);
//      }
//    }
//    cout << "Total plane patches #: " << plane_points_.size() << endl;
//
//    while (!viewer->wasStopped()) {
//      viewer->spinOnce(1); // ms
//      if (type_ == TUM_LIST || type_ == SYN)
//        break;
//    }
//  }

//  void PlaneSegment::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
//  {
//    if (msg->data.empty()) {
//      cerr << "HoPE: Cloud_XYZRGB is empty." << endl;
//      return;
//    }
//    Cloud_XYZRGB::Ptr src_temp(new Cloud_XYZRGB);
//    Cloud_XYZRGB::Ptr temp(new PointCloud);
//
//    pcl::PCLPointCloud2 pcl_pc2;
//    pcl_conversions::toPCL(*msg, pcl_pc2);
//    pcl::fromPCLPointCloud2(pcl_pc2, *src_temp);
//
//    Utilities::getCloudByZ(src_temp, src_z_inliers_, temp,
//                           th_min_depth_, th_max_depth_);
//
//    tf_->doTransform(temp, src_rgb_cloud_, roll_, pitch_, yaw_);
//    Utilities::convertCloudType<Cloud_XYZRGB::Ptr, Cloud_XYZ::Ptr>(src_rgb_cloud_, src_mono_cloud_);
//  }

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


//PlaneSegment::PlaneSegment(data_type mode, float th_xy, float th_z, string base_frame, const string& cloud_topic) :
//    type_(mode),
//    src_mono_cloud_(new Cloud_XYZ),
//    src_rgb_cloud_(new Cloud_XYZRGB),
//    cloud_norm_fit_mono_(new Cloud_XYZ),
//    cloud_norm_fit_(new Cloud_N),
//    src_sp_mono_(new Cloud_XYZ),
//    src_sp_rgb_(new Cloud_XYZRGB),
//    src_normals_(new Cloud_N),
//    idx_norm_fit_(new pcl::PointIndices),
//    src_z_inliers_(new pcl::PointIndices),
//    tf_(new Transform),
//    base_frame_(std::move(base_frame)),
//    viewer(new pcl::visualization::PCLVisualizer("HoPE Result")),
//    hst_("total")
//{
//  th_grid_rsl_ = th_xy;
//  th_z_rsl_ = th_z;
//  th_theta_ = th_z_rsl_ / th_grid_rsl_;
//  th_angle_ = atan(th_theta_);
//  th_norm_ = sqrt(1 / (1 + 2 * pow(th_theta_, 2)));
//
//  // For storing max hull id and area
//  global_size_temp_ = 0;
//
//  // Register the callback if using real point cloud data
//  sub_pointcloud_ = nh_.subscribe<sensor_msgs::PointCloud2>(cloud_topic, 1,
//                                                            &PlaneSegment::cloudCallback, this);
//
//  viewer->setBackgroundColor(0.8, 0.83, 0.86);
//  viewer->initCameraParameters();
//  viewer->setCameraPosition(1,0,2,0,0,1);
//  viewer->addCoordinateSystem(0.1);
//}
//
//void PlaneSegment::visualizeProcess(Cloud_XYZRGB::Ptr cloud)
//{
//  // Clear temps
//  viewer->removeAllPointClouds();
//  viewer->removeAllShapes();
//  string name;
//
//  /// Point size must be set AFTER adding point cloud
//  // Add source colored cloud for reference
//  name = Utilities::getName(0, "source_", -1);
//  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> src_rgb(cloud);
//  if (!viewer->updatePointCloud(cloud, src_rgb, name)){
//    viewer->addPointCloud<pcl::PointXYZRGB>(cloud, src_rgb, name);
//    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, name);
//  }
//
//  while (!viewer->wasStopped()) {
//    viewer->spinOnce(1);
//  }
//}
//
//void PlaneSegment::setRPY(float roll = 0.0, float pitch = 0.0, float yaw = 0.0)
//{
//  roll_ = roll;
//  pitch_ = pitch;
//  yaw_ = yaw;
//}
//
//void PlaneSegment::setQ(float qx = 0.0, float qy = 0.0,
//                        float qz = 0.0, float qw = 1.0)
//{
//  qx_ = qx;
//  qy_ = qy;
//  qz_ = qz;
//  qw_ = qw;
//}
//
//void PlaneSegment::setT(float tx = 0.0, float ty = 0.0, float tz = 0.0)
//{
//  tx_ = tx;
//  ty_ = ty;
//  tz_ = tz;
//}
//
//// Notice that the point cloud may not transformed before this function
//void PlaneSegment::getHorizontalPlanes(Cloud_XYZRGB::Ptr cloud)
//{
//  Cloud_XYZRGB::Ptr temp(new Cloud_XYZRGB);
//  if (type_ == SYN) {
//    // If using real data, the transform from camera frame to base frame
//    // need to be provided
//    getSourceCloud();
//  }
//  else if (type_ == POINT_CLOUD) {
//    src_rgb_cloud_ = cloud;
//  }
//  else if (type_ >= TUM_SINGLE) {
//    // To remove Nan and unreliable points with z value
//    Utilities::getCloudByZ(cloud, src_z_inliers_, temp, th_min_depth_, th_max_depth_);
//    //visualizeProcess(temp);
//
//    if (type_ <= TUM_LIST) {
//      tf_->doTransform(temp, src_rgb_cloud_, tx_, ty_, tz_, qx_, qy_, qz_, qw_);
//      //tf_->doTransform(temp, src_cloud_xyzrgb_, 0, 0, 0, qx_, qy_, qz_, qw_);
//    }
//    else {
//      tf_->doTransform(temp, src_rgb_cloud_, roll_, pitch_, yaw_);
//    }
//  }
//  Utilities::convertCloudType<Cloud_XYZRGB::Ptr, Cloud_XYZ::Ptr>(src_rgb_cloud_, src_mono_cloud_);
//
//  //visualizeProcess(src_cloud_xyzrgb_);
//  //pcl::io::savePCDFile("~/src.pcd", *src_cloud_xyzrgb_);
//
//  Utilities::downSampling(src_mono_cloud_, src_sp_mono_, th_grid_rsl_, th_z_rsl_);
//  Utilities::downSampling(src_rgb_cloud_, src_sp_rgb_, th_grid_rsl_, th_z_rsl_);
//
//  //visualizeProcess(dsp_cloud_xyzrgb_);
//  cout << "Point number after down sampling: #" << src_sp_rgb_->points.size() << endl;
//
//  if (src_sp_mono_->points.empty()) {
//    ROS_WARN("PlaneSegment: Source cloud is empty.");
//    return;
//  }
//
//  // Clear temp and get the candidates of horizontal plane points using normal
//  reset();
//  computeNormalAndFilter();
//
//  //pcl::io::savePCDFile("~/normal_filter.pcd", *normal_fitted_cloud_xyz_);
//
//  // Start timer
//  hst_.start();
//
//  findAllPlanes();
//
//  /* You can alternatively use RANSAC or Region Growing instead of HoPE
//   * to carry out the comparision experiments in the paper
//   */
//  //findAllPlanesRANSAC(true, 500, 1.01*th_xy_, 0.001);
//  //findAllPlanesRG(20, 20, 8.0, 1.0);
//
//  // Stop timer and get total processing time
//  hst_.stop();
//  hst_.print();
//
//  setID();
//  visualizeResult(true, true, false, cal_hull_);
//}
//
//void PlaneSegment::findAllPlanesRG(int norm_k, int num_n, float s_th, float c_th)
//{
//  cout << "Threshold for RG:" << endl;
//  cout << "K search for normal computation: " << norm_k << endl;
//  cout << "RG number of neighbours: " << num_n << endl;
//  cout << "smooth threshold: " << s_th << endl;
//  cout << "curvature threshold: " << c_th << endl;
//
//  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >
//      (new pcl::search::KdTree<pcl::PointXYZ>);
//  pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
//  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
//  normal_estimator.setSearchMethod(tree);
//  normal_estimator.setInputCloud(cloud_norm_fit_mono_);
//  normal_estimator.setKSearch(norm_k);
//  normal_estimator.compute(*normals);
//
//  pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
//  reg.setMinClusterSize(3);
//  reg.setMaxClusterSize(INT_MAX);
//  reg.setSearchMethod(tree);
//  reg.setNumberOfNeighbours(num_n);
//  reg.setInputCloud(cloud_norm_fit_mono_);
//  reg.setInputNormals(normals);
//  reg.setSmoothnessThreshold(s_th / 180.0 * M_PI);
//  reg.setCurvatureThreshold(c_th);
//  vector<pcl::PointIndices> clusters;
//  reg.extract(clusters);
//  //cout << "Number of clusters: " << clusters.size () << endl;
//
//  for (size_t i = 0; i < clusters.size(); ++i) {
//    Cloud_XYZ::Ptr cloud_p (new Cloud_XYZ);
//    pcl::PointIndices::Ptr idx_seed(new pcl::PointIndices);
//    idx_seed->indices = clusters[i].indices;
//    Utilities::getCloudByInliers(cloud_norm_fit_mono_, cloud_p, idx_seed, false, false);
//    plane_points_.push_back(cloud_p);
//
//    float z_mean, z_max, z_min, z_mid;
//    Utilities::getCloudZInfo(cloud_p, z_mean, z_max, z_min, z_mid);
//    setFeatures(z_mean, cloud_p);
//  }
//}
//
//void PlaneSegment::findAllPlanes()
//{
//  zClustering(cloud_norm_fit_mono_); // -> z_clustered_indices_list_
//  getMeanZofEachCluster(cloud_norm_fit_mono_); // -> plane_z_values_
//  extractPlaneForEachZ(cloud_norm_fit_mono_);
//}
//
//void PlaneSegment::findAllPlanesRANSAC(bool isOptimize, int maxIter,
//                                       float disThresh, float omit)
//{
//  cout << "Threshold for RANSAC:" << endl;
//  cout << "Is optimize: " << isOptimize << endl;
//  cout << "Max iteration: " << maxIter << endl;
//  cout << "Distance threshold: " << disThresh << endl;
//  cout << "Omit rate: " << omit << endl;
//
//  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
//  pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
//  // Create the segmentation object
//  pcl::SACSegmentation<pcl::PointXYZ> seg;
//  // Optional
//  seg.setOptimizeCoefficients(isOptimize);
//  // Mandatory
//  seg.setModelType(pcl::SACMODEL_PLANE);
//  seg.setMethodType(pcl::SAC_RANSAC);
//  seg.setMaxIterations(maxIter);
//  seg.setDistanceThreshold(disThresh);
//
//  int i = 0;
//  int n_points = (int)cloud_norm_fit_mono_->points.size();
//
//  while (cloud_norm_fit_mono_->points.size() > omit * n_points) {
//    // Segment the largest planar component from the remaining cloud
//    Cloud_XYZ::Ptr  cloud_p(new Cloud_XYZ);
//    Cloud_XYZ::Ptr  cloud_f(new Cloud_XYZ);
//    seg.setInputCloud(cloud_norm_fit_mono_);
//    seg.segment(*inliers, *coefficients);
//    if (inliers->indices.size() == 0) {
//      cerr << "Could not estimate a planar model for the given dataset." << endl;
//      break;
//    }
//    // Extract the inliers
//    Utilities::getCloudByInliers(cloud_norm_fit_mono_, cloud_p, inliers, false, false);
//    cout << "Cloud_XYZRGB representing the planar component: " << cloud_p->points.size() << " data points." << endl;
//    plane_points_.push_back(cloud_p);
//
//    if (cal_hull_) {
//      Cloud_XYZRGB::Ptr cloud_2d_rgb(new Cloud_XYZRGB);
//      Cloud_XYZ::Ptr cloud_proj(new Cloud_XYZ);
//
//      Utilities::projectCloudTo2D(coefficients, cloud_p, cloud_proj);
//      Utilities::convertToColorCloud(cloud_proj, cloud_2d_rgb, 100, 100, 100);
//      computeHull(cloud_2d_rgb);
//    }
//
//    setFeatures(coefficients->values[3], cloud_p);
//
//    //std::stringstream ss;
//    //ss << "plane_" << i << ".pcd";
//    //writer.write<pcl::PointXYZ>(ss.str(), *cloud_p, false);
//
//    // Create the filtering object
//    Utilities::getCloudByInliers(cloud_norm_fit_mono_, cloud_f, inliers, true, false);
//    cloud_norm_fit_mono_.swap(cloud_f);
//    i++;
//  }
//}
//
//void PlaneSegment::getMeanZofEachCluster(Cloud_XYZ::Ptr cloud_norm_fit_mono)
//{
//  if (seed_clusters_indices_.empty())
//    ROS_DEBUG("PlaneSegment: Region growing get nothing.");
//
//  else {
//    size_t k = 0;
//    // Traverse each part to determine its mean Z value
//    for (vector<pcl::PointIndices>::const_iterator it = seed_clusters_indices_.begin();
//         it != seed_clusters_indices_.end(); ++it) {
//      Cloud_XYZ::Ptr cloud_fit_part(new Cloud_XYZ);
//
//      pcl::PointIndices::Ptr idx_seed(new pcl::PointIndices);
//      idx_seed->indices = it->indices;
//      Utilities::getCloudByInliers(cloud_norm_fit_mono, cloud_fit_part, idx_seed, false, false);
//
//      if (show_cluster_) {
//        string name = Utilities::getName(k, "part_", -1);
//        Vec3f c = Utilities::getColorWithID(k);
//
//        viewer->addPointCloud<pcl::PointXYZ>(cloud_fit_part, name);
//        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10.0, name);
//        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0.7, 0, name);
//      }
//
//      float z_mean, z_max, z_min, z_mid;
//      Utilities::getCloudZInfo<Cloud_XYZ::Ptr>(cloud_fit_part, z_mean, z_max, z_min, z_mid);
//      //cout << "Cluster has " << idx_seed->indices.size() << " points at z: " << z_mean << endl;
//      plane_z_values_.push_back(z_mean);
//      k++;
//    }
//
//    ROS_DEBUG("Hypothesis plane number: %d", int(plane_z_values_.size()));
//    // Z is ordered from small to large, i.e., low to high
//    //sort(planeZVector_.begin(), planeZVector_.end());
//  }
//}
//
//void PlaneSegment::zClustering(Cloud_XYZ::Ptr cloud_norm_fit_mono)
//{
//  ZGrowing zg;
//  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >
//      (new pcl::search::KdTree<pcl::PointXYZ>);
//
//  zg.setMinClusterSize(3);
//  zg.setMaxClusterSize(INT_MAX);
//  zg.setSearchMethod(tree);
//  zg.setNumberOfNeighbours(8);
//  zg.setInputCloud(cloud_norm_fit_mono);
//  zg.setZThreshold(th_z_rsl_);
//
//  zg.extract(seed_clusters_indices_);
//}
//
//void PlaneSegment::extractPlaneForEachZ(Cloud_XYZ::Ptr cloud_norm_fit)
//{
//  size_t id = 0;
//  for (auto cit = plane_z_values_.begin();
//       cit != plane_z_values_.end(); cit++) {
//    getPlane(id, *cit, cloud_norm_fit);
//    id++;
//  }
//}
//
//void PlaneSegment::getPlane(size_t id, float z_in, Cloud_XYZ::Ptr &cloud_norm_fit_mono)
//{
//  pcl::ModelCoefficients::Ptr cluster_coeff(new pcl::ModelCoefficients);
//  // Plane function: ax + by + cz + d = 0, here coeff[3] = d = -cz
//  cluster_coeff->values.push_back(0.0);
//  cluster_coeff->values.push_back(0.0);
//  cluster_coeff->values.push_back(1.0);
//  cluster_coeff->values.push_back(-z_in);
//
//  pcl::PointIndices::Ptr idx_seed (new pcl::PointIndices);
//  idx_seed->indices = seed_clusters_indices_[id].indices;
//
//  // Extract the plane points indexed by idx_seed
//  Cloud_XYZ::Ptr cluster_near_z(new Cloud_XYZ);
//  Utilities::getCloudByInliers(cloud_norm_fit_mono, cluster_near_z, idx_seed, false, false);
//
//  // If the points do not pass the error test, return
//  Cloud_XYZRGB::Ptr cluster_2d_rgb(new Cloud_XYZRGB);
//  if (!gaussianImageAnalysis(id)) return;
//
//  // If the cluster of points pass the check,
//  // push it and corresponding projected points into resulting vectors
//  plane_results_.push_back(cluster_2d_rgb);
//  plane_points_.push_back(cluster_near_z);
//
//  // Use convex hull to represent the plane patch
//  if (cal_hull_) {
//    computeHull(cluster_2d_rgb);
//  }
//
//  setFeatures(z_in, cluster_near_z);
//
//  // Update the data of the max plane detected
//  //  if (cluster_2d_rgb->points.size() > max_plane_points_num_) {
//  //    plane_max_result_ = cluster_2d_rgb;
//  //    max_plane_cloud_ = cluster_near_z;
//  //    max_coeff_ = feature;
//  //    max_plane_points_num_ = cluster_2d_rgb->points.size();
//  //  }
//}
//
//void PlaneSegment::computeHull(Cloud_XYZRGB::Ptr cluster_2d_rgb)
//{
//  Cloud_XYZRGB::Ptr cluster_hull(new Cloud_XYZRGB);
//  pcl::ConvexHull<pcl::PointXYZRGB> hull;
//  pcl::PolygonMesh cluster_mesh;
//
//  hull.setInputCloud(cluster_2d_rgb);
//  hull.setComputeAreaVolume(true);
//  hull.reconstruct(*cluster_hull);
//  hull.reconstruct(cluster_mesh);
//
//  plane_hull_.push_back(cluster_hull);
//  plane_mesh_.push_back(cluster_mesh);
//  plane_max_hull_ = cluster_hull;
//  plane_max_mesh_ = cluster_mesh;
//}
//
//void PlaneSegment::setFeatures(float z_in, Cloud_XYZ::Ptr cluster)
//{
//  // Prepare the feature vector for each plane to identify its id
//  vector<float> feature;
//  feature.push_back(z_in); // z value
//  pcl::PointXYZ minPt, maxPt;
//  pcl::getMinMax3D(*cluster, minPt, maxPt);
//  feature.push_back(minPt.x); // cluster min x
//  feature.push_back(minPt.y); // cluster min y
//  feature.push_back(maxPt.x); // cluster max x
//  feature.push_back(maxPt.y); // cluster max y
//  plane_coeff_.push_back(feature);
//}
//
//bool PlaneSegment::gaussianImageAnalysis(size_t id)
//{
//  /// Get normal cloud for current cluster
//  pcl::PointIndices::Ptr idx_seed (new pcl::PointIndices);
//  idx_seed->indices = seed_clusters_indices_[id].indices;
//
//  // Extract the plane points indexed by idx_seed
//  Cloud_N::Ptr cluster_normal(new Cloud_N);
//  Utilities::getCloudByInliers(cloud_norm_fit_, cluster_normal, idx_seed, false, false);
//
//  /// Construct a Pointcloud to store normal points
//  if (show_egi_) {
//    Cloud_XYZ::Ptr cloud(new Cloud_XYZ);
//    cloud->width = cluster_normal->width;
//    cloud->height = cluster_normal->height;
//    cloud->resize(cluster_normal->width * cluster_normal->height);
//
//    size_t k = 0;
//    for (Cloud_XYZ::const_iterator pit = cloud->begin();
//         pit != cloud->end(); ++pit) {
//      cloud->points[k].x = cluster_normal->points[k].normal_x;
//      cloud->points[k].y = cluster_normal->points[k].normal_y;
//      cloud->points[k].z = fabs(cluster_normal->points[k].normal_z);
//      k++;
//    }
//    pcl::visualization::PCLVisualizer viewer ("EGI and normals distribution");
//    viewer.setBackgroundColor(0.8, 0.83, 0.86);
//    viewer.addPointCloud(cloud, "normals");
//    viewer.addCoordinateSystem(0.5);
//
//    /// Use wire frame sphere
//    //pcl::PointXYZ p;
//    //p.x = 0; p.y = 0; p.z = 0;
//    //viewer.addSphere(p, 1.0, 0, 0.6, 0.8, "EGI");
//    //viewer.setRepresentationToWireframeForAllActors();
//
//    /// Use point cloud sphere
//    int bandwidth = 128;
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere (new pcl::PointCloud<pcl::PointXYZRGB>);
//    sphere->resize (4 * bandwidth * bandwidth);
//    uint8_t r = 0, g = 0, b = 0;
//    uint32_t rgb = 0;
//    float tmp_theta = 0.0;
//    float tmp_phi = 0.0;
//    for (size_t i = 0; i < 2 * bandwidth; i++)
//    {
//      tmp_theta = (2 * i + 1) * M_PI / 4 / bandwidth;
//      for (size_t j = 0; j < 2 * bandwidth; j++)
//      {
//        tmp_phi = M_PI * j / bandwidth;
//        sphere->points[i * 2 * bandwidth + j].x = cos (tmp_phi) * sin (tmp_theta);
//        sphere->points[i * 2 * bandwidth + j].y = sin (tmp_phi) * sin (tmp_theta);
//        sphere->points[i * 2 * bandwidth + j].z = cos (tmp_theta);
//        Utilities::heatmapRGB(float(i)/bandwidth/4, r, g, b);
//        rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
//        sphere->points[i * 2 * bandwidth + j].rgb = *reinterpret_cast<float*> (&rgb);
//      }
//    }
//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> sp_rgb(sphere);
//    viewer.addPointCloud<pcl::PointXYZRGB>(sphere, sp_rgb, "egi");
//    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "egi");
//
//    // Show normal distribution on the sphere
//    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "normals");
//    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0.4, 0, "normals");
//
//    while (!viewer.wasStopped()) {
//      viewer.spinOnce(1); // ms
//    }
//  }
//
//  return Utilities::normalAnalysis(cluster_normal, th_angle_);
//}
//
//void PlaneSegment::setID()
//{
//  if (global_id_temp_.empty()) {
//    // Initialize the global id temp with the first detection
//    for (size_t i = 0; i < plane_coeff_.size(); ++i) {
//      global_id_temp_.push_back(i);
//      global_coeff_temp_.push_back(plane_coeff_[i]);
//    }
//  }
//  else {
//    vector<int> local_id_temp;
//    Utilities::matchID(global_coeff_temp_, plane_coeff_, global_id_temp_, local_id_temp, 5);
//
//    // Update global result temp
//    global_coeff_temp_.clear();
//    global_id_temp_.clear();
//    for (size_t i = 0; i < plane_coeff_.size(); ++i) {
//      global_id_temp_.push_back(local_id_temp[i]);
//      global_coeff_temp_.push_back(plane_coeff_[i]);
//    }
//  }
//}
//
//int PlaneSegment::checkSimilar(vector<float> coeff)
//{
//  int id = -1;
//  float distemp = FLT_MAX;
//  for (size_t i = 0; i < global_coeff_temp_.size(); ++i) {
//    vector<float> coeff_prev = global_coeff_temp_[i];
//    float dis = Utilities::getDistance(coeff_prev, coeff);
//    if (dis < distemp) {
//      distemp = dis;
//      id = global_id_temp_[i];
//    }
//  }
//  return id;
//}
//
//void PlaneSegment::visualizeResult(bool display_source, bool display_raw,
//                                   bool display_err, bool display_hull)
//{
//  // For visualizing in RViz
//  //publishCloud(src_cloud_xyzrgb_, plane_cloud_publisher_);
//  //publishCloud(max_plane_cloud_, max_plane_cloud_publisher_);
//
//  // Clear temps
//  viewer->removeAllPointClouds();
//  viewer->removeAllShapes();
//  string name;
//
//  /// Point size must be set AFTER adding point cloud
//  if (display_source) {
//    // Add source colored cloud for reference
//    name = Utilities::getName(0, "source_", -1);
//
//    //viewer->addPointCloud<pcl::PointXYZRGB>(dsp_cloud_xyzrgb_, name);
//    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, name);
//    //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 0, name);
//    //viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (dsp_cloud_xyzrgb_, src_cloud_n_, 1, 0.05, "normals");
//
//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> src_rgb(src_sp_rgb_);
//    if (!viewer->updatePointCloud(src_sp_rgb_, src_rgb, name)){
//      viewer->addPointCloud<pcl::PointXYZRGB>(src_sp_rgb_, src_rgb, name);
//      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, name);
//    }
//  }
//
//  for (size_t i = 0; i < plane_points_.size(); i++) {
//    int id = global_id_temp_[i];
//    Vec3f c = Utilities::getColorWithID(id);
//    if (display_raw) {
//      // Add raw plane points
//      name = Utilities::getName(i, "plane_", -1);
//      viewer->addPointCloud<pcl::PointXYZ>(plane_points_[i], name);
//      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10.0, name);
//      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, c[0], c[1], c[2], name);
//    }
//    if (display_err) {
//      // Add results with error display
//      name = Utilities::getName(i, "error_", -1);
//      pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> err_rgb(plane_results_[i]);
//      if (!viewer->updatePointCloud(plane_results_[i], err_rgb, name)){
//        viewer->addPointCloud<pcl::PointXYZRGB>(plane_results_[i], err_rgb, name);
//        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10.0, name);
//      }
//    }
//    if (cal_hull_ && display_hull) {
//      // Add hull points
//      //name = utl_->getName(i, "hull_", -1);
//      //viewer->addPointCloud<pcl::PointXYZRGB>(plane_contour_list_[i], name);
//      //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 10.0, name);
//      //viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, c[0], c[1], c[2], name);
//      // Add hull mesh
//      name = Utilities::getName(i, "mesh_", -1);
//      viewer->addPolygonMesh(plane_mesh_[i], name);
//      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.9, name);
//      viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, c[0], c[1], c[2], name);
//    }
//  }
//  cout << "Total plane patches #: " << plane_points_.size() << endl;
//
//  while (!viewer->wasStopped()) {
//    viewer->spinOnce(1); // ms
//    if (type_ == TUM_LIST || type_ == SYN)
//      break;
//  }
//}
//
//void PlaneSegment::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
//{
//  if (msg->data.empty()) {
//    cerr << "HoPE: Cloud_XYZRGB is empty." << endl;
//    return;
//  }
//  Cloud_XYZRGB::Ptr src_temp(new Cloud_XYZRGB);
//  Cloud_XYZRGB::Ptr temp(new Cloud_XYZRGB);
//
//  pcl::PCLPointCloud2 pcl_pc2;
//  pcl_conversions::toPCL(*msg, pcl_pc2);
//  pcl::fromPCLPointCloud2(pcl_pc2, *src_temp);
//
//  Utilities::getCloudByZ(src_temp, src_z_inliers_, temp,
//                         th_min_depth_, th_max_depth_);
//
//  tf_->doTransform(temp, src_rgb_cloud_, roll_, pitch_, yaw_);
//  Utilities::convertCloudType<Cloud_XYZRGB::Ptr, Cloud_XYZ::Ptr>(src_rgb_cloud_, src_mono_cloud_);
//}
//
//void PlaneSegment::poisson_reconstruction(Cloud_XYZN::Ptr point_cloud,
//                                          pcl::PolygonMesh& mesh)
//{
//  // Initialize poisson reconstruction
//  pcl::Poisson<pcl::PointNormal> poisson;
//
//  /*
//   * Set the maximum depth of the tree used in Poisson surface reconstruction.
//   * A higher value means more iterations which could lead to better results but
//   * it is also more computationally heavy.
//   */
//  poisson.setDepth(10);
//  poisson.setInputCloud(point_cloud);
//
//  // Perform the Poisson surface reconstruction algorithm
//  poisson.reconstruct(mesh);
//}
//
///**
// * Reconstruct a point cloud to a mesh by estimate the normals of the point cloud
// *
// * @param point_cloud The input point cloud that will be reconstructed
// * @return Returns a reconstructed mesh
// */
//pcl::PolygonMesh PlaneSegment::mesh(const Cloud_XYZ::Ptr point_cloud,
//                                    Cloud_N::Ptr normals)
//{
//  // Add the normals to the point cloud
//  Cloud_XYZN::Ptr cloud_with_normals(new Cloud_XYZN);
//  pcl::concatenateFields(*point_cloud, *normals, *cloud_with_normals);
//
//  // Point cloud to mesh reconstruction
//  pcl::PolygonMesh mesh;
//  poisson_reconstruction(cloud_with_normals, mesh);
//
//  return mesh;
//}
//
//void PlaneSegment::reset()
//{
//  // Clear temp
//  plane_results_.clear();
//  plane_points_.clear();
//  plane_coeff_.clear();
//  plane_hull_.clear();
//  plane_mesh_.clear();
//
//  plane_z_values_.clear();
//  cloud_fit_parts_.clear();
//  seed_clusters_indices_.clear();
//
//  global_size_temp_ = 0;
//  // Reset timer
//  hst_.reset();
//}
//
//bool PlaneSegment::getSourceCloud()
//{
//  while (ros::ok()) {
//    if (!src_rgb_cloud_->points.empty())
//      return true;
//
//    // Handle callbacks and sleep for a small amount of time
//    // before looping again
//    ros::spinOnce();
//    ros::Duration(0.001).sleep();
//  }
//}
//
//void PlaneSegment::computeNormalAndFilter()
//{
//  Utilities::estimateNorm(src_sp_mono_, src_normals_, 1.01 * th_grid_rsl_);
//  Utilities::getCloudByNorm(src_normals_, idx_norm_fit_, th_norm_);
//
//  if (idx_norm_fit_->indices.empty()) return;
//
//  Utilities::getCloudByInliers(src_sp_mono_, cloud_norm_fit_mono_, idx_norm_fit_, false, false);
//  Utilities::getCloudByInliers(src_normals_, cloud_norm_fit_, idx_norm_fit_, false, false);
//}
//
//PlaneSegmentRT::PlaneSegmentRT(float th_xy, float th_z, ros::NodeHandle nh, string base_frame, const string &cloud_topic) :
//    nh_(nh),
//    src_mono_cloud_(new Cloud_XYZ),
//    cloud_norm_fit_mono_(new Cloud_XYZ),
//    cloud_norm_fit_(new Cloud_N),
//    src_dsp_mono_(new Cloud_XYZ),
//    src_normals_(new Cloud_N),
//    idx_norm_fit_(new pcl::PointIndices),
//    src_z_inliers_(new pcl::PointIndices),
//    tf_(new Transform),
//    hst_("total"),
//    pe_(new PoseEstimation(th_grid_rsl_)),
//    base_frame_(std::move(base_frame)),
//    max_plane_z_(-1000.0f),
//    origin_height_(0.0f),
//    aggressive_merge_(true)
//{
//  th_grid_rsl_ = th_xy;
//  th_z_rsl_ = th_z;
//  th_theta_ = th_z_rsl_ / th_grid_rsl_;
//  th_angle_ = atan(th_theta_);
//  th_norm_ = sqrt(1 / (1 + 2 * pow(th_theta_, 2)));
//
//  // For storing max hull id and area
//  max_plane_points_num_ = 0;
//
//  // Register the callback if using real point cloud data
//  cloud_subscriber_ = nh_.subscribe<sensor_msgs::PointCloud2>(cloud_topic, 1,
//                                                         &PlaneSegmentRT::cloudCallback, this);
//
//  // Set up dynamic reconfigure callback
//  dynamic_reconfigure::Server<hope::hopeConfig>::CallbackType f;
//
//  f = boost::bind(&PlaneSegmentRT::configCallback, this, _1, _2);
//  config_server_.setCallback(f);
//
//  extract_on_top_server_ = nh_.advertiseService("extract_object_on_top", &PlaneSegmentRT::extractOnTopCallback, this);
//
//  // Detect table surface as an obstacle
//  plane_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("plane_points", 1);
//  max_plane_cloud_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("max_plane", 1);
//  max_plane_hull_publisher_ = nh_.advertise<sensor_msgs::PointCloud2>("max_contour", 1);
//  on_plane_obj_publisher_ = nh_.advertise<geometry_msgs::PoseArray>("obj_poses", 1, true);
//}
//
//void PlaneSegmentRT::getHorizontalPlanes() {
//  // If using real data, the transform from camera frame to base frame
//  // need to be provided
//  getSourceCloud();
//
//  // Down sampling
//  Utilities::downSampling(src_mono_cloud_, src_dsp_mono_, th_grid_rsl_, th_z_rsl_);
//
//  if (!Utilities::isPointCloudValid(src_dsp_mono_)) {
//    ROS_ERROR("HoPE: Down sampled source cloud is empty.");
//    return;
//  }
//
//  reset();
//  computeNormalAndFilter();
//  findAllPlanes();
//  visualizeResult();
//}
//
//void PlaneSegmentRT::getSourceCloud()
//{
//  src_mono_cloud_.reset(new Cloud_XYZ);
//  while (ros::ok()) {
//    if (Utilities::isPointCloudValid(src_mono_cloud_))
//      return;
//    ros::spinOnce();
//  }
//}
//
//void PlaneSegmentRT::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &msg)
//{
//  if (msg->data.empty()) {
//    ROS_WARN("HoPE: Source point cloud is empty.");
//    return;
//  }
//  Cloud_XYZ::Ptr src_temp(new Cloud_XYZ);
//  Cloud_XYZ::Ptr src_clamped(new Cloud_XYZ);
//
//  pcl::PCLPointCloud2 pcl_pc2;
//  pcl_conversions::toPCL(*msg, pcl_pc2);
//  pcl::fromPCLPointCloud2(pcl_pc2, *src_temp);
//
//  Utilities::getCloudByZ(src_temp, src_z_inliers_, src_clamped,
//                         th_min_depth_, th_max_depth_);
//
//  if (!tf_->getTransform(base_frame_, msg->header.frame_id)) return;
//  tf_->doTransform(src_clamped, src_mono_cloud_);
//}
//
//void PlaneSegmentRT::configCallback(hope::hopeConfig &config, uint32_t level) {
//  min_height_ = config.min_height_cfg;
//  max_height_ = config.max_height_cfg;
//}
//
//bool PlaneSegmentRT::extractOnTopCallback(hope::ExtractObjectOnTop::Request &req,
//                                          hope::ExtractObjectOnTop::Response &res)
//{
//  ROS_INFO("HoPE Service: Received extract on top object %s call.", req.goal_id.id.c_str());
//  string object_type = req.goal_id.id;
//  bool do_cluster;
//  if (req.goal_id.id == req.CYLINDER) {
//    origin_height_ = req.origin_height;
//    object_type = "cylinder";
//    do_cluster = true;
//  } else if (req.goal_id.id == req.MESH) {
//    object_type = "mesh";
//    object_model_path_ = req.mesh_path;
//    do_cluster = false;
//  } else if (req.goal_id.id == req.BOX) {
//    origin_height_ = req.origin_height;
//    object_type = "box";
//    do_cluster = true;
//  } else if (req.goal_id.id == req.BOX_TOP) {
//    origin_heights_ = req.origin_heights;
//    object_type = "box_top";
//    do_cluster = true;
//  } else if (req.goal_id.id == "debug") {
//    origin_height_ = req.origin_height;
//    object_type = "cylinder";
//    do_cluster = true;
//    req.header.stamp = ros::Time::now();
//  }
//  else {
//    ROS_ERROR("HoPE: Unknown object type given in goal_id.id: %s", object_type.c_str());
//    res.result_status = res.FAILED;
//    return true;
//  }
//
//
//  aggressive_merge_ = req.aggressive_merge;
//
//  bool ok = postProcessing(do_cluster, object_type);
//  if (ok) {
//    int time_interval = on_top_object_poses_.header.stamp.sec - req.header.stamp.sec;
//    if (time_interval > 2) {
//      ROS_WARN("HoPE Service: Extract on top object failed due to lagging %d.", time_interval);
//      res.result_status = res.SUCCEEDED;
//      res.obj_poses = on_top_object_poses_;
//      res.categories = on_top_object_categories_;
//    } else if (time_interval < 0) {
//      ROS_WARN("HoPE service: Extract on top object failed due to looking into past %d.", time_interval);
//      res.result_status = res.FAILED;
//    } else {
//      ROS_INFO("HoPE Service: Extract on top object succeeded.");
//      res.result_status = res.SUCCEEDED;
//      res.obj_poses = on_top_object_poses_;
//      res.categories = on_top_object_categories_;
//    }
//  } else {
//    res.result_status = res.FAILED;
//  }
//  return true;
//}
//
//void PlaneSegmentRT::computeNormalAndFilter()
//{
//  Utilities::estimateNorm(src_dsp_mono_, src_normals_, 1.01 * th_grid_rsl_);
//  Utilities::getCloudByNorm(src_normals_, idx_norm_fit_, th_norm_);
//  if (idx_norm_fit_->indices.empty()) {
//    ROS_WARN("HoPE: No point fits the normal criteria");
//    return;
//  }
//  Utilities::getCloudByInliers(src_dsp_mono_, cloud_norm_fit_mono_, idx_norm_fit_, false, false);
//  Utilities::getCloudByInliers(src_normals_, cloud_norm_fit_, idx_norm_fit_, false, false);
//}
//
//void PlaneSegmentRT::findAllPlanes()
//{
//  zClustering(cloud_norm_fit_mono_); // -> z_clustered_indices_list_
//  getMeanZofEachCluster(cloud_norm_fit_mono_); // -> plane_z_values_
//  extractPlaneForEachZ(cloud_norm_fit_mono_);
//}
//
//void PlaneSegmentRT::reset()
//{
//  max_plane_cloud_.reset(new Cloud_XYZ);
//  max_plane_contour_.reset(new Cloud_XYZ);
//
//  plane_z_values_.clear();
//  seed_clusters_indices_.clear();
//
//  max_plane_points_num_ = 0;
//  //hst_.reset();
//}
//
//void PlaneSegmentRT::getMeanZofEachCluster(Cloud_XYZ::Ptr cloud_norm_fit_mono)
//{
//  if (seed_clusters_indices_.empty())
//    ROS_WARN("PlaneSegment: Z growing got nothing.");
//
//  else {
//    size_t k = 0;
//    // Traverse each part to determine its mean Z value
//    for (const auto & seed_clusters_indice : seed_clusters_indices_) {
//      Cloud_XYZ::Ptr cloud_fit_part(new Cloud_XYZ);
//
//      pcl::PointIndices::Ptr idx_seed(new pcl::PointIndices);
//      idx_seed->indices = seed_clusters_indice.indices;
//      Utilities::getCloudByInliers(cloud_norm_fit_mono, cloud_fit_part, idx_seed, false, false);
//
//      float z_mean, z_max, z_min, z_mid;
//      Utilities::getCloudZInfo<Cloud_XYZ::Ptr>(cloud_fit_part, z_mean, z_max, z_min, z_mid);
//      plane_z_values_.push_back(z_mean);
//      k++;
//    }
//
//    ROS_DEBUG("Hypothesis plane number: %d", int(plane_z_values_.size()));
//    // Z is ordered from small to large, i.e., low to high
//    //sort(planeZVector_.begin(), planeZVector_.end());
//  }
//}
//
//void PlaneSegmentRT::extractPlaneForEachZ(Cloud_XYZ::Ptr cloud_norm_fit)
//{
//  size_t id = 0;
//  for (float & plane_z_value : plane_z_values_) {
//    getPlane(id, plane_z_value, cloud_norm_fit);
//    id++;
//  }
//}
//
//void PlaneSegmentRT::getPlane(size_t id, float z_in, Cloud_XYZ::Ptr &cloud_norm_fit_mono)
//{
//  if (z_in > min_height_ && z_in < max_height_) {
//    pcl::PointIndices::Ptr idx_seed(new pcl::PointIndices);
//    idx_seed->indices = seed_clusters_indices_[id].indices;
//
//    // Extract the plane points indexed by idx_seed
//    Cloud_XYZ::Ptr cloud_z(new Cloud_XYZ);
//    Utilities::getCloudByInliers(cloud_norm_fit_mono, cloud_z, idx_seed, false, false);
//
//    // If the points do not pass the error test, return
//    if (!gaussianImageAnalysis(id)) return;
//
//    if (aggressive_merge_) {
//      if (Utilities::isPointCloudValid(max_plane_cloud_)) {
//        if (fabs(max_plane_z_ - z_in) <= th_z_rsl_) {
//          Cloud_XYZ::Ptr cloud_combined(new Cloud_XYZ);
//          Utilities::combineCloud(cloud_z, max_plane_cloud_, cloud_combined);
//          cloud_z = cloud_combined;
//        }
//      }
//    }
//
//    // Update the data of the max plane detected
//    if (cloud_z->points.size() > max_plane_points_num_) {
//      max_plane_cloud_ = cloud_z;
//      // Use convex hull to represent the plane patch
//      Utilities::computeHull(max_plane_cloud_, max_plane_contour_);
//      max_plane_z_ = z_in;
//      max_plane_points_num_ = cloud_z->points.size();
//    }
//  }
//}
//
//void PlaneSegmentRT::zClustering(const Cloud_XYZ::Ptr& cloud_norm_fit_mono)
//{
//  ZGrowing zg;
//  pcl::search::Search<pcl::PointXYZ>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> >
//      (new pcl::search::KdTree<pcl::PointXYZ>);
//
//  zg.setMinClusterSize(3);
//  zg.setMaxClusterSize(INT_MAX);
//  zg.setSearchMethod(tree);
//  zg.setNumberOfNeighbours(8);
//  zg.setInputCloud(cloud_norm_fit_mono);
//  zg.setZThreshold(th_z_rsl_);
//
//  zg.extract(seed_clusters_indices_);
//}
//
//void PlaneSegmentRT::visualizeResult()
//{
//  // For visualizing in RViz
//  if (Utilities::isPointCloudValid(max_plane_cloud_)) {
//    Utilities::publishCloud(max_plane_cloud_, max_plane_cloud_publisher_, base_frame_);
//    Utilities::publishCloud(max_plane_contour_, max_plane_hull_publisher_, base_frame_);
//    ROS_INFO("Max plane z=%.3f detected in height range %.3f, %.3f", max_plane_z_, min_height_, max_height_);
//  } else {
//    ROS_WARN("No plane detected in height range %.3f, %.3f", min_height_, max_height_);
//  }
//}
//
//bool PlaneSegmentRT::gaussianImageAnalysis(size_t id)
//{
//  /// Get normal cloud for current cluster
//  pcl::PointIndices::Ptr idx_seed (new pcl::PointIndices);
//  idx_seed->indices = seed_clusters_indices_[id].indices;
//
//  // Extract the plane points indexed by idx_seed
//  Cloud_N::Ptr cluster_normal(new Cloud_N);
//  Utilities::getCloudByInliers(cloud_norm_fit_, cluster_normal, idx_seed, false, false);
//
//  /// Construct a Pointcloud to store normal points
//  if (show_egi_) {
//    Cloud_XYZ::Ptr cloud(new Cloud_XYZ);
//    cloud->width = cluster_normal->width;
//    cloud->height = cluster_normal->height;
//    cloud->resize(cluster_normal->width * cluster_normal->height);
//
//    size_t k = 0;
//    for (Cloud_XYZ::const_iterator pit = cloud->begin();
//         pit != cloud->end(); ++pit) {
//      cloud->points[k].x = cluster_normal->points[k].normal_x;
//      cloud->points[k].y = cluster_normal->points[k].normal_y;
//      cloud->points[k].z = fabs(cluster_normal->points[k].normal_z);
//      k++;
//    }
//    pcl::visualization::PCLVisualizer viewer ("EGI and normals distribution");
//    viewer.setBackgroundColor(0.8, 0.83, 0.86);
//    viewer.addPointCloud(cloud, "normals");
//    viewer.addCoordinateSystem(0.5);
//
//    /// Use wire frame sphere
//    //pcl::PointXYZ p;
//    //p.x = 0; p.y = 0; p.z = 0;
//    //viewer.addSphere(p, 1.0, 0, 0.6, 0.8, "EGI");
//    //viewer.setRepresentationToWireframeForAllActors();
//
//    /// Use point cloud sphere
//    int bandwidth = 128;
//    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere (new pcl::PointCloud<pcl::PointXYZRGB>);
//    sphere->resize (4 * bandwidth * bandwidth);
//    uint8_t r = 0, g = 0, b = 0;
//    uint32_t rgb = 0;
//    float tmp_theta = 0.0;
//    float tmp_phi = 0.0;
//    for (size_t i = 0; i < 2 * bandwidth; i++) {
//      tmp_theta = (2 * i + 1) * M_PI / 4 / bandwidth;
//      for (size_t j = 0; j < 2 * bandwidth; j++) {
//        tmp_phi = M_PI * j / bandwidth;
//        sphere->points[i * 2 * bandwidth + j].x = cos (tmp_phi) * sin (tmp_theta);
//        sphere->points[i * 2 * bandwidth + j].y = sin (tmp_phi) * sin (tmp_theta);
//        sphere->points[i * 2 * bandwidth + j].z = cos (tmp_theta);
//        Utilities::heatmapRGB(float(i)/bandwidth/4, r, g, b);
//        rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
//        sphere->points[i * 2 * bandwidth + j].rgb = *reinterpret_cast<float*> (&rgb);
//      }
//    }
//    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> sp_rgb(sphere);
//    viewer.addPointCloud<pcl::PointXYZRGB>(sphere, sp_rgb, "egi");
//    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.0, "egi");
//
//    // Show normal distribution on the sphere
//    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2.0, "normals");
//    viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0.4, 0, "normals");
//
//    while (!viewer.wasStopped()) {
//      viewer.spinOnce(1); // ms
//    }
//  }
//
//  return Utilities::normalAnalysis(cluster_normal, th_angle_);
//}
//
//bool PlaneSegmentRT::postProcessing(bool do_cluster, string type) {
//  if (Utilities::isPointCloudValid(max_plane_contour_)) {
//    vector<Cloud_XYZ::Ptr> clusters;
//    if (do_cluster) {
//      bool ok = Utilities::getClustersUponPlane(src_mono_cloud_, max_plane_contour_, clusters);
//      ROS_INFO("HoPE: Object clusters on plane #: %d", int(clusters.size()));
//      if (!ok) return false;
//    } else {
//      pcl::PointIndices::Ptr indices(new pcl::PointIndices);
//      Cloud_XYZ::Ptr upper_cloud(new Cloud_XYZ);
//      Utilities::getCloudByZ(src_mono_cloud_, indices, upper_cloud, max_plane_z_ + 0.05f, 1000.0f);
//      if (!Utilities::isPointCloudValid(upper_cloud)) {
//        ROS_WARN("HoPE: No point cloud on the max plane.");
//        return false;
//      }
//      clusters.push_back(upper_cloud);
//    }
//
//    on_top_object_poses_.poses.clear();
//    on_top_object_categories_.clear();
//
//    if (type != "mesh") {
//      for (auto & cloud : clusters) {
//        geometry_msgs::Pose pose;
//        if (type == "cylinder") {
//          bool ok = Utilities::getCylinderPose(cloud, pose, origin_height_);
//          if (!ok) continue;
//        } else if (type == "box") {
//          try {
//            bool ok = Utilities::getBoxPose(cloud, pose, origin_height_);
//            if (!ok) continue;
//          } catch (cv::Exception) {
//            ROS_WARN("HoPE: Exception raised during getting box pose");
//            continue;
//          }
//        } else if (type == "box_top") {
//          try {
//            int category;
//            bool ok = Utilities::getBoxTopPose(cloud, pose, category, origin_heights_);
//            if (!ok) continue;
//            on_top_object_categories_.push_back(category);
//          } catch (cv::Exception) {
//            ROS_WARN("HoPE: Exception raised during getting box pose");
//            continue;
//          }
//        } else {
//          ROS_WARN("HoPE: Unknown object type %s", type.c_str());
//          return false;
//        }
//        on_top_object_poses_.poses.push_back(pose);
//      }
//    } else {
//      Cloud_XYZN::Ptr scene_cloud(new Cloud_XYZN);
//      Utilities::convertCloudType(clusters[0], scene_cloud);
//      Eigen::Matrix4f trans;
//      pe_->estimate(scene_cloud, trans);
//      Utilities::matrixToPoseArray(trans, on_top_object_poses_);
//    }
//    on_top_object_poses_.header.stamp = ros::Time::now();
//    on_top_object_poses_.header.frame_id = base_frame_;
//    on_plane_obj_publisher_.publish(on_top_object_poses_);
//    return true;
//  } else {
//    ROS_WARN("HoPE: No valid plane for extracting objects on top.");
//    return false;
//  }
//}
