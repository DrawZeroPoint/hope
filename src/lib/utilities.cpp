#include "hope/utilities.h"

using namespace std;
using namespace cv;

Vec3b pascal_map[] = {
    Vec3b(0,0,0),
    Vec3b(128,0,0),
    Vec3b(0,128,0),
    Vec3b(128,128,0),
    Vec3b(0,0,128),
    Vec3b(128,0,128),
    Vec3b(0,128,128),
    Vec3b(128,128,128),
    Vec3b(64,0,0),
    Vec3b(192,0,0),
    Vec3b(64,128,0),
    Vec3b(192,128,0),
    Vec3b(64,0,128),
    Vec3b(192,0,128),
    Vec3b(64,128,128),
    Vec3b(192,128,128),
    Vec3b(0,64,0),
    Vec3b(128,64,0),
    Vec3b(0,192,0),
    Vec3b(128,192,0),
    Vec3b(0,64,128),
    Vec3b(128,64,128),
    Vec3b(0,192,128),
    Vec3b(128,192,128),
    Vec3b(64,64,0),
    Vec3b(192,64,0),
    Vec3b(64,192,0),
    Vec3b(192,192,0),
    Vec3b(64,64,128),
    Vec3b(192,64,128),
    Vec3b(64,192,128),
    Vec3b(192,192,128),
    Vec3b(0,0,64),
    Vec3b(128,0,64),
    Vec3b(0,128,64),
    Vec3b(128,128,64),
    Vec3b(0,0,192),
    Vec3b(128,0,192),
    Vec3b(0,128,192),
    Vec3b(128,128,192),
    Vec3b(64,0,64),
    Vec3b(192,0,64),
    Vec3b(64,128,64),
    Vec3b(192,128,64),
    Vec3b(64,0,192),
    Vec3b(192,0,192),
    Vec3b(64,128,192),
    Vec3b(192,128,192),
    Vec3b(0,64,64),
    Vec3b(128,64,64),
    Vec3b(0,192,64),
    Vec3b(128,192,64),
    Vec3b(0,64,192),
    Vec3b(128,64,192),
    Vec3b(0,192,192),
    Vec3b(128,192,192),
    Vec3b(64,64,64),
    Vec3b(192,64,64),
    Vec3b(64,192,64),
    Vec3b(192,192,64),
    Vec3b(64,64,192),
    Vec3b(192,64,192),
    Vec3b(64,192,192),
    Vec3b(192,192,192)
};

Utilities::Utilities() = default;

float Utilities::determinant(float v1, float v2, float v3, float v4)
{
  return (v1 * v3 - v2 * v4);
}

std::string Utilities::getName(int count, const string& pref, int surf)
{
  std::string name;
  std::ostringstream ost;
  ost << count;
  if (surf >= 0)
    ost << surf;
  std::string temp(ost.str());
  name = pref + temp;
  return name;
}

void Utilities::getOccupancyMap(const Cloud_XYZ::Ptr& cloud_src, const Cloud_XYZ::Ptr& cloud_upper,
                                std::vector<int> occupy, Cloud_XYZRGB::Ptr &cloud_out)
{
  int minv, maxv;
  getMinMax(occupy, minv, maxv);
  cloud_out->resize(cloud_src->size() + cloud_upper->size());

  size_t k = 0;
  for (auto pit : *cloud_src) {
    float rgb = Utilities::shortRainbowColorMap(occupy[k], minv, maxv);

    cloud_out->points[k].x = pit.x;
    cloud_out->points[k].y = pit.y;
    cloud_out->points[k].z = pit.z;
    cloud_out->points[k].rgb = rgb;
    k++;
  }

  size_t j = 0;
  for (Cloud_XYZ::const_iterator pit = cloud_upper->begin();
       pit != cloud_upper->end(); ++pit) {
    cloud_out->points[cloud_src->size() + j].x = pit->x;
    cloud_out->points[cloud_src->size() + j].y = pit->y;
    cloud_out->points[cloud_src->size() + j].z = pit->z;
    cloud_out->points[cloud_src->size() + j].r = 255;
    cloud_out->points[cloud_src->size() + j].g = 0;
    cloud_out->points[cloud_src->size() + j].b = 0;
    j++;
  }
}

void Utilities::getPointByZ(float z, const Cloud_XYZ::Ptr& cloud_in, pcl::PointXYZ &pt)
{
  for (auto pit : *cloud_in) {
    if (pit.z == z) {
      pt.x = pit.x;
      pt.y = pit.y;
      pt.z = pit.z;
      break;
    }
  }
}

void Utilities::msgToCloud(const Cloud_XYZRGB::ConstPtr& msg, const Cloud_XYZ::Ptr& cloud)
{
  cloud->height = msg->height;
  cloud->width  = msg->width;
  cloud->is_dense = false;
  cloud->resize(cloud->height * cloud->width);

  size_t i = 0;
  for (const auto & pit : *msg) {
    cloud->points[i].x = pit.x;
    cloud->points[i].y = pit.y;
    cloud->points[i].z = pit.z;
    ++i;
  }
}

bool Utilities::normalAnalysis(const Cloud_N::Ptr& cloud, double th_angle)
{
  size_t sz = cloud->points.size();
  if (sz <= 2) return false;

  Eigen::Matrix3Xf data(3, sz);
  for (long i = 0; i < sz; ++i) {
    data(0, i) = cloud->points[i].normal_x;
    data(1, i) = cloud->points[i].normal_y;
    // Notice that the normal direction can be both positive and negative
    data(2, i) = fabs(cloud->points[i].normal_z);
  }

  Eigen::Vector3f mean = data.rowwise().mean();

  // Check mean
  Eigen::Vector2f norm_proj;
  norm_proj(0) = mean(0);
  norm_proj(1) = mean(1);

  float grad = asin(norm_proj.norm() / mean.norm());
  if (grad > th_angle)
    return false;

  /// Divide the data points into 2 distinct parts using PCA
  Eigen::Matrix2Xf data_2d(2, sz);
  for (long i = 0; i < sz; ++i) {
    data_2d(0, i) = cloud->points[i].normal_x;
    data_2d(1, i) = cloud->points[i].normal_y;
  }

  Eigen::Vector2f mean_2d = data_2d.rowwise().mean();

  Eigen::MatrixXf tmp(2, sz);
  tmp = data_2d.colwise() - mean_2d;

  Eigen::MatrixXf C = (tmp * tmp.transpose()) / (sz - 1);

  Eigen::EigenSolver<Eigen::MatrixXf> es(C);
  // The result is complex number
  complex<float> lambda0 = es.eigenvalues()[0];
  complex<float> lambda1 = es.eigenvalues()[1];
  Eigen::MatrixXcf V = es.eigenvectors();

  // The first and second principal axes
  complex<float> val_x_1;
  complex<float> val_y_1;

  if (lambda0.real() > lambda1.real()) {
    val_x_1 = V.col(0)[0];
    val_y_1 = V.col(0)[1];
  }
  else {
    val_x_1 = V.col(1)[0];
    val_y_1 = V.col(1)[1];
  }

  Eigen::Vector2f axis0;
  axis0(0) = val_x_1.real();
  axis0(1) = val_y_1.real();

  vector<int> part1;
  vector<int> part2;
  for (long i = 0; i < sz; ++i) {
    Eigen::Vector2f p;
    p(0) = tmp(0, i);
    p(1) = tmp(1, i);
    if (axis0.transpose() * p > 0) {
      part1.push_back(i);
    }
    else {
      part2.push_back(i);
    }
  }

  Eigen::Vector3f mean_part1;
  Eigen::Vector3f mean_part2;
  if (!calNormalMean(data, part1, part2, mean_part1, mean_part2))
    return false;
  float mu = mean_part2.transpose() * mean_part1;
  float rad_mu = acos(mu / (mean_part1.norm() * mean_part2.norm()));

  /// Using standard deviation value or 68-95-97 rule to judge whether the distribution
  /// is a gaussian distribution is less accurate and relies on extra thresholds
  //  Eigen::Vector3f std(3, 1);
  //  std = ((data.colwise() - mean).array().pow(2).rowwise().sum() / (sz-1)).abs().sqrt();
  //  int pn = 0.7 * sz;
  //  int num_in_one_theta_x = 0;
  //  for (size_t i = 0; i < sz; ++i) {
  //    if (fabs(data(0, i) - mean(0)) <= std(0)) {
  //      num_in_one_theta_x++;
  //    }
  //  }

  //  int num_in_one_theta_y = 0;
  //  for (size_t i = 0; i < sz; ++i) {
  //    if (fabs(data(1, i) - mean(1)) <= std(1)) {
  //      num_in_one_theta_y++;
  //    }
  //  }
  //|| std(0) > 0.15 || std(1) > 0.15 || (num_in_one_theta_x >= pn || num_in_one_theta_y >= pn)
  /// Only for reference, DO NOT unlock the code above

  return rad_mu <= th_angle;
}

bool Utilities::calNormalMean(Eigen::Matrix3Xf data, vector<int> part1, vector<int> part2,
                              Eigen::Vector3f &mean_part1, Eigen::Vector3f &mean_part2)
{
  if (part1.empty() || part2.empty())
    return false;
  Eigen::Matrix3Xf data1(3, part1.size());
  Eigen::Matrix3Xf data2(3, part2.size());

  int j = 0;
  int k = 0;
  for (int i : part1) {
    data1(0, j) = data(0, i);
    data1(1, j) = data(1, i);
    data1(2, j) = data(2, i);
    j++;
  }
  for (int i : part2) {
    data2(0, k) = data(0, i);
    data2(1, k) = data(1, i);
    data2(2, k) = data(2, i);
    k++;
  }
  mean_part1 = data1.rowwise().mean();
  mean_part2 = data2.rowwise().mean();
  return true;
}

bool Utilities::calRANSAC(const Cloud_XYZ::ConstPtr& cloud_3d_in, float dt, float &grad)
{
  size_t sz = cloud_3d_in->points.size();
  if (sz <= 3) return false;
  // Pose heuristics
  pcl::SACSegmentation<pcl::PointXYZ> planeSegmenter;
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  // set the segmentaion parameters
  planeSegmenter.setOptimizeCoefficients(true);
  planeSegmenter.setModelType(pcl::SACMODEL_PLANE);
  planeSegmenter.setMethodType(pcl::SAC_RANSAC);
  planeSegmenter.setMaxIterations(100);
  planeSegmenter.setDistanceThreshold(dt);

  // Extract the global (environment) dominant plane
  planeSegmenter.setInputCloud(cloud_3d_in);
  planeSegmenter.segment(*inliers, *coefficients);

  Eigen::Vector3f normal;
  Eigen::Vector2f norm_proj;
  normal(0) = coefficients->values[0];
  normal(1) = coefficients->values[1];
  normal(2) = coefficients->values[2];
  norm_proj(0) = normal(0);
  norm_proj(1) = normal(1);

  grad = asin(norm_proj.norm() / normal.norm());
  return true;
}

void Utilities::getFurthestPointsAlongAxis(Eigen::Vector2f axis, Eigen::MatrixXf data,
                                           vector<int> &inlist, int &id_max, int &id_min)
{
  int sz = data.cols();
  float vmin = FLT_MAX;
  float vmax = -FLT_MAX;
  id_max = -1;
  id_min = -1;

  Eigen::Vector2f data_point;
  int pos;
  for (size_t i = 0; i < sz; ++i) {
    data_point(0) = data(0, i);
    data_point(1) = data(1, i);
    float v = axis.transpose() * data_point; // Dot product
    if (v > vmax) {
      if (!isInVector(i, inlist, pos)) {
        vmax = v;
        id_max = i;
      }
    }
    if (v < vmin) {
      if (!isInVector(i, inlist, pos)) {
        vmin = v;
        id_min = i;
      }
    }
  }
  inlist.push_back(id_max);
  inlist.push_back(id_min);
}

void Utilities::calRegionGrowing(Cloud_XYZRGBN::Ptr cloud_in, int minsz, int maxsz, int nb, int smooth,
                                 pcl::PointCloud<pcl::Normal>::Ptr normals, vector<pcl::PointIndices> &inliers)
{
  pcl::RegionGrowing<pcl::PointXYZRGBNormal, pcl::Normal> reg;
  pcl::search::Search<pcl::PointXYZRGBNormal>::Ptr tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZRGBNormal> >
      (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);

  reg.setMinClusterSize(minsz);
  reg.setMaxClusterSize(maxsz);
  reg.setSearchMethod(tree);
  reg.setNumberOfNeighbours(nb);
  reg.setInputCloud(cloud_in);
  reg.setInputNormals(normals);
  reg.setSmoothnessThreshold(smooth / 180.0 * M_PI);

  reg.extract(inliers);
}

void Utilities::estimateNorm(const Cloud_XYZ::Ptr& cloud_in,
                             Cloud_XYZRGBN::Ptr &cloud_out,
                             Cloud_N::Ptr &normals_out,
                             float norm_r)
{
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

  // Compute the normals
  ne.compute(*normals_out);

  /// Do it in parallel
  //  // Declare PCL objects needed to perform normal estimation
  //  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> normal_estimation;
  //  pcl::search::KdTree<pcl::PointXYZ>::Ptr search_tree(new pcl::search::KdTree<pcl::PointXYZ>);

  //  // Set input parameters for normal estimation
  //  search_tree->setInputCloud(cloud_in);
  //  normal_estimation.setInputCloud(cloud_in);
  //  normal_estimation.setSearchMethod(search_tree);

  //  /*
  //     * When estimating normals, the algorithm looks at the nearest neighbors of every point
  //     * and fits a plane to these points as close as it can. The normal of this plane is
  //     * the estimated normal of the point.
  //     * This sets how many of the nearest neighbors to look at when estimating normals.
  //     * Is a rough setting for accuracy that can be adjusted.
  //     * A lower number here means that corners in the point cloud will be more accurate,
  //     * too low a number will cause problems.
  //     */
  //  normal_estimation.setKSearch(6);

  //  // Perform normal estimation algorithm
  //  normal_estimation.compute(*normals_out);

  cloud_out->height = cloud_in->height;
  cloud_out->width  = cloud_in->width;
  cloud_out->is_dense = false;
  cloud_out->resize(cloud_out->height * cloud_out->width);

  // Generate rgbn cloud
  for (size_t i = 0; i < cloud_out->size(); ++i) {
    cloud_out->points[i].x = cloud_in->points[i].x;
    cloud_out->points[i].y = cloud_in->points[i].y;
    cloud_out->points[i].z = cloud_in->points[i].z;
    cloud_out->points[i].r = 255;
    cloud_out->points[i].g = 255;
    cloud_out->points[i].b = 255;
    cloud_out->points[i].normal_x = normals_out->points[i].normal_x;
    cloud_out->points[i].normal_y = normals_out->points[i].normal_y;
    cloud_out->points[i].normal_z = normals_out->points[i].normal_z;
  }
}

void Utilities::estimateNorm(const Cloud_XYZ::Ptr& cloud_in,
                             Cloud_N::Ptr &normals_out,
                             double norm_r)
{
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

  // Compute the normals
  ne.compute(*normals_out);
}

void Utilities::estimateNormals(const Cloud_XYZN::Ptr &cloud_in, Cloud_XYZN::Ptr &cloud_out, float dsp_th) {
  pcl::NormalEstimationOMP<PointN, PointN> nest;
  nest.setRadiusSearch(dsp_th * 2);
  nest.setInputCloud(cloud_in);
  nest.compute(*cloud_out);
}

void Utilities::downSampling(const Cloud_XYZ::Ptr& cloud_in,
                             Cloud_XYZ::Ptr &cloud_out,
                             float grid_sz, float z_sz)
{
  if (grid_sz > 0 && z_sz > 0) {
    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(cloud_in);
    vg.setLeafSize(grid_sz, grid_sz, z_sz);
    vg.filter(*cloud_out);
  } else {
    cloud_out = cloud_in;
  }
}

void Utilities::downSampling(const Cloud_XYZRGB::Ptr& cloud_in,
                             Cloud_XYZRGB::Ptr &cloud_out,
                             float grid_sz, float z_sz)
{
  if (grid_sz > 0 && z_sz > 0) {
    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZRGB> vg;
    vg.setInputCloud(cloud_in);
    vg.setLeafSize(grid_sz, grid_sz, z_sz);
    vg.filter(*cloud_out);
  } else {
    cloud_out = cloud_in;
  }
}

void Utilities::downSampling(const Cloud_XYZN::Ptr &cloud_in,
                             Cloud_XYZN::Ptr &cloud_out,
                             float grid_sz, float z_sz)
{
  if (grid_sz > 0 && z_sz > 0) {
    // Create the filtering object
    pcl::VoxelGrid<PointN> vg;
    vg.setInputCloud(cloud_in);
    vg.setLeafSize(grid_sz, grid_sz, z_sz);
    vg.filter(*cloud_out);
  } else {
    cloud_out = cloud_in;
  }
}

void Utilities::planeTo2D(float z, Cloud_XYZ::Ptr cloud_in,
                          Cloud_XYZ::Ptr &cloud_out)
{
  cloud_out->width = cloud_in->width;
  cloud_out->height = cloud_in->height;
  cloud_out->resize(cloud_out->width *cloud_out->height);

  size_t k = 0;
  for (Cloud_XYZ::const_iterator pit = cloud_in->begin();
       pit != cloud_in->end(); ++pit) {
    cloud_out->points[k].x = pit->x;
    cloud_out->points[k].y = pit->y;
    cloud_out->points[k].z = z;
    k++;
  }
}

void Utilities::convertToColorCloud(const Cloud_XYZ::Ptr& cloud_in,
                                    Cloud_XYZRGB::Ptr &cloud_out,
                                    int r, int g, int b)
{
  cloud_out->resize(cloud_in->size());

  for (size_t i = 0; i < cloud_in->points.size(); i++) {
    cloud_out->points[i].x = cloud_in->points[i].x;
    cloud_out->points[i].y = cloud_in->points[i].y;
    cloud_out->points[i].z = cloud_in->points[i].z;
    cloud_out->points[i].r = r;
    cloud_out->points[i].g = g;
    cloud_out->points[i].b = b;
  }
}

template <typename T, typename U>
void Utilities::sliceCloudWithPlane(pcl::ModelCoefficients::Ptr coeff_in, float th_distance,
                                    T cloud_in, U &cloud_out)
{
  Eigen::Vector4f coeff(coeff_in->values[0], coeff_in->values[1],
                        coeff_in->values[2], coeff_in->values[3]);

  Cloud_XYZ::Ptr cloud_in_mono(new Cloud_XYZ);
  std::vector<int> inliers;
  convertCloudType(cloud_in, cloud_in_mono);
  pcl::SampleConsensusModelPlane<pcl::PointXYZ> scmp(cloud_in_mono);
  int slice_count = 5;
  while (slice_count) {
    inliers.clear();
    scmp.selectWithinDistance(coeff, th_distance, inliers);
    if (inliers.size() >= 4) break;
    else th_distance += 0.001;
    slice_count--;
  }
  assert(!inliers.empty());
  scmp.projectPoints(inliers, coeff, *cloud_out, false);
}

void Utilities::extractClusters(Cloud_XYZ::Ptr cloud_in,
                                vector<pcl::PointIndices> &cluster_indices,
                                float th_cluster, int minsize, int maxsize)
{
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
  ec.setClusterTolerance(th_cluster);

  // 3 is used for minsize and 640*480 is used for maxsize
  // cause the filter of cluster should be performed somewhere else
  ec.setMinClusterSize(minsize);
  ec.setMaxClusterSize(maxsize);
  ec.setSearchMethod(tree);
  ec.setInputCloud(cloud_in);
  ec.extract(cluster_indices);
}

void Utilities::projectCloudTo2D(const pcl::ModelCoefficients::Ptr& coeff_in,
                                 const Cloud_XYZ::Ptr& cloud_in,
                                 Cloud_XYZ::Ptr &cloud_out)
{
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(cloud_in);
  proj.setModelCoefficients(coeff_in);
  proj.filter(*cloud_out);
}

void Utilities::rotateCloudXY(const Cloud_XYZRGBN::Ptr& cloud_in, Cloud_XYZRGBN::Ptr &cloud_out,
                              float rx, float ry, Eigen::Matrix4f &transform_inv)
{
  Eigen::Matrix4f transform_x = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f transform_y = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f transform_ = Eigen::Matrix4f::Identity();

  //the math function cos etc. operate angle in radius
  transform_x(1,1) = cos(rx);
  transform_x(2,1) = -sin(rx);
  transform_x(1,2) = sin(rx);
  transform_x(2,2) = cos(rx);
  //		std::cout << "trans x: "<< transform_x << std::endl;

  transform_y(0,0) = cos(ry);
  transform_y(0,2) = -sin(ry);
  transform_y(2,0) = sin(ry);
  transform_y(2,2) = cos(ry);
  //		std::cout << "trans y: "<< transform_y << std::endl;

  transform_ = transform_y * transform_x;
  //		std::cout << "total trans: "<< transform_ << std::endl;
  transform_inv = transform_.inverse();
  //		std::cout << "trans_inv: "<< transform_inv << std::endl;

  // Executing the transformation
  pcl::transformPointCloudWithNormals(*cloud_in, *cloud_out, transform_);
}

void Utilities::getCloudByNorm(const Cloud_XYZRGBN::Ptr& cloud_in,
                               pcl::PointIndices::Ptr &inliers,
                               float th_norm)
{
  size_t i = 0;
  for (const auto & pit : *cloud_in) {
    float n_z = pit.normal_z;
    // If point normal fulfill this criterion, consider it from plane
    // Here we use absolute value cause the normal direction may be opposite to
    // the z-axis due to the algorithm's settings
    if (fabs(n_z) > th_norm)
      inliers->indices.push_back(i);
    ++i;
  }
}

void Utilities::getCloudByNorm(const Cloud_N::Ptr& cloud_in,
                               pcl::PointIndices::Ptr &inliers,
                               double th_norm)
{
  inliers->indices.clear();
  int i = 0;
  for (const auto & p : *cloud_in) {
    float n_z = p.normal_z;
    // If point normal fulfill this criterion, consider it from plane
    // Here we use absolute value cause the normal direction may be opposite to
    // the z-axis due to the algorithm's settings
    /// The normal is an unit vector
    if (fabs(n_z) > th_norm)
      inliers->indices.push_back(i);
    ++i;
  }
}

void Utilities::getCloudByZ(const Cloud_XYZ::Ptr& cloud_in,
                            pcl::PointIndices::Ptr &inliers,
                            Cloud_XYZ::Ptr &cloud_out,
                            double z_min, double z_max)
{
  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(z_min, z_max);
  //pass.setFilterLimitsNegative (true);
  pass.filter(inliers->indices);
  pass.filter(*cloud_out);
}

void Utilities::getCloudByZ(const Cloud_XYZRGB::Ptr& cloud_in,
                            pcl::PointIndices::Ptr &inliers,
                            Cloud_XYZRGB::Ptr &cloud_out,
                            float z_min, float z_max)
{
  // Create the filtering object
  pcl::PassThrough<pcl::PointXYZRGB> pass;
  pass.setInputCloud(cloud_in);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(z_min, z_max);
  //pass.setFilterLimitsNegative (true);
  pass.filter(inliers->indices);
  pass.filter(*cloud_out);
}

void Utilities::getCloudByInliers(const Cloud_XYZ::Ptr& cloud_in,
                                  Cloud_XYZ::Ptr &cloud_out,
                                  const pcl::PointIndices::Ptr& inliers,
                                  bool negative, bool organized)
{
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setNegative(negative);
  extract.setInputCloud(cloud_in);
  extract.setIndices(inliers);
  extract.setKeepOrganized(organized);
  extract.filter(*cloud_out);
}

void Utilities::getCloudByInliers(const Cloud_N::Ptr& cloud_in,
                                  Cloud_N::Ptr &cloud_out,
                                  const pcl::PointIndices::Ptr& inliers,
                                  bool negative, bool organized)
{
  pcl::ExtractIndices<pcl::Normal> extract;
  extract.setNegative(negative);
  extract.setInputCloud(cloud_in);
  extract.setIndices(inliers);
  extract.setKeepOrganized(organized);
  extract.filter(*cloud_out);
}

void Utilities::getCloudByInliers(const Cloud_XYZRGBN::Ptr& cloud_in,
                                  Cloud_XYZRGBN::Ptr &cloud_out,
                                  const pcl::PointIndices::Ptr& inliers,
                                  bool negative, bool organized)
{
  pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
  extract.setNegative(negative);
  extract.setInputCloud(cloud_in);
  extract.setIndices(inliers);
  extract.setKeepOrganized(organized);
  extract.filter(*cloud_out);
}

bool Utilities::checkWithIn(const pcl::PointIndices::Ptr& ref_inliers,
                            const pcl::PointIndices::Ptr& tgt_inliers)
{
  int size_ref = ref_inliers->indices.size();
  int size_tgt = tgt_inliers->indices.size();
  int half_size = size_tgt * 0.5f;

  // We take the advantage that all inliers are in order small to large
  // So sort is unnecessary
  //sort(ref_inliers->indices.begin(), ref_inliers->indices.end());
  //sort(tgt_inliers->indices.begin(), tgt_inliers->indices.end());

  int within = 0;

  if (tgt_inliers->indices[size_tgt - 1] < ref_inliers->indices[0] ||
      tgt_inliers->indices[0] > ref_inliers->indices[size_ref - 1]) {
    return false;
  }

  size_t start_pos = 0;

  for (size_t e = 0; e < size_tgt; ++e) {
    for (size_t i = start_pos; i < size_ref; ++i) {
      if (tgt_inliers->indices[e] == ref_inliers->indices[i]) {
        within++;
        start_pos = i + 1;
        break;
      }
    }
    if (within > half_size) {
      return true;
    }
  }
}

template <typename T>
void Utilities::getCloudZInfo(T cloud_in, float &z_mean, float &z_max, float &z_min, float &z_mid)
{
  z_max = -1000;
  z_min = 1000;
  float mid = 0.0;
  size_t ct = 0;
  for (auto pit = cloud_in->begin();
       pit != cloud_in->end(); ++pit) {
    if (isfinite(pit->z)) {
      if (pit->z > z_max) z_max = pit->z;
      if (pit->z < z_min) z_min = pit->z;
      mid += pit->z;
      ct++;
    }
  }
  z_mean = mid / ct;
  z_mid = (z_max + z_min) / 2.0f;
}

Vec3f Utilities::getColorWithID(int id)
{
  // ids range from 0 to 63
  id = id % 64;
  Vec3f cf(0.0, 0.0, 0.0);
  Vec3b c = pascal_map[id + 1];
  cf[0] = float(c[0]) * 0.0039; // =/256
  cf[1] = float(c[1]) * 0.0039;
  cf[2] = float(c[2]) * 0.0039;
  return cf;
}

float Utilities::getDistance(vector<float> v1, vector<float> v2)
{
  float rst = 0.0;
  assert(v1.size() == v2.size());
  for (size_t i = 0; i < v1.size(); ++i) {
    float f1 = v1[i];
    float f2 = v2[i];
    rst += fabs(f1 - f2);
  }
  return rst;
}

pcl::PolygonMesh Utilities::getMesh(const Cloud_XYZ::Ptr point_cloud,
                                    Cloud_N::Ptr normals)
{
  //    Cloud_N::Ptr normals(new Cloud_N);
  //    normals->height = plane_contour_list_[i]->height;
  //    normals->width  = plane_contour_list_[i]->width;
  //    normals->is_dense = true;
  //    normals->resize(normals->height * normals->width);
  //    for (size_t j = 0; j < normals->size(); ++j) {
  //      normals->points[j].normal_x = 0;
  //      normals->points[j].normal_y = 0;
  //      normals->points[j].normal_z = 1;
  //    }

  // Add the normals to the point cloud
  Cloud_XYZN::Ptr cloud_with_normals(new Cloud_XYZN);
  pcl::concatenateFields(*point_cloud, *normals, *cloud_with_normals);
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>);
  tree->setInputCloud(cloud_with_normals);
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  gp3.setSearchRadius(0.08);
  gp3.setMu(2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI);
  gp3.setMinimumAngle(0);
  gp3.setMaximumAngle(2*M_PI/3);
  gp3.setNormalConsistency(false);
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod(tree);
  gp3.reconstruct(triangles);
}

void Utilities::getMinMax(std::vector<int> vec, int &minv, int &maxv)
{
  // Small to large
  sort(vec.begin(), vec.end());
  minv = vec[0];
  maxv = vec[vec.size() - 1];
}

void Utilities::shrinkHull(const Cloud_XYZ::Ptr& cloud,
                           Cloud_XYZ::Ptr &cloud_sk, float dis)
{
  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D(*cloud, minPt, maxPt);

  size_t i = 0;
  float center_x = (maxPt.x + minPt.x) / 2;
  float center_y = (maxPt.y + minPt.y) / 2;
  for (Cloud_XYZ::const_iterator pit = cloud->begin();
       pit != cloud->end(); ++pit) {
    if (pit->x == center_x) {
      if (pit->y > center_y) {
        cloud_sk->points[i].y = (pit->y - dis)>center_y?(pit->y - dis):pit->y;
      }
      else {
        cloud_sk->points[i].y = (pit->y + dis)<center_y?(pit->y + dis):pit->y;
      }
      cloud_sk->points[i].x = pit->x;
    }
    else {
      float d_x = pit->x - center_x;
      float d_y = pit->y - center_y;
      float theta = atan(d_y/d_x);
      if (d_x > 0 && d_y >= 0) {
        cloud_sk->points[i].x = (pit->x - fabs(dis*sin(theta)))>center_x?
                                (pit->x - fabs(dis*sin(theta))):pit->x;
        cloud_sk->points[i].y = (pit->y - fabs(dis*cos(theta)))>center_y?
                                (pit->y - fabs(dis*cos(theta))):pit->y;
      }
      else if (d_x < 0 && d_y >= 0) {
        cloud_sk->points[i].x = (pit->x + fabs(dis*sin(theta)))<center_x?
                                (pit->x + fabs(dis*sin(theta))):pit->x;
        cloud_sk->points[i].y = (pit->y - fabs(dis*cos(theta)))>center_y?
                                (pit->y - fabs(dis*cos(theta))):pit->y;
      }
      else if (d_x < 0 && d_y < 0) {
        cloud_sk->points[i].x = (pit->x + fabs(dis*sin(theta)))<center_x?
                                (pit->x + fabs(dis*sin(theta))):pit->x;
        cloud_sk->points[i].y = (pit->y + fabs(dis*cos(theta)))<center_y?
                                (pit->y + fabs(dis*cos(theta))):pit->y;
      }
      else {
        cloud_sk->points[i].x = (pit->x - fabs(dis*sin(theta)))>center_x?
                                (pit->x - fabs(dis*sin(theta))):pit->x;
        cloud_sk->points[i].y = (pit->y + fabs(dis*cos(theta)))<center_y?
                                (pit->y + fabs(dis*cos(theta))):pit->y;
      }
    }
  }
}

bool Utilities::tryExpandROI(int &minx, int &miny, int &maxx, int &maxy,
                             int pad, int width, int height)
{
  if (minx >= maxx || miny >= maxy) {
    return false;
  }
  minx -= pad;
  maxx += pad;
  miny -= pad;
  maxy += pad;
  if (minx < 0) minx = 0;
  if (maxx > width) maxx = width - 1;
  if (miny < 0) miny = 0;
  if (maxy > height) maxy = height - 1;
}

bool Utilities::isIntersect(pcl::PointXY p1, pcl::PointXY p2,
                            pcl::PointXY p3, pcl::PointXY p4)
{
  float delta = determinant(p2.x-p1.x, p3.x-p4.x, p2.y-p1.y, p3.y-p4.y);
  if ( delta<=(1e-6) && delta>=-(1e-6) ) {
    return false;
  }
  float d = determinant(p3.x - p1.x, p3.x - p4.x, p3.y - p1.y, p3.y - p4.y) / delta;
  if (d > 1 || d < 0 ) {
    return false;
  }
  float miu = determinant(p2.x-p1.x, p3.x-p1.x, p2.y-p1.y, p3.y-p1.y) / delta;
  return !(miu > 1 || miu < 0);
}

void Utilities::matFill(vector<vector<float> > features, Mat &out)
{
  // Each row corresponds to a plane while each column to a feature
  for (size_t r = 0; r < features.size(); ++r) {
    for (size_t c = 0; c < features[r].size(); ++c) {
      out.at<float>(r, c) = features[r][c];
    }
  }
}

void Utilities::matNormalize(Mat query_in, Mat train_in,
                             Mat &query_out, Mat &train_out)
{
  query_out = Mat(query_in.rows, query_in.cols, query_in.type());
  train_out = Mat(train_in.rows, train_in.cols, train_in.type());

  for (size_t c = 0; c < query_in.cols; ++c) {

    float acc = 0;
    for (size_t r = 0; r < query_in.rows; ++r) {
      float val = query_in.at<float>(r, c);
      acc += val;
    }
    for (size_t r = 0; r < train_in.rows; ++r) {
      float val = train_in.at<float>(r, c);
      acc += val;
    }
    float r_mean = acc / float(query_in.rows + train_in.rows);

    acc = 0;
    for (size_t r = 0; r < query_in.rows; ++r) {
      float val = query_in.at<float>(r, c);
      acc += pow(val - r_mean, 2);
    }
    for (size_t r = 0; r < train_in.rows; ++r) {
      float val = train_in.at<float>(r, c);
      acc += pow(val - r_mean, 2);
    }

    float sd = sqrt(acc / float(query_in.rows + train_in.rows));

    for (size_t r = 0; r < query_in.rows; ++r) {
      float val = query_in.at<float>(r, c);
      val = (val - r_mean) / sd;
      query_out.at<float>(r, c) = val;
    }
    for (size_t r = 0; r < train_in.rows; ++r) {
      float val = train_in.at<float>(r, c);
      val = (val - r_mean) / sd;
      train_out.at<float>(r, c) = val;
    }
  }
}

// Note that the limit restricted the total number of planes being found
void Utilities::searchAvailableID(vector<int> id_used,
                                  vector<int> &id_ava,
                                  size_t limit=1024)
{
  //  sort(id_used.begin(),id_used.end());
  //  int min_id = id_used[0];
  //  int max_id = id_used[id_used.size()-1];
  //  for (size_t i = max_id + 1; i < max_id + limit; ++i) {
  //    int p = i;
  //    if (p >= limit) {
  //      p -= limit;
  //    }
  //    if (p > max_id || p < min_id)
  //      id_ava.push_back(p);
  //  }
  for (size_t i = 0; i < limit; ++i) {
    bool i_used = false;
    for (int n : id_used) {
      if (i == n) {
        i_used = true;
        break;
      }
    }
    if (!i_used)
      id_ava.push_back(i);
  }
}

void Utilities::getClosestPoint(pcl::PointXY p1, pcl::PointXY p2,
                                pcl::PointXY p, pcl::PointXY &pc)
{
  float A = p1.x - p2.x;
  float B = p1.y - p2.y;

  if (A == 0) {
    pc.x = p1.x;
    pc.y = p.y;
    return;
  }
  if (B == 0) {
    pc.x = p.x;
    pc.y = p1.y;
    return;
  }

  //pc.x = (A*(A*p.x + B*p.y)/B - A*p1.y + B*p1.x)/(B + A*A/B);
  //pc.y = B*(pc.x - p1.x)/A + p1.y;

  pc.y = (p.x - p1.x + A/B*p1.y + B/A*p.y)/(A/B + B/A);
  pc.x = A/B*(pc.y - p1.y) + p1.x;
}

bool Utilities::isInVector(int id, vector<int> vec, int &pos)
{
  for (size_t i = 0; i < vec.size(); ++i) {
    if (id == vec[i]) {
      pos = i;
      return true;
    }
  }
  return false;
}

bool Utilities::isInVector(int id, vector<int> &vec)
{
  for (auto it = vec.begin();
       it != vec.end(); ++it) {
    if (*it == id) {
      vec.erase(it);
      return true;
    }
  }
  return false;
}

bool Utilities::isInVector(int id, vector<vector<int> > vec, int &pos)
{
  for (size_t e = 0; e < vec.size(); ++e) {
    for (size_t i = 0; i < vec[i].size(); ++i) {
      if (id == vec[e][i]) {
        pos = e;
        return true;
      }
    }
    return false;
  }
}

void Utilities::matchID(vector<vector<float> > global, vector<vector<float> > local,
                        vector<int> in, vector<int> &out, int feature_dim)
{
  // Get new id that can be used
  vector<int> id_ava;
  searchAvailableID(in, id_ava);

  // Allocate space for output
  out.resize(local.size());

  // Rearrange the data into Mat
  Mat m1(Size(feature_dim, local.size()), CV_32FC1); // Size w*h
  Mat m2(Size(feature_dim, global.size()), CV_32FC1);

  matFill(local, m1);
  matFill(global, m2);

  FlannBasedMatcher matcher;
  vector<DMatch> matches;

  Mat query, train;
  matNormalize(m1, m2, query, train);

  matcher.match(query, train, matches);

  //  pair<int, int> best;
  //  float mind = 100;
  //  for (size_t j = 0; j < local.size(); ++j) {
  //    if (matches[j].distance < mind) {
  //      best.first = j;
  //      best.second = matches[j].trainIdx;
  //      mind = matches[j].distance;
  //    }
  //  }
  //  for (size_t j = 0; j < local.size(); ++j) {
  //    if (j == best.first) {
  //      out[j] = in[best.second];
  //    }
  //    else {
  //      out[j] = -1;
  //    }
  //  }

  // One query feature can only have one corresponding train feature and vise versa
  vector<float> g_dist_temp(global.size(), 10000);
  vector<int> match_for_g(global.size(), -1);
  for (size_t i = 0; i < global.size(); ++i) {
    for (auto & match : matches) {
      if (i == match.trainIdx) {
        if (match.distance < g_dist_temp[i]) {
          g_dist_temp[i] = match.distance;
          match_for_g[i] = match.queryIdx;
        }
      }
    }
  }

  // One global feature can have multiple query matchers if they together yield smaller distance
  //  vector<float> g_dist_temp(global.size(), 10000);
  //  vector<vector<int> > match_for_g(global.size()); // store query features' ids for each train feature
  //  for (size_t g_idx = 0; g_idx < global.size(); ++g_idx) {
  //    // local size is equal to matches size
  //    for (size_t l_idx = 0; l_idx < matches.size(); ++l_idx) {
  //      if (g_idx == matches[l_idx].trainIdx) {
  //        if (matches[l_idx].distance < g_dist_temp[g_idx]) {
  //          if (match_for_g[g_idx].size() == 0) {
  //            match_for_g[g_idx].push_back(matches[l_idx].queryIdx);
  //          }
  //          else {
  //            //if (mergeOrReplace(g_idx, match_for_g[g_idx], matches[l_idx].queryIdx, global, local)) {
  //              // merge
  //              match_for_g[g_idx].push_back(matches[l_idx].queryIdx);
  //            //}
  //            //else {
  //            //  // replace
  //            //  match_for_g[g_idx].erase(match_for_g[g_idx].begin());
  //            //  match_for_g[g_idx].push_back(matches[l_idx].queryIdx);
  //            //}
  //          }
  //        }
  //      }
  //    }
  //  }

  for (size_t j = 0; j < local.size(); ++j) {
    int pos = -1;
    if (isInVector(j, match_for_g, pos)) {
      out[j] = in[pos];
      //cout << "current: " << j << " matches: " << pos << " score " << matches[j].distance << endl;
    }
    else {
      out[j] = id_ava[0];
      id_ava.erase(id_ava.begin());
      //cout << "assign: " << j << " to: " << id_ava[0] << " score " << matches[j].distance << endl;
    }
  }
}

bool Utilities::mergeOrReplace(size_t g_id, vector<int> l_ids, size_t q_id,
                               vector<vector<float> > global, vector<vector<float> > local)
{
  //  vector<float> g_feat = global(g_id);

  //  // Prepare feature vector for previous optimal
  //  vector<float> p_feat(5, 9999);
  //  for (size_t i = 0; i < l_ids.size(); ++i) {
  //    vector<float> temp = local[l_ids[i]];
  //    for (size_t j = 0; j < temp.size(); ++j) {
  //      if (j == 0)
  //        p_feat[j] += temp[j];
  //      // xmin ymin xmax ymax
  //      else if (j == 1 || j == 2) {
  //        if (p_feat[j] == 9999) {
  //          p_feat[j] = temp[j];
  //        }
  //        if (temp[j] < p_feat[j] ) {
  //          p_feat[j] = temp[j];
  //        }
  //      }
  //      else {
  //        if (p_feat[j] == 9999) {
  //          p_feat[j] = temp[j];
  //        }
  //        if (temp[j] > p_feat[j] ) {
  //          p_feat[j] = temp[j];
  //        }
  //      }
  //    }
  //  }
  //  p_feat[0] /= l_ids.size();

  //  // Current feature vector
  //  vector<float> q_feat = local[q_id];

  //  // Compare the distance
  //  float disPrev = computeFeatureDis(g_feat, p_feat);
  //  float disCurr = computeFeatureDis(g_feat, q_feat);
  //  float disComb = computeFeatureDis(g_feat, q_combine);

  //  if (disPrev < disCurr) {
  //    if (disPrev < disComb) {
  //      return -1; // ignore
  //    }
  //    else {
  //      if (disComb < disCurr) {
  //        return 1; // merge
  //      }

  //    }
  //  }
  //  else {
  //    if (disCurr < disComb)
  //  }

  //  return true; // merge
  return false; // replace
}

float Utilities::shortRainbowColorMap(const double &value,
                                      const double &min,
                                      const double &max) {
  uint8_t r, g, b;

  // Normalize value to [0, 1]
  double value_normalized = (value - min) / (max - min);

  double a = (1.0f - value_normalized) / 0.25f;
  int X = static_cast<int>(floor(a));
  int Y = static_cast<int>(floor(255.0f * (a - X)));

  switch (X) {
    case 0:
      r = 255;
      g = Y;
      b = 0;
      break;
    case 1:
      r = 255 - Y;
      g = 255;
      b = 0;
      break;
    case 2:
      r = 0;
      g = 255;
      b = Y;
      break;
    case 3:
      r = 0;
      g = 255-Y;
      b = 255;
      break;
    case 4:
      r = 0;
      g = 0;
      b = 255;
      break;
  }

  uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
  return *reinterpret_cast<float*>(&rgb);
}

float Utilities::pointToSegDist(float x, float y, float x1, float y1, float x2, float y2)
{
  float cross = (x2 - x1) * (x - x1) + (y2 - y1) * (y - y1);
  if (cross <= 0)
    return sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));

  float d2 = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
  if (cross >= d2)
    return sqrt((x - x2) * (x - x2) + (y - y2) * (y - y2));

  float r = cross / d2;
  float px = x1 + (x2 - x1) * r;
  float py = y1 + (y2 - y1) * r;
  return sqrt((x - px) * (x - px) + (py - y) * (py - y));
}

void Utilities::heatmapRGB(float gray, uint8_t &r, uint8_t &g, uint8_t &b)
{
  // From: https://github.com/junhaoxiao/TAMS-Planar-Surface-Based-Perception/
  // blob/master/fftw_correlate/src/fftw_correlate.cpp
  if (gray >= 0.0 && gray <= 0.125) {
    r = 0;
    g = 0;
    b = 127 + floor (gray * 128 / 0.125);
  }
  else if (gray > 0.125 && gray <= 0.375)
  {
    r = 0;
    g = floor ((gray - 0.125) * 255 / 0.25);
    b = 255;
  }
  else if (gray > 0.375 && gray <= 0.625)
  {
    r = floor ((gray - 0.375) * 255 / 0.25);
    g = 255;
    b = 255 - floor ((gray - 0.375) * 255 / 0.25);
  }
  else if (gray > 0.625 && gray <= 0.875)
  {
    r = 255;
    g = 255 - floor ((gray - 0.625) * 255 / 0.25);
    b = 0;
  }
  else if (gray > 0.875 && gray <= 1.0)
  {
    r = 255 - floor ((gray - 0.875) * 128 / 0.125);
    g = 0;
    b = 0;
  }
}

template <typename T>
void Utilities::publishCloud(T cloud, const ros::Publisher& pub, string cloud_frame)
{
  sensor_msgs::PointCloud2 ros_cloud;
  pcl::toROSMsg(*cloud, ros_cloud);
  ros_cloud.header.frame_id = std::move(cloud_frame);
  ros_cloud.header.stamp = ros::Time::now();
  pub.publish(ros_cloud);
}

template<typename T, typename U>
void Utilities::convertCloudType(T cloud_in, U &cloud_out) {
  cloud_out->resize(cloud_in->size());

  for (size_t i = 0; i < cloud_in->points.size(); i++) {
    cloud_out->points[i].x = cloud_in->points[i].x;
    cloud_out->points[i].y = cloud_in->points[i].y;
    cloud_out->points[i].z = cloud_in->points[i].z;
  }
}

bool Utilities::isInContour(const Cloud_XYZ::Ptr& contour, pcl::PointXY p) {
  float angle_sum = 0;

  for (size_t i = 0; i < contour->points.size(); i++) {
    pcl::PointXY v_i{};
    pcl::PointXY v_j{};
    v_i.x = contour->points[i].x;
    v_i.y = contour->points[i].y;
    if (i < contour->points.size() - 1) {
      v_j.x = contour->points[i + 1].x;
      v_j.y = contour->points[i + 1].y;
    } else {
      v_j.x = contour->points[0].x;
      v_j.y = contour->points[0].y;
    }
    Eigen::Vector3f pvi(v_i.x - p.x, v_i.y - p.y, 0);  // add z dim since cross only apply to Vector3
    Eigen::Vector3f pvj(v_j.x - p.x, v_j.y - p.y, 0);

    // This function calculate the included angle between pvi and pvj
    float angle = atan2(pvi.cross(pvj).norm(), pvi.dot(pvj));
    angle_sum += fabs(angle);
  }
  return fabs(angle_sum - 2 * M_PI) < 0.01;
}

//template<typename T>
bool Utilities::getClustersUponPlane(const Cloud_XYZ::Ptr& src_cloud, const Cloud_XYZ::Ptr& contour,
                                     vector<Cloud_XYZ::Ptr> &clusters) {
  // Get cloud upon the given contour from src_cloud
  float z_mean, z_max, z_min, z_mid;
  getCloudZInfo<Cloud_XYZ::Ptr>(contour, z_mean, z_max, z_min, z_mid);

  Cloud_XYZ::Ptr temp(new Cloud_XYZ);
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

  vector<pcl::PointXY> rect;
  pcl::PointXY center{};
  float width, height;
  getStraightRect2D(contour, rect, center, width, height);

  for (size_t i = 0; i < src_cloud->points.size(); i++) {
    if (src_cloud->points[i].z < z_max + 0.01)
      continue;
    if (fabs(src_cloud->points[i].x - center.x) > width * 0.5 ||
        fabs(src_cloud->points[i].y - center.y) > height * 0.5 ) {
      continue;
    }

    pcl::PointXY p{};
    p.x = src_cloud->points[i].x;
    p.y = src_cloud->points[i].y;

    if (isInContour(contour, p)) {
      inliers->indices.push_back(i);
    }
  }
  getCloudByInliers(src_cloud, temp, inliers, false, false);

  // Cluster the extracted cloud into different objects
  vector<pcl::PointIndices> cluster_inliers_list;
  extractClusters(temp, cluster_inliers_list, 0.01, 10, 240000);

  for (auto & i : cluster_inliers_list) {
    Cloud_XYZ::Ptr cluster_temp(new Cloud_XYZ);
    pcl::PointIndices::Ptr indices_temp(new pcl::PointIndices);
    indices_temp->indices = i.indices;
    getCloudByInliers(temp, cluster_temp, indices_temp, false, false);
    if (!isPointCloudValid(cluster_temp)) continue;
    clusters.push_back(cluster_temp);
  }
  return !clusters.empty();
}

void Utilities::matrixToPoseArray(const Eigen::Matrix4f &mat, geometry_msgs::PoseArray &array) {
  // make sure the array.poses is clear when input
  Eigen::Quaternion<float> q;
  quaternionFromMatrix(mat, q);

  geometry_msgs::Pose pose;
  pose.position.x = mat(0, 3);
  pose.position.y = mat(1, 3);
  pose.position.z = mat(2, 3);
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  array.poses.push_back(pose);
}

bool Utilities::getCylinderPose(const Cloud_XYZ::Ptr& cloud, geometry_msgs::Pose &pose, float z) {
  float z_mean, z_max, z_min, z_mid, z_origin;
  getCloudZInfo(cloud, z_mean, z_max, z_min, z_mid);
  (z == 0) ? (z_origin = z_mid) : (z_origin = z);

  pcl::ModelCoefficients::Ptr coeff = getPlaneCoeff(z_mid);
  Cloud_XYZ::Ptr slice_2d(new Cloud_XYZ);
  sliceCloudWithPlane(coeff, 0.001, cloud, slice_2d);

  int sz = slice_2d->points.size();
  if (sz <= 2) return false;  // not enough for constructing a circum circle

  int half_sz = sz / 2;
  float a[] = {slice_2d->points[0].x, slice_2d->points[0].y, slice_2d->points[0].z};
  float b[] = {slice_2d->points[half_sz].x, slice_2d->points[half_sz].y, slice_2d->points[half_sz].z};
  float c[] = {slice_2d->points[sz-1].x, slice_2d->points[sz-1].y, slice_2d->points[sz-1].z};
  pcl::PointXY res{};
  triCircumCenter2D(a, b, c, res);
  pose.position.x = res.x;
  pose.position.y = res.y;
  pose.position.z = z_origin;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 1;
  return true;
}

bool Utilities::getBoxPose(const Cloud_XYZ::Ptr& cloud, geometry_msgs::Pose &pose, float z) {
  float z_mean, z_max, z_min, z_mid, z_origin;
  getCloudZInfo(cloud, z_mean, z_max, z_min, z_mid);
  (z == 0) ? (z_origin = z_mid) : (z_origin = z);

  pcl::ModelCoefficients::Ptr coeff = getPlaneCoeff(z_mid);
  Cloud_XYZ::Ptr slice_2d(new Cloud_XYZ);
  sliceCloudWithPlane(coeff, 0.001, cloud, slice_2d);
  if (slice_2d->points.size() <= 2)
    return false;  // not enough for constructing a convex hull

  vector<pcl::PointXY> rect;
  pcl::PointXY center{};
  pcl::PointXY edge_center{};
  float width, height, rotation;
  getRotatedRect2D(slice_2d, rect, center, edge_center, width, height, rotation);

  Eigen::Quaternion<float> q;
  quaternionFromPlanarRotation(rotation, q);
  pose.position.x = edge_center.x;
  pose.position.y = edge_center.y;
  pose.position.z = z_origin;
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();
  return true;
}

bool Utilities::getBoxTopPose(const Cloud_XYZ::Ptr& cloud, geometry_msgs::Pose &pose,
                              int& category, std::vector<double> z_list) {
  float z_mean, z_max, z_min, z_mid;
  getCloudZInfo(cloud, z_mean, z_max, z_min, z_mid);

  pcl::ModelCoefficients::Ptr coeff = getPlaneCoeff(z_mean);
  Cloud_XYZ::Ptr slice_2d(new Cloud_XYZ);
  sliceCloudWithPlane(coeff, 0.01, cloud, slice_2d);
  if (slice_2d->points.size() <= 4) {
    ROS_WARN("not enough point");
    return false;  // not enough for constructing a convex hull
  }

  vector<pcl::PointXY> rect;
  pcl::PointXY center{};
  pcl::PointXY edge_center{};
  float width, height, rotation;
  getRotatedRect2D(slice_2d, rect, center, edge_center, width, height, rotation);

  Eigen::Quaternion<float> q;
  quaternionFromPlanarRotation(rotation + M_PI_2, q);
  pose.position.x = center.x;
  pose.position.y = center.y;
  pose.position.z = z_mean;
  pose.orientation.x = q.x();
  pose.orientation.y = q.y();
  pose.orientation.z = q.z();
  pose.orientation.w = q.w();

  category = -1;
  if (z_list.size() == 1) {
    category = 0;
  } else {
    for (int i = 0; i < z_list.size(); ++i) {
      if (z_mean <= z_list[i]) {
        category = i;
        break;
      } else if (i + 1 < z_list.size()) {
        if (fabs(z_mean - z_list[i]) < fabs(z_mean - z_list[i + 1])) {
          category = i;
          break;
        }
      }
    }
  }
  return category >= 0;
}

void Utilities::computeHull(Cloud_XYZ::Ptr cloud_2d, Cloud_XYZ::Ptr &cloud_hull)
{
  pcl::ConvexHull<pcl::PointXYZ> hull;
  hull.setInputCloud(cloud_2d);
  hull.reconstruct(*cloud_hull);
}

void Utilities::getStraightRect2D(const Cloud_XYZ::Ptr &cloud, vector<pcl::PointXY> &rect,
                                  pcl::PointXY &center, float &width, float &height) {

  pcl::PointXYZ min_pt{};
  pcl::PointXYZ max_pt{};
  pcl::getMinMax3D(*cloud, min_pt, max_pt);
  float min_x = min_pt.x;
  float min_y = min_pt.y;
  float max_x = max_pt.x;
  float max_y = max_pt.y;
  pcl::PointXY corner_1{min_x, min_y};
  pcl::PointXY corner_2{max_x, min_y};
  pcl::PointXY corner_3{max_x, max_y};
  pcl::PointXY corner_4{min_x, max_y};
  rect.push_back(corner_1);
  rect.push_back(corner_2);
  rect.push_back(corner_3);
  rect.push_back(corner_4);
  center.x = (max_x + min_x) * 0.5f;
  center.y = (max_y + min_y) * 0.5f;
  width = max_x - min_x;
  height = max_y - min_y;
}

void Utilities::getRotatedRect2D(const Cloud_XYZ::Ptr &cloud_2d, std::vector<pcl::PointXY> &rect,
                                 pcl::PointXY &center, pcl::PointXY &edge_center,
                                 float &width, float &height, float &rotation)
{
  Cloud_XYZ::Ptr hull(new Cloud_XYZ);
  computeHull(cloud_2d, hull);
  vector<cv::Point2f> points = cloudToCVPoints(hull);
  cv::RotatedRect rr = cv::minAreaRect(points);

  cv::Point2f vertices[4];
  rr.points(vertices);
  for (size_t i = 0; i < 4; ++i) {
    pcl::PointXY p;
    p.y = vertices[i].x;  // swap back xy
    p.x = vertices[i].y;
    rect.push_back(p);
  }
  center.y = rr.center.x;
  center.x = rr.center.y;
  width = rr.size.width;
  height = rr.size.height;

  float dist_01 = pcl::squaredEuclideanDistance(rect[0], rect[1]);
  float dist_12 = pcl::squaredEuclideanDistance(rect[1], rect[2]);
  // the +x axis points along the observing direction and +y axis to the left
  if (dist_01 > dist_12) {
    pcl::PointXY mid_01{(rect[0].x + rect[1].x) / 2.0f, (rect[0].y + rect[1].y) / 2.0f};
    pcl::PointXY mid_23{(rect[2].x + rect[3].x) / 2.0f, (rect[2].y + rect[3].y) / 2.0f};
    mid_01.x > mid_23.x ? edge_center = mid_23 : edge_center = mid_01;
  } else {
    pcl::PointXY mid_12{(rect[1].x + rect[2].x) / 2.0f, (rect[1].y + rect[2].y) / 2.0f};
    pcl::PointXY mid_30{(rect[3].x + rect[0].x) / 2.0f, (rect[3].y + rect[0].y) / 2.0f};
    mid_12.x > mid_30.x ? edge_center = mid_30 : edge_center = mid_12;
  }
  //  float angle_deg;
  //  if (rr.size.width < rr.size.height) {
  //    angle_deg = 90 - rr.angle;
  //  } else {
  //    angle_deg = -rr.angle;
  //  }
  //  // The rr.angle is positive in clockwise direction, here we transfer that to positive
  //  // in anticlockwise direction. see cite for details.
  //  rotation = (90.0f - angle_deg) / 180.0f * M_PI;
  rotation = atan2(edge_center.y - center.y, edge_center.x - center.x);
  (rotation >= 0) ? (rotation -= M_PI) : (rotation += M_PI);
  //cerr << center << " " << edge_center << endl;
  //cerr << " rotation in deg "<< rotation / M_PI * 180 << endl;
}

void Utilities::estimateFPFH(Cloud_XYZN::Ptr cloud_in, PointCloudFPFH::Ptr &features_out, float dsp_th) {
  FeatureEstimationFPFH fest;
  fest.setRadiusSearch(dsp_th * 5);
  fest.setInputCloud(cloud_in);
  fest.setInputNormals(cloud_in);
  fest.compute(*features_out);
}

bool Utilities::alignmentWithFPFH(Cloud_XYZN::Ptr src_cloud, PointCloudFPFH::Ptr src_features,
                                  Cloud_XYZN::Ptr tgt_cloud, PointCloudFPFH::Ptr tgt_features,
                                  Eigen::Matrix4f &transformation, Cloud_XYZN::Ptr &src_aligned, float leaf) {
  pcl::SampleConsensusPrerejective<PointN, PointN, FeatureFPFH> align;
  align.setInputSource(src_cloud);
  align.setSourceFeatures(src_features);
  align.setInputTarget(tgt_cloud);
  align.setTargetFeatures(tgt_features);
  align.setMaximumIterations(50000); // Number of RANSAC iterations
  align.setNumberOfSamples(3); // Number of points to sample for generating/prerejecting a pose
  align.setCorrespondenceRandomness(5); // Number of nearest features to use
  align.setSimilarityThreshold(0.9f); // Polygonal edge length similarity threshold
  align.setMaxCorrespondenceDistance(2.5f * leaf); // Inlier threshold
  align.setInlierFraction(0.25f); // Required inlier fraction for accepting a pose hypothesis
  align.align(*src_aligned);

  if (align.hasConverged()) {
    transformation = align.getFinalTransformation();
    return true;
  } else {
    return false;
  }
}

void Utilities::quaternionFromMatrix(Eigen::Matrix4f mat, Eigen::Quaternion<float> &q) {
  auto t = mat.trace();
  if (t > mat(3, 3)) {
    q.w() = t;
    q.z() = mat(1, 0) - mat(0, 1);
    q.y() = mat(0, 2) - mat(2, 0);
    q.x() = mat(2, 1) - mat(1, 2);
  } else {
    int i = 0;
    int j = 1;
    int k = 2;
    if (mat(1, 1) > mat(0, 0)) {
      i = 1;
      j = 2;
      k = 0;
    }
    if (mat(2, 2) > mat(i, i)) {
      i = 2;
      j = 0;
      k = 1;
    }
    t = mat(i, i) - (mat(j, j) + mat(k, k)) + mat(3, 3);
    if (i == 0) q.x() = t;
    else if (i == 1) q.y() = t;
    else q.z() = t;

    if (j == 0) q.x() = mat(i, j) + mat(j, i);
    else if (j == 1) q.y() = mat(i, j) + mat(j, i);
    else q.z() = mat(i, j) + mat(j, i);

    if (k == 0) q.x() = mat(k, i) + mat(i, k);
    else if (k == 1) q.y() = mat(k, i) + mat(i, k);
    else q.z() = mat(k, i) + mat(i, k);
    q.w() = mat(k, j) - mat(j, k);
  }
  auto d = 0.5 / sqrt(t * mat(3, 3));
  q.x() *= d;
  q.y() *= d;
  q.z() *= d;
  q.w() *= d;
}

pcl::ModelCoefficients::Ptr Utilities::getPlaneCoeff(float z) {
  // Plane function: ax + by + cz + d = 0, here coeff[3] = d = -cz
  pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
  coeff->values.push_back(0.0);
  coeff->values.push_back(0.0);
  coeff->values.push_back(1.0);
  coeff->values.push_back(-z);
  return coeff;
}

std::vector<cv::Point2f> Utilities::cloudToCVPoints(Cloud_XYZ::Ptr cloud_hull) {
  std::vector<cv::Point2f> points;
  for (std::size_t i = 0; i < cloud_hull->size(); ++i) {
    cv::Point2f p;
    p.y = cloud_hull->points[i].x;  // swap x y
    p.x = cloud_hull->points[i].y;
    points.push_back(p);
  }
  return points;
}

void Utilities::quaternionFromPlanarRotation(float rotation, Eigen::Quaternion<float> &q) {
  float cos_r = cos(rotation);
  float sin_r = sin(rotation);
  Eigen::Matrix4f mat = Eigen::Matrix<float, 4, 4>::Identity();
  mat(0, 0) = cos_r;
  mat(0, 1) = -sin_r;
  mat(1, 0) = sin_r;
  mat(1, 1) = cos_r;
  quaternionFromMatrix(mat, q);
}

void Utilities::combineCloud(Cloud_XYZ::Ptr cloud_a, Cloud_XYZ::Ptr cloud_b, Cloud_XYZ::Ptr &cloud_out) {
  int sz_a = cloud_a->points.size();
  int sz_b = cloud_b->points.size();
  cloud_out->is_dense = false;
  for (std::size_t a = 0; a < sz_a; ++a) {
    cloud_out->points.push_back(cloud_a->points[a]);
  }
  for (std::size_t b = 0; b < sz_b; ++b) {
    cloud_out->points.push_back(cloud_b->points[b]);
  }
}


// declare all templates use case, otherwise undefined symbol error will raise
// refer: https://raymii.org/s/snippets/Cpp_template_definitions_in_a_cpp_file_instead_of_header.html

template void Utilities::publishCloud<Cloud_XYZRGB::Ptr>(Cloud_XYZRGB::Ptr cloud, const ros::Publisher &pub, string cloud_frame);
template void Utilities::publishCloud<Cloud_XYZ::Ptr>(Cloud_XYZ::Ptr cloud, const ros::Publisher &pub, string cloud_frame);

template void Utilities::convertCloudType<Cloud_XYZRGB::Ptr, Cloud_XYZ::Ptr>(Cloud_XYZRGB::Ptr cloud_in, Cloud_XYZ::Ptr &cloud_out);
template void Utilities::convertCloudType<Cloud_XYZRGBN::Ptr, Cloud_XYZ::Ptr>(Cloud_XYZRGBN::Ptr cloud_in, Cloud_XYZ::Ptr &cloud_out);
template void Utilities::convertCloudType<Cloud_XYZ::Ptr, Cloud_XYZN::Ptr>(Cloud_XYZ::Ptr cloud_in, Cloud_XYZN::Ptr &cloud_out);
template void Utilities::convertCloudType<Cloud_XYZ::Ptr, Cloud_XYZRGB::Ptr>(Cloud_XYZ::Ptr cloud_in, Cloud_XYZRGB::Ptr &cloud_out);

template void Utilities::getCloudZInfo<Cloud_XYZ::Ptr>(Cloud_XYZ::Ptr cloud_in, float &z_mean, float &z_max, float &z_min, float &z_mid);
template void Utilities::getCloudZInfo<Cloud_XYZRGBN::Ptr>(Cloud_XYZRGBN::Ptr cloud_in, float &z_mean, float &z_max, float &z_min, float &z_mid);

template void Utilities::sliceCloudWithPlane<Cloud_XYZ::Ptr, Cloud_XYZ::Ptr>(pcl::ModelCoefficients::Ptr coeff_in,
                                                                             float th_distance, Cloud_XYZ::Ptr cloud_in,
                                                                             Cloud_XYZ::Ptr &cloud_out);

