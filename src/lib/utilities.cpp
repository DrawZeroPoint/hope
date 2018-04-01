#include "utilities.h"

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

Utilities::Utilities()
{
}

float Utilities::determinant(float v1, float v2, float v3, float v4)
{
  return (v1 * v3 - v2 * v4);
}

std::string Utilities::getName(int count, string pref, int surf)
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

void Utilities::getOccupancyMap(PointCloudMono::Ptr cloud_src, PointCloudMono::Ptr cloud_upper, 
                                std::vector<int> occupy, PointCloud::Ptr &cloud_out)
{
  int minv, maxv;
  getMinMax(occupy, minv, maxv);
  cloud_out->resize(cloud_src->size() + cloud_upper->size());
  
  size_t k = 0;
  for (PointCloudMono::const_iterator pit = cloud_src->begin();
       pit != cloud_src->end(); ++pit) {
    float rgb = Utilities::shortRainbowColorMap(occupy[k], minv, maxv);
    
    cloud_out->points[k].x = pit->x;
    cloud_out->points[k].y = pit->y;
    cloud_out->points[k].z = pit->z;
    cloud_out->points[k].rgb = rgb;
    k++;
  }
  
  size_t j = 0;
  for (PointCloudMono::const_iterator pit = cloud_upper->begin();
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

void Utilities::getPointByZ(float z, PointCloudMono::Ptr cloud_in, pcl::PointXYZ &pt)
{
  for (PointCloudMono::const_iterator pit = cloud_in->begin();
       pit != cloud_in->end(); ++pit) {
    if (pit->z == z) {
      pt.x = pit->x;
      pt.y = pit->y;
      pt.z = pit->z;
      break;
    }
  }
}

void Utilities::msgToCloud(const PointCloud::ConstPtr msg,
                           PointCloudMono::Ptr cloud)
{
  cloud->height = msg->height;
  cloud->width  = msg->width;
  cloud->is_dense = false;
  cloud->resize(cloud->height * cloud->width);
  
  size_t i = 0;
  for (PointCloud::const_iterator pit = msg->begin();
       pit != msg->end(); ++pit) {
    cloud->points[i].x = pit->x;
    cloud->points[i].y = pit->y;
    cloud->points[i].z = pit->z;
    ++i;
  }
}

bool Utilities::pcaAnalyse(pcl::PointXYZ pointMaxZ, pcl::PointXYZ pointMinZ,
                           float &proj_long, float &proj_short,
                           const PointCloud::ConstPtr cloud_2d_in, float &max_dis)
{
  size_t sz = cloud_2d_in->points.size();
  if (sz <= 1) return false;
  else if (sz == 2) {
    max_dis = pcl::euclideanDistance(cloud_2d_in->points[0], cloud_2d_in->points[1]);
    return true;
  }

  Eigen::Vector2f maxDeltaZVector;
  maxDeltaZVector(0) = pointMaxZ.x - pointMinZ.x;
  maxDeltaZVector(1) = pointMaxZ.y - pointMinZ.y;
  
  Eigen::Matrix2Xf data(2, sz);
  for (size_t i = 0; i < sz; ++i) {
    data(0, i) = cloud_2d_in->points[i].x;
    data(1, i) = cloud_2d_in->points[i].y;
  }
  
  Eigen::Vector2f mean = data.rowwise().mean();
  
  Eigen::MatrixXf tmp(2, sz);
  for (int r = 0; r < 2; ++r) {
    for (int c = 0; c < sz; ++c) {
      tmp(r, c) = data(r, c) - mean(r);
    }
  }
  
  Eigen::MatrixXf C = (tmp * tmp.transpose()) / (sz - 1);
  
  Eigen::EigenSolver<Eigen::MatrixXf> es(C);
  // The result is complex number
  complex<float> lambda0 = es.eigenvalues()[0];
  complex<float> lambda1 = es.eigenvalues()[1];
  Eigen::MatrixXcf V = es.eigenvectors();
  
  // The max axis
  complex<float> val_x_1;
  complex<float> val_y_1;
  complex<float> val_x_2;
  complex<float> val_y_2;
  if (lambda0.real() > lambda1.real()) {
    val_x_1 = V.col(0)[0];
    val_y_1 = V.col(0)[1];
    val_x_2 = V.col(1)[0];
    val_y_2 = V.col(1)[1];
  }
  else {
    val_x_1 = V.col(1)[0];
    val_y_1 = V.col(1)[1];
    val_x_2 = V.col(0)[0];
    val_y_2 = V.col(0)[1];
  }
  
  Eigen::Vector2f axis0;
  axis0(0) = val_x_1.real();
  axis0(1) = val_y_1.real();
  Eigen::Vector2f axis1;
  axis1(0) = val_x_2.real();
  axis1(1) = val_y_2.real();

  proj_long = fabs(axis0.transpose() * maxDeltaZVector);
  // Note that both axis0 and axis1 are unit vector
  //proj_long /= axis0.norm();
  proj_short = fabs(axis1.transpose() * maxDeltaZVector);
  //proj_short /= axis1.norm();
  
  Eigen::Vector2f data_point;
  float vmin = FLT_MAX;
  float vmax = -FLT_MAX;
  int idmax = -1;
  int idmin = -1;
  for (size_t i = 0; i < sz; ++i) {
    data_point(0) = tmp(0, i);
    data_point(1) = tmp(1, i);
    
    float v = axis0.transpose() * data_point; // Dot product
    if (v > vmax) {
      vmax = v;
      idmax = i;
    }
    if (v < vmin) {
      vmin = v;
      idmin = i;
    }
  }
  if (idmin >= 0 && idmax >= 0) {
    max_dis = pcl::euclideanDistance(cloud_2d_in->points[idmin], cloud_2d_in->points[idmax]);
    return true;
  }
  else
    return false;
  
  // For debug
  /*
  sz = 10;
  vector<vector<float>> vec{ { 2.5f, 0.5f, 2.2f, 1.9f, 3.1f, 2.3, 2, 1, 1.5, 1.1},
  { 2.4f, 0.7f, 2.9f, 2.2f, 3.0, 2.7, 1.6, 1.1, 1.6, 0.9 }};
  const int rows{ 2 }, cols{ 10 };
  
  std::vector<float> vec_;
  for (int i = 0; i < 2; ++i) {
    vec_.insert(vec_.begin() + i * cols, vec[i].begin(), vec[i].end());
  }
  Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> data(vec_.data(), rows, cols);

  //fprintf(stderr, "source matrix:\n");
  //std::cout << data << std::endl;
  
  Eigen::Vector2f mean = data.rowwise().mean();
  //std::cout << "print mean: " << std::endl << mean << std::endl;
  
  Eigen::MatrixXf tmp(2, sz);
  for (int r = 0; r < 2; ++r) {
    for (int c = 0; c < sz; ++c) {
      tmp(r, c) = data(r, c) - mean(r);
    }
  }
  
  Eigen::MatrixXf C = (tmp * tmp.transpose()) / (sz - 1);
  std::cout << "print covariance matrix: " << std::endl << tmp << std::endl;
  
  Eigen::EigenSolver<Eigen::MatrixXf> es(C);
  
  // The result is complex number
  complex<float> lambda0 = es.eigenvalues()[0];
  complex<float> lambda1 = es.eigenvalues()[1];
  //cout << "... and A * v = " << endl << A.cast<complex<double> >() * v << endl << endl;
  Eigen::MatrixXcf V = es.eigenvectors();
  
  // The max axis
  complex<float> val_x;
  complex<float> val_y;
  if (lambda0.real() > lambda1.real()) {
    val_x = V.col(0)[0];
    val_y = V.col(0)[1];
  }
  else {
    val_x = V.col(1)[0];
    val_y = V.col(1)[1];
  }
  
  Eigen::Vector2f axis0;
  axis0(0) = val_x.real();
  axis0(1) = val_y.real();
  std::cout << "Main axis: " << std::endl << axis0(0) << axis0(1) << std::endl;
  
  Eigen::Vector2f data_point;
  float vmin = FLT_MAX;
  float vmax = -FLT_MAX;
  int idmax = -1;
  int idmin = -1;
  for (size_t i = 0; i < sz; ++i) {
    data_point(0) = tmp(0, i);
    data_point(1) = tmp(1, i);
    
    float v = axis0.transpose()*data_point;
    std::cout << "v: " << std::endl << v << std::endl;
    if (v > vmax) {
      vmax = v;
      idmax = i;
    }
    if (v < vmin) {
      vmin = v;
      idmin = i;
    }
  }
  std::cout << "Farest point id: " << std::endl << idmin << idmax << std::endl;
  if (idmin >= 0 && idmax >= 0) {
    return true;
  }
  else
    return false;
  */
}

void Utilities::calRegionGrowing(PointCloudRGBN::Ptr cloud_in, int minsz, int maxsz, int nb, int smooth,
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

void Utilities::estimateNorm(PointCloudMono::Ptr cloud_in, 
                             PointCloudRGBN::Ptr &cloud_out,
                             NormalCloud::Ptr &normals_out,
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

void Utilities::estimateNorm(PointCloudMono::Ptr cloud_in, 
                             NormalCloud::Ptr &normals_out,
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
}

void Utilities::downSampling(PointCloudMono::Ptr cloud_in, 
                             PointCloudMono::Ptr &cloud_out,
                             float gird_sz, float z_sz)
{
  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud_in);
  vg.setLeafSize(gird_sz, gird_sz, z_sz);
  vg.filter(*cloud_out);
}

void Utilities::downSampling(PointCloud::Ptr cloud_in,
                             PointCloud::Ptr &cloud_out,
                             float gird_sz, float z_sz)
{
  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZRGB> vg;
  vg.setInputCloud(cloud_in);
  vg.setLeafSize(gird_sz, gird_sz, z_sz);
  vg.filter(*cloud_out);
}

void Utilities::planeTo2D(float z, PointCloudMono::Ptr cloud_in, 
                          PointCloudMono::Ptr &cloud_out)
{
  cloud_out->width = cloud_in->width;
  cloud_out->height = cloud_in->height;
  cloud_out->resize(cloud_out->width *cloud_out->height);
  
  size_t k = 0;
  for (PointCloudMono::const_iterator pit = cloud_in->begin();
       pit != cloud_in->end(); ++pit) {
    cloud_out->points[k].x = pit->x;
    cloud_out->points[k].y = pit->y;
    cloud_out->points[k].z = z;
    k++;
  }
}

void Utilities::pointTypeTransfer(PointCloudRGBN::Ptr cloud_in, 
                                  PointCloudMono::Ptr &cloud_out)
{
  cloud_out->resize(cloud_in->size());
  
  for (size_t i = 0; i < cloud_in->points.size(); i++) {
    cloud_out->points[i].x = cloud_in->points[i].x;
    cloud_out->points[i].y = cloud_in->points[i].y;
    cloud_out->points[i].z = cloud_in->points[i].z;
  }
}

void Utilities::pointTypeTransfer(PointCloud::Ptr cloud_in, 
                                  PointCloudMono::Ptr &cloud_out)
{
  cloud_out->resize(cloud_in->size());
  
  for (size_t i = 0; i < cloud_in->points.size(); i++) {
    cloud_out->points[i].x = cloud_in->points[i].x;
    cloud_out->points[i].y = cloud_in->points[i].y;
    cloud_out->points[i].z = cloud_in->points[i].z;
  }
}

void Utilities::pointTypeTransfer(PointCloudMono::Ptr cloud_in, 
                                  PointCloud::Ptr &cloud_out,
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

void Utilities::cutCloud(pcl::ModelCoefficients::Ptr coeff_in, float th_distance,
                         PointCloudRGBN::Ptr cloud_in, vector<int> &inliers_cut,
                         PointCloudMono::Ptr &cloud_out)
{
  Eigen::Vector4f coeffs(coeff_in->values[0], coeff_in->values[1],
      coeff_in->values[2], coeff_in->values[3]);
  
  PointCloudMono::Ptr cloudSourceFiltered_t(new PointCloudMono);
  pointTypeTransfer(cloud_in, cloudSourceFiltered_t);
  pcl::SampleConsensusModelPlane<pcl::PointXYZ> scmp(cloudSourceFiltered_t);
  scmp.selectWithinDistance(coeffs, th_distance, inliers_cut);
  scmp.projectPoints(inliers_cut, coeffs, *cloud_out, false);
}

void Utilities::cutCloud(pcl::ModelCoefficients::Ptr coeff_in, float th_distance,
                         PointCloudMono::Ptr cloud_in, vector<int> &inliers_cut,
                         PointCloudMono::Ptr &cloud_out)
{
  Eigen::Vector4f coeffs(coeff_in->values[0], coeff_in->values[1],
      coeff_in->values[2], coeff_in->values[3]);
  
  pcl::SampleConsensusModelPlane<pcl::PointXYZ> scmp(cloud_in);
  scmp.selectWithinDistance(coeffs, th_distance, inliers_cut);
  scmp.projectPoints(inliers_cut, coeffs, *cloud_out, false);
}

void Utilities::clusterExtract(PointCloudMono::Ptr cloud_in, 
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

void Utilities::projectCloud(pcl::ModelCoefficients::Ptr coeff_in, 
                             PointCloudMono::Ptr cloud_in,
                             PointCloudMono::Ptr &cloud_out)
{
  pcl::ProjectInliers<pcl::PointXYZ> proj;
  proj.setModelType(pcl::SACMODEL_PLANE);
  proj.setInputCloud(cloud_in);
  proj.setModelCoefficients(coeff_in);
  proj.filter(*cloud_out);
}

void Utilities::rotateCloudXY(PointCloudRGBN::Ptr cloud_in, PointCloudRGBN::Ptr &cloud_out,
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

void Utilities::rotateBack(PointCloudMono::Ptr cloud_in, PointCloudMono::Ptr &cloud_out,
                           Eigen::Matrix4f transform_inv)
{
  // Executing the transformation
  pcl::transformPointCloud(*cloud_in, *cloud_out, transform_inv);
}

void Utilities::getAverage(PointCloudMono::Ptr cloud_in, 
                           float &avr, float &deltaz)
{
  avr = 0.0;
  deltaz = 0.0;
  size_t sz = cloud_in->points.size();
  for (PointCloudMono::const_iterator pit = cloud_in->begin();
       pit != cloud_in->end(); ++pit)
    avr += pit->z;
  
  avr = avr / sz;
  
  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D(*cloud_in, minPt, maxPt);
  deltaz = maxPt.z - minPt.z;
}

void Utilities::getCloudByNorm(PointCloudRGBN::Ptr cloud_in, 
                               pcl::PointIndices::Ptr &inliers,
                               float th_norm)
{
  size_t i = 0;
  for (PointCloudRGBN::const_iterator pit = cloud_in->begin();
       pit != cloud_in->end();++pit) {
    float n_z = pit->normal_z;
    // If point normal fulfill this criterion, consider it from plane
    // Here we use absolute value cause the normal direction may be opposite to
    // the z-axis due to the algorithm's settings
    if (fabs(n_z) > th_norm)
      inliers->indices.push_back(i);
    ++i;
  }
}

void Utilities::getCloudByNorm(NormalCloud::Ptr cloud_in, 
                               pcl::PointIndices::Ptr &inliers,
                               float th_norm)
{
  inliers->indices.clear();
  size_t i = 0;
  for (NormalCloud::const_iterator pit = cloud_in->begin();
       pit != cloud_in->end();++pit) {
    float n_z = pit->normal_z;
    // If point normal fulfill this criterion, consider it from plane
    // Here we use absolute value cause the normal direction may be opposite to
    // the z-axis due to the algorithm's settings
    if (fabs(n_z) > th_norm)
      inliers->indices.push_back(i);
    ++i;
  }
}

void Utilities::getCloudByZ(PointCloudMono::Ptr cloud_in, 
                            pcl::PointIndices::Ptr &inliers,
                            PointCloudMono::Ptr &cloud_out,
                            float z_min, float z_max)
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

void Utilities::getCloudByZ(PointCloud::Ptr cloud_in, 
                            pcl::PointIndices::Ptr &inliers,
                            PointCloud::Ptr &cloud_out,
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

void Utilities::getCloudByInliers(PointCloudMono::Ptr cloud_in, 
                                  PointCloudMono::Ptr &cloud_out,
                                  pcl::PointIndices::Ptr inliers,
                                  bool negative, bool organized)
{
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  extract.setNegative(negative);
  extract.setInputCloud(cloud_in);
  extract.setIndices(inliers);
  extract.setKeepOrganized(organized);
  extract.filter(*cloud_out);
}

void Utilities::getCloudByInliers(NormalCloud::Ptr cloud_in, 
                                  NormalCloud::Ptr &cloud_out,
                                  pcl::PointIndices::Ptr inliers,
                                  bool negative, bool organized)
{
  pcl::ExtractIndices<pcl::Normal> extract;
  extract.setNegative(negative);
  extract.setInputCloud(cloud_in);
  extract.setIndices(inliers);
  extract.setKeepOrganized(organized);
  extract.filter(*cloud_out);
}

void Utilities::getCloudByInliers(PointCloudRGBN::Ptr cloud_in, 
                                  PointCloudRGBN::Ptr &cloud_out,
                                  pcl::PointIndices::Ptr inliers,
                                  bool negative, bool organized)
{
  pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
  extract.setNegative(negative);
  extract.setInputCloud(cloud_in);
  extract.setIndices(inliers);
  extract.setKeepOrganized(organized);
  extract.filter(*cloud_out);
}

bool Utilities::checkWithIn(pcl::PointIndices::Ptr ref_inliers, 
                            pcl::PointIndices::Ptr tgt_inliers)
{
  int size_ref = ref_inliers->indices.size();
  int size_tgt = tgt_inliers->indices.size();
  int half_size = size_tgt * 0.5;
  
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

float Utilities::getCloudMeanZ(PointCloudMono::Ptr cloud_in)
{
  // A simple yet easy to understand method
  float mid = 0.0;
  size_t ct = 0;
  for (PointCloudMono::const_iterator pit = cloud_in->begin();
       pit != cloud_in->end(); ++pit) {
    if (!isfinite(pit->z))
      cerr << "nan" << endl;
    else {
      mid += pit->z;
      ct++;
    }
  }
  return mid/ct;
}

float Utilities::getCloudMeanZ(PointCloudRGBN::Ptr cloud_in)
{
  // A simple yet easy to understand method
  float mid = 0.0;
  size_t ct = 0;
  for (PointCloudRGBN::const_iterator pit = cloud_in->begin();
       pit != cloud_in->end(); ++pit) {
    if (!isfinite(pit->z))
      cerr << "nan" << endl;
    else {
      mid += pit->z;
      ct++;
    }
  }
  return mid/ct;
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

void Utilities::getHullCenter(PointCloud::Ptr hull, float &x, float &y)
{
  float sumx = 0;
  float sumy = 0;
  for (size_t i = 0; i < hull->points.size(); ++i) {
    sumx += hull->points[i].x;
    sumy += hull->points[i].y;
  }
  x = sumx / hull->points.size();
  y = sumy / hull->points.size();
}

pcl::PolygonMesh Utilities::getMesh(const PointCloudMono::Ptr point_cloud, 
                                    NormalCloud::Ptr normals)
{
  //    NormalCloud::Ptr normals(new NormalCloud);
  //    normals->height = plane_hull_[i]->height;
  //    normals->width  = plane_hull_[i]->width;
  //    normals->is_dense = true;
  //    normals->resize(normals->height * normals->width);
  //    for (size_t j = 0; j < normals->size(); ++j) {
  //      normals->points[j].normal_x = 0;
  //      normals->points[j].normal_y = 0;
  //      normals->points[j].normal_z = 1;
  //    }
  
  // Add the normals to the point cloud
  NormalPointCloud::Ptr cloud_with_normals(new NormalPointCloud);
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

void Utilities::shrinkHull(PointCloudMono::Ptr cloud, 
                           PointCloudMono::Ptr &cloud_sk, float dis)
{
  pcl::PointXYZ minPt, maxPt;
  pcl::getMinMax3D(*cloud, minPt, maxPt);
  
  size_t i = 0;
  float center_x = (maxPt.x + minPt.x) / 2;
  float center_y = (maxPt.y + minPt.y) / 2;
  for (PointCloudMono::const_iterator pit = cloud->begin();
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

bool Utilities::isInHull(PointCloudMono::Ptr hull, pcl::PointXY p_in, 
                         pcl::PointXY &offset, pcl::PointXY &p_closest)
{
  //pcl::isXYPointIn2DXYPolygon(p_in, *hull);
  // Step 1: get cloest point of p_in in each sector
  size_t i = 0;
  bool has_sector_1 = false;
  bool has_sector_2 = false;
  bool has_sector_3 = false;
  bool has_sector_4 = false;
  float dis_1 = 10.0;
  float dis_2 = 10.0;
  float dis_3 = 10.0;
  float dis_4 = 10.0;
  pcl::PointXY p_1, p_2, p_3, p_4;
  for (PointCloudMono::const_iterator pit = hull->begin();
       pit != hull->end(); ++pit) {
    float delta_x = pit->x - p_in.x;
    float delta_y = pit->y - p_in.y;
    float dis = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
    if (delta_x > 0 && delta_y > 0) {
      has_sector_1 = true;
      if (dis < dis_1) {
        dis_1 = dis;
        p_1.x = pit->x;
        p_1.y = pit->y;
      }
    }
    else if (delta_x <= 0 && delta_y > 0) {
      has_sector_2 = true;
      if (dis < dis_2) {
        dis_2 = dis;
        p_2.x = pit->x;
        p_2.y = pit->y;
      }
    }
    else if (delta_x <= 0 && delta_y <=0 ) {
      has_sector_3 = true;
      if (dis < dis_3) {
        dis_3 = dis;
        p_3.x = pit->x;
        p_3.y = pit->y;
      }
    }
    else {
      has_sector_4 = true;
      if (dis < dis_4) {
        dis_4 = dis;
        p_4.x = pit->x;
        p_4.y = pit->y;
      }
    }
    ++i;
  }
  if (has_sector_1 && has_sector_2 && has_sector_3 && has_sector_4) {
    offset.x = 0;
    offset.y = 0;
    return true;
  }
  else {
    pcl::PointXYZ minPt, maxPt;
    pcl::getMinMax3D(*hull, minPt, maxPt);
    pcl::PointXY p_c;
    p_c.x = (minPt.x + maxPt.x)/2;
    p_c.y = (minPt.y + maxPt.y)/2;
    p_closest.x = p_in.x + 0.1 * (p_c.x - p_in.x);
    p_closest.y = p_in.y + 0.1 * (p_c.y - p_in.y);
    
    offset.x = p_closest.x - p_in.x;
    offset.y = p_closest.y - p_in.y;
    return false;
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
  float lameda = determinant(p3.x-p1.x, p3.x-p4.x, p3.y-p1.y, p3.y-p4.y) / delta;
  if ( lameda > 1 || lameda < 0 ) {
    return false;
  }
  float miu = determinant(p2.x-p1.x, p3.x-p1.x, p2.y-p1.y, p3.y-p1.y) / delta;
  if ( miu > 1 || miu < 0 ) {
    return false;
  }
  return true;
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
    float r_mean = acc / (query_in.rows + train_in.rows);

    acc = 0;
    for (size_t r = 0; r < query_in.rows; ++r) {
      float val = query_in.at<float>(r, c);
      acc += pow(val - r_mean, 2);
    }
    for (size_t r = 0; r < train_in.rows; ++r) {
      float val = train_in.at<float>(r, c);
      acc += pow(val - r_mean, 2);
    }

    float sd = sqrt(acc / (query_in.rows + train_in.rows));

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

void Utilities::searchAvailableID(vector<int> id_used,
                                  vector<int> &id_ava,
                                  size_t limit=64)
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
    for (size_t n = 0; n < id_used.size(); ++n) {
      if (i == id_used[n]) {
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
    for (size_t j = 0; j < matches.size(); ++j) {
      if (i == matches[j].trainIdx) {
        if (matches[j].distance < g_dist_temp[i]) {
          g_dist_temp[i] = matches[j].distance;
          match_for_g[i] = matches[j].queryIdx;
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

float Utilities::shortRainbowColorMap(const double value, 
                                      const double min,
                                      const double max) {
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
