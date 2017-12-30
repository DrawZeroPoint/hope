#include "utilities.h"


Utilities::Utilities()
{
}

void Utilities::generateName(int count, string pref, string surf, string &name)
{
  std::ostringstream ost;
  ost << count;
  std::string temp(ost.str());
  name = pref + temp + surf;
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

void Utilities::estimateNorm(PointCloudMono::Ptr cloud_in, 
                             PointCloudRGBN::Ptr &cloud_out,
                             float norm_r, float grid_sz, bool down_sp)
{
  PointCloudMono::Ptr cloud_fit(new PointCloudMono);
  if (down_sp)
    preProcess(cloud_in, cloud_fit, grid_sz);
  else
    cloud_fit = cloud_in;
  
  cloud_out->height = cloud_fit->height;
  cloud_out->width  = cloud_fit->width;
  cloud_out->is_dense = false;
  cloud_out->resize(cloud_out->height * cloud_out->width);
  
  // Create the normal estimation class, and pass the input dataset to it
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  ne.setInputCloud(cloud_fit);
  
  // Create an empty kdtree representation, and pass it to the normal estimation object.
  // Its content will be filled inside the object, based on the given input dataset 
  // (as no other search surface is given).
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  ne.setSearchMethod(tree);
  ne.setRadiusSearch(norm_r); // mm
  
  // Compute the features
  pcl::PointCloud<pcl::Normal>::Ptr cloud_norm(new pcl::PointCloud<pcl::Normal>);
  ne.compute(*cloud_norm);
  
  for (size_t i = 0; i < cloud_out->size(); ++i) {
    cloud_out->points[i].x = cloud_fit->points[i].x;
    cloud_out->points[i].y = cloud_fit->points[i].y;
    cloud_out->points[i].z = cloud_fit->points[i].z;
    cloud_out->points[i].r = 1;
    cloud_out->points[i].g = 1;
    cloud_out->points[i].b = 1;
    cloud_out->points[i].normal_x = cloud_norm->points[i].normal_x;
    cloud_out->points[i].normal_y = cloud_norm->points[i].normal_y;
    cloud_out->points[i].normal_z = cloud_norm->points[i].normal_z;
  }
}

void Utilities::preProcess(PointCloudMono::Ptr cloud_in, 
                           PointCloudMono::Ptr &cloud_out,
                           float gird_sz)
{
  // Create the filtering object
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud_in);
  vg.setLeafSize(gird_sz, gird_sz, gird_sz);
  vg.filter(*cloud_out);
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

void Utilities::cutCloud(pcl::ModelCoefficients::Ptr coeff_in, float th_distance,
                         PointCloudRGBN::Ptr cloud_in, 
                         PointCloudMono::Ptr &cloud_out)
{
  vector<int> inliers_cut;
  Eigen::Vector4f coeffs(coeff_in->values[0], coeff_in->values[1],
      coeff_in->values[2], coeff_in->values[3]);
  
  PointCloudMono::Ptr cloudSourceFiltered_t(new PointCloudMono);
  pointTypeTransfer(cloud_in, cloudSourceFiltered_t);
  pcl::SampleConsensusModelPlane<pcl::PointXYZ> scmp(cloudSourceFiltered_t);
  scmp.selectWithinDistance(coeffs, th_distance, inliers_cut);
  scmp.projectPoints(inliers_cut, coeffs, *cloud_out, false);
}

void Utilities::cutCloud(pcl::ModelCoefficients::Ptr coeff_in, float th_distance,
                         PointCloudMono::Ptr cloud_in, 
                         PointCloudMono::Ptr &cloud_out)
{
  vector<int> inliers_cut;
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

float Utilities::determinant(float v1, float v2, float v3, float v4)
{  
  return (v1 * v3 - v2 * v4);  
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

void Utilities::smartOffset(pcl::PointXYZ &p_in, float off_xy, float off_z)
{
  //  float y_off = off_xy / sqrt(1 + pow(p_in.x / p_in.y, 2)) * p_in.y / fabs(p_in.y);
  //  float x_off = p_in.x / p_in.y * y_off;
  
  float rad = atan2(p_in.y, p_in.x);
  float y_off = off_xy * sin(rad);
  float x_off = off_xy * cos(rad);  
  p_in.x += x_off;
  p_in.y += y_off;
  // To solve the issue that the detected point is higher than optimal position
  p_in.z += off_z;
}
