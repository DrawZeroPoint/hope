#ifndef HOPE_H
#define HOPE_H

#include <pcl/segmentation/region_growing.h>

#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_base.h>

#include <queue>
#include <list>
#include <cmath>
#include <time.h>


class PCL_EXPORTS Hope : public pcl::PCLBase<pcl::PointXYZ>
{
public:
  typedef pcl::search::Search <pcl::PointXYZ> KdTree;
  typedef typename KdTree::Ptr KdTreePtr;
  typedef pcl::PointCloud <pcl::PointXYZ> PointCloud;
  
  using pcl::PCLBase <pcl::PointXYZ>::input_;
  using pcl::PCLBase <pcl::PointXYZ>::indices_;
  using pcl::PCLBase <pcl::PointXYZ>::initCompute;
  using pcl::PCLBase <pcl::PointXYZ>::deinitCompute;
  
public:
  Hope();
  
  virtual
  ~Hope();
  
  /** \brief Get the minimum number of points that a cluster needs to contain in order to be considered valid. */
  int
  getMinClusterSize ();
  
  /** \brief Set the minimum number of points that a cluster needs to contain in order to be considered valid. */
  void
  setMinClusterSize (int min_cluster_size);
  
  /** \brief Get the maximum number of points that a cluster needs to contain in order to be considered valid. */
  int
  getMaxClusterSize ();
  
  /** \brief Set the maximum number of points that a cluster needs to contain in order to be considered valid. */
  void
  setMaxClusterSize (int max_cluster_size);
  
  /** \brief Returns the flag value. This flag signalizes which mode of algorithm will be used.
        * If it is set to true than it will work as said in the article. This means that
        * it will be testing the angle between normal of the current point and it's neighbours normal.
        * Otherwise, it will be testing the angle between normal of the current point
        * and normal of the initial point that was chosen for growing new segment.
        */
  bool
  getSmoothModeFlag () const;
  
  /** \brief This function allows to turn on/off the smoothness constraint.
        * \param[in] value new mode value, if set to true then the smooth version will be used.
        */
  void
  setSmoothModeFlag (bool value);
  
  /** \brief Returns smoothness threshold. */
  float
  getSmoothnessThreshold () const;
  
  /** \brief Allows to set smoothness threshold used for testing the points.
    * \param[in] theta new threshold value for the angle between normals
    */
  void
  setSmoothnessThreshold (float theta);
  
  
  /** \brief Returns the number of nearest neighbours used for KNN. */
  unsigned int
  getNumberOfNeighbours () const;
  
  /** \brief Allows to set the number of neighbours. For more information check the article.
          * \param[in] neighbour_number number of neighbours to use
          */
  void
  setNumberOfNeighbours (unsigned int neighbour_number);
  
  /** \brief Returns the pointer to the search method that is used for KNN. */
  KdTreePtr
  getSearchMethod () const;
  
  /** \brief Allows to set search method that will be used for finding KNN.
        * \param[in] tree pointer to a KdTree
        */
  void
  setSearchMethod (const KdTreePtr& tree);
  
  /** \brief This method launches the segmentation algorithm and returns the clusters that were
        * obtained during the segmentation.
        * \param[out] clusters clusters that were obtained. Each cluster is an array of point indices.
        */
  virtual void
  extract (std::vector <pcl::PointIndices>& clusters);
  
  /** \brief For a given point this function builds a segment to which it belongs and returns this segment.
        * \param[in] index index of the initial point which will be the seed for growing a segment.
        * \param[out] cluster cluster to which the point belongs.
        */
  virtual void
  getSegmentFromPoint (int index, pcl::PointIndices& cluster);
  
protected:
  
  /** \brief This method simply checks if it is possible to execute the segmentation algorithm with
        * the current settings. If it is possible then it returns true.
        */
  virtual bool
  prepareForSegmentation ();
  
  /** \brief This method finds KNN for each point and saves them to the array
        * because the algorithm needs to find KNN a few times.
        */
  virtual void
  findPointNeighbours ();
  
  /** \brief This function implements the algorithm described in the article
        * "Segmentation of point clouds using smoothness constraint"
        * by T. Rabbania, F. A. van den Heuvelb, G. Vosselmanc.
        */
  void
  applySmoothRegionGrowingAlgorithm ();
  
  /** \brief This method grows a segment for the given seed point. And returns the number of its points.
        * \param[in] initial_seed index of the point that will serve as the seed point
        * \param[in] segment_number indicates which number this segment will have
        */
  int
  growRegion (int initial_seed, int segment_number);
  
  /** \brief This function is checking if the point with index 'nghbr' belongs to the segment.
        * If so, then it returns true. It also checks if this point can serve as the seed.
        * \param[in] initial_seed index of the initial point that was passed to the growRegion() function
        * \param[in] point index of the current seed point
        * \param[in] nghbr index of the point that is neighbour of the current seed
        * \param[out] is_a_seed this value is set to true if the point with index 'nghbr' can serve as the seed
        */
  virtual bool
  validatePoint (int initial_seed, int point, int nghbr, bool& is_a_seed) const;
  
  /** \brief This function simply assembles the regions from list of point labels.
        * Each cluster is an array of point indices.
        */
  void
  assembleRegions ();
  
protected:
  
  /** \brief Stores the minimum number of points that a cluster needs to contain in order to be considered valid. */
  int min_pts_per_cluster_;
  
  /** \brief Stores the maximum number of points that a cluster needs to contain in order to be considered valid. */
  int max_pts_per_cluster_;
  
  /** \brief Flag that signalizes if the smoothness constraint will be used. */
  bool smooth_mode_flag_;
  
  /** \brief If set to true then curvature test will be done during segmentation. */
  bool curvature_flag_;
  
  /** \brief If set to true then residual test will be done during segmentation. */
  bool residual_flag_;
  
  /** \brief Thershold used for testing the smoothness between points. */
  float z_threshold_;
  
  /** \brief Thershold used in residual test. */
  float residual_threshold_;
  
  /** \brief Thershold used in curvature test. */
  float curvature_threshold_;
  
  /** \brief Number of neighbours to find. */
  unsigned int neighbour_number_;
  
  /** \brief Serch method that will be used for KNN. */
  KdTreePtr search_;
  
  /** \brief Contains neighbours of each point. */
  std::vector<std::vector<int> > point_neighbours_;
  
  /** \brief Point labels that tells to which segment each point belongs. */
  std::vector<int> point_labels_;
  
  /** \brief If set to true then normal/smoothness test will be done during segmentation.
        * It is always set to true for the usual region growing algorithm. It is used for turning on/off the test
        * for smoothness in the child class RegionGrowingRGB.*/
  bool normal_flag_;
  
  /** \brief Tells how much points each segment contains. Used for reserving memory. */
  std::vector<int> num_pts_in_segment_;
  
  /** \brief After the segmentation it will contain the segments. */
  std::vector <pcl::PointIndices> clusters_;
  
  /** \brief Stores the number of segments. */
  int number_of_segments_;
  
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif // HOPE_H
