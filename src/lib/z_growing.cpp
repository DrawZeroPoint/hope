#include "z_growing.h"

ZGrowing::ZGrowing() :
  min_pts_per_cluster_ (1),
  max_pts_per_cluster_ (std::numeric_limits<int>::max ()),
  smooth_mode_flag_ (true),
  curvature_flag_ (true),
  residual_flag_ (false),
  z_threshold_ (0.003),
  neighbour_number_ (30),
  search_ (),
  point_neighbours_ (0),
  point_labels_ (0),
  normal_flag_ (true),
  num_pts_in_segment_ (0),
  clusters_ (0),
  number_of_segments_ (0)
{
  
}

ZGrowing::~ZGrowing ()
{
  if (search_ != 0)
    search_.reset ();
  
  point_neighbours_.clear ();
  point_labels_.clear ();
  num_pts_in_segment_.clear ();
  clusters_.clear ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
ZGrowing::getMinClusterSize ()
{
  return (min_pts_per_cluster_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
ZGrowing::setMinClusterSize (int min_cluster_size)
{
  min_pts_per_cluster_ = min_cluster_size;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
ZGrowing::getMaxClusterSize ()
{
  return (max_pts_per_cluster_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
ZGrowing::setMaxClusterSize (int max_cluster_size)
{
  max_pts_per_cluster_ = max_cluster_size;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
ZGrowing::getSmoothModeFlag () const
{
  return (smooth_mode_flag_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
ZGrowing::setSmoothModeFlag (bool value)
{
  smooth_mode_flag_ = value;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float
ZGrowing::getZThreshold () const
{
  return (z_threshold_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
ZGrowing::setZThreshold (float theta)
{
  z_threshold_ = theta;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
unsigned int
ZGrowing::getNumberOfNeighbours () const
{
  return (neighbour_number_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
ZGrowing::setNumberOfNeighbours (unsigned int neighbour_number)
{
  neighbour_number_ = neighbour_number;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ZGrowing::KdTreePtr
ZGrowing::getSearchMethod () const
{
  return (search_);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
ZGrowing::setSearchMethod (const KdTreePtr& tree)
{
  if (search_ != 0)
    search_.reset ();
  
  search_ = tree;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
ZGrowing::extract (std::vector <pcl::PointIndices>& clusters)
{
  clusters_.clear ();
  clusters.clear ();
  point_neighbours_.clear ();
  point_labels_.clear ();
  num_pts_in_segment_.clear ();
  number_of_segments_ = 0;
  
  bool segmentation_is_possible = initCompute ();
  if ( !segmentation_is_possible )
  {
    deinitCompute ();
    return;
  }
  
  segmentation_is_possible = prepareForSegmentation ();
  if ( !segmentation_is_possible )
  {
    deinitCompute ();
    return;
  }
  
  findPointNeighbours ();
  applySmoothRegionGrowingAlgorithm ();
  assembleRegions ();
  
  clusters.resize (clusters_.size ());
  std::vector<pcl::PointIndices>::iterator cluster_iter_input = clusters.begin ();
  for (std::vector<pcl::PointIndices>::const_iterator cluster_iter = clusters_.begin (); cluster_iter != clusters_.end (); cluster_iter++)
  {
    if ((static_cast<int> (cluster_iter->indices.size ()) >= min_pts_per_cluster_) &&
        (static_cast<int> (cluster_iter->indices.size ()) <= max_pts_per_cluster_))
    {
      *cluster_iter_input = *cluster_iter;
      cluster_iter_input++;
    }
  }
  
  clusters_ = std::vector<pcl::PointIndices> (clusters.begin (), cluster_iter_input);
  clusters.resize(clusters_.size());
  
  deinitCompute ();
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
ZGrowing::prepareForSegmentation ()
{
  // if user forgot to pass point cloud or if it is empty
  if ( input_->points.size () == 0 )
    return (false);
  
  // from here we check those parameters that are always valuable
  if (neighbour_number_ == 0)
    return (false);
  
  // if user didn't set search method
  if (!search_)
    search_.reset (new pcl::search::KdTree<pcl::PointXYZ>);
  
  if (indices_)
  {
    if (indices_->empty ())
      PCL_ERROR ("[hope::prepareForSegmentation] Empty given indices!\n");
    search_->setInputCloud (input_, indices_);
  }
  else
    search_->setInputCloud (input_);
  
  return (true);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
ZGrowing::findPointNeighbours ()
{
  int point_number = static_cast<int> (indices_->size ());
  std::vector<int> neighbours;
  std::vector<float> distances;
  
  point_neighbours_.resize (input_->points.size (), neighbours);
  if (input_->is_dense)
  {
    for (int i_point = 0; i_point < point_number; i_point++)
    {
      int point_index = (*indices_)[i_point];
      neighbours.clear ();
      search_->nearestKSearch (i_point, neighbour_number_, neighbours, distances);
      point_neighbours_[point_index].swap (neighbours);
    }
  }
  else
  {
    for (int i_point = 0; i_point < point_number; i_point++)
    {
      neighbours.clear ();
      int point_index = (*indices_)[i_point];
      if (!pcl::isFinite (input_->points[point_index]))
        continue;
      search_->nearestKSearch (i_point, neighbour_number_, neighbours, distances);
      point_neighbours_[point_index].swap (neighbours);
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
ZGrowing::applySmoothRegionGrowingAlgorithm ()
{
  int num_of_pts = static_cast<int> (indices_->size ());
  point_labels_.resize (input_->points.size (), -1);
  
  std::vector< std::pair<float, int> > point_residual;
  std::pair<float, int> pair;
  point_residual.resize (num_of_pts, pair);
  
  if (normal_flag_)
  {
    for (int i_point = 0; i_point < num_of_pts; i_point++)
    {
      int point_index = (*indices_)[i_point];
      point_residual[i_point].second = point_index;
    }
    std::sort (point_residual.begin (), point_residual.end (), pcl::comparePair);
  }
  else
  {
    for (int i_point = 0; i_point < num_of_pts; i_point++)
    {
      int point_index = (*indices_)[i_point];
      point_residual[i_point].first = 0;
      point_residual[i_point].second = point_index;
    }
  }
  int seed_counter = 0;
  int seed = point_residual[seed_counter].second;
  
  int segmented_pts_num = 0;
  int number_of_segments = 0;
  while (segmented_pts_num < num_of_pts)
  {
    int pts_in_segment;
    pts_in_segment = growRegion (seed, number_of_segments);
    segmented_pts_num += pts_in_segment;
    num_pts_in_segment_.push_back (pts_in_segment);
    number_of_segments++;
    
    //find next point that is not segmented yet
    for (int i_seed = seed_counter + 1; i_seed < num_of_pts; i_seed++)
    {
      int index = point_residual[i_seed].second;
      if (point_labels_[index] == -1)
      {
        seed = index;
        seed_counter = i_seed;
        break;
      }
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
int
ZGrowing::growRegion (int initial_seed, int segment_number)
{
  std::queue<int> seeds;
  seeds.push (initial_seed);
  point_labels_[initial_seed] = segment_number;
  
  int num_pts_in_segment = 1;
  
  while (!seeds.empty ())
  {
    int curr_seed;
    curr_seed = seeds.front ();
    seeds.pop ();
    
    size_t i_nghbr = 0;
    while ( i_nghbr < neighbour_number_ && i_nghbr < point_neighbours_[curr_seed].size () )
    {
      int index = point_neighbours_[curr_seed][i_nghbr];
      if (point_labels_[index] != -1)
      {
        i_nghbr++;
        continue;
      }
      
      bool is_a_seed = false;
      bool belongs_to_segment = validatePoint (initial_seed, curr_seed, index, is_a_seed);
      
      if (belongs_to_segment == false)
      {
        i_nghbr++;
        continue;
      }
      
      point_labels_[index] = segment_number;
      num_pts_in_segment++;
      
      if (is_a_seed)
      {
        seeds.push (index);
      }
      
      i_nghbr++;
    }// next neighbour
  }// next seed
  
  return (num_pts_in_segment);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool
ZGrowing::validatePoint (int initial_seed, int point, int nghbr, bool& is_a_seed) const
{
  is_a_seed = true;
  
  float z_threshold = z_threshold_;
  
  float seed_z = input_->points[point].z;
  
  // check delta z between neighbors
  if (smooth_mode_flag_ == true)
  {
    float nghbr_z = input_->points[nghbr].z;
    float delta_z = fabs(seed_z - nghbr_z);
    if (delta_z < z_threshold)
    {
      return true;
    }
  }
  else
  {
    float nghbr_z = input_->points[nghbr].z;
    float initial_seed_z = input_->points[initial_seed].z;
    float delta_z = fabs(initial_seed_z - nghbr_z);
    if (delta_z < z_threshold)
      return true;
  }

  return false;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
ZGrowing::assembleRegions ()
{
  int number_of_segments = static_cast<int> (num_pts_in_segment_.size ());
  int number_of_points = static_cast<int> (input_->points.size ());
  
  pcl::PointIndices segment;
  clusters_.resize (number_of_segments, segment);
  
  for (int i_seg = 0; i_seg < number_of_segments; i_seg++)
  {
    clusters_[i_seg].indices.resize ( num_pts_in_segment_[i_seg], 0);
  }
  
  std::vector<int> counter;
  counter.resize (number_of_segments, 0);
  
  for (int i_point = 0; i_point < number_of_points; i_point++)
  {
    int segment_index = point_labels_[i_point];
    if (segment_index != -1)
    {
      int point_index = counter[segment_index];
      clusters_[segment_index].indices[point_index] = i_point;
      counter[segment_index] = point_index + 1;
    }
  }
  
  number_of_segments_ = number_of_segments;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void
ZGrowing::getSegmentFromPoint (int index, pcl::PointIndices& cluster)
{
  cluster.indices.clear ();
  
  bool segmentation_is_possible = initCompute ();
  if ( !segmentation_is_possible )
  {
    deinitCompute ();
    return;
  }
  
  // first of all we need to find out if this point belongs to cloud
  bool point_was_found = false;
  int number_of_points = static_cast <int> (indices_->size ());
  for (int point = 0; point < number_of_points; point++)
    if ( (*indices_)[point] == index)
    {
      point_was_found = true;
      break;
    }
  
  if (point_was_found)
  {
    if (clusters_.empty ())
    {
      point_neighbours_.clear ();
      point_labels_.clear ();
      num_pts_in_segment_.clear ();
      number_of_segments_ = 0;
      
      segmentation_is_possible = prepareForSegmentation ();
      if ( !segmentation_is_possible )
      {
        deinitCompute ();
        return;
      }
      
      findPointNeighbours ();
      applySmoothRegionGrowingAlgorithm ();
      assembleRegions ();
    }
    // if we have already made the segmentation, then find the segment
    // to which this point belongs
    std::vector <pcl::PointIndices>::iterator i_segment;
    for (i_segment = clusters_.begin (); i_segment != clusters_.end (); i_segment++)
    {
      bool segment_was_found = false;
      for (size_t i_point = 0; i_point < i_segment->indices.size (); i_point++)
      {
        if (i_segment->indices[i_point] == index)
        {
          segment_was_found = true;
          cluster.indices.clear ();
          cluster.indices.reserve (i_segment->indices.size ());
          std::copy (i_segment->indices.begin (), i_segment->indices.end (), std::back_inserter (cluster.indices));
          break;
        }
      }
      if (segment_was_found)
      {
        break;
      }
    }// next segment
  }// end if point was found
  
  deinitCompute ();
}
