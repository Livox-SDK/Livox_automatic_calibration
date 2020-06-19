#ifndef POINT_CLOUD_MAPPER_H
#define POINT_CLOUD_MAPPER_H



#include <pcl/octree/octree_search.h>

#include <mutex>
#include <thread>
#include"common.h"

class PointCloudMapper {
 public:
	 typedef pcl::PointCloud<pcl::POINT_TYPE> PointCloud;
	 typedef pcl::octree::OctreePointCloudSearch<pcl::POINT_TYPE> Octree;

  PointCloudMapper();
  ~PointCloudMapper();

  bool Initialize();


  void Reset();

  bool InsertPoints(const PointCloud::ConstPtr& points,
                    PointCloud* incremental_points);
  bool ApproxNearestNeighbors(const PointCloud& points, PointCloud* neighbors);

  //void PublishMap();

 private:
  // Node initialization.
  bool LoadParameters();


  // The node's name.
  std::string name_;

  // Frame id for publishing map.
  std::string fixed_frame_id_;

  // Boolean initialization flag that is set after success from LoadParameters.
  bool initialized_;

  // Boolean to only publish the map if it has been updated recently.
  bool map_updated_;

  // When a loop closure occurs, this flag enables a user to unsubscribe from
  // and resubscribe to the incremental map topic in order to re-draw the map.
  bool incremental_unsubscribed_;

  // Containers storing the map and its structure.

  Octree::Ptr map_octree_;

  // Map parameters.
  double octree_resolution_;


  std::thread publish_thread_;
  mutable std::mutex map_mutex_;

public:
	PointCloud::Ptr map_data_;
};

#endif
