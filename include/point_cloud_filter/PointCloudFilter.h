#ifndef POINT_CLOUD_FILTER_H
#define POINT_CLOUD_FILTER_H

//#include <ros/ros.h>
//#include <pcl_ros/point_cloud.h>// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include "common.h"

class PointCloudFilter {
 public:
	 typedef pcl::PointCloud<pcl::POINT_TYPE> PointCloud;

  PointCloudFilter();
  ~PointCloudFilter();

  bool Initialize();

  // Filter an incoming point cloud
  bool Filter(const PointCloud::ConstPtr& points,
              PointCloud::Ptr points_filtered) const;

 private:
  // Node initialization.
  bool LoadParameters();

  // The node's name.
  std::string name_;

  struct Parameters {
    // Apply a voxel grid filter.
    bool grid_filter;

    // Resolution of voxel grid filter.
    double grid_res;

    // Apply a random downsampling filter.
    bool random_filter;

    // Percentage of points to discard. Must be between 0.0 and 1.0;
    double decimate_percentage;

    // Apply a statistical outlier filter.
    bool outlier_filter;

    // Standard deviation threshold in distance to neighbors for outlier
    // removal.
    double outlier_std;

    // Number of nearest neighbors to use for outlier filter.
    unsigned int outlier_knn;

    // Apply a radius outlier filter.
    bool radius_filter;

    // Size of the radius filter.
    double radius;

    // If this number of neighbors are not found within a radius around each
    // point, remove that point.
    unsigned int radius_knn;
  } params_;
};

#endif
