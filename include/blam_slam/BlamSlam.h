#ifndef BLAM_SLAM_H
#define BLAM_SLAM_H


#include <point_cloud_filter/PointCloudFilter.h>
#include <point_cloud_odometry/PointCloudOdometry.h>
#include <point_cloud_localization/PointCloudLocalization.h>
#include <point_cloud_mapper/PointCloudMapper.h>

//// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include<string>
#include"common.h"


class BlamSlam {
 public:
	 typedef pcl::PointCloud<pcl::POINT_TYPE> PointCloud;

  BlamSlam();
  ~BlamSlam();

  bool Initialize();
  void ProcessPointCloudMessage(const PointCloud::ConstPtr& msg);
  bool showPointCloud(int FrameCounter);
  std::string itos(int i);   // ½«int ×ª»»³Éstring 

private:

  // The node's name.
  std::string name_;
  // Update rates and callback timers.
  double estimate_update_rate_;
  double visualization_update_rate_;

  // Names of coordinate frames.
  std::string fixed_frame_id_;
  std::string base_frame_id_;
public:
  PointCloudFilter filter_;
  PointCloudOdometry odometry_;
  PointCloudLocalization localization_;
  PointCloudMapper mapper_;
};

#endif
