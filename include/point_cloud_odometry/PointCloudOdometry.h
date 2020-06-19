#ifndef POINT_CLOUD_ODOMETRY_H
#define POINT_CLOUD_ODOMETRY_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/registration/icp.h>

#include <geometry_utils/Transform3.h>
#include"common.h"
//#include <pcl_ros/point_cloud.h>
//#include <tf2_ros/transform_broadcaster.h>

class PointCloudOdometry {
 public:
	 typedef pcl::PointCloud<pcl::POINT_TYPE> PointCloud;

  PointCloudOdometry();
  ~PointCloudOdometry();

  bool Initialize(/*const ros::NodeHandle& n*/);

  bool UpdateEstimate(const PointCloud& points);

  // Get pose estimates.
  const geometry_utils::Transform3& GetIncrementalEstimate() const;
  const geometry_utils::Transform3& GetIntegratedEstimate() const;

  // not initialized.
  bool GetLastPointCloud(PointCloud::Ptr& out) const;

 private:
  // Node initialization.
  bool LoadParameters(/*const ros::NodeHandle& n*/);

  bool UpdateICP();

  

  // The node's name.
  std::string name_;

  // Pose estimates.
  geometry_utils::Transform3 integrated_estimate_;
  geometry_utils::Transform3 incremental_estimate_;

  

  // Coordinate frames.
  std::string fixed_frame_id_;
  std::string odometry_frame_id_;


  // For initialization.
  bool initialized_;

  // Point cloud containers.
  PointCloud::Ptr query_;
  PointCloud::Ptr reference_;

  // Parameters for filtering, and ICP.
  struct Parameters {
    // Stop ICP if the transformation from the last iteration was this small.
    double icp_tf_epsilon;

    // During ICP, two points won't be considered a correspondence if they are
    // at least this far from one another.
    double icp_corr_dist;

    // Iterate ICP this many times.
    unsigned int icp_iterations;
  } params_;

  // Maximum acceptable translation and rotation tolerances. If
  // transform_thresholding_ is set to false, neither of these thresholds are
  // considered.
  bool transform_thresholding_;
  double max_translation_;
  double max_rotation_;
};

#endif
