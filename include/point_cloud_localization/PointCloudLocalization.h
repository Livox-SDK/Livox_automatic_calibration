#ifndef POINT_CLOUD_LOCALIZATION_H
#define POINT_CLOUD_LOCALIZATION_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <ros/ros.h>
#include <geometry_utils/Transform3.h>

//#include <pcl_ros/point_cloud.h>
//#include <tf2_ros/transform_broadcaster.h>
#include"common.h"

class PointCloudLocalization {
 public:
	 typedef pcl::PointCloud<pcl::POINT_TYPE> PointCloud;

  PointCloudLocalization();
  ~PointCloudLocalization();

  bool Initialize();

  bool TransformPointsToFixedFrame(const PointCloud& points,
                                   PointCloud* points_transformed) const;

  bool TransformPointsToSensorFrame(const PointCloud& points,
                                    PointCloud* points_transformed) const;

  bool MotionUpdate(const geometry_utils::Transform3& incremental_odom);

  bool MeasurementUpdate(const PointCloud::Ptr& query,
                         const PointCloud::Ptr& reference,
                         PointCloud* aligned_query);

  const geometry_utils::Transform3& GetIncrementalEstimate() const;
  const geometry_utils::Transform3& GetIntegratedEstimate() const;

  void SetIntegratedEstimate(
      const geometry_utils::Transform3& integrated_estimate);

 private:
  // Node initialization.
  bool LoadParameters();

  
  // The node's name.
  std::string name_;

  // Pose estimate.
  geometry_utils::Transform3 incremental_estimate_;
  geometry_utils::Transform3 integrated_estimate_;

  

  // Coordinate frames.
  std::string fixed_frame_id_;
  std::string base_frame_id_;

  // Parameters for filtering and ICP.
  struct Parameters {
    // Stop ICP if the transformation from the last iteration was this small.
    double tf_epsilon;

    // During ICP, two points won't be considered a correspondence if they are
    // at least this far from one another.
    double corr_dist;

    // Iterate ICP this many times.
    unsigned int iterations;
  } params_;


  bool transform_thresholding_;
  double max_translation_;
  double max_rotation_;
};

#endif
