/*
The mapping algorithm is an advanced implementation of the following open source project:
  [blam](https://github.com/erik-nelson/blam). 
Modifier: livox               dev@livoxtech.com


Copyright (c) 2015, The Regents of the University of California (Regents).
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are
met:

   1. Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above
      copyright notice, this list of conditions and the following
      disclaimer in the documentation and/or other materials provided
      with the distribution.

   3. Neither the name of the copyright holder nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/


#include <point_cloud_localization/PointCloudLocalization.h>
#include <parameter_utils/slamBase.h>
//#include <parameter_utils/ParameterUtils.h>
#include <pcl/registration/gicp.h>

namespace gu = geometry_utils;
//namespace gr = gu::ros;
//namespace pu = parameter_utils;

using pcl::GeneralizedIterativeClosestPoint;
using pcl::PointCloud;
using pcl::POINT_TYPE;
using pcl::transformPointCloud;

PointCloudLocalization::PointCloudLocalization() {}
PointCloudLocalization::~PointCloudLocalization() {}

bool PointCloudLocalization::Initialize() {
  
	if (!LoadParameters()) {

		return false;
	}
	return true;
}

bool PointCloudLocalization::LoadParameters() {

	ParameterReader pd;
	fixed_frame_id_ = pd.getData("fixed");
	base_frame_id_ = pd.getData("base");
	// Load initial position.
	double init_x = 0.0, init_y = 0.0, init_z = 0.0;
	double init_roll = 0.0, init_pitch = 0.0, init_yaw = 0.0;
	init_x = atof(pd.getData("position_x").c_str());
	init_y = atof(pd.getData("position_y").c_str());
	init_z = atof(pd.getData("position_z").c_str());

	init_roll = atof(pd.getData("orientation_roll").c_str());
	init_pitch = atof(pd.getData("orientation_pitch").c_str());
	init_yaw = atof(pd.getData("orientation_yaw").c_str());
	integrated_estimate_.translation = gu::Vec3(init_x, init_y, init_z);
	integrated_estimate_.rotation = gu::Rot3(init_roll, init_pitch, init_yaw);

	params_.tf_epsilon = atof(pd.getData("tf_epsilon").c_str());
	params_.corr_dist = atof(pd.getData("corr_dist").c_str());
	params_.iterations = atoi(pd.getData("iterations").c_str());
	
	transform_thresholding_ = atoi(pd.getData("transform_thresholding").c_str());
	max_translation_ = atof(pd.getData("max_translation").c_str());
	max_rotation_ = atof(pd.getData("max_rotation").c_str());

	return true;
}


const gu::Transform3& PointCloudLocalization::GetIncrementalEstimate() const {
  return incremental_estimate_;
}

const gu::Transform3& PointCloudLocalization::GetIntegratedEstimate() const {
  return integrated_estimate_;
}

void PointCloudLocalization::SetIntegratedEstimate(
    const gu::Transform3& integrated_estimate) {
  integrated_estimate_ = integrated_estimate;

}

bool PointCloudLocalization::MotionUpdate(
    const gu::Transform3& incremental_odom) {
  // Store the incremental transform from odometry.
  incremental_estimate_ = incremental_odom;
  return true;
}

bool PointCloudLocalization::TransformPointsToFixedFrame(
    const PointCloud& points, PointCloud* points_transformed) const {
  if (points_transformed == NULL) {
    return false;
  }

  // Compose the current incremental estimate (from odometry) with the
  // integrated estimate, and transform the incoming point cloud.
  const gu::Transform3 estimate =
      gu::PoseUpdate(integrated_estimate_, incremental_estimate_);
  const Eigen::Matrix<double, 3, 3> R = estimate.rotation.Eigen();
  const Eigen::Matrix<double, 3, 1> T = estimate.translation.Eigen();

  Eigen::Matrix4d tf;
  tf.block(0, 0, 3, 3) = R;
  tf.block(0, 3, 3, 1) = T;

  pcl::transformPointCloud(points, *points_transformed, tf);

  return true;
}

bool PointCloudLocalization::TransformPointsToSensorFrame(
    const PointCloud& points, PointCloud* points_transformed) const {
  if (points_transformed == NULL) {
    return false;
  }

  // Compose the current incremental estimate (from odometry) with the
  // integrated estimate, then invert to go from world to sensor frame.
  const gu::Transform3 estimate = gu::PoseInverse(
      gu::PoseUpdate(integrated_estimate_, incremental_estimate_));//integrated_estimate from config parameters
  const Eigen::Matrix<double, 3, 3> R = estimate.rotation.Eigen();
  const Eigen::Matrix<double, 3, 1> T = estimate.translation.Eigen();

  Eigen::Matrix4d tf;
  tf.block(0, 0, 3, 3) = R;
  tf.block(0, 3, 3, 1) = T;

  pcl::transformPointCloud(points, *points_transformed, tf);

  return true;
}

bool PointCloudLocalization::MeasurementUpdate(const PointCloud::Ptr& query,
                                               const PointCloud::Ptr& reference,
                                               PointCloud* aligned_query) {
  if (aligned_query == NULL) {
    return false;
  }

// Store time stamp.
//  stamp_.fromNSec(query->header.stamp*1e3);

  // ICP-based alignment. Generalized ICP does (roughly) plane-to-plane
  // matching, and is much more robust than standard ICP.
  GeneralizedIterativeClosestPoint<pcl::POINT_TYPE, pcl::POINT_TYPE> icp;
  icp.setTransformationEpsilon(params_.tf_epsilon);
  icp.setMaxCorrespondenceDistance(params_.corr_dist);
  icp.setMaximumIterations(params_.iterations);
  icp.setRANSACIterations(0);
  icp.setMaximumOptimizerIterations(50); // default 20

  icp.setInputSource(query);
  icp.setInputTarget(reference);

  PointCloud unused;
  icp.align(unused);

  // Retrieve transformation and estimate and update.
  const Eigen::Matrix4f T = icp.getFinalTransformation();
  pcl::transformPointCloud(*query, *aligned_query, T);

  gu::Transform3 pose_update;
  pose_update.translation = gu::Vec3(T(0, 3), T(1, 3), T(2, 3));
  pose_update.rotation = gu::Rot3(T(0, 0), T(0, 1), T(0, 2),
                                  T(1, 0), T(1, 1), T(1, 2),
                                  T(2, 0), T(2, 1), T(2, 2));

  // Only update if the transform is small enough.
  if (!transform_thresholding_ ||
      (pose_update.translation.Norm() <= max_translation_ &&
       pose_update.rotation.ToEulerZYX().Norm() <= max_rotation_)) {
    incremental_estimate_ = gu::PoseUpdate(incremental_estimate_, pose_update);
  } else {
  
  }

  integrated_estimate_ =
      gu::PoseUpdate(integrated_estimate_, incremental_estimate_);

  
  return true;
}
