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


#include <point_cloud_odometry/PointCloudOdometry.h>
#include <parameter_utils/slamBase.h>


#include <pcl/registration/gicp.h>
#include"common.h"

namespace gu = geometry_utils;
//namespace gr = gu::ros;
//namespace pu = parameter_utils;

using pcl::copyPointCloud;
using pcl::GeneralizedIterativeClosestPoint;
using pcl::PointCloud;
using pcl::POINT_TYPE;

PointCloudOdometry::PointCloudOdometry() : initialized_(false) {
  query_.reset(new PointCloud);
  reference_.reset(new PointCloud);
}

PointCloudOdometry::~PointCloudOdometry() {}

bool PointCloudOdometry::Initialize(/*const ros::NodeHandle& n*/) {
	if (!LoadParameters()) {

		return false;
	}
	return true;
}

bool PointCloudOdometry::LoadParameters() {
	ParameterReader pd;
	fixed_frame_id_ = pd.getData("fixed");
	odometry_frame_id_ = pd.getData("odometry");

	// Load initial position.
	double init_x = 0.0, init_y = 0.0, init_z = 0.0;
	double init_roll = 0.0, init_pitch = 0.0, init_yaw = 0.0;
	init_x = atof(pd.getData("position_x").c_str());
	init_y = atof(pd.getData("position_y").c_str());
	init_z = atof(pd.getData("position_z").c_str());

	init_roll = atof(pd.getData("orientation_roll").c_str());
	init_pitch = atof(pd.getData("orientation_pitch").c_str());
	init_yaw = atof(pd.getData("orientation_yaw").c_str());
	gu::Transform3 init;
	init.translation = gu::Vec3(init_x, init_y, init_z);
	init.rotation = gu::Rot3(init_roll, init_pitch, init_yaw);
	integrated_estimate_ = init;

	params_.icp_tf_epsilon = atof(pd.getData("tf_epsilon").c_str());
	params_.icp_corr_dist = atof(pd.getData("corr_dist").c_str());
	params_.icp_iterations = atoi(pd.getData("iterations").c_str());

	transform_thresholding_ = atoi(pd.getData("transform_thresholding").c_str());
	max_translation_ = atof(pd.getData("max_translation").c_str());
	max_rotation_ = atof(pd.getData("max_rotation").c_str());

	return true;
}


bool PointCloudOdometry::UpdateEstimate(const PointCloud& points) {
  if (!initialized_) {
    copyPointCloud(points, *query_);
    initialized_ = true;
    return false;
  }
  copyPointCloud(*query_, *reference_);
  copyPointCloud(points, *query_);
  return UpdateICP();
}

const gu::Transform3& PointCloudOdometry::GetIncrementalEstimate() const {
  return incremental_estimate_;
}

const gu::Transform3& PointCloudOdometry::GetIntegratedEstimate() const {
  return integrated_estimate_;
}

bool PointCloudOdometry::GetLastPointCloud(PointCloud::Ptr& out) const {
  if (!initialized_ || query_ == NULL) {
    return false;
  }

  out = query_;
  return true;
}

bool PointCloudOdometry::UpdateICP() {
  // Compute the incremental transformation.
	GeneralizedIterativeClosestPoint<POINT_TYPE, POINT_TYPE> icp;
  icp.setTransformationEpsilon(params_.icp_tf_epsilon);
  icp.setMaxCorrespondenceDistance(params_.icp_corr_dist);
  icp.setMaximumIterations(params_.icp_iterations);
  icp.setRANSACIterations(0);

  icp.setInputSource(query_);
  icp.setInputTarget(reference_);

  PointCloud unused_result;
  icp.align(unused_result);

  const Eigen::Matrix4f T = icp.getFinalTransformation();

  // Update pose estimates.
  incremental_estimate_.translation = gu::Vec3(T(0, 3), T(1, 3), T(2, 3));
  incremental_estimate_.rotation = gu::Rot3(T(0, 0), T(0, 1), T(0, 2),
                                            T(1, 0), T(1, 1), T(1, 2),
                                            T(2, 0), T(2, 1), T(2, 2));

  // Only update if the incremental transform is small enough.
  if (!transform_thresholding_ ||
      (incremental_estimate_.translation.Norm() <= max_translation_ &&
       incremental_estimate_.rotation.ToEulerZYX().Norm() <= max_rotation_)) {
    integrated_estimate_ =
        gu::PoseUpdate(integrated_estimate_, incremental_estimate_);
  } else {
   
  }


  return true;
}
