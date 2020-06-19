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


#include <point_cloud_filter/PointCloudFilter.h>
#include <parameter_utils/slamBase.h>
//#include <parameter_utils/ParameterUtils.h>


#include <pcl/filters/filter.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>

//namespace pu = parameter_utils;

PointCloudFilter::PointCloudFilter() {}
PointCloudFilter::~PointCloudFilter() {}

bool PointCloudFilter::Initialize() {
  if (!LoadParameters()) {
  
    return false;
  }
  return true;
}

bool PointCloudFilter::LoadParameters() {
  // Load filtering parameters.
  ParameterReader pd;
  params_.grid_filter = atoi(pd.getData("grid_filter").c_str());
  params_.grid_res = atof(pd.getData("grid_res").c_str());
  params_.random_filter = atoi(pd.getData("random_filter").c_str());
  params_.decimate_percentage = atof(pd.getData("decimate_percentage").c_str());
  params_.outlier_filter = atoi(pd.getData("outlier_filter").c_str());
  params_.outlier_std = atof(pd.getData("outlier_std").c_str());
  params_.outlier_knn = atoi(pd.getData("outlier_knn").c_str());

  params_.radius_filter = atoi(pd.getData("radius_filter").c_str());
  params_.radius = atof(pd.getData("radius").c_str());
  params_.radius_knn = atoi(pd.getData("radius_knn").c_str());

  // Cap to [0.0, 1.0].
  params_.decimate_percentage =
      std::min(1.0, std::max(0.0, params_.decimate_percentage));

  return true;
}


bool PointCloudFilter::Filter(const PointCloud::ConstPtr& points,
                              PointCloud::Ptr points_filtered) const {
  if (points_filtered == NULL) {
  //  ROS_ERROR("%s: Output is null.", name_.c_str());
    return false;
  }

  // Copy input points.
  *points_filtered = *points;

  // Apply a random downsampling filter to the incoming point cloud.
  if (params_.random_filter) {
    const int n_points = static_cast<int>((1.0 - params_.decimate_percentage) *
                                          points_filtered->size());
	pcl::RandomSample<pcl::POINT_TYPE> random_filter;
    random_filter.setSample(n_points);
    random_filter.setInputCloud(points_filtered);
    random_filter.filter(*points_filtered);
  }

  // Apply a voxel grid filter to the incoming point cloud.
  if (params_.grid_filter) {
	  pcl::VoxelGrid<pcl::POINT_TYPE> grid;
    grid.setLeafSize(params_.grid_res, params_.grid_res, params_.grid_res);
    grid.setInputCloud(points_filtered);
    grid.filter(*points_filtered);
  }

  // Remove statistical outliers in incoming the point cloud.
  if (params_.outlier_filter) {
	  pcl::StatisticalOutlierRemoval<pcl::POINT_TYPE> sor;
    sor.setInputCloud(points_filtered);
    sor.setMeanK(params_.outlier_knn);
    sor.setStddevMulThresh(params_.outlier_std);
    sor.filter(*points_filtered);
  }

  // Remove points without a threshold number of neighbors within a specified
  // radius.
  if (params_.radius_filter) {
	  pcl::RadiusOutlierRemoval<pcl::POINT_TYPE> rad;
    rad.setInputCloud(points_filtered);
    rad.setRadiusSearch(params_.radius);
    rad.setMinNeighborsInRadius(params_.radius_knn);
    rad.filter(*points_filtered);
  }

  return true;
}
