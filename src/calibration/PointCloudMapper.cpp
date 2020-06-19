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


#include <point_cloud_mapper/PointCloudMapper.h>
#include <parameter_utils/slamBase.h>

//namespace pu = parameter_utils;

PointCloudMapper::PointCloudMapper()
    : initialized_(false),
      map_updated_(false),
      incremental_unsubscribed_(false) {
  // Initialize map data container.
  map_data_.reset(new PointCloud);
}

PointCloudMapper::~PointCloudMapper() {
  if (publish_thread_.joinable()) {
    publish_thread_.join();
  }
}

bool PointCloudMapper::Initialize(/*const ros::NodeHandle& n*/) {
	if (!LoadParameters()) {

		return false;
	}
	return true;
}

bool PointCloudMapper::LoadParameters() {
	ParameterReader pd;
	fixed_frame_id_ = pd.getData("fixed");
	map_data_->header.frame_id = fixed_frame_id_;
	octree_resolution_ = atof(pd.getData("octree_resolution").c_str());
	// Initialize the map octree.
	map_octree_.reset(new Octree(octree_resolution_));
	map_octree_->setInputCloud(map_data_);

	initialized_ = true;

	return true;
}

void PointCloudMapper::Reset() {
  map_data_.reset(new PointCloud);
  map_data_->header.frame_id = fixed_frame_id_;
  map_octree_.reset(new Octree(octree_resolution_));
  map_octree_->setInputCloud(map_data_);

  initialized_ = true;
}

bool PointCloudMapper::InsertPoints(const PointCloud::ConstPtr& points,
                                    PointCloud* incremental_points) {
  if (!initialized_) {
    return false;
  }

  if (incremental_points == NULL) {
    return false;
  }
  incremental_points->clear();

  
  if (map_mutex_.try_lock()) {
   
    for (size_t ii = 0; ii < points->points.size(); ++ii) {
		const pcl::POINT_TYPE p = points->points[ii];
      if (!map_octree_->isVoxelOccupiedAtPoint(p)) {
        map_octree_->addPointToCloud(p, map_data_);
        incremental_points->push_back(p);
      }
    }
    map_mutex_.unlock();
  } else {
   
  }

  // Publish the incremental map update.
  incremental_points->header = points->header;
  incremental_points->header.frame_id = fixed_frame_id_;
//  PublishMapUpdate(*incremental_points);

  map_updated_ = true;
  return true;
}

bool PointCloudMapper::ApproxNearestNeighbors(const PointCloud& points,
                                              PointCloud* neighbors) {
  if (!initialized_) {
  }

  if (neighbors == NULL) {
  }

  neighbors->points.clear();

  for (size_t ii = 0; ii < points.points.size(); ++ii) {
    float unused = 0.f;
    int result_index = -1; 
    map_octree_->approxNearestSearch(points.points[ii], result_index, unused);
    if (result_index >= 0)
      neighbors->push_back(map_data_->points[result_index]);
  }
    
  return neighbors->points.size() > 0;
}
