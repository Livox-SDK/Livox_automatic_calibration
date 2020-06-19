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

#include <blam_slam/BlamSlam.h>
#include <geometry_utils/Transform3.h>

namespace gu = geometry_utils;


BlamSlam::BlamSlam() {
}

BlamSlam::~BlamSlam() {}



bool BlamSlam::Initialize() {

	if (!filter_.Initialize()) {
		return false;
	}

	if (!odometry_.Initialize()) {
		return false;
	}
	
	if (!localization_.Initialize()) {
		return false;
	}

	if (!mapper_.Initialize()) {
		return false;
	}
	return true;
}



void BlamSlam::ProcessPointCloudMessage(const PointCloud::ConstPtr& msg) {
	PointCloud::Ptr msg_filtered(new PointCloud);
	filter_.Filter(msg, msg_filtered);

	//IF  Fitst Frame ,do :
	if (!odometry_.UpdateEstimate(*msg_filtered)) {
		PointCloud::Ptr unused(new PointCloud);
		mapper_.InsertPoints(msg_filtered, unused.get());
		//loop_closure_.AddKeyScanPair(0, msg);
		return;
	}
	//ELSE

	PointCloud::Ptr msg_transformed(new PointCloud);
	PointCloud::Ptr msg_neighbors(new PointCloud);
	PointCloud::Ptr msg_base(new PointCloud);
	PointCloud::Ptr msg_fixed(new PointCloud);

	

	localization_.MotionUpdate(odometry_.GetIncrementalEstimate()); 

	localization_.TransformPointsToFixedFrame(*msg_filtered,msg_transformed.get());

	

	mapper_.ApproxNearestNeighbors(*msg_transformed, msg_neighbors.get());

	

	localization_.TransformPointsToSensorFrame(*msg_neighbors, msg_neighbors.get());

	

	localization_.MeasurementUpdate(msg_filtered, msg_neighbors, msg_base.get());

	
	localization_.MotionUpdate(gu::Transform3::Identity());
	localization_.TransformPointsToFixedFrame(*msg, msg_fixed.get());
	PointCloud::Ptr unused(new PointCloud);
	mapper_.InsertPoints(msg_fixed, unused.get());

	
}
std::string BlamSlam::itos(int i)   {
	std::stringstream s;
	s << i;
	return s.str();
}
