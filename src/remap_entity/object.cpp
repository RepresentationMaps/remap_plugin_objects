// Copyright 2025 PAL Robotics, S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "remap_entity/object.hpp"

namespace remap
{
namespace entity
{
	Object::Object(
	  const std::string & entity_id,
	  const std::string & entity_type,
	  const double & starting_time,
	  const std::vector<pcl::PointXYZ> & points) : 
			entity_id_(entity_id),
			entity_type_(entity_type),
			last_updated_(starting_time),
			points_(points) {}

	void Object::addPoints(const std::vector<pcl::PointXYZ> & points) {
		// We should optimize this operation in the plugin
		points_.insert(points_.end(), points.begin(), points.end());
	}

	float Object::computeIoU(
    const openvdb::CoordBBox & bbox) const
  {
    if (!object_bbox_.empty() && !bbox.empty()) {
      auto intersection = object_bbox_;
      intersection.intersect(bbox);
      float intersection_volume = intersection.volume();
      float union_volume = object_bbox_.volume() + bbox.volume() - intersection_volume;
      return (union_volume > 0.0f) ? (intersection_volume / union_volume) : 0.0f;
    }
    return 0.0f;
  }

  float Object::computIntersectionRatio(
    const openvdb::CoordBBox & bbox) const
  {
    if (!object_bbox_.empty() && !bbox.empty()) {
      auto intersection = object_bbox_;
      intersection.intersect(bbox);
      float intersection_volume = intersection.volume();
      return (intersection_volume > 0.0f) ? (intersection_volume / bbox.volume()) : 0.0f;
    }
    return 0.0f;
  }

}
}