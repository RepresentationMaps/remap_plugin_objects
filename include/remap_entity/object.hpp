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

#ifndef REMAP_ENTITY__OBJECT_HPP_
#define REMAP_ENTITY__OBJECT_HPP_

#include <functional>
#include <string>
#include <vector>

#include <openvdb/openvdb.h>

#include <pcl/point_types.h>

namespace remap
{
namespace entity
{
class Object
{
protected:
  std::string entity_id_;
  std::string entity_type_;
  double last_updated_;

  std::vector<std::string> facts_;

  // Let's start from something simple
  // We check whether the two objects:
  // - have the same type
  // - have some threshold intersection
  openvdb::CoordBBox object_bbox_;

  // We store the object points
  std::vector<pcl::PointXYZ> points_;

public:
  Object(){}

  Object(
  	const std::string & entity_id,
		const std::string & entity_type,
		const double & starting_time,
		const std::vector<pcl::PointXYZ> & points = {});

  inline void setBBox(const openvdb::CoordBBox & bbox)
  {
  	object_bbox_ = bbox;
  }

  inline void expandBBox(const openvdb::CoordBBox & bbox)
  {
  	if (!object_bbox_.empty() && !bbox.empty()) {
			object_bbox_.expand(bbox);
		} else if (!bbox.empty()) {
			object_bbox_ = bbox;
		}
  }

	inline void updateTime(const double & time)
	{
		last_updated_ = time;
	}

	inline void setPoints(const std::vector<pcl::PointXYZ> & points)
	{
		points_ = points;
	}

	inline const std::vector<pcl::PointXYZ> & getPoints() const
	{
		return points_;
	}

	inline std::string getEntityId() const
	{
		return entity_id_;
	}

	inline std::string getEntityType() const
	{
		return entity_type_;
	}

	inline std::vector<std::string> getFacts() const
	{
		return facts_;
	}

	void addPoints(const std::vector<pcl::PointXYZ> & points);

	float computeIoU(const openvdb::CoordBBox & bbox) const;

	float computIntersectionRatio(const openvdb::CoordBBox & bbox) const;
};
}  // namespace entity
}  // namespace remap
#endif  // REMAP_ENTITY__OBJECT_HPP_