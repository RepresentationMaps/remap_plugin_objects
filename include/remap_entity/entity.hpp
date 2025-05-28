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

#ifndef REMAP_ENTITY__ENTITY_HPP_
#define REMAP_ENTITY__ENTITY_HPP_

#include <functional>
#include <string>
#include <vector>

#include <openvdb/openvdb.h>

namespace remap
{
namespace entity
{
class Entity
{
protected:
  std::string entity_id_;
  std::string entity_type_;
  std::function<void(const std::string &)> f_;
  std::function<void(const std::string &)> remove_f_;
  double last_updated_;

  std::vector<std::string> facts_;

  // Let's start from something simple
  // We check whether the two objects:
  // - have the same type
  // - have some threshold intersection
  openvdb::CoordBBox object_bbox_;

public:
  Entity()
  {
    f_ = [](const std::string &) {};
    remove_f_ = [](const std::string &) {};
  }

  Entity(
    const std::string & entity_id,
    const std::string & entity_type,
    const double & starting_time);

  void update_f(const std::function<void(const std::string &)> & f);
  void updateRemove_f(const std::function<void(const std::string &)> & f);

  void updateTime(const double & time);

  void map(const bool & update_time = false);
  void remove();

  bool checkTime(
    const double & current_time,
    const double & time_threshold = 1.0);

  void storeFact(const std::string & fact);

  // Getters
  std::string getEntityId() const
  {
    return entity_id_;
  }

  std::string getEntityType() const
  {
    return entity_type_;
  }

  std::vector<std::string> getFacts() const
  {
    return facts_;
  }

  void storeBBox(
    const openvdb::CoordBBox & bbox)
  {
    object_bbox_ = bbox;
  }

  void mergeBBox(
    const openvdb::CoordBBox & bbox)
  {
    if (!object_bbox_.empty() && !bbox.empty()) {
      object_bbox_.expand(bbox);
    } else if (!bbox.empty()) {
      object_bbox_ = bbox;
    }
  }

  float computeIoU(
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

  float computIntersectionRatio(
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
};
}  // namespace entity
}  // namespace remap
#endif  // REMAP_ENTITY__ENTITY_HPP_
