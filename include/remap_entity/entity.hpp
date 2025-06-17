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

namespace remap
{
namespace entity
{
class Entity
{
protected:
  std::string entity_id_;
  std::string anon_entity_id_;
  std::string entity_type_;
  std::function<void(const std::string &)> f_;
  std::function<void(const std::string &)> remove_f_;
  double last_updated_;

  std::vector<std::string> facts_;

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
    const double & time_threshold = 3.0);

  void storeFact(const std::string & fact);

  // Setters
  void setAnonEntityId(const std::string & anon_entity_id)
  {
    anon_entity_id_ = anon_entity_id;
  }

  // Getters
  std::string getEntityId() const
  {
    return entity_id_;
  }

  std::string getAnonEntityId() const
  {
    return anon_entity_id_;
  }

  std::string getEntityType() const
  {
    return entity_type_;
  }

  std::vector<std::string> getFacts() const
  {
    return facts_;
  }
};
}  // namespace entity
}  // namespace remap
#endif  // REMAP_ENTITY__ENTITY_HPP_
