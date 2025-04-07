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

#include <iostream>
#include <chrono>

#include "remap_entity/entity.hpp"

namespace remap
{
namespace entity
{
Entity::Entity(
  const std::string & entity_id,
  const std::string & entity_type,
  const double & starting_time)
: entity_id_(entity_id),
  entity_type_(entity_type),
  last_updated_(starting_time)
{
  f_ = [](const std::string &) {};
  remove_f_ = [](const std::string &) {};
}

void Entity::update_f(const std::function<void(const std::string &)> & f)
{
  f_ = f;
}

void Entity::updateRemove_f(const std::function<void(const std::string &)> & f)
{
  remove_f_ = f;
}

void Entity::map(const bool & update_time)
{
  if (f_) {
    f_(entity_id_);
  }
  if (update_time) {
    last_updated_ = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();
  }
}

void Entity::remove()
{
  if (remove_f_) {
    remove_f_(entity_id_);
  }
}

void Entity::updateTime(const double & time)
{
  last_updated_ = time;
}

bool Entity::checkTime(
  const double & current_time,
  const double & time_threshold)
{
  return (current_time - last_updated_) < time_threshold;
}

void Entity::storeFact(const std::string & fact)
{
  facts_.push_back(fact);
}
}  // namespace entity
}  // namespace remap
