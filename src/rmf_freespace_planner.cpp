/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include "rmf_freespace_planner/rmf_freespace_planner.hpp"

#include <iostream>

namespace rmf_freespace_planner {
FreespacePlanner::FreespacePlanner(
  rmf_utils::clone_ptr<rmf_traffic::agv::RouteValidator> validator)
: validator(std::move(validator))
{
}

bool FreespacePlanner::has_conflict(
  const std::string& map,
  rmf_traffic::Trajectory trajectory)
{
  return validator->find_conflict({map, std::move(trajectory)}).has_value();
}
}
