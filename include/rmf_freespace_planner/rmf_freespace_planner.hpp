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

#ifndef RMF_FREESPACE_PLANNER__RMF_FREESPACE_PLANNER_HPP
#define RMF_FREESPACE_PLANNER__RMF_FREESPACE_PLANNER_HPP

#include <rmf_traffic/agv/RouteValidator.hpp>
#include <rmf_traffic/Route.hpp>

namespace rmf_freespace_planner {

class FreespacePlanner
{
public:
  explicit FreespacePlanner(
    rmf_utils::clone_ptr<rmf_traffic::agv::RouteValidator> validator);

  std::vector<rmf_traffic::Route> make_plan(
    const std::vector<rmf_traffic::Route>& route);

protected:
  bool has_conflict(rmf_traffic::Trajectory trajectory);

private:
  virtual rmf_traffic::Trajectory plan(
    const rmf_traffic::Trajectory::Waypoint& start,
    const rmf_traffic::Trajectory::Waypoint& goal) = 0;

  rmf_utils::clone_ptr<rmf_traffic::agv::RouteValidator> validator;

  std::string map;
};
}

#endif //RMF_FREESPACE_PLANNER__RMF_FREESPACE_PLANNER_HPP
