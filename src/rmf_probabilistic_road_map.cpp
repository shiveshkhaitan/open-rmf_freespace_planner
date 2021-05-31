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

#include "rmf_freespace_planner/rmf_probabilistic_road_map.hpp"
#include <rmf_traffic/agv/Interpolate.hpp>
#include <rmf_traffic/agv/Planner.hpp>
#include <rmf_traffic/DetectConflict.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/Motion.hpp>

namespace rmf_freespace_planner {
namespace rmf_probabilistic_road_map {
ProbabilisticRoadMap::ProbabilisticRoadMap(
  rmf_utils::clone_ptr<rmf_traffic::agv::RouteValidator> validator,
  int num_close_vertices,
  double dist_close_vertices,
  std::shared_ptr<rmf_traffic::schedule::ItineraryViewer> itinerary_viewer,
  std::optional<std::unordered_set<rmf_traffic::schedule::ParticipantId>> excluded_participants)
: FreespacePlanner(std::move(validator)),
  num_close_vertices(num_close_vertices),
  dist_close_vertices(dist_close_vertices),
  itinerary_viewer(std::move(itinerary_viewer)),
  excluded_participants(std::move(excluded_participants))
{
}

ProbabilisticRoadMap::InternalState::InternalState(
  std::string map,
  Eigen::Vector3d start_position,
  Eigen::Vector3d goal_position,
  rmf_traffic::Time start_time,
  rmf_traffic::agv::VehicleTraits traits,
  std::optional<std::vector<Obstacle>> obstacles,
  Eigen::Matrix<double, 2, 2> state_limits,
  ProbabilisticRoadMap* probabilistic_road_map)
: map(std::move(map)),
  start_position(std::move(start_position)),
  goal_position(std::move(goal_position)),
  start_time(start_time),
  traits(std::move(traits)),
  obstacles(std::move(obstacles)),
  state_limits(std::move(state_limits)),
  probabilistic_road_map(probabilistic_road_map),
  gen(rd())
{
}

std::vector<rmf_traffic::Route> ProbabilisticRoadMap::plan(
  const Start& start,
  const Goal& goal,
  const rmf_traffic::agv::VehicleTraits& traits,
  const std::optional<std::vector<Obstacle>>& obstacles,
  const std::string& map)
{
  double length = (start.position.head<2>() - goal.position.head<2>()).norm();

  Eigen::Matrix<double, 2, 2> state_limits;
  state_limits << 0, length,
    -0.5, 0.5;

  InternalState internal_state(
    map,
    start.position,
    goal.position,
    start.time,
    traits,
    obstacles,
    state_limits,
    this
  );

  auto graph = internal_state.create_graph();

  const auto obstacle_validator =
    rmf_traffic::agv::ScheduleRouteValidator::make(
    itinerary_viewer,
    *excluded_participants.value().begin(),
    traits.profile());
  rmf_traffic::agv::Planner planner({graph, traits}, {obstacle_validator});

  rmf_traffic::agv::Planner::Start planner_start{
    start.time,
    0,
    start.position.z()};
  rmf_traffic::agv::Planner::Goal planner_goal{graph.num_waypoints() - 1};
  auto result = planner.plan(planner_start, planner_goal);
  while (!result.success())
  {
    graph = internal_state.expand_graph();
    planner = rmf_traffic::agv::Planner(
      {graph, traits}, {obstacle_validator});
    result = planner.plan(planner_start, planner_goal);
  }
  return result->get_itinerary();
}

rmf_traffic::agv::Graph ProbabilisticRoadMap::InternalState::expand_graph(
  std::size_t num_additional_vertices)
{
  for (std::size_t i = 0; i < num_additional_vertices; ++i)
  {
    auto position = generate_random_waypoint();
    while (has_conflict(position))
    {
      position = generate_random_waypoint();
    }
    connect_waypoint(graph.add_waypoint(map, position));
  }
  return graph;
}

bool ProbabilisticRoadMap::InternalState::has_conflict(
  const Eigen::Vector2d& position) const
{
  if (!obstacles.has_value())
  {
    return false;
  }

  for (const auto& obstacle : obstacles.value())
  {
    if ((position - obstacle.position).norm() <
      traits.profile().footprint()->get_characteristic_length() +
      obstacle.radius)
    {
      return true;
    }
  }
  return false;
}

bool ProbabilisticRoadMap::InternalState::has_conflict(
  const Eigen::Vector2d& position1,
  const Eigen::Vector2d& position2) const
{
  if (!obstacles.has_value())
  {
    return false;
  }

  for (const auto& obstacle : obstacles.value())
  {
    double distance = minimum_distance(position1, position2, obstacle.position);
    if (distance <
      traits.profile().footprint()->get_characteristic_length() +
      obstacle.radius)
    {
      return true;
    }
  }
  return false;
}

double ProbabilisticRoadMap::minimum_distance(
  Eigen::Vector2d segment_point1,
  Eigen::Vector2d segment_point2,
  Eigen::Vector2d point)
{
  Eigen::Vector2d segment;
  segment(0) = segment_point2(0) - segment_point1(0);
  segment(1) = segment_point2(1) - segment_point1(1);

  // vector point_to_point1
  Eigen::Vector2d point_to_point1;
  point_to_point1(0) = point(0) - segment_point1(0);
  point_to_point1(1) = point(1) - segment_point1(1);

  // vector point_to_point2
  Eigen::Vector2d point_to_point2;
  point_to_point2(0) = point(0) - segment_point2(0);
  point_to_point2(1) = point(1) - segment_point2(1);

  // Variables to store dot product
  double dot_product1, dot_product2;

  // Calculating the dot product
  dot_product1 =
    (segment(0) * point_to_point2(0) + segment(1) * point_to_point2(1));
  dot_product2 =
    (segment(0) * point_to_point1(0) + segment(1) * point_to_point1(1));

  // Minimum distance from
  // point point to the line segment
  double min_distance;

  if (dot_product1 > 0)
  {
    // Finding the magnitude
    double y = point(1) - segment_point2(1);
    double x = point(0) - segment_point2(0);
    min_distance = std::hypot(x, y);
  }
  else if (dot_product2 < 0)
  {
    double y = point(1) - segment_point1(1);
    double x = point(0) - segment_point1(0);
    min_distance = std::hypot(x, y);
  }
  else
  {
    // Finding the perpendicular distance
    double x1 = segment(0);
    double y1 = segment(1);
    double x2 = point_to_point1(0);
    double y2 = point_to_point1(1);
    double mod = std::hypot(x1, y1);
    min_distance = abs(x1 * y2 - y1 * x2) / mod;
  }
  return min_distance;
}

rmf_traffic::agv::Graph ProbabilisticRoadMap::InternalState::create_graph()
{
  graph.add_waypoint(map, start_position.head<2>());

  auto seed_vertices = get_seed_vertices();

  while (!seed_vertices.empty())
  {
    auto position = seed_vertices.front();
    seed_vertices.pop();
    if (has_conflict(position))
    {
      continue;
    }
    graph.add_waypoint(map, position);
  }

  graph.add_waypoint(map, goal_position.head<2>());

  connect_waypoints();

  return graph;
}

Eigen::Vector2d ProbabilisticRoadMap::InternalState::generate_random_waypoint()
const
{
  std::uniform_real_distribution<> x(state_limits(0, 0), state_limits(0, 1));
  std::uniform_real_distribution<> y(state_limits(1, 0), state_limits(1, 1));

  Eigen::Vector2d position(x(gen), y(gen));

  return transform_point(position);
}

std::queue<Eigen::Vector2d>
ProbabilisticRoadMap::InternalState::get_seed_vertices() const
{
  std::vector<Eigen::Vector3d> input_positions;
  input_positions.push_back(start_position);
  input_positions.push_back(goal_position);

  auto trajectory = rmf_traffic::agv::Interpolate::positions(
    traits, start_time, input_positions);

  rmf_traffic::Profile profile{rmf_traffic::geometry::make_final_convex(
      rmf_traffic::geometry::Circle(0.5))};

  std::queue<Eigen::Vector2d> queue;

  for (const auto& participant_id :
    probabilistic_road_map->itinerary_viewer->participant_ids())
  {
    if (probabilistic_road_map->excluded_participants.has_value())
    {
      if (probabilistic_road_map->excluded_participants.value().find(
          participant_id) !=
        probabilistic_road_map->excluded_participants.value().end())
      {
        continue;
      }
    }
    const auto& itinerary =
      probabilistic_road_map->itinerary_viewer->get_itinerary(participant_id);
    const auto& participant =
      probabilistic_road_map->itinerary_viewer->get_participant(participant_id);
    if (!participant)
    {
      continue;
    }

    const auto& participant_profile = participant->profile();
    if (itinerary.has_value())
    {
      const auto& routes = itinerary.value();
      for (const auto& route : routes)
      {
        auto time = rmf_traffic::DetectConflict::between(
          participant_profile, route->trajectory(), profile, trajectory);
        if (time.has_value())
        {
          const auto& motion = rmf_traffic::Motion::compute_cubic_splines(
            trajectory);
          auto position = motion->compute_position(time.value());

          Eigen::Vector2d seed_point;
          seed_point <<
          (position.head<2>() - start_position.head<2>()).norm() + 0.7, -0.5;
          seed_point = transform_point(seed_point);
          queue.push(seed_point);

          seed_point <<
          (position.head<2>() - start_position.head<2>()).norm() + 0.7, 0.5;
          seed_point = transform_point(seed_point);
          queue.push(seed_point);
        }
      }
    }
  }
  return queue;
}

Eigen::Vector2d ProbabilisticRoadMap::InternalState::transform_point(
  const Eigen::Vector2d& point) const
{
  double x1 = start_position.x();
  double y1 = start_position.y();
  double x2 = goal_position.x();
  double y2 = goal_position.y();

  Eigen::Isometry2d transformed_point = Eigen::Isometry2d::Identity();
  transformed_point.translate(Eigen::Vector2d(x1, y1));
  transformed_point.rotate(Eigen::Rotation2Dd(atan2(y2 - y1, x2 - x1)));
  transformed_point.translate(point);

  return transformed_point.translation().transpose();
}

void ProbabilisticRoadMap::InternalState::connect_waypoint(
  const rmf_traffic::agv::Graph::Waypoint& waypoint)
{
  std::vector<std::pair<double, std::size_t>> distance_list;
  const auto& position1 = waypoint.get_location();
  for (std::size_t j = 0; j < graph.num_waypoints(); ++j)
  {
    if (waypoint.index() == j)
    {
      continue;
    }
    const auto& position2 = graph.get_waypoint(j).get_location();
    auto distance = (position1 - position2).norm();
    if (distance > probabilistic_road_map->dist_close_vertices)
    {
      continue;
    }
    if (graph.lane_from(waypoint.index(), j) != nullptr)
    {
      continue;
    }
    if (!has_conflict(position1, position2))
    {
      distance_list.emplace_back(distance, j);
    }
  }
  std::sort(distance_list.begin(), distance_list.end());
  std::size_t num_lanes =
    std::min(probabilistic_road_map->num_close_vertices, distance_list.size());
  for (std::size_t j = 0; j < num_lanes; ++j)
  {
    graph.add_lane(waypoint.index(), distance_list.at(j).second);
    graph.add_lane(distance_list.at(j).second, waypoint.index());
  }
}

void ProbabilisticRoadMap::InternalState::connect_waypoints()
{
  for (std::size_t i = 0; i < graph.num_waypoints(); ++i)
  {
    connect_waypoint(graph.get_waypoint(i));
  }
}

}
}
