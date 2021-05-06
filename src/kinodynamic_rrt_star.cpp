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

#include "rmf_freespace_planner/kinodynamic_rrt_star.hpp"

#include <rmf_traffic/DetectConflict.hpp>
#include <rmf_traffic/geometry/Circle.hpp>
#include <rmf_traffic/Motion.hpp>
#include <rmf_traffic/schedule/Viewer.hpp>

#include <stack>
#include <utility>

namespace rmf_freespace_planner {
namespace kinodynamic_rrt_star {

KinodynamicRRTStar::State::State(
  Eigen::Vector3d position,
  Eigen::Vector3d velocity,
  Eigen::Vector2d distance)
: position(std::move(position)),
  velocity(std::move(velocity)),
  distance(std::move(distance))
{
}

KinodynamicRRTStar::State::State(
  const rmf_traffic::Trajectory::Waypoint& waypoint,
  Eigen::Vector2d distance)
: State(waypoint.position(), waypoint.velocity(), std::move(distance))
{
}

KinodynamicRRTStar::Vertex::Vertex(
  State state,
  std::shared_ptr<Vertex> parent,
  double cost_to_root,
  double cost_to_parent)
: state(std::move(state)),
  parent(std::move(parent)),
  cost_to_root(cost_to_root),
  cost_to_parent(cost_to_parent)
{
}

KinodynamicRRTStar::KinodynamicRRTStar(
  rmf_utils::clone_ptr<rmf_traffic::agv::RouteValidator> validator,
  std::shared_ptr<rmf_traffic::schedule::ItineraryViewer> itinerary_viewer,
  std::optional<std::unordered_set<rmf_traffic::schedule::ParticipantId>> excluded_participants,
  double sample_time)
: FreespacePlanner(std::move(validator)),
  itinerary_viewer(std::move(itinerary_viewer)),
  excluded_participants(std::move(excluded_participants)),
  sample_time(sample_time),
  goal_vertex(nullptr),
  gen(rd()),
  estimated_total_cost(0.0),
  velocity(1.0),
  min_goal_distance(std::numeric_limits<double>::max())
{
}

rmf_traffic::Trajectory KinodynamicRRTStar::plan(
  const rmf_traffic::Trajectory::Waypoint& start,
  const rmf_traffic::Trajectory::Waypoint& goal)
{
  vertex_list.clear();

  const auto start_vertex = std::make_shared<Vertex>(
    State{start, {0.0, 0.0}},
    nullptr,
    0.0);
  start_vertex->trajectory.insert(start);
  vertex_list.push_back(start_vertex);

  double length =
    (start.position().head<2>() - goal.position().head<2>()).norm();

  estimated_total_cost = length * velocity;
  min_goal_distance = length;

  goal_vertex = std::make_shared<Vertex>(
    State{goal, {length, 0.0}},
    nullptr,
    std::numeric_limits<double>::max());

  Eigen::Matrix<double, 6, 2> state_limits;
  state_limits << 0, length,
    -0.5, 0.5,
    goal.position().z(), goal.position().z(),
    -2.0, 2.0,
    -2.0, 2.0,
    0, 0;

  auto seed_vertices = get_seed_vertices(start, goal, state_limits);

  while (!goal_vertex->parent)
  {
    std::shared_ptr<Vertex> vertex;

    if (seed_vertices.empty())
    {
      vertex = generate_random_vertex(start, goal, state_limits);
    }
    else
    {
      vertex = seed_vertices.front();
      seed_vertices.pop_front();
    }

    const auto& [closest_vertex, close_vertices, trajectory, cost] =
      find_close_vertices(vertex);

    if (closest_vertex)
    {
      vertex->parent = closest_vertex;
      vertex->trajectory = std::move(trajectory);
      vertex->cost_to_root = cost;
      vertex->cost_to_parent = cost - closest_vertex->cost_to_root;

      update_estimated_total_cost(vertex);

      for (auto close_vertex : close_vertices)
      {
        rewire(vertex, close_vertex);
      }
      rewire(vertex, goal_vertex);
      vertex_list.push_back(vertex);
    }
  }

  return construct_trajectory(start_vertex, start);
}

std::shared_ptr<KinodynamicRRTStar::Vertex> KinodynamicRRTStar::
generate_random_vertex(
  const rmf_traffic::Trajectory::Waypoint& start,
  const rmf_traffic::Trajectory::Waypoint& goal,
  const Eigen::Matrix<double, 6, 2>& state_limits)
{
  std::uniform_real_distribution<> x(state_limits(0, 0), state_limits(0, 1));
  std::uniform_real_distribution<> y(state_limits(1, 0), state_limits(1, 1));
  std::uniform_real_distribution<> yaw(state_limits(2, 0), state_limits(2, 1));
  std::uniform_real_distribution<> vx(state_limits(3, 0), state_limits(3, 1));
  std::uniform_real_distribution<> vy(state_limits(4, 0), state_limits(4, 1));
  std::uniform_real_distribution<> vyaw(state_limits(5, 0), state_limits(5, 1));

  Eigen::Vector3d position(x(gen), y(gen), state_limits(2, 0));
  Eigen::Vector3d velocity(vx(gen), vy(gen), vyaw(gen));

  position.head<2>() = transform_point(start, goal, position.head<2>());

  return std::make_shared<Vertex>(
    State{position, velocity, {position.x(), position.y()}}, nullptr, 0.0);
}

bool KinodynamicRRTStar::compare_vertices(
  const std::shared_ptr<const Vertex>& vertex1,
  const std::shared_ptr<const Vertex>& vertex2)
{
  return vertex1->state.distance.x() < vertex2->state.distance.x();
}

std::deque<std::shared_ptr<KinodynamicRRTStar::Vertex>> KinodynamicRRTStar::
get_seed_vertices(
  const rmf_traffic::Trajectory::Waypoint& start,
  const rmf_traffic::Trajectory::Waypoint& goal,
  const Eigen::Matrix<double, 6, 2>& state_limits)
{
  std::deque<std::shared_ptr<Vertex>> seed_vertices;

  rmf_traffic::Trajectory trajectory;
  trajectory.insert(start);
  trajectory.insert(goal);

  rmf_traffic::Profile profile{rmf_traffic::geometry::make_final_convex(
      rmf_traffic::geometry::Circle(0.5))};

  for (const auto& participant_id : itinerary_viewer->participant_ids())
  {
    if (excluded_participants.has_value())
    {
      if (excluded_participants->find(participant_id) !=
        excluded_participants->end())
      {
        continue;
      }
    }
    const auto& itinerary = itinerary_viewer->get_itinerary(participant_id);
    const auto& participant = itinerary_viewer->get_participant(participant_id);
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

          Eigen::Vector3d seed_point;

          seed_point << (position - start.position()).norm(), -0.5,
            state_limits(2, 0);
          seed_point.head<2>() =
            transform_point(start, goal, seed_point.head<2>());

          seed_vertices.push_back(std::make_shared<Vertex>(
              State{seed_point, Eigen::Vector3d::Zero(),
                {(position - start.position()).norm(), -0.5}},
              nullptr, 0.0));

          seed_point << (position - start.position()).norm(), 0.5, state_limits(
            2, 0);
          seed_point.head<2>() =
            transform_point(start, goal, seed_point.head<2>());
          seed_vertices.push_back(std::make_shared<Vertex>(
              State{seed_point, Eigen::Vector3d::Zero(),
                {(position - start.position()).norm(), 0.5}},
              nullptr, 0.0));
        }
      }
    }
  }
  std::sort(seed_vertices.begin(), seed_vertices.end(), compare_vertices);
  seed_vertices.push_front(goal_vertex);
  return seed_vertices;
}

Eigen::Vector2d KinodynamicRRTStar::transform_point(
  const rmf_traffic::Trajectory::Waypoint& start,
  const rmf_traffic::Trajectory::Waypoint& goal,
  const Eigen::Vector2d& point)
{
  double x1 = start.position().x();
  double y1 = start.position().y();
  double x2 = goal.position().x();
  double y2 = goal.position().y();

  Eigen::Isometry2d transformed_point = Eigen::Isometry2d::Identity();
  transformed_point.translate(Eigen::Vector2d(x1, y1));
  transformed_point.rotate(Eigen::Rotation2Dd(atan2(y2 - y1, x2 - x1)));
  transformed_point.translate(point);

  return transformed_point.translation().transpose();
}

KinodynamicRRTStar::Neighborhood KinodynamicRRTStar::find_close_vertices(
  const std::shared_ptr<Vertex>& new_vertex)
{
  double cost = std::numeric_limits<double>::max();
  std::shared_ptr<Vertex> closest_vertex;
  rmf_traffic::Trajectory expansion_trajectory;

  Neighborhood neighborhood;

  for (const auto& vertex : vertex_list)
  {
    if (new_vertex->state.distance.x() <= vertex->state.distance.x())
    {
      continue;
    }

    auto test_trajectory = compute_trajectory(vertex, new_vertex);

    if (!test_trajectory.has_value())
    {
      continue;
    }

    if (!has_conflict(test_trajectory->trajectory))
    {
      neighborhood.close_vertices.push_back(vertex);
      if (cost > test_trajectory->cost + vertex->cost_to_root)
      {
        cost = test_trajectory->cost + vertex->cost_to_root;
        closest_vertex = vertex;
        expansion_trajectory = std::move(test_trajectory->trajectory);
      }
    }
  }
  if (closest_vertex)
  {
    neighborhood.trajectory = std::move(expansion_trajectory);
    neighborhood.cost = cost;
  }
  neighborhood.closest_vertex = std::move(closest_vertex);
  return neighborhood;
}

void KinodynamicRRTStar::rewire(
  const std::shared_ptr<Vertex>& random_vertex,
  const std::shared_ptr<Vertex>& vertex_to_rewire)
{
  auto test_trajectory = compute_trajectory(random_vertex, vertex_to_rewire);
  if (!test_trajectory.has_value())
  {
    return;
  }
  if (vertex_to_rewire->cost_to_root >
    random_vertex->cost_to_root + test_trajectory->cost &&
    !has_conflict(test_trajectory->trajectory))
  {
    vertex_to_rewire->cost_to_root =
      random_vertex->cost_to_root + test_trajectory->cost;
    vertex_to_rewire->parent = random_vertex;
    vertex_to_rewire->trajectory = std::move(test_trajectory->trajectory);
    vertex_to_rewire->cost_to_parent = test_trajectory->cost;

    update_estimated_total_cost(vertex_to_rewire);
    propagate_cost(vertex_to_rewire);
  }
}

void KinodynamicRRTStar::propagate_cost(
  const std::shared_ptr<Vertex>& initial_parent)
{
  std::stack<std::shared_ptr<Vertex>> stack;
  stack.push(initial_parent);
  while (!stack.empty())
  {
    const auto& parent_vertex = stack.top();
    stack.pop();
    for (const auto& vertex : vertex_list)
    {
      if (vertex->parent == parent_vertex)
      {
        vertex->cost_to_root = parent_vertex->cost_to_root +
          vertex->cost_to_parent;
        update_estimated_total_cost(vertex);
        stack.push(vertex);
      }
    }
  }
}

rmf_traffic::Trajectory KinodynamicRRTStar::construct_trajectory(
  const std::shared_ptr<Vertex>& start_vertex,
  const rmf_traffic::Trajectory::Waypoint& start)
{
  rmf_traffic::Trajectory trajectory;
  trajectory.insert(start);

  std::shared_ptr<Vertex> vertex = goal_vertex;
  std::stack<std::shared_ptr<Vertex>> path;
  while (vertex != start_vertex)
  {
    path.push(vertex);
    vertex = vertex->parent;
  }
  while (!path.empty())
  {
    vertex = path.top();
    for (const auto& waypoint : vertex->trajectory)
    {
      trajectory.insert(waypoint);
    }
    path.pop();
  }

  return trajectory;
}

void KinodynamicRRTStar::update_estimated_total_cost(
  const std::shared_ptr<Vertex>& vertex)
{
  if (goal_vertex->parent)
  {
    estimated_total_cost = goal_vertex->cost_to_root;
  }
  double goal_distance =
    (vertex->state.position.head<2>() - goal_vertex->state.position.head<2>())
    .norm();

  if (goal_distance < min_goal_distance)
  {
    min_goal_distance = goal_distance;
    estimated_total_cost = vertex->cost_to_root + goal_distance * velocity;
  }
}
}
}
