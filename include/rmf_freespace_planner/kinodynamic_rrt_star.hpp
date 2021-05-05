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

#ifndef RMF_FREESPACE_PLANNER__KINODYNAMIC_RRT_STAR_HPP
#define RMF_FREESPACE_PLANNER__KINODYNAMIC_RRT_STAR_HPP

#include "rmf_freespace_planner/rmf_freespace_planner.hpp"

#include <Eigen/Core>

#include <deque>
#include <random>

namespace rmf_freespace_planner {
namespace kinodynamic_rrt_star {
class KinodynamicRRTStar : public FreespacePlanner
{
public:
  KinodynamicRRTStar(
    rmf_utils::clone_ptr<rmf_traffic::agv::RouteValidator> validator,
    std::shared_ptr<rmf_traffic::schedule::ItineraryViewer> itinerary_viewer,
    std::optional<std::unordered_set<rmf_traffic::schedule::ParticipantId>> excluded_participants,
    double sample_time);

  struct State
  {
    State(
      Eigen::Vector3d position,
      Eigen::Vector3d velocity,
      Eigen::Vector2d distance);

    State(const rmf_traffic::Trajectory::Waypoint& waypoint,
      Eigen::Vector2d distance);

    double x() const
    {
      return position.x();
    }

    double y() const
    {
      return position.y();
    }

    double yaw() const
    {
      return position.z();
    }

    double vx() const
    {
      return velocity.x();
    }

    double vy() const
    {
      return velocity.y();
    }

    double vyaw() const
    {
      return velocity.z();
    }

    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector2d distance;
  };

  struct Vertex
  {
    Vertex(
      State state,
      std::shared_ptr<Vertex> parent = nullptr,
      double cost_to_root = 0.0,
      double cost_to_parent = 0.0);

    State state;
    std::shared_ptr<Vertex> parent;
    double cost_to_root;
    double cost_to_parent;

    rmf_traffic::Trajectory trajectory;
  };

  struct ComputedTrajectory
  {
    rmf_traffic::Trajectory trajectory;
    double cost;
  };

  struct Neighborhood
  {
    std::shared_ptr<Vertex> closest_vertex;
    std::vector<std::shared_ptr<Vertex>> close_vertices;
    rmf_traffic::Trajectory trajectory;
    double cost;
  };

  std::vector<std::shared_ptr<Vertex>> get_vertices()
  {
    return vertex_list;
  }

  double get_estimated_total_cost() const
  {
    return estimated_total_cost;
  }

protected:
  double sample_time;

private:
  rmf_traffic::Trajectory plan(
    const rmf_traffic::Trajectory::Waypoint& start,
    const rmf_traffic::Trajectory::Waypoint& goal) override;

  std::shared_ptr<Vertex> generate_random_vertex(
    const rmf_traffic::Trajectory::Waypoint& start,
    const rmf_traffic::Trajectory::Waypoint& goal,
    const Eigen::Matrix<double, 6, 2>& state_limits);

  static bool compare_vertices(
    const std::shared_ptr<const Vertex>& vertex1,
    const std::shared_ptr<const Vertex>& vertex2);

  std::deque<std::shared_ptr<Vertex>> get_seed_vertices(
    const rmf_traffic::Trajectory::Waypoint& start,
    const rmf_traffic::Trajectory::Waypoint& goal,
    const Eigen::Matrix<double, 6, 2>& state_limits);

  static Eigen::Vector2d transform_point(
    const rmf_traffic::Trajectory::Waypoint& start,
    const rmf_traffic::Trajectory::Waypoint& goal,
    const Eigen::Vector2d& point);

  Neighborhood find_close_vertices(const std::shared_ptr<Vertex>& new_vertex);

  void rewire(
    const std::shared_ptr<Vertex>& new_vertex,
    const std::shared_ptr<Vertex>& vertex_to_rewire);

  void propagate_cost(const std::shared_ptr<Vertex>& parent_vertex);

  virtual std::optional<ComputedTrajectory> compute_trajectory(
    const std::shared_ptr<Vertex>& start,
    const std::shared_ptr<Vertex>& end) = 0;

  rmf_traffic::Trajectory construct_trajectory(
    const std::shared_ptr<Vertex>& start_vertex,
    const rmf_traffic::Trajectory::Waypoint& start);

  void update_estimated_total_cost(const std::shared_ptr<Vertex>& vertex);

  std::shared_ptr<Vertex> goal_vertex;

  std::vector<std::shared_ptr<Vertex>> vertex_list;

  std::random_device rd;

  std::mt19937 gen;

  double estimated_total_cost;

  double velocity;

  double min_goal_distance;

  std::shared_ptr<rmf_traffic::schedule::ItineraryViewer> itinerary_viewer;

  std::optional<std::unordered_set<rmf_traffic::schedule::ParticipantId>>
  excluded_participants;
};
}
}

#endif //RMF_FREESPACE_PLANNER__KINODYNAMIC_RRT_STAR_HPP
