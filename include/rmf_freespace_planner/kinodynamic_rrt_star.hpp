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

#ifndef RMF_FREESPACE_PLANNER_KINODYNAMIC_RRT_STAR_HPP_
#define RMF_FREESPACE_PLANNER_KINODYNAMIC_RRT_STAR_HPP_

#include "rmf_freespace_planner/rmf_freespace_planner.hpp"

#include <Eigen/Core>

#include <queue>
#include <random>

namespace rmf_freespace_planner {
namespace kinodynamic_rrt_star {
class KinodynamicRRTStar : public FreespacePlanner
{
public:
  KinodynamicRRTStar(
    rmf_utils::clone_ptr<rmf_traffic::agv::RouteValidator> validator,
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

  std::queue<std::shared_ptr<Vertex>> get_seed_vertices(
    const rmf_traffic::Trajectory::Waypoint& start,
    const rmf_traffic::Trajectory::Waypoint& goal,
    const Eigen::Matrix<double, 6, 2>& state_limits);

  static void transform_point(
    const rmf_traffic::Trajectory::Waypoint& start,
    const rmf_traffic::Trajectory::Waypoint& goal,
    double& x,
    double& y);

  std::shared_ptr<Vertex> find_close_vertices(
    const std::shared_ptr<Vertex>& new_vertex,
    std::vector<std::shared_ptr<Vertex>>& close_vertices);

  bool rewire(
    const std::shared_ptr<Vertex>& vertex,
    const std::vector<std::shared_ptr<Vertex>>& close_vertices);

  void propagate_cost(const std::shared_ptr<Vertex>& parent_vertex);

  virtual double compute_trajectory(
    const std::shared_ptr<Vertex>& start,
    const std::shared_ptr<Vertex>& end,
    const std::shared_ptr<rmf_traffic::Trajectory>& trajectory) = 0;

  void construct_trajectory(
    const std::shared_ptr<Vertex>& start_vertex,
    const std::shared_ptr<rmf_traffic::Trajectory>& trajectory);

  void update_estimated_total_cost(const std::shared_ptr<Vertex>& vertex);

  std::shared_ptr<Vertex> goal_vertex;

  std::vector<std::shared_ptr<Vertex>> vertex_list;

  std::random_device rd;

  std::mt19937 gen;

  double estimated_total_cost;

  double velocity;

  double min_goal_distance;
};
}
}

#endif //RMF_FREESPACE_PLANNER_KINODYNAMIC_RRT_STAR_HPP_
