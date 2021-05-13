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

#ifndef RMF_FREESPACE_PLANNER__RMF_PROBABILISTIC_ROAD_MAP_HPP
#define RMF_FREESPACE_PLANNER__RMF_PROBABILISTIC_ROAD_MAP_HPP

#include <rmf_freespace_planner/rmf_freespace_planner.hpp>

#include <rmf_traffic/agv/Graph.hpp>
#include <rmf_traffic/schedule/Viewer.hpp>

#include <queue>
#include <random>

namespace rmf_freespace_planner {
namespace rmf_probabilistic_road_map {
class ProbabilisticRoadMap : public FreespacePlanner
{
public:
  ProbabilisticRoadMap(
    rmf_utils::clone_ptr<rmf_traffic::agv::RouteValidator> validator,
    int num_close_vertices,
    double dist_close_vertices,
    std::shared_ptr<rmf_traffic::schedule::ItineraryViewer> itinerary_viewer,
    std::optional<std::unordered_set<rmf_traffic::schedule::ParticipantId>> excluded_participants);

  std::vector<rmf_traffic::Route> plan(
    const Start& start,
    const Goal& goal,
    const rmf_traffic::agv::VehicleTraits& traits,
    const std::optional<std::vector<Obstacle>>& obstacles,
    const std::string& map) override;

private:
  class InternalState
  {
  public:
    InternalState(
      std::string map,
      Eigen::Vector3d start_position,
      Eigen::Vector3d goal_position,
      rmf_traffic::Time start_time,
      rmf_traffic::agv::VehicleTraits traits,
      std::optional<std::vector<Obstacle>> obstacles,
      Eigen::Matrix<double, 2, 2> state_limits,
      ProbabilisticRoadMap* probabilistic_road_map
    );

    rmf_traffic::agv::Graph create_graph();

    rmf_traffic::agv::Graph expand_graph(
      std::size_t num_additional_vertices = 1);

  private:
    bool has_conflict(const Eigen::Vector2d& position) const;

    bool has_conflict(
      const Eigen::Vector2d& position1,
      const Eigen::Vector2d& position2) const;

    Eigen::Vector2d generate_random_waypoint() const;

    std::queue<Eigen::Vector2d> get_seed_vertices() const;

    Eigen::Vector2d transform_point(const Eigen::Vector2d& point) const;

    void connect_waypoint(const rmf_traffic::agv::Graph::Waypoint& waypoint);

    void connect_waypoints();

    const std::string map;

    const Eigen::Vector3d start_position;

    const Eigen::Vector3d goal_position;

    const rmf_traffic::Time start_time;

    const rmf_traffic::agv::VehicleTraits traits;

    const std::optional<std::vector<Obstacle>> obstacles;

    const Eigen::Matrix<double, 2, 2> state_limits;

    const ProbabilisticRoadMap* probabilistic_road_map;

    std::random_device rd;

    mutable std::mt19937 gen;

    rmf_traffic::agv::Graph graph;
  };

  static double minimum_distance(
    Eigen::Vector2d segment_point1,
    Eigen::Vector2d segment_point2,
    Eigen::Vector2d point);

  std::size_t num_close_vertices;

  double dist_close_vertices;

  std::shared_ptr<rmf_traffic::schedule::ItineraryViewer> itinerary_viewer;

  std::optional<std::unordered_set<rmf_traffic::schedule::ParticipantId>>
  excluded_participants;
};
}
}

#endif //RMF_FREESPACE_PLANNER__RMF_PROBABILISTIC_ROAD_MAP_HPP
