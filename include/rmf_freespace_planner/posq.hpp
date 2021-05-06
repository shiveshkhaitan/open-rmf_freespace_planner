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

#ifndef RMF_FREESPACE_PLANNER__POSQ_HPP
#define RMF_FREESPACE_PLANNER__POSQ_HPP

#include "rmf_freespace_planner/kinodynamic_rrt_star.hpp"

namespace rmf_freespace_planner {
namespace kinodynamic_rrt_star {
class Posq : public KinodynamicRRTStar
{
public:
  Posq(
    rmf_utils::clone_ptr<rmf_traffic::agv::RouteValidator> validator,
    std::shared_ptr<rmf_traffic::schedule::ItineraryViewer> itinerary_viewer,
    std::optional<std::unordered_set<rmf_traffic::schedule::ParticipantId>> excluded_participants,
    double sample_time);

private:
  std::optional<ComputedTrajectory> compute_trajectory(
    const std::shared_ptr<Vertex>& start,
    const std::shared_ptr<Vertex>& end) override;

  struct PosqState
  {
    double theta;

    bool eot;

    struct
    {
      double forward;
      double rotational;
    } DiffDriveVelocity;
  };

  PosqState step(
    double t,
    const Eigen::Vector3d& start,
    const Eigen::Vector3d& end,
    const PosqState& posq_state,
    bool forward = true) const;

  static double norm_angle(double theta, double start = 0.0);

  double k_v;
  double k_rho;
  double k_alpha;
  double k_theta;
  double rho_end;
  double base;
  double vmax;
};
}
}

#endif //RMF_FREESPACE_PLANNER__POSQ_HPP
