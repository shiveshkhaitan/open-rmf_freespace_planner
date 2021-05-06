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

#include "rmf_freespace_planner/posq.hpp"

namespace rmf_freespace_planner {
namespace kinodynamic_rrt_star {
Posq::Posq(
  rmf_utils::clone_ptr<rmf_traffic::agv::RouteValidator> validator,
  std::shared_ptr<rmf_traffic::schedule::ItineraryViewer> itinerary_viewer,
  std::optional<std::unordered_set<rmf_traffic::schedule::ParticipantId>> excluded_participants,
  double sample_time)
: KinodynamicRRTStar(std::move(validator),
    std::move(itinerary_viewer),
    std::move(excluded_participants)),
  sample_time(sample_time)
{
  k_v = 3.8;
  k_rho = 1.0;
  k_alpha = 6.0;
  k_theta = -1.0;
  rho_end = 0.00510;
  base = 0.4;
  vmax = 1.0;
}

std::optional<Posq::ComputedTrajectory> Posq::compute_trajectory(
  const std::shared_ptr<Vertex>& start,
  const std::shared_ptr<Vertex>& end) const
{
  Eigen::Vector3d x0(start->state.x(), start->state.y(), start->state.yaw());
  Eigen::Vector3d x1(end->state.x(), end->state.y(), end->state.yaw());

  double t = 0.0;

  double sl = 0.0;
  double sr = 0.0;
  double old_sl = 0.0;
  double old_sr = 0.0;

  PosqState posq_state{};
  posq_state.theta = 0;
  posq_state.eot = false;

  double cost = 0.0;

  rmf_traffic::Trajectory trajectory;
  trajectory.insert(start->trajectory.back());

  while (!posq_state.eot)
  {
    double dSl = sl - old_sl;
    double dSr = sr - old_sr;
    double dSm = (dSl + dSr) / 2.0;
    double dSd = (dSr - dSl) / base;

    x0(0) = x0(0) + dSm * cos(x0(2) + dSd / 2.0);
    x0(1) = x0(1) + dSm * sin(x0(2) + dSd / 2.0);
    x0(2) = norm_angle(x0(2) + dSd, -M_PI);

    posq_state = step(x0, x1, posq_state, true);
    double vl = posq_state.DiffDriveVelocity.forward -
      posq_state.DiffDriveVelocity.rotational * base / 2.0;
    vl = std::clamp(vl, -vmax, vmax);

    double vr = posq_state.DiffDriveVelocity.forward +
      posq_state.DiffDriveVelocity.rotational * base / 2.0;
    vr = std::clamp(vr, -vmax, vmax);

    t = t + sample_time;

    old_sl = sl;
    old_sr = sr;
    sl += vl * sample_time;
    sr += vr * sample_time;

    trajectory.insert(
      rmf_traffic::time::apply_offset(trajectory.back().time(), sample_time),
      x0,
      Eigen::Vector3d(
        posq_state.DiffDriveVelocity.forward,
        0.0,
        posq_state.DiffDriveVelocity.rotational));
    cost += sample_time;
    cost += 0.01 *
      (posq_state.DiffDriveVelocity.rotational *
      posq_state.DiffDriveVelocity.rotational);
  }

  if (trajectory.size() < 2)
  {
    return std::nullopt;
  }
  return ComputedTrajectory{trajectory, cost};
}

Posq::PosqState Posq::step(
  const Eigen::Vector3d& start,
  const Eigen::Vector3d& end,
  const PosqState& posq_state,
  bool forward) const
{
  PosqState posq_state_next;

  double dx = end(0) - start(0);
  double dy = end(1) - start(1);
  double rho = std::hypot(dx, dy);
  double f_rho = std::min(rho, vmax / k_rho);

  double alpha = norm_angle(atan2(dy, dx) - start(2), -M_PI);
  if (forward)
  {
    if (alpha > M_PI_2)
    {
      f_rho = -f_rho;
      alpha = alpha - M_PI;
    }
    else if (alpha <= -M_PI_2)
    {
      f_rho = -f_rho;
      alpha = alpha + M_PI;
    }
  }
  else
  {
    f_rho = -f_rho;
    alpha = alpha + M_PI;
    if (alpha > M_PI)
    {
      alpha = alpha - 2 * M_PI;
    }
  }

  double phi = end(2) - start(2);
  phi = norm_angle(phi, -M_PI);
  double theta = norm_angle(phi - alpha, -M_PI);
  if (abs(posq_state.theta - theta) > M_PI)
  {
    theta = posq_state.theta;
  }
  posq_state_next.theta = theta;

  double vm = k_rho * tanh(f_rho * k_v);
  double vd = (k_alpha * alpha + k_theta * theta);
  posq_state_next.eot = (rho < rho_end);
  posq_state_next.DiffDriveVelocity = {vm, vd};

  return posq_state_next;
}

double Posq::norm_angle(double theta, double start)
{
  if (theta < std::numeric_limits<double>::infinity())
  {
    while (theta >= start + 2 * M_PI)
    {
      theta -= 2 * M_PI;
    }
    while (theta < start)
    {
      theta += 2 * M_PI;
    }
    return theta;
  }
  else
  {
    return std::numeric_limits<double>::infinity();
  }
}
}
}
