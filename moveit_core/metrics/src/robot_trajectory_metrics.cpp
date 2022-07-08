/*******************************************************************************
 * BSD 3-Clause License
 *
 * Copyright (c) 2022, PickNik Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

/* Author: Sebastian Jahr, Ryan Luna */

#include <moveit/metrics/robot_trajectory_metrics.hpp>
#include <rclcpp/logging.hpp>

namespace moveit
{
namespace core
{
namespace
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("metrics.robot_trajectory_metrics");
}

double calculateTrajectoryLength(robot_trajectory::RobotTrajectory const& trajectory)
{
  auto trajectory_length = 0.0;
  for (std::size_t index = 1; index < trajectory.getWayPointCount(); ++index)
  {
    trajectory_length += trajectory.getWayPoint(index - 1).distance(trajectory.getWayPoint(index));
  }
  return trajectory_length;
}

std::optional<double> calculateTrajectorySmoothness(robot_trajectory::RobotTrajectory const& trajectory)
{
  if (trajectory.getWayPointCount() > 2)
  {
    auto smoothness = 0.0;
    double a = trajectory.getWayPoint(0).distance(trajectory.getWayPoint(1));
    for (std::size_t k = 2; k < trajectory.getWayPointCount(); ++k)
    {
      // view the path as a sequence of segments, and look at the triangles it forms:
      //          s1
      //          /\          s4
      //      a  /  \ b       |
      //        /    \        |
      //       /......\_______|
      //     s0    c   s2     s3

      // use Pythagoras generalized theorem to find the cos of the angle between segments a and b
      double b = trajectory.getWayPoint(k - 1).distance(trajectory.getWayPoint(k));
      double cdist = trajectory.getWayPoint(k - 2).distance(trajectory.getWayPoint(k));
      double acos_value = (a * a + b * b - cdist * cdist) / (2.0 * a * b);
      if (acos_value > -1.0 && acos_value < 1.0)
      {
        // the smoothness is actually the outside angle of the one we compute
        double angle = (M_PI - acos(acos_value));

        // and we normalize by the length of the segments
        double u = 2.0 * angle;  /// (a + b);
        smoothness += u * u;
      }
      a = b;
    }
    smoothness /= (double)trajectory.getWayPointCount();
    return smoothness;
  }
  // In case the path is to short, no value is returned
  RCLCPP_ERROR(LOGGER, "%zu waypoints are too few to calculate the trajectory smoothness",
               trajectory.getWayPointCount());
  return std::nullopt;
}

std::optional<double> calculateTrajectoryWaypointDensity(robot_trajectory::RobotTrajectory const& trajectory)
{
  if (trajectory.getWayPointCount() == 0)
  {
    RCLCPP_ERROR(LOGGER, "Cannot calculate trajectory density because the trajectory does not contain any points");
    return std::nullopt;
  }

  // Calculate path length
  auto const length = calculateTrajectoryLength(trajectory);
  if (length > 0.0)
  {
    auto density = length / (double)trajectory.getWayPointCount();
    return density;
  }
  // In case the path length is zero, no value is returned
  RCLCPP_ERROR(LOGGER, "Cannot calculate trajectory density, path length is 0.0");
  return std::nullopt;
}

}  // namespace core
}  // namespace moveit
