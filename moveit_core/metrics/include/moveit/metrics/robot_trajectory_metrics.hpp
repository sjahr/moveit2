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

/* Author: Sebastian Jahr, Ryan Luna
   Description: Metrics to gather data about a RobotTrajectory
 */

#pragma once

#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/robot_state/robot_state.h>
#include <optional>

namespace moveit
{
namespace core
{
/// \brief Calculate the path length of a given trajectory based on the accumulated robot state distances. The distance
/// between two robot states is calculated based on the sum of active joint distances between the two states (L1 norm).
/// This function is adapted from
/// https://github.com/ros-planning/moveit2/blob/main/moveit_ros/benchmarks/src/BenchmarkExecutor.cpp#L872 \param[in]
/// trajectory Given robot trajectory \return Length of the robot trajectory [rad]
[[nodiscard]] double calculateTrajectoryLength(robot_trajectory::RobotTrajectory const& trajectory);

/// \brief Calculate the smoothness of a given trajectory
/// This functionality is adapted from
/// https://github.com/ros-planning/moveit2/blob/33b075d15f8d9ec64ca8fea98cf1573b7c581c47/moveit_ros/benchmarks/src/BenchmarkExecutor.cpp#L916
/// \param[in] trajectory Given robot trajectory
/// \return Smoothness of the given trajectory or nullopt if it is not possible to calculate the smoothness
[[nodiscard]] std::optional<double> calculateTrajectorySmoothness(robot_trajectory::RobotTrajectory const& trajectory);

/// \brief Calculate the waypoint density of a trajectory
/// \param[in] trajectory Given robot trajectory
/// \return Waypoint density of the given trajectory or nullopt if it is not possible to calculate the density
[[nodiscard]] std::optional<double>
calculateTrajectoryWaypointDensity(robot_trajectory::RobotTrajectory const& trajectory);

/// Correctness and clearance
/// Similarity
}  // namespace core
}  // namespace moveit
