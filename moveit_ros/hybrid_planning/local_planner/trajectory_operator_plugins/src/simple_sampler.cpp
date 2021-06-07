/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of PickNik Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sebastian Jahr
 */

#include <moveit/trajectory_operator_plugins/simple_sampler.h>

#include <moveit/kinematic_constraints/utils.h>

namespace moveit_hybrid_planning
{
const rclcpp::Logger LOGGER = rclcpp::get_logger("local_planner_component");

bool SimpleSampler::initialize(const rclcpp::Node::SharedPtr& node, const moveit::core::RobotModelConstPtr& robot_model,
                               const std::string& group_name)
{
  // Load parameter & initialize member variables
  if (node->has_parameter("pass_through"))
    node->get_parameter<bool>("pass_through", pass_through_);
  else
    pass_through_ = node->declare_parameter<bool>("pass_through", false);

  reference_trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(robot_model, group_name);
  next_waypoint_index_ = 0;
  return true;
}

bool SimpleSampler::addTrajectorySegment(const robot_trajectory::RobotTrajectory& new_trajectory)
{
  // Reset trajectory operator to delete old reference trajectory
  reset();

  // Throw away old reference trajectory and use trajectory update
  reference_trajectory_ = std::make_shared<robot_trajectory::RobotTrajectory>(new_trajectory);

  // Parametrize trajectory and calculate velocity and accelerations
  time_parametrization_.computeTimeStamps(*reference_trajectory_);
  return true;
}

bool SimpleSampler::reset()
{
  // Reset index
  next_waypoint_index_ = 0;
  reference_trajectory_->clear();
  return true;
}
robot_trajectory::RobotTrajectory SimpleSampler::getLocalTrajectory(const moveit::core::RobotState& current_state)
{
  robot_trajectory::RobotTrajectory local_trajectory(reference_trajectory_->getRobotModel(),
                                                     reference_trajectory_->getGroupName());
  if (pass_through_)
  {
    // Use reference_trajectory as local trajectory
    local_trajectory.swap(*reference_trajectory_);
  }
  else
  {
    // Get next desired robot state
    moveit::core::RobotState next_desired_goal_state = reference_trajectory_->getWayPoint(next_waypoint_index_);

    // Check if state reached
    if (next_desired_goal_state.distance(current_state) <= 0.1)
    {
      // Update index (and thus desired robot state)
      next_waypoint_index_ += 1;
    }

    // Construct local trajectory containing the next three global trajectory waypoints
    local_trajectory.addSuffixWayPoint(reference_trajectory_->getWayPoint(next_waypoint_index_),
                                       reference_trajectory_->getWayPointDurationFromPrevious(next_waypoint_index_));
  }
  return local_trajectory;
}

double SimpleSampler::getTrajectoryProgress(const moveit::core::RobotState& current_state)
{
  // Check if trajectory is unwinded
  if (next_waypoint_index_ >= reference_trajectory_->getWayPointCount() - 1)
  {
    return 1.0;
  }
  return 0.0;
}
}  // namespace moveit_hybrid_planning

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(moveit_hybrid_planning::SimpleSampler, moveit_hybrid_planning::TrajectoryOperatorInterface);
