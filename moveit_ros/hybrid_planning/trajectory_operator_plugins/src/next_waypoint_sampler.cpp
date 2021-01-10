/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020, PickNik LLC
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
 *   * Neither the name of PickNik LLC nor the names of its
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

#include <moveit/trajectory_operator_plugins/next_waypoint_sampler.h>

#include <moveit/kinematic_constraints/utils.h>

namespace moveit_hybrid_planning
{
bool NextWaypointSampler::initialize(const rclcpp::Node::SharedPtr& node)
{
  index_ = 0;
  return true;
}

bool NextWaypointSampler::addTrajectorySegment(const robot_trajectory::RobotTrajectory& new_trajectory,
                                               std::string group)
{
  reference_trajectory_->append(
      new_trajectory,
      0.01);  // TODO Remove magic number by interpolation between both end and start point of the trajectories
  return true;
}

moveit_msgs::msg::Constraints NextWaypointSampler::getCurrentGoal(moveit::core::RobotState current_state)
{
  moveit::core::RobotState desired_goal_state = reference_trajectory_->getWayPoint(index_);
  if (desired_goal_state.distance(current_state) <= 0.1)
  {
    index_ += 1;
    desired_goal_state = reference_trajectory_->getWayPoint(index_);
  }
  moveit_msgs::msg::Constraints local_goal_constraint = kinematic_constraints::constructGoalConstraints(
      desired_goal_state, desired_goal_state.getJointModelGroup(group_), 0.1);  // TODO Remove magic number!
  return local_goal_constraint;
}

double NextWaypointSampler::getTrajectoryProgress(moveit::core::RobotState current_state)
{
  return 0.0;  // TODO Implement this!
}
}  // namespace moveit_hybrid_planning

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(moveit_hybrid_planning::NextWaypointSampler, moveit_hybrid_planning::TrajectoryOperatorInterface);