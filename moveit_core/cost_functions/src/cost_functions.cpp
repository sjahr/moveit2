/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2023, PickNik Inc.
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

/* Author: Sebastian Jahr */

#include <moveit/cost_functions/cost_functions.hpp>

namespace moveit
{
namespace cost_functions
{
[[nodiscard]] ::planning_interface::StateCostFn
getClearanceCostFn(moveit::core::RobotState& robot_state, const std::string& group_name,
                   const planning_scene::PlanningSceneConstPtr& planning_scene)
{
  // Create cost function
  return [robot_state, group_name, planning_scene](const Eigen::VectorXd& state_vector) mutable {
    robot_state.setJointGroupActivePositions(group_name, state_vector);
    auto const shortest_distance_to_collision = planning_scene->distanceToCollisionUnpadded(robot_state);

    // Return cost based on shortest_distance if the robot is not in contact or penetrated a collision object
    if (shortest_distance_to_collision > 0.0)
    {
      // The closer the collision object the higher the cost
      return 1.0 / shortest_distance_to_collision;
    }
    return std::numeric_limits<double>::infinity();  // Return a max cost cost by default
  };
}

/*
[[nodiscard]] ::planning_interface::StateCostFn
getWeightedCostFnSum(std::vector<std::pair<double, ::planning_interface::StateCostFn>> weight_cost_vector)
{
  return [weight_cost_vector](const moveit::core::RobotState& robot_state,
                              const planning_interface::MotionPlanRequest& request,
                              const planning_scene::PlanningSceneConstPtr& planning_scene) {
    auto weighted_sum = 0.0;
    for (const auto& weight_cost_pair : weight_cost_vector)
    {
      weighted_sum += weight_cost_pair.first * weight_cost_pair.second(robot_state, request, planning_scene);
    }
    return weighted_sum;
  };
}*/

}  // namespace cost_functions
}  // namespace moveit
