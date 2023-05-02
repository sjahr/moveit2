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

#include <moveit/ompl_interface/detail/planning_interface_objective.hpp>
#include <moveit/ompl_interface/parameterization/model_based_state_space.h>

namespace ompl_interface
{
PlanningInterfaceObjective::PlanningInterfaceObjective(const ompl::base::SpaceInformationPtr& si,
                                                       const planning_scene::PlanningSceneConstPtr& planning_scene,
                                                       const planning_interface::MotionPlanRequest& request,
                                                       const planning_interface::StateCostFn& state_cost_function)
  : OptimizationObjective(si)
  , planning_scene_(planning_scene)
  , request_(request)
  , state_cost_function_(state_cost_function)
{
  description_ = "Planning Interface Objective";
}

ompl::base::Cost PlanningInterfaceObjective::stateCost(const ompl::base::State* state_ptr) const
{
  auto robot_state = moveit::core::RobotState(planning_scene_->getRobotModel());
  robot_state.setJointGroupPositions(request_.group_name,
                                     state_ptr->as<ompl_interface::ModelBasedStateSpace::StateType>()->values);
  return ompl::base::Cost(state_cost_function_(robot_state, request_, planning_scene_));
}

ompl::base::Cost PlanningInterfaceObjective::motionCost(const ompl::base::State* state_1,
                                                        const ompl::base::State* state_2) const
{
  // Adapted from ompl::base::StateCostIntegralObjective: Uses the trapezoidal rule to approximate the integral of the
  // cost between two states of distance and cost of state 1 and cost of state 2*/
  return trapezoid(stateCost(state_1), stateCost(state_2), si_->distance(state_1, state_2));
}

ompl::base::Cost PlanningInterfaceObjective::motionCostBestEstimate(const ompl::base::State* state_1,
                                                                    const ompl::base::State* state_2) const
{
  // Adapted from ompl::base::StateCostIntegralObjective: Uses the trapezoidal rule to approximate the integral of the
  // cost between two states of distance and cost of state 1 and cost of state 2*/
  return trapezoid(stateCost(state_1), stateCost(state_2), si_->distance(state_1, state_2));
}
}  // namespace ompl_interface
