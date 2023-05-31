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

/* Author: Sebastian Jahr
   Description: TODO
*/

#pragma once

#include <moveit/cost_functions/cost_functions.hpp>
#include <moveit/planning_scene/planning_scene.h>
#include <ompl/base/OptimizationObjective.h>

namespace ompl_interface
{
/** \brief Flexible objective that can be configured using the MoveIt planning interface cost API. */
class PlanningInterfaceObjective : public ompl::base::OptimizationObjective
{
public:
  PlanningInterfaceObjective(const ompl::base::SpaceInformationPtr& si,
                             const ::planning_interface::StateCostFn& state_cost_function);

  /** \brief Returns for a state calculated with the state_cost_function */
  ompl::base::Cost stateCost(const ompl::base::State* state) const override;

  /** \brief Adapted from ompl::base::StateCostIntegralObjective: Compute the cost of a path segment from \e state_1 to
     \e state_2 (including endpoints) \param state_1 start state of the motion to be evaluated \param state_2 final
     state of the motion to be evaluated \param cost the cost of the motion segment

      By default, this function computes
      \f{eqnarray*}{
      \mbox{cost} &=& \frac{cost(s_1) + cost(s_2)}{2}\vert s_1 - s_2 \vert
      \f}
  */
  ompl::base::Cost motionCost(const ompl::base::State* state_1, const ompl::base::State* state_2) const override;

  /** \brief Adapted from ompl::base::StateCostIntegralObjective: Estimate the cost of a path segment from \e state_1 to
     \e state_2 (including endpoints). \param state_1 start state of the motion to be evaluated \param state_2 final
     state of the motion to be evaluated \param cost the cost of the motion segment

      This function computes
      \f{eqnarray*}{
      \mbox{cost} &=& \frac{cost(s_1) + cost(s_2)}{2}\vert s_1 - s_2 \vert
      \f}
      regardless of whether enableMotionCostInterpolation was specified as true in
      constructing this object.
  */
  ompl::base::Cost motionCostBestEstimate(const ompl::base::State* state_1, const ompl::base::State* state_2) const;

protected:
  const ::planning_interface::StateCostFn state_cost_function_;

  /** \brief Adapted from ompl::base::StateCostIntegralObjective: Helper method which uses the trapezoidal rule
      to approximate the integral of the cost between two
      states of distance \e dist and costs \e c1 and \e
      c2 */
  ompl::base::Cost trapezoid(ompl::base::Cost cost_1, ompl::base::Cost cost_2, double distance) const
  {
    return ompl::base::Cost(0.5 * distance * (cost_1.value() + cost_2.value()));
  }
};
}  // namespace ompl_interface
