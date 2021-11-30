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
   Description: The hybrid planning manager component node that serves as the control unit of the whole architecture.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit_msgs/action/local_planner.hpp>
#include <moveit_msgs/action/global_planner.hpp>
#include <moveit_msgs/action/hybrid_planner.hpp>

#include <moveit/hybrid_planning_manager/hybrid_planner_interface.h>
#include <moveit/hybrid_planning_manager/planner_logic_interface.h>

#include <pluginlib/class_loader.hpp>

namespace moveit::hybrid_planning
{
/**
 * Class HybridPlanningManager - ROS 2 component node that implements the hybrid planning manager.
 */
class HybridPlanningManager
{
public:
  /** \brief Constructor */
  HybridPlanningManager(const rclcpp::NodeOptions& options);

  // This function is required to make this class a valid NodeClass
  // see https://docs.ros2.org/foxy/api/rclcpp_components/register__node__macro_8hpp.html
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface()
  {
    return node_->get_node_base_interface();
  }

private:
  std::shared_ptr<rclcpp::Node> node_;

  // Planner logic plugin loader
  std::unique_ptr<pluginlib::ClassLoader<PlannerLogicInterface>> planner_logic_plugin_loader_ = nullptr;

  // Planner logic instance to implement reactive behavior
  std::shared_ptr<PlannerLogicInterface> planner_logic_instance_ = nullptr;

  // Interface to interact with the other components of the Hybrid Planning Architecture
  std::shared_ptr<HybridPlannerInterface> hybrid_planner_interface_ = nullptr;
};
}  // namespace moveit::hybrid_planning
