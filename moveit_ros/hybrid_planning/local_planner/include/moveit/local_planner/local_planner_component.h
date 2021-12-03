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
   Description: A local planner component node that is customizable through plugins that implement the local planning
   problem solver algorithm and the trajectory matching and blending.
 */

#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <moveit_msgs/action/local_planner.hpp>
#include <moveit_msgs/msg/motion_plan_response.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>

#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/local_planner/local_constraint_solver_interface.h>
#include <moveit/local_planner/trajectory_operator_interface.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

namespace moveit_hybrid_planning
{
// TODO(sjahr) Refactor and use repository wide solution
template <typename T>
void declareOrGetParam(const std::string& param_name, T& output_value, const T& default_value,
                       const rclcpp::Node::SharedPtr& node)
{
  try
  {
    if (node->has_parameter(param_name))
      node->get_parameter<T>(param_name, output_value);
    else
      output_value = node->declare_parameter<T>(param_name, default_value);
  }
  catch (const rclcpp::exceptions::InvalidParameterTypeException& e)
  {
    RCLCPP_ERROR_STREAM(node->get_logger(),
                        "Error getting parameter \'" << param_name << "\', check parameter type in YAML file");
    throw e;
  }
}

/// Internal local planner states
/// TODO(sjahr) Use lifecycle node?
enum class LocalPlannerState : int8_t
{
  ABORT = -1,
  ERROR = 0,
  READY = 1,
  UNCONFIGURED = 2,
  AWAIT_GLOBAL_TRAJECTORY = 3,
  LOCAL_PLANNING_ACTIVE = 4
};

/**
 * Class LocalPlannerComponent - ROS 2 component node that implements a local planner.
 */
class LocalPlannerComponent : public rclcpp::Node
{
public:
  /// Struct that contains configuration of the local planner component node
  struct LocalPlannerConfig
  {
    void load(const rclcpp::Node::SharedPtr& node)
    {
      std::string undefined = "<undefined>";
      declareOrGetParam<std::string>("trajectory_operator_plugin_name", trajectory_operator_plugin_name, undefined,
                                     node);
      declareOrGetParam<std::string>("local_constraint_solver_plugin_name", local_constraint_solver_plugin_name,
                                     undefined, node);
      declareOrGetParam<double>("local_planning_frequency", local_planning_frequency, 1.0, node);
      declareOrGetParam<std::string>("global_solution_topic", global_solution_topic, undefined, node);
      declareOrGetParam<std::string>("local_solution_topic", local_solution_topic, undefined, node);
    }

    std::string robot_description;
    std::string robot_description_semantic;
    std::string publish_planning_scene_topic;
    std::string trajectory_operator_plugin_name;
    std::string local_constraint_solver_plugin_name;
    std::string global_solution_topic;
    std::string local_solution_topic;
    double local_planning_frequency;
  };

  /** \brief Constructor */
  LocalPlannerComponent(const rclcpp::NodeOptions& options);

  /**
   * Initialize and start planning scene monitor to listen to the planning scene topic.
   * Load trajectory_operator and constraint solver plugin.
   * Initialize ROS 2 interfaces
   * @return true if scene monitor and plugins are successfully initialized
   */
  bool initialize();

  /**
   * Handle the planners current job based on the internal state each loop run when the planner is started.
   */
  void executePlanningLoopRun();

private:
  // Planner configuration
  LocalPlannerConfig config_;

  // Current planner state
  LocalPlannerState state_;

  // Timer to periodically call executePlanningLoopRun()
  rclcpp::TimerBase::SharedPtr timer_;

  // Current action goal handle
  std::shared_ptr<rclcpp_action::ServerGoalHandle<moveit_msgs::action::LocalPlanner>> local_planning_goal_handle_;

  // Planning scene monitor to get the current planning scene
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Global solution listener
  rclcpp::Subscription<moveit_msgs::msg::MotionPlanResponse>::SharedPtr global_solution_subscriber_;

  // Local planning request action server
  rclcpp_action::Server<moveit_msgs::action::LocalPlanner>::SharedPtr local_planning_request_server_;

  // Local solution publisher
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr local_solution_publisher_;

  // Local constraint solver plugin loader
  std::unique_ptr<pluginlib::ClassLoader<moveit_hybrid_planning::LocalConstraintSolverInterface>>
      local_constraint_solver_plugin_loader_;

  // Local constrain solver instance to compute a local solution each loop run
  std::shared_ptr<moveit_hybrid_planning::LocalConstraintSolverInterface> local_constraint_solver_instance_;

  // Trajectory operator plugin
  std::unique_ptr<pluginlib::ClassLoader<moveit_hybrid_planning::TrajectoryOperatorInterface>>
      trajectory_operator_loader_;

  // Trajectory_operator instance handle trajectory matching and blending
  std::shared_ptr<moveit_hybrid_planning::TrajectoryOperatorInterface> trajectory_operator_instance_;
};
}  // namespace moveit_hybrid_planning
