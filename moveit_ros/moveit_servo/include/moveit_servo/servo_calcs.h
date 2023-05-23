/*******************************************************************************
 *      Title     : servo_calcs.h
 *      Project   : moveit_servo
 *      Created   : 1/11/2019
 *      Author    : Brian O'Neil, Andy Zelenak, Blake Anderson
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
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

#pragma once

// C++
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <thread>

// ROS
#include <control_msgs/msg/joint_jog.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/int8.hpp>
#include <std_srvs/srv/empty.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>

// moveit_core
#include <moveit/kinematics_base/kinematics_base.h>

// moveit_servo
#include <moveit_servo/status_codes.h>
#include <moveit/online_signal_smoothing/smoothing_base_class.h>
#include <moveit_servo_lib_parameters.hpp>

namespace moveit_servo
{
enum class ServoType
{
  CARTESIAN_SPACE,
  JOINT_SPACE
};

class ServoCalcs
{
public:
  ServoCalcs(const rclcpp::Node::SharedPtr& node,
             const planning_scene_monitor::PlanningSceneMonitorPtr& planning_scene_monitor,
             const std::shared_ptr<const servo::ParamListener>& servo_param_listener);

  ~ServoCalcs();

  /**
   * Start the timer where we do work and publish outputs
   *
   * @exception can throw a std::runtime_error if the setup was not completed
   */
  void start();

  /** \brief Stop the currently running thread */
  void stop();

  /**
   * Check for parameter update, and apply updates if any
   * All dynamic parameters must be checked and updated within this method
   */
  void updateParams();

  /**
   * Get the MoveIt planning link transform.
   * The transform from the MoveIt planning frame to robot_link_command_frame
   *
   * @param transform the transform that will be calculated
   * @return true if a valid transform was available
   */
  bool getCommandFrameTransform(Eigen::Isometry3d& transform);
  bool getCommandFrameTransform(geometry_msgs::msg::TransformStamped& transform);

  /**
   * Get the End Effector link transform.
   * The transform from the MoveIt planning frame to EE link
   *
   * @param transform the transform that will be calculated
   * @return true if a valid transform was available
   */
  bool getEEFrameTransform(Eigen::Isometry3d& transform);
  bool getEEFrameTransform(geometry_msgs::msg::TransformStamped& transform);

protected:
  /** \brief Run the main calculation loop */
  void mainCalcLoop();

  /** \brief Do calculations for a single iteration. Publish one outgoing command */
  void calculateSingleIteration();

  /** \brief Do servoing calculations for Cartesian twist commands. */
  bool cartesianServoCalcs(geometry_msgs::msg::TwistStamped& cmd,
                           trajectory_msgs::msg::JointTrajectory& joint_trajectory);

  /** \brief Do servoing calculations for direct commands to a joint. */
  bool jointServoCalcs(const control_msgs::msg::JointJog& cmd, trajectory_msgs::msg::JointTrajectory& joint_trajectory);

  /**
   * Checks a JointJog msg for valid (non-NaN) velocities
   * @param cmd the desired joint servo command
   * @return true if this represents a valid joint servo command, false otherwise
   */
  bool checkValidCommand(const control_msgs::msg::JointJog& cmd);

  /**
   * Checks a TwistStamped msg for valid (non-NaN) velocities
   * @param cmd the desired twist servo command
   * @return true if this represents a valid servo twist command, false otherwise
   */
  bool checkValidCommand(const geometry_msgs::msg::TwistStamped& cmd);

  /** \brief If incoming velocity commands are from a unitless joystick, scale them to physical units.
   * Also, multiply by timestep to calculate a position change.
   * @return a vector of position deltas
   */
  Eigen::VectorXd scaleCartesianCommand(const geometry_msgs::msg::TwistStamped& command);

  /** \brief If incoming velocity commands are from a unitless joystick, scale them to physical units.
   * Also, multiply by timestep to calculate a position change.
   * @return a vector of position deltas
   */
  Eigen::VectorXd scaleJointCommand(const control_msgs::msg::JointJog& command);

  /** \brief Come to a halt in a smooth way. Apply a smoothing plugin, if one is configured.
   */
  void filteredHalt(trajectory_msgs::msg::JointTrajectory& joint_trajectory);

  /** \brief Suddenly halt for a joint limit or other critical issue.
   * Is handled differently for position vs. velocity control.
   */
  void suddenHalt(sensor_msgs::msg::JointState& joint_state,
                  const std::vector<const moveit::core::JointModel*>& joints_to_halt) const;

  /** \brief Avoid overshooting joint limits
      \return Vector of the joints that would move farther past position margin limits
   */
  std::vector<const moveit::core::JointModel*> enforcePositionLimits(sensor_msgs::msg::JointState& joint_state) const;

  /** \brief Compose the outgoing JointTrajectory message */
  void composeJointTrajMessage(const sensor_msgs::msg::JointState& joint_state,
                               trajectory_msgs::msg::JointTrajectory& joint_trajectory);

  /** \brief Set the filters to the specified values */
  void resetLowPassFilters(const sensor_msgs::msg::JointState& joint_state);

  /** \brief Handles all aspects of the servoing after the desired joint commands are established
   * Joint and Cartesian calcs feed into here
   * Handles limit enforcement, internal state updated, collision scaling, and publishing the commands
   * @param delta_theta Eigen vector of joint delta's, from joint or Cartesian servo calcs
   * @param joint_trajectory Output trajectory message
   * @param servo_type The type of servoing command being used
   */
  bool internalServoUpdate(Eigen::ArrayXd& delta_theta, trajectory_msgs::msg::JointTrajectory& joint_trajectory,
                           const ServoType servo_type);

  /* \brief Command callbacks */
  void twistStampedCB(const geometry_msgs::msg::TwistStamped::ConstSharedPtr& msg);
  void jointCmdCB(const control_msgs::msg::JointJog::ConstSharedPtr& msg);
  void collisionVelocityScaleCB(const std_msgs::msg::Float64::ConstSharedPtr& msg);

  // Pointer to the ROS node
  std::shared_ptr<rclcpp::Node> node_;

  // Servo parameters
  const std::shared_ptr<const servo::ParamListener> servo_param_listener_;
  servo::Params servo_params_;

  // Pointer to the collision environment
  planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;

  // Flag for staying inactive while there are no incoming commands
  bool wait_for_servo_commands_ = true;

  // Flag saying if the filters were updated during the timer callback
  bool updated_filters_ = false;

  // Incoming command messages
  geometry_msgs::msg::TwistStamped twist_stamped_cmd_;
  control_msgs::msg::JointJog joint_servo_cmd_;

  const moveit::core::JointModelGroup* joint_model_group_;

  moveit::core::RobotStatePtr current_state_;

  // These variables are mutex protected
  // previous_joint_state holds the state q(t - dt)
  // current_joint_state holds the  state q(t) as retrieved from the planning scene monitor.
  // next_joint_state holds the computed state q(t + dt)

  sensor_msgs::msg::JointState previous_joint_state_, current_joint_state_, next_joint_state_;
  std::map<std::string, std::size_t> joint_state_name_map_;

  // Smoothing algorithm (loads a plugin)
  pluginlib::UniquePtr<online_signal_smoothing::SmoothingBaseClass> smoother_;

  trajectory_msgs::msg::JointTrajectory::SharedPtr last_sent_command_;

  // ROS
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_stamped_sub_;
  rclcpp::Subscription<control_msgs::msg::JointJog>::SharedPtr joint_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr collision_velocity_scale_sub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr status_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_outgoing_cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr multiarray_outgoing_cmd_pub_;

  // Main tracking / result publisher loop
  std::thread thread_;
  bool stop_requested_;

  // Status
  StatusCode status_ = StatusCode::NO_WARNING;
  bool twist_command_is_stale_ = false;
  bool joint_command_is_stale_ = false;
  double collision_velocity_scale_ = 1.0;

  // Use ArrayXd type to enable more coefficient-wise operations
  Eigen::ArrayXd delta_theta_;

  unsigned int num_joints_;

  // main_loop_mutex_ is used to protect the input state and dynamic parameters
  mutable std::mutex main_loop_mutex_;
  Eigen::Isometry3d tf_moveit_to_robot_cmd_frame_;
  Eigen::Isometry3d tf_moveit_to_ee_frame_;
  geometry_msgs::msg::TwistStamped::ConstSharedPtr latest_twist_stamped_;
  control_msgs::msg::JointJog::ConstSharedPtr latest_joint_cmd_;
  rclcpp::Time latest_twist_command_stamp_ = rclcpp::Time(0., RCL_ROS_TIME);
  rclcpp::Time latest_joint_command_stamp_ = rclcpp::Time(0., RCL_ROS_TIME);

  // input condition variable used for low latency mode
  std::condition_variable input_cv_;
  bool new_input_cmd_ = false;

  // Load a smoothing plugin
  pluginlib::ClassLoader<online_signal_smoothing::SmoothingBaseClass> smoothing_loader_;

  kinematics::KinematicsBaseConstPtr ik_solver_ = nullptr;
};
}  // namespace moveit_servo
