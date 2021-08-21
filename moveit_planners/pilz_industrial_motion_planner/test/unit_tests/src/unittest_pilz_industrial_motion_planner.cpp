/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018 Pilz GmbH & Co. KG
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
 *   * Neither the name of Pilz GmbH & Co. KG nor the names of its
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

#include <gtest/gtest.h>

#include <iostream>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include "pilz_industrial_motion_planner/pilz_industrial_motion_planner.h"
#include "test_utils.h"

#include "rclcpp/rclcpp.hpp"

const std::string PARAM_MODEL_NO_GRIPPER_NAME{ "robot_description" };
const std::string PARAM_MODEL_WITH_GRIPPER_NAME{ "robot_description_pg70" };

static const rclcpp::Logger LOGGER = rclcpp::get_logger("unittest_pilz_industrial_motion_planner");

class CommandPlannerTest : public testing::TestWithParam<std::string>
{
protected:
  void SetUp() override
  {
    createPlannerInstance();
  }

  /**
   * @brief initialize the planner plugin
   * The planner is loaded using the pluginlib. Checks that the planner was
   * loaded properly.
   * Exceptions will cause test failure.
   *
   * This function should be called only once during initialization of the
   * class.
   */
  void createPlannerInstance()
  {
    std::cout << "Tres";
    // Load planner name from parameter server
    if (!node_->has_parameter("planning_plugin"))
    {
      RCLCPP_FATAL_STREAM(LOGGER, "Could not find planner plugin name");
      FAIL();
    }
    else {
      node_->get_parameter<std::string>("planning_plugin", planner_plugin_name_);
      std::cout << "Found: " << planner_plugin_name_.c_str();
    }

    // Load the plugin
    try
    {
      planner_plugin_loader_.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
          "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
      RCLCPP_FATAL_STREAM(LOGGER, "Exception while creating planning plugin loader " << ex.what());
      FAIL();
    }
    std::cout << "Loaded planning plugin";

    // Create planner
    try
    {
      planner_instance_.reset(planner_plugin_loader_->createUnmanagedInstance(planner_plugin_name_));
      ASSERT_TRUE(planner_instance_->initialize(robot_model_, node_, "robot_description_planning")) // TODO(sjahr) Namespace correct?
          << "Initialzing the planner instance failed.";
    }
    catch (pluginlib::PluginlibException& ex)
    {
      FAIL() << "Could not create planner " << ex.what() << "\n";
    }
    std::cout << "Created planner";
  }

  void TearDown() override
  {
    planner_plugin_loader_->unloadLibraryForClass(planner_plugin_name_);
  }

protected:
  // ros stuff
  rclcpp::Node::SharedPtr node_;
  moveit::core::RobotModelConstPtr robot_model_{robot_model_loader::RobotModelLoader(node_, GetParam()).getModel() };

  std::string planner_plugin_name_;

  std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader_;

  planning_interface::PlannerManagerPtr planner_instance_;
};

// Instantiate the test cases for robot model with and without gripper
INSTANTIATE_TEST_SUITE_P(InstantiationName, CommandPlannerTest,
                         ::testing::Values(PARAM_MODEL_NO_GRIPPER_NAME, PARAM_MODEL_WITH_GRIPPER_NAME));

/**
 * @brief Test that PTP can be loaded
 * This needs to be extended with every new planning Algorithm
 */
TEST_P(CommandPlannerTest, ObtainLoadedPlanningAlgorithms)
{
  std::cout << "Quatre?";
  // Check for the algorithms
  std::vector<std::string> algs;

  std::cout << "Check for the algorithms";
  planner_instance_->getPlanningAlgorithms(algs);
  ASSERT_EQ(3u, algs.size()) << "Found more or less planning algorithms as expected! Found:"
                             << ::testing::PrintToString(algs);

  // Collect the algorithms, check for uniqueness
  std::set<std::string> algs_set;
  std::cout << "Check for the algorithms";
  for (const auto& alg : algs)
  {
    algs_set.insert(alg);
  }
  ASSERT_EQ(algs.size(), algs_set.size()) << "There are two or more algorithms with the same name!";
  ASSERT_TRUE(algs_set.find("LIN") != algs_set.end());
  ASSERT_TRUE(algs_set.find("PTP") != algs_set.end());
  ASSERT_TRUE(algs_set.find("CIRC") != algs_set.end());
}

/**
 * @brief Check that all announced planning algorithms can perform the service
 * request if the planner_id is set.
 */
TEST_P(CommandPlannerTest, CheckValidAlgorithmsForServiceRequest)
{
  // Check for the algorithms
  std::vector<std::string> algs;
  planner_instance_->getPlanningAlgorithms(algs);

  for (const auto& alg : algs)
  {
    planning_interface::MotionPlanRequest req;
    req.planner_id = alg;

    EXPECT_TRUE(planner_instance_->canServiceRequest(req));
  }
}

/**
 * @brief Check that canServiceRequest(req) returns false if planner_id is not
 * supported
 */
TEST_P(CommandPlannerTest, CheckInvalidAlgorithmsForServiceRequest)
{
  planning_interface::MotionPlanRequest req;
  req.planner_id = "NON_EXISTEND_ALGORITHM_HASH_da39a3ee5e6b4b0d3255bfef95601890afd80709";

  EXPECT_FALSE(planner_instance_->canServiceRequest(req));
}

/**
 * @brief Check that canServiceRequest(req) returns false if planner_id is empty
 */
TEST_P(CommandPlannerTest, CheckEmptyPlannerIdForServiceRequest)
{
  planning_interface::MotionPlanRequest req;
  req.planner_id = "";

  EXPECT_FALSE(planner_instance_->canServiceRequest(req));
}

/**
 * @brief Check integrety against empty input
 */
TEST_P(CommandPlannerTest, CheckPlanningContextRequestNull)
{
  moveit_msgs::msg::MotionPlanRequest req;
  moveit_msgs::msg::MoveItErrorCodes error_code;
  EXPECT_EQ(nullptr, planner_instance_->getPlanningContext(nullptr, req, error_code));
}

/**
 * @brief Check that for the announced algorithmns getPlanningContext does not
 * return nullptr
 */
TEST_P(CommandPlannerTest, CheckPlanningContextRequest)
{
  moveit_msgs::msg::MotionPlanRequest req;
  moveit_msgs::msg::MoveItErrorCodes error_code;

  // Check for the algorithms
  std::vector<std::string> algs;
  planner_instance_->getPlanningAlgorithms(algs);

  for (const auto& alg : algs)
  {
    req.planner_id = alg;

    EXPECT_NE(nullptr, planner_instance_->getPlanningContext(nullptr, req, error_code));
  }
}

/**
 * @brief Check the description can be obtained and is not empty
 */
TEST_P(CommandPlannerTest, CheckPlanningContextDescriptionNotEmptyAndStable)
{
  std::string desc = planner_instance_->getDescription();
  EXPECT_GT(desc.length(), 0u);
}

int main(int argc, char** argv)
{
  std::cout << "Hello there\n";
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  std::cout << "Uno\n";
  int result = RUN_ALL_TESTS();
    std::cout << "D\n";
  return result;
}
