set(UNIT_TEST_SOURCEFILES
  unittest_pilz_industrial_motion_planner_direct
  unittest_velocity_profile_atrap
  unittest_trajectory_generator
  #unittest_trajectory_functions
  #unittest_trajectory_blender_transition_window
  #unittest_trajectory_generator_common
  #unittest_trajectory_generator_circ
  #unittest_trajectory_generator_lin
  #unittest_trajectory_generator_ptp
  #unittest_pilz_industrial_motion_planner
  unittest_joint_limit
  #unittest_joint_limits_aggregator
  #unittest_joint_limits_container
  #unittest_joint_limits_validator
  unittest_cartesian_limits_aggregator
  #unittest_planning_context_loaders
  #unittest_planning_context
  unittest_get_solver_tip_frame
)

## Add pure gtest based unittests and launch them with ros_test
ament_add_gtest_executable(unittest_pilz_industrial_motion_planner_direct
  src/unittest_pilz_industrial_motion_planner.cpp
  ${CMAKE_SOURCE_DIR}/src/planning_context_loader_ptp.cpp)
target_link_libraries(unittest_pilz_industrial_motion_planner_direct ${PROJECT_NAME}_lib ${PROJECT_NAME}_testhelpers)

ament_add_gtest_executable(unittest_velocity_profile_atrap
  src/unittest_velocity_profile_atrap.cpp
  ${CMAKE_SOURCE_DIR}/src/velocity_profile_atrap.cpp)
ament_target_dependencies(unittest_velocity_profile_atrap rclcpp)
target_link_libraries(unittest_velocity_profile_atrap ${PROJECT_NAME}_lib)

ament_add_gtest_executable(unittest_trajectory_generator
  src/unittest_trajectory_generator.cpp
  ${CMAKE_SOURCE_DIR}/src/trajectory_generator.cpp)
ament_target_dependencies(unittest_trajectory_generator rclcpp ${PROJECT_NAME}_testutils)
target_link_libraries(unittest_trajectory_generator ${PROJECT_NAME}_lib)

#   # Trajectory Functions Unit Test
#   add_rostest_gtest(unittest_trajectory_functions
#     test/unittest_trajectory_functions.test
#     test/unittest_trajectory_functions.cpp
#   )
#   target_link_libraries(unittest_trajectory_functions
#     ${catkin_LIBRARIES}
#     ${PROJECT_NAME}_testhelpers
#   )
#
#   # unittest for trajectory blender transition window
#   add_rostest_gtest(unittest_trajectory_blender_transition_window
#     test/unittest_trajectory_blender_transition_window.test
#     test/unittest_trajectory_blender_transition_window.cpp
#     src/trajectory_blender_transition_window.cpp
#   )
#   target_link_libraries(unittest_trajectory_blender_transition_window
#     ${catkin_LIBRARIES}
#     ${${PROJECT_NAME}_testutils_LIBRARIES}
#     ${PROJECT_NAME}_testhelpers
#   )
#
#   # trajectory generator Unit Test
#   add_rostest_gtest(unittest_trajectory_generator_common
#     test/unittest_trajectory_generator_common.test
#     test/unittest_trajectory_generator_common.cpp
#   )
#   target_link_libraries(unittest_trajectory_generator_common
#     ${catkin_LIBRARIES}
#     ${PROJECT_NAME}_testhelpers
#   )
#
#   # trajectory generator circ Unit Test
#   add_rostest_gtest(unittest_trajectory_generator_circ
#     test/unittest_trajectory_generator_circ.test
#     test/unittest_trajectory_generator_circ.cpp
#   )
#   target_link_libraries(unittest_trajectory_generator_circ
#     ${catkin_LIBRARIES}
#     ${${PROJECT_NAME}_testutils_LIBRARIES}
#     ${PROJECT_NAME}_testhelpers
#   )
#
#   # trajectory generator lin Unit Test
#   add_rostest_gtest(unittest_trajectory_generator_lin
#     test/unittest_trajectory_generator_lin.test
#     test/unittest_trajectory_generator_lin.cpp
#   )
#   target_link_libraries(unittest_trajectory_generator_lin
#     ${catkin_LIBRARIES}
#     ${${PROJECT_NAME}_testutils_LIBRARIES}
#     ${PROJECT_NAME}_testhelpers
#   )
#
#   # trajectory generator ptp Unit Test
#   add_rostest_gtest(unittest_trajectory_generator_ptp
#     test/unittest_trajectory_generator_ptp.test
#     test/unittest_trajectory_generator_ptp.cpp
#   )
#   target_link_libraries(unittest_trajectory_generator_ptp
#     ${catkin_LIBRARIES}
#     ${PROJECT_NAME}_testhelpers
#   )
#
# Command Planner Unit Test
#ament_add_gtest_executable(unittest_pilz_industrial_motion_planner src/unittest_pilz_industrial_motion_planner.cpp) TODO(sjahr): Debug
#target_link_libraries(unittest_pilz_industrial_motion_planner ${PROJECT_NAME}_lib)
#add_ros_test(launch/unittest_pilz_industrial_motion_planner.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
#
# JointLimits Unit Test
ament_add_gtest_executable(unittest_joint_limit src/unittest_joint_limit.cpp)
target_link_libraries(unittest_joint_limit ${PROJECT_NAME}_lib)
add_ros_test(launch/unittest_joint_limit.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

# JointLimitsAggregator Unit Test
#  ament_add_gtest_executable(unittest_joint_limits_aggregator src/unittest_joint_limits_aggregator.cpp)
#  target_link_libraries(unittest_joint_limits_aggregator ${PROJECT_NAME}_testhelpers ${PROJECT_NAME}_lib)
#  add_ros_test(launch/unittest_joint_limits_aggregator.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")
#   add_rostest_gtest(unittest_joint_limits_aggregator
#     test/unittest_joint_limits_aggregator.test
#     test/unittest_joint_limits_aggregator.cpp
#   )
#   target_link_libraries(unittest_joint_limits_aggregator
#     ${catkin_LIBRARIES}
#     ${PROJECT_NAME}_lib
#   )
#
#   # JointLimitsContainer Unit Test
#   catkin_add_gtest(unittest_joint_limits_container
#     test/unittest_joint_limits_container.cpp
#   )
#   target_link_libraries(unittest_joint_limits_container
#     ${catkin_LIBRARIES}
#     ${PROJECT_NAME}_lib
#   )
#
#   # JointLimitsValidator Unit Test
#   catkin_add_gtest(unittest_joint_limits_validator
#     test/unittest_joint_limits_validator.cpp
#   )
#   target_link_libraries(unittest_joint_limits_validator
#     ${catkin_LIBRARIES}
#     ${PROJECT_NAME}_lib
#   )
#
# Cartesian Limits Aggregator Unit Test
ament_add_gtest_executable(unittest_cartesian_limits_aggregator src/unittest_cartesian_limits_aggregator.cpp)
target_link_libraries(unittest_cartesian_limits_aggregator ${PROJECT_NAME}_lib)
add_ros_test(launch/unittest_cartesian_limits_aggregator.test.py ARGS "test_binary_dir:=${CMAKE_CURRENT_BINARY_DIR}")

#   # PlanningContextLoaderPTP Unit Test
#   add_rostest_gtest(unittest_planning_context_loaders
#     test/unittest_planning_context_loaders.test
#     test/unittest_planning_context_loaders.cpp
#   )
#   target_link_libraries(unittest_planning_context_loaders
#     ${catkin_LIBRARIES}
#     ${PROJECT_NAME}_testhelpers
#   )
#
#   # PlanningContext Unit Test (Typed test)
#   add_rostest_gtest(unittest_planning_context
#     test/unittest_planning_context.test
#     test/unittest_planning_context.cpp
#     src/planning_context_loader_circ.cpp
#     src/planning_context_loader_lin.cpp
#     src/planning_context_loader_ptp.cpp
#   )
#   target_link_libraries(unittest_planning_context
#     ${catkin_LIBRARIES}
#     ${PROJECT_NAME}_testhelpers
#   )
#
## GetTipFrames Unit Test
ament_add_gmock(unittest_get_solver_tip_frame src/unittest_get_solver_tip_frame.cpp)
target_link_libraries(unittest_get_solver_tip_frame ${PROJECT_NAME}_lib)

install(TARGETS ${UNIT_TEST_SOURCEFILES}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
