#import os
#import yaml
#import xacro
#from ament_index_python.packages import get_package_share_directory
#
#def load_file(package_name, file_path):
#    package_path = get_package_share_directory(package_name)
#    absolute_file_path = os.path.join(package_path, file_path)
#
#    try:
#        with open(absolute_file_path, "r") as file:
#            return file.read()
#    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
#        return None
#
#def load_yaml(package_name, file_path):
#    package_path = get_package_share_directory(package_name)
#    absolute_file_path = os.path.join(package_path, file_path)
#
#    try:
#        with open(absolute_file_path, "r") as file:
#            return yaml.safe_load(file)
#    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
#        return None
#
#class PilzPlannerTestContext():
#  robot_description = None
#  robot_description_semantic = None
#  joint_limits = None
#  cartesian_limits = None
#  kinematics = None
#
#  def __init__(self)
#
##TODO Use jafar's moveit_config_builder!
#
#def create_test_context(gripper):
#  # Load universal robot description format (URDF)
#  robot_description_config = xacro.process_file(
#      os.path.join(
#          get_package_share_directory("moveit_resources_prbt_support"),
#          "urdf",
#          "prbt.xacro",
#      ), gripper:=gripper)
#
#  robot_description = {"robot_description": robot_description_config.toxml()}
#
#
#  # The semantic description that corresponds to the URDF
#  robot_description_semantic_config = xacro.process_file(
#      os.path.join(
#          get_package_share_directory("moveit_resources_prbt_moveit_config"),
#          "config",
#          "prbt.srdf.xacro",
#      ), gripper:=gripper)
#
#  # Load limits again in default namespace (which may not be configurable for planners)
#  if gripper == "":
#    joint_limits = load_yaml("moveit_resources_prbt_moveit_config", "config/cartesian_limits.yaml")
#  else:
#    joint_limits = load_yaml("moveit_resources_prbt_"+ gripper + "_support", "config/cartesian_limits.yaml")
#
#  cartesian_limits = load_yaml(
#        "moveit_resources_prbt_moveit_config", "config/cartesian_limits.yaml"
#    )
#
#  limits = {
#        "robot_description_planning": {joint_limits, cartesian_limits}
#  }
#
#  # Load default settings for kinematics; these settings are overridden by settings in a node's namespace
#  kinematics_yaml = load_yaml(
#        "moveit_resources_prbt_moveit_config", "config/kinematics.yaml"
#    )
#  return robot_description, robot_description_semantic_config, limits, kinematics_yaml