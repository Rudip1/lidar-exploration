cmake_minimum_required(VERSION 3.0.2)
project(lidar_based_exploration)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
  message_generation
)

add_message_files(
  FILES
  GoalStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

catkin_install_python(PROGRAMS
  src/laser_scan_to_point_cloud_node.py
  src/sampling_based_exploration_node.py
  src/sampling_based_exploration_node_real.py
  src/turtlebot_online_path_planning_node.py
  src/turtlebot_online_path_planning_node_real.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# The nodes import utils_lib relative to their own location, so install it next to them.
install(DIRECTORY src/utils_lib
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  FILES_MATCHING PATTERN "*.py"
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
