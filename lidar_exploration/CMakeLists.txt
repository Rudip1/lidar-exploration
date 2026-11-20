cmake_minimum_required(VERSION 3.22)
project(lidar_exploration LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include(cmake/learn_motion_planning.cmake)  # target: motion_planning

# ---- ROS-free algorithm core
add_library(exploration_core STATIC
    src/grid_map.cpp src/frontier.cpp src/planner.cpp src/path_follower.cpp src/explorer.cpp)
target_include_directories(exploration_core PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
target_link_libraries(exploration_core PUBLIC motion_planning Eigen3::Eigen)

# ---- ROS 2 node
add_executable(explorer_node src/explorer_node.cpp)
target_link_libraries(explorer_node exploration_core)
ament_target_dependencies(explorer_node rclcpp geometry_msgs nav_msgs std_msgs visualization_msgs tf2 tf2_ros)

install(TARGETS explorer_node DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/evaluate_exploration.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_core test/test_core.cpp)
    target_link_libraries(test_core exploration_core)
    target_compile_definitions(test_core PRIVATE
        WALLS_JSON="${CMAKE_CURRENT_SOURCE_DIR}/../lidar_exploration_gazebo/worlds/office_floor_walls.json")
    ament_add_gtest(test_exploration test/test_exploration.cpp TIMEOUT 900)
    target_link_libraries(test_exploration exploration_core)
endif()

ament_package()
