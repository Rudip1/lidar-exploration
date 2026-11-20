"""Simulation + slam_toolbox + the explorer, optionally with the evaluation and RViz.

    ros2 launch lidar_exploration exploration.launch.py                            # Gazebo window and RViz
    ros2 launch lidar_exploration exploration.launch.py gui:=false rviz:=false evaluate:=true output:=run.csv
"""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("lidar_exploration"))
    sim = Path(get_package_share_directory("lidar_exploration_gazebo")) / "launch" / "sim.launch.py"
    # slam_toolbox is a lifecycle node on Jazzy; its own launch file configures and activates it.
    slam = Path(get_package_share_directory("slam_toolbox")) / "launch" / "online_async_launch.py"
    arg = LaunchConfiguration

    evaluator = Node(package="lidar_exploration", executable="evaluate_exploration.py", output="screen",
                     parameters=[{"use_sim_time": True, "output": arg("output")}],
                     condition=IfCondition(arg("evaluate")))
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="Gazebo window"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("evaluate", default_value="false",
                              description="compare the map with the floor plan when exploration ends, then shut down"),
        DeclareLaunchArgument("output", default_value="exploration_run.csv", description="CSV of the evaluation"),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(sim)), launch_arguments={"gui": arg("gui")}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(slam)),
                                 launch_arguments={"slam_params_file": str(share / "config" / "slam_toolbox.yaml"),
                                                   "use_sim_time": "true"}.items()),
        Node(package="lidar_exploration", executable="explorer_node", name="explorer", output="screen",
             parameters=[str(share / "config" / "explorer.yaml")]),
        evaluator,
        RegisterEventHandler(OnProcessExit(target_action=evaluator, on_exit=[EmitEvent(event=Shutdown())])),
        Node(package="rviz2", executable="rviz2", arguments=["-d", str(share / "rviz" / "exploration.rviz")],
             parameters=[{"use_sim_time": True}], condition=IfCondition(arg("rviz"))),
    ])
