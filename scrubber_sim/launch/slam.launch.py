import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("scrubber_sim")
    slam_params = PathJoinSubstitution([pkg_share, "config", "slam_toolbox.yaml"])

    # 先起仿真 + 机器人 + bridge
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, "launch", "bringup.launch.py"])
        ])
    )

    # slam_toolbox 是 lifecycle 节点；async 节点自身不自动激活，
    # 用 nav2_lifecycle_manager 自动 configure+activate。
    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params, {"use_sim_time": True}],
    )

    lifecycle_mgr = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "autostart": True,
            "node_names": ["slam_toolbox"],
        }],
    )

    return LaunchDescription([bringup, slam, lifecycle_mgr])
