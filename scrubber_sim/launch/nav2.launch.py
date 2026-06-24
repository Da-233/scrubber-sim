import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("scrubber_sim")
    nav2_params = PathJoinSubstitution([pkg_share, "config", "nav2.yaml"])
    slam_params = PathJoinSubstitution([pkg_share, "config", "slam_toolbox.yaml"])
    bt_xml = PathJoinSubstitution([pkg_share, "config", "ackermann_bt.xml"])

    # 仿真 + 机器人 + bridge
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, "launch", "bringup.launch.py"])
        ])
    )

    # SLAM（提供 map->odom）
    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params, {"use_sim_time": True}],
    )

    # Nav2 核心节点（精简：只起单点导航必需的，不含 collision_monitor/route_server/smoother）
    nav2_nodes = [
        Node(package="nav2_controller", executable="controller_server", output="screen",
             parameters=[nav2_params]),
        Node(package="nav2_planner", executable="planner_server", name="planner_server",
             output="screen", parameters=[nav2_params]),
        Node(package="nav2_behaviors", executable="behavior_server", name="behavior_server",
             output="screen", parameters=[nav2_params]),
        Node(package="nav2_bt_navigator", executable="bt_navigator", name="bt_navigator",
             output="screen",
             parameters=[nav2_params,
                         {"default_nav_to_pose_bt_xml": bt_xml},
                         {"default_nav_through_poses_bt_xml": bt_xml}]),
    ]

    # slam_toolbox 用独立的 lifecycle_manager（它的 bond 与 nav2 manager 不完全兼容，
    # 混在一起会 bond 超时；拆开各管各的）
    lifecycle_mgr_slam = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "autostart": True,
            "node_names": ["slam_toolbox"],
            "bond_timeout": 0.0,
        }],
    )

    nav2_lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
    ]

    lifecycle_mgr_nav = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_nav",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "autostart": True,
            "node_names": nav2_lifecycle_nodes,
        }],
    )

    # 延迟启动 Nav2，等仿真/SLAM 先就绪
    delayed = TimerAction(period=8.0, actions=nav2_nodes + [lifecycle_mgr_nav])

    return LaunchDescription([bringup, slam, lifecycle_mgr_slam, delayed])
