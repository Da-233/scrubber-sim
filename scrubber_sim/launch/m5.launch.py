"""M5 弓字形全覆盖 launch.

栈：bringup(gz+urdf+wheel_odom+bridge) + SLAM(slam_toolbox)
    + Nav2 控制面(controller/behavior/bt_navigator+coverage_server)
    + 独立 lifecycle_manager × 2（slam 一个、nav 一个，bond 不兼容拆开）

与 nav2.launch.py 的差异：
  - bt_navigator 多注册 navigate_complete_coverage 这个 navigator
  - 新加 coverage_server 节点（Fields2Cover 算法层）
  - bt_xml 用 opennav_coverage_bt 自带的 navigate_w_basic_complete_coverage.xml
    （内置 ComputeCoveragePath → FollowPath，阿卡曼安全，无 Spin/BackUp）
  - planner_server 仍起着，给可能的 nav_to_pose / fallback 用
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription, TimerAction, DeclareLaunchArgument,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("scrubber_sim")
    nav2_params = PathJoinSubstitution([pkg_share, "config", "nav2.yaml"])
    coverage_params = PathJoinSubstitution([pkg_share, "config", "coverage.yaml"])
    slam_params = PathJoinSubstitution([pkg_share, "config", "slam_toolbox.yaml"])

    # M5.3 A6: 自定义带 recovery 的弓字形 BT（ComputeCoveragePath → FollowPath
    # 外包双层 RecoveryNode：清 costmap + Wait，无 Spin/BackUp，阿卡曼安全）。
    # 替代 opennav_coverage_bt 自带的无 recovery 版。
    coverage_bt_xml = PathJoinSubstitution([
        pkg_share, "config", "coverage_recovery_bt.xml",
    ])

    # M5.3 A7: world 可切换，默认 room_7x7.sdf（7×7房+2箱障，M5.3 场景）
    world_arg = DeclareLaunchArgument(
        "world", default_value="room_7x7.sdf",
        description="worlds/ 下的 sdf 文件名",
    )

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_share, "launch", "bringup.launch.py"])
        ]),
        launch_arguments=[("world", LaunchConfiguration("world"))],
    )

    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[slam_params, {"use_sim_time": True}],
    )

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

    # Nav2 控制面 + coverage_server
    # 注：每个节点都同时加载 nav2.yaml 与 coverage.yaml。
    # ROS 参数装载是后者覆盖前者的同名段：bt_navigator 的 plugin_lib_names/navigators
    # 在 coverage.yaml 里整个列出（含 nav 原生 + opennav 专属），所以会完整覆盖。
    common_params = [nav2_params, coverage_params]

    controller = Node(
        package="nav2_controller", executable="controller_server",
        name="controller_server", output="screen",
        parameters=common_params,
    )
    planner = Node(
        package="nav2_planner", executable="planner_server",
        name="planner_server", output="screen",
        parameters=common_params,
    )
    behavior = Node(
        package="nav2_behaviors", executable="behavior_server",
        name="behavior_server", output="screen",
        parameters=common_params,
    )
    coverage_server = Node(
        package="opennav_coverage", executable="opennav_coverage",
        name="coverage_server", output="screen",
        parameters=common_params,
    )
    bt_navigator = Node(
        package="nav2_bt_navigator", executable="bt_navigator",
        name="bt_navigator", output="screen",
        parameters=common_params + [
            # M5 只跑 NavigateCompleteCoverage，不注册 to_pose/through_poses（避免
            # NavigateToPoseNavigator 内置注册与 plugin_lib_names 重复 ID FATAL）。
            # default_nav_complete_coverage_bt_xml 显式注入更稳：
            {"default_nav_complete_coverage_bt_xml": coverage_bt_xml},
        ],
    )

    nav2_lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "coverage_server",
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

    # 延迟 8s 启 Nav2 控制面（让 gz/SLAM 先就绪、TF 树成形）
    delayed = TimerAction(
        period=8.0,
        actions=[controller, planner, behavior, coverage_server, bt_navigator,
                 lifecycle_mgr_nav],
    )

    return LaunchDescription([world_arg, bringup, slam, lifecycle_mgr_slam, delayed])
