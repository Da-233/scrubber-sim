import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution, Command, FindExecutable, LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare("scrubber_sim")
    pkg_gz = FindPackageShare("ros_gz_sim")

    # M5.3 A7: world 可切换（默认 simple_world.sdf 保持 M4 兼容；
    # M5.3 用 room_7x7.sdf）。m5.launch.py 透传 world:= 参数。
    world_arg = DeclareLaunchArgument(
        "world", default_value="simple_world.sdf",
        description="worlds/ 下的 sdf 文件名",
    )

    urdf_path = PathJoinSubstitution([pkg_share, "urdf", "scrubber.urdf.xacro"])
    world_path = PathJoinSubstitution([
        pkg_share, "worlds", LaunchConfiguration("world"),
    ])

    robot_description = Command([
        PathJoinSubstitution([FindExecutable(name="xacro")]), " ", urdf_path
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([pkg_gz, "launch", "gz_sim.launch.py"])
        ]),
        launch_arguments=[("gz_args", [world_path, " -s -r -v 4"])]
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description},
            {"use_sim_time": True}
        ],
    )

    gz_spawn = Node(
        package="ros_gz_sim",
        executable="create",
        # spawn 在 world 原点 (0,0,0)；SLAM 把 map 原点设在这
        arguments=["-topic", "/robot_description", "-name", "scrubber", "-x", "0", "-y", "0", "-z", "0.2"],
        output="screen",
    )

    # 注意：不再桥 gz 的 /odom，也不桥 /model/scrubber/tf。
    # odom->base_footprint 由自研 wheel_odometry 节点发布（gz 自算 odom 失真）。
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[
            {"use_sim_time": True}
        ],
        arguments=[
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/imu/data@sensor_msgs/msg/Imu[gz.msgs.IMU",
        ],
        output="screen",
    )

    # 自研轮式里程计：订阅 /joint_states，发布 /odom + odom->base_footprint TF
    wheel_odom = Node(
        package="scrubber_sim",
        executable="wheel_odometry",
        name="wheel_odometry",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([
        world_arg,
        gz_sim,
        robot_state_pub,
        gz_spawn,
        gz_bridge,
        wheel_odom,
    ])
