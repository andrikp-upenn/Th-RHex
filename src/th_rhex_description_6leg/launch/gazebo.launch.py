import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "th_rhex_description_6leg"
    pkg_share = get_package_share_directory(package_name)

    # Resolve package:// → file:// (meshes) and $(find ...) → abs path (controllers.yaml)
    urdf_path = os.path.join(pkg_share, "urdf", "6leg_Th_Rhex_v1.urdf")
    with open(urdf_path, "r") as f:
        robot_description = f.read() \
            .replace(f"package://{package_name}", f"file://{pkg_share}") \
            .replace(f"$(find {package_name})", pkg_share)

    default_world = PathJoinSubstitution(
        [FindPackageShare(package_name), "worlds", "minimal_world.sdf"]
    )
    gz_sim_launch = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )

    world = LaunchConfiguration("world")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=default_world,
                description="Absolute path to a Gazebo SDF world.",
            ),

            # 1. Start Gazebo
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_sim_launch),
                launch_arguments={"gz_args": ["-r ", world]}.items(),
            ),

            # 2. Bridge Gazebo /clock → ROS2 /clock
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
                output="screen",
            ),

            # 3. Robot state publisher - broadcasts TF transforms
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {"robot_description": robot_description},
                    {"use_sim_time": True},
                ],
                output="screen",
            ),

            # 4. Spawn robot into Gazebo
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-topic",
                    "robot_description",
                    "-name",
                    "th_rhex",
                    "-z",
                    "0.25",
                ],
                output="screen",
            ),

            # 5. Spawn joint_state_broadcaster after short delay
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=["joint_state_broadcaster"],
                        output="screen",
                    ),
                ]
            ),

            # 6. Spawn leg velocity controller after joint_state_broadcaster
            TimerAction(
                period=4.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=["leg_velocity_controller"],
                        output="screen",
                    ),
                ]
            ),

        ]
    )
