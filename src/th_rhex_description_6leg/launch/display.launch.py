from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "th_rhex_description_6leg"
    default_model = PathJoinSubstitution(
        [FindPackageShare(package_name), "urdf", "6leg_Th_Rhex_v1.urdf"]
    )
    default_rviz_config = PathJoinSubstitution(
        [FindPackageShare(package_name), "rviz", "urdf.rviz"]
    )

    model = LaunchConfiguration("model")
    rviz_config = LaunchConfiguration("rviz_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=default_model,
                description="Absolute path to a URDF or Xacro file.",
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=default_rviz_config,
                description="Absolute path to an RViz config file.",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": ParameterValue(
                            Command(["xacro", " ", model]),
                            value_type=str,
                        )
                    }
                ],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
