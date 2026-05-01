from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_name = "th_rhex_description"
    default_model = PathJoinSubstitution(
        [FindPackageShare(package_name), "urdf", "th_rhex.urdf.xacro"]
    )
    default_world = PathJoinSubstitution(
        [FindPackageShare(package_name), "worlds", "minimal_world.sdf"]
    )
    gz_sim_launch = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )

    model = LaunchConfiguration("model")
    world = LaunchConfiguration("world")

    # Robot description from xacro
    robot_description = ParameterValue(
        Command(["xacro", " ", model]),
        value_type=str
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model",
                default_value=default_model,
                description="Absolute path to a URDF or Xacro file.",
            ),
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

            # 2. Robot state publisher - broadcasts TF transforms
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{robot_description: robot_description}],
                output="screen",
            ),

            # 3. Spawn robot into Gazebo
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

            # 4. Spawn joint_state_broadcaster after short delay
            TimerAction(
                period=3.0, #seconds
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=["joint_state_broadcaster"],
                        output="screen",
                    ),
                ]
            ),

            # 5. Spawn leg velocity controller after joint_state_broadcaster
            TimerAction(
                period=4.0, #seconds
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
