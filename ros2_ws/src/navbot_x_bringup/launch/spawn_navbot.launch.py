from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world_file = PathJoinSubstitution([
        FindPackageShare('navbot_x_sim'),
        'worlds',
        'empty.world'
    ])

    gz_launch = PathJoinSubstitution([
        FindPackageShare('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    ])

    xacro_file = PathJoinSubstitution([
        FindPackageShare('navbot_x_description'),
        'urdf',
        'navbot_x.urdf.xacro'
    ])

    bridge_launch = PathJoinSubstitution([
        FindPackageShare('navbot_x_bringup'),
        'launch',
        'bridge.launch.py'
    ])

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]),
        value_type=str
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_launch),
            launch_arguments={'gz_args': ['-r ', world_file]}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bridge_launch)
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
            output='screen'
        ),

        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=[
                '-name', 'navbot_x',
                '-topic', 'robot_description',
                '-x', '0',
                '-y', '0',
                '-z', '0.2'
            ],
            output='screen'
        )
    ])