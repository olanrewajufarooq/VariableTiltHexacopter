from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    declare_world = DeclareLaunchArgument(
        'world_name',
        default_value='empty.sdf',
        description='SDF world file name'
    )

    world_path = PathJoinSubstitution([
        FindPackageShare('hexacopter_description'),
        'worlds',
        LaunchConfiguration('world_name')
    ])

    gz_sim_launch = PathJoinSubstitution([
        FindPackageShare('ros_gz_sim'),
        'launch',
        'gz_sim.launch.py'
    ])

    xacro_file = PathJoinSubstitution([
        FindPackageShare('hexacopter_description'),
        'urdf',
        'variable_tilt_hexacopter.urdf.xacro'
    ])

    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([
        declare_world,

        # Start Gazebo Sim by including its launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gz_sim_launch),
            launch_arguments={
                'gz_args': [world_path]
            }.items()
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
                '-name', 'hexacopter', 
                '-topic', 'robot_description',
                '-x', '0', '-y', '0', '-z', '1.0'
                ],
            output='screen'
        ),
    ])
