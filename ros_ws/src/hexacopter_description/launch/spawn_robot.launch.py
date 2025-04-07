import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    # Package paths
    pkg_hex = get_package_share_directory('hexacopter_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    sdf_file = os.path.join(pkg_hex, 'urdf', 'variable_tilt_hexacopter.sdf')

    # Load world
    world_path = os.path.join(pkg_hex, 'worlds', 'empty.sdf')

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_path}'}.items(),
    )

    # Spawn Robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='create_hexacopter_entity',
        output='screen',
        arguments=[
            '-name', 'variable_tilt_hexacopter',
            '-file', sdf_file,
            # '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '1.0'
        ]
    )

    # ROS-Gz Bridge (extend as needed)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='hexacopter_bridge',
        output='screen',
        arguments=[
            # Clock
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',

            # Odometry
            '/model/variable_tilt_hexacopter/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/model/variable_tilt_hexacopter/odometry_with_covariance@nav_msgs/msg/Odometry[gz.msgs.Odometry',

            # Pose (base)
            '/model/variable_tilt_hexacopter/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',

            # Joint states
            '/world/empty/model/variable_tilt_hexacopter/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ]
    )

    return LaunchDescription([
        gazebo,
        spawn_entity,
        bridge,
    ])
