import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression

def generate_launch_description():

    declare_start_pos = DeclareLaunchArgument(
        'start_pos',
        default_value='0.0 0.0 1.0',
        description='Initial position of the hexacopter in Gazebo'
    )

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
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
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
            '-x', PythonExpression(["'", LaunchConfiguration('start_pos'), "'.split()[0]"]),
            '-y', PythonExpression(["'", LaunchConfiguration('start_pos'), "'.split()[1]"]),
            '-z', PythonExpression(["'", LaunchConfiguration('start_pos'), "'.split()[2]"]),
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

            # Pose (base)
            '/model/variable_tilt_hexacopter/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose',

            # Joint states
            '/world/empty/model/variable_tilt_hexacopter/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',

            # IMU 
            '/model/variable_tilt_hexacopter/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ]
    )

    return LaunchDescription([
        declare_start_pos,
        gazebo,
        spawn_entity,
        bridge,
    ])
