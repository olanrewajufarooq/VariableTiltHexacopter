import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
# import xacro

# import subprocess
# import tempfile

def generate_launch_description():

    # Package paths
    pkg_hex = get_package_share_directory('hexacopter_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ros_gz_bridge = get_package_share_directory('ros_gz_bridge')

    # # Process Xacro file
    # xacro_file = os.path.join(pkg_hex, 'urdf', 'variable_tilt_hexacopter.urdf.xacro')
    # urdf_xml = xacro.process_file(xacro_file).toxml()

    sdf_file = os.path.join(pkg_hex, 'urdf', 'variable_tilt_hexacopter.sdf')

    # # Robot Description
    # robot_description = {'robot_description': urdf_xml}

    # # Create temporary file for SDF
    # tmp_sdf_file = os.path.join(tempfile.gettempdir(), "variable_tilt_hexacopter.sdf")
    # with open(tmp_sdf_file, 'w') as f:
    #     f.write(sdf_xml)

    # Load world
    world_path = os.path.join(pkg_hex, 'worlds', 'empty.sdf')

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_path}'}.items(),
    )

    # # Robot State Publisher
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[robot_description]
    # )

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

            # Optional: World state and pose info
            '/world/empty/pose/info@gz.msgs.Pose_V',
            '/world/empty/state@gz.msgs.SerializedState',


            '/model/variable_tilt_hexacopter/command/motor_speed@std_msgs/msg/Float_V[gz.msgs.Float_V',
        ]
    )

    # # RViz for joint/pose visualization (optional)
    # rviz = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', os.path.join(pkg_hex, 'rviz', 'hexacopter.rviz')],
    #     output='screen',
    #     condition=None  # Optional
    # )

    return LaunchDescription([
        gazebo,
        # robot_state_publisher,
        spawn_entity,
        bridge,
        # rviz,
    ])
