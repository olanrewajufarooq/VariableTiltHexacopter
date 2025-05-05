import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution

def generate_launch_description():

    declare_start_pos = DeclareLaunchArgument(
        'start_pos',
        default_value='0.0 0.0 1.0',
        description='Initial position of the hexacopter in Gazebo'
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='World file to load in Gazebo',
    )
    
    # Package paths
    pkg_hex = get_package_share_directory('hexacopter_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # SDF file path
    sdf_file = os.path.join(pkg_hex, 'urdf', 'variable_tilt_hexacopter_adaptive.sdf')
    attached_mass_file = os.path.join(pkg_hex, 'urdf', 'attached_mass.sdf')
    world_path = PathJoinSubstitution([pkg_hex, 'worlds', LaunchConfiguration('world')])


    # Gazebo Simulation
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={
            'gz_args': PythonExpression(["'-r -v 1 ' + '", world_path, "'"])
        }.items(),
    )

    # Spawn the attached mass
    spawn_mass = Node(
        package='ros_gz_sim',
        executable='create',
        name='create_attached_mass_entity',
        output='screen',
        arguments=[
            '-name', 'attached_mass',
            '-file', attached_mass_file,
            '-x', PythonExpression(["'", LaunchConfiguration('start_pos'), "'.split()[0]"]),
            '-y', PythonExpression(["'", LaunchConfiguration('start_pos'), "'.split()[1]"]),
            '-z', '0.04',
        ],
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

            # Image data
            '/model/variable_tilt_hexacopter/camera@sensor_msgs/msg/Image[gz.msgs.Image',
            '/model/variable_tilt_hexacopter/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',

            # Motor Speed Command
            "/model/variable_tilt_hexacopter/command/motor_speed@actuator_msgs/msg/Actuators]gz.msgs.Actuators",

            # Joint commands (ROS → Gazebo)
            *[f"/model/variable_tilt_hexacopter/joint_arm_{i}/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double" for i in range(1, 7)],

            # Joint command for sliders
            "/model/variable_tilt_hexacopter/joint_slider_left/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",
            "/model/variable_tilt_hexacopter/joint_slider_right/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double",

            # Payload Gazebo Attach/Detach
            "/model/variable_tilt_hexacopter/payload/attach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/model/variable_tilt_hexacopter/payload/detach@std_msgs/msg/Empty]gz.msgs.Empty",
            "/model/variable_tilt_hexacopter/payload/state@std_msgs/msg/Bool[gz.msgs.Boolean",
        ]
    )

    # Payload Manager Node
    payload_manager = Node(
        package='hexacopter_description',
        executable='payload_manager',
        name='payload_manager',
        output='screen',
    )

    return LaunchDescription([
        declare_start_pos,
        declare_world,
        gazebo,
        spawn_entity,
        spawn_mass,
        bridge,
        payload_manager,
    ])
