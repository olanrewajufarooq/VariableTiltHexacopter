from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

package_name = 'hexacopter_description'

def generate_launch_description():
    # Launch args
    allocation_method = DeclareLaunchArgument(
        'allocation_method',
        default_value='fixed_tilt', # or 'variable_tilt'
        description='Method of control allocation (fixed_tilt or variable_tilt)'
    )

    tilt_angle = DeclareLaunchArgument(
        'tilt_angle', # for fixed tilt only
        default_value='0.00',
        description='Tilt angle for fixed tilt allocation (radians)'
    )

    # Paths
    hex_pkg = get_package_share_directory(package_name)
    spawn_launch_file = os.path.join(hex_pkg, 'launch', 'spawn_robot.launch.py')
    hex_params = os.path.join(hex_pkg, 'config', 'hexacopter_params.yaml')

    # Include robot spawner
    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_launch_file)
    )

    # Define controller node (not launched yet)
    controller_node = Node(
        package=package_name,
        executable='hover_control_node',
        output='screen',
        parameters=[hex_params]
    )

    control_allocator_node = Node(
        package=package_name,
        executable='control_allocation_node',
        output='screen',
        parameters=[
            hex_params,
            {'allocation_method': LaunchConfiguration('allocation_method'), 
             'tilt_angle': LaunchConfiguration('tilt_angle')},
        ]
    )

    delayed_controller_node = TimerAction(
        period=0.0,
        actions=[controller_node, control_allocator_node]
    )

    # Generate joint bridge arguments using a loop
    joint_bridges = [f"/model/variable_tilt_hexacopter/joint_arm_{i}/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double" for i in range(1, 7)]

    # Add the motor speed bridge
    motor_bridge = "/model/variable_tilt_hexacopter/command/motor_speed@actuator_msgs/msg/Actuators]gz.msgs.Actuators"


    # Bridge joint commands (ROS → Gazebo)
    bridge_commands = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='command_bridge',
        output='screen',
        arguments=joint_bridges + [motor_bridge],
    )

    return LaunchDescription([
        allocation_method,
        tilt_angle,
        spawn_launch,
        delayed_controller_node,
        bridge_commands,
    ])
