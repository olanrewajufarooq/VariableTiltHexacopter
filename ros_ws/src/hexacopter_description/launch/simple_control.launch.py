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
    declare_angles = DeclareLaunchArgument(
        'angles',
        default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
        description='List of 6 joint tilt angles (radians)'
    )

    declare_motor_speeds = DeclareLaunchArgument(
        'motor_speeds',
        default_value='[400.0, 400.0, 400.0, 400.0, 400.0, 400.0]',
        description='List of 6 motor speeds (rad/s)'
    )

    # Paths
    hex_pkg = get_package_share_directory(package_name)
    spawn_launch_file = os.path.join(hex_pkg, 'launch', 'spawn_robot.launch.py')

    # Include robot spawner
    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_launch_file)
    )

    # Define controller node (not launched yet)
    controller_node = Node(
        package=package_name,
        executable='simple_control_node',
        # name='simple_control_node',
        output='screen',
        parameters=[{
            'angles': LaunchConfiguration('angles'),
            'motor_speeds': LaunchConfiguration('motor_speeds')
        }]
    )

    delayed_controller_node = TimerAction(
        period=5.0,
        actions=[controller_node]
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
        declare_angles,
        declare_motor_speeds,
        spawn_launch,
        delayed_controller_node,
        bridge_commands,
    ])
