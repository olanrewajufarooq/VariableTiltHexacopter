from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os, yaml

package_name = 'hexacopter_description'

def generate_launch_description():
    # Launch args
    declare_allocation_method = DeclareLaunchArgument(
        'allocation_method',
        default_value='fixed_tilt', # or 'variable_tilt'
        description='Method of control allocation (fixed_tilt or variable_tilt)'
    )

    declare_tilt_angle = DeclareLaunchArgument(
        'tilt_angle', # for fixed tilt only
        default_value='0.57', # radians
        description='Tilt angle for fixed tilt allocation (radians)'
    )

    declare_hover_altitude = DeclareLaunchArgument(
        'hover_altitude',
        default_value='5.0',  # meters
        description='Desired hover altitude'
    )

    declare_hover_gain = DeclareLaunchArgument(
        'hover_gain',
        default_value='1.0',  # Kp gain for hover error correction
        description='Kp gain for hover error correction'
    )

    # Paths
    hex_pkg = get_package_share_directory(package_name)
    spawn_launch_file = os.path.join(hex_pkg, 'launch', 'spawn_robot.launch.py')
    hex_params = os.path.join(hex_pkg, 'config', 'hexacopter_params.yaml')

    # Load YAML parameters
    with open(hex_params, 'r') as f:
        all_params = yaml.safe_load(f)
        
    hover_params = all_params.get('hover_control_node', {}).get('ros__parameters', {})
    hover_params['hover_altitude'] = LaunchConfiguration('hover_altitude')
    hover_params['hover_gain'] = LaunchConfiguration('hover_gain')

    control_allocation_params = all_params.get('control_allocation_node', {}).get('ros__parameters', {})
    control_allocation_params['allocation_method'] = LaunchConfiguration('allocation_method')
    control_allocation_params['tilt_angle'] = LaunchConfiguration('tilt_angle')

    # Include robot spawner
    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_launch_file),
        launch_arguments={'start_pos': PythonExpression([
            "'0.0 0.0 ' + str(", LaunchConfiguration('hover_altitude'), ")" # spawning at hover altitude
        ])}.items(),
    )

    # Define controller node
    controller_node = Node(
        package=package_name,
        executable='hover_control_node',
        output='screen',
        parameters=[hover_params]
    )

    control_allocation_node = Node(
        package=package_name,
        executable='control_allocation_node',
        output='screen',
        parameters=[control_allocation_params],
    )

    return LaunchDescription([
        declare_allocation_method,
        declare_tilt_angle,
        declare_hover_altitude,
        declare_hover_gain,
        spawn_launch,
        controller_node,
        control_allocation_node,
    ])
