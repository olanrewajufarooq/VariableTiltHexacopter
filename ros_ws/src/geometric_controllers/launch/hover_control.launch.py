from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os, yaml

hex_desc_pkg_name = 'hexacopter_description'
package_name = 'geometric_controllers'

def generate_launch_description():

    # Paths
    hex_pkg = get_package_share_directory(hex_desc_pkg_name)
    spawn_launch_file = os.path.join(hex_pkg, 'launch', 'spawn_robot.launch.py')
    hex_params = os.path.join(hex_pkg, 'config', 'hexacopter_params.yaml')

    # Load YAML parameters
    with open(hex_params, 'r') as f:
        all_params = yaml.safe_load(f)

    control_allocation_params = all_params.get('control_allocation_node', {}).get('ros__parameters', {})
    control_allocation_params['allocation_method'] = LaunchConfiguration('allocation_method')
    control_allocation_params['tilt_angle'] = LaunchConfiguration('tilt_angle')

    # Include robot spawner
    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_launch_file),
        launch_arguments={'start_pos': "0.0 0.0 1.0"}.items(),
    )

    # Define controller node
    controller_node = Node(
        package=package_name,
        executable='hover_control_node',
        output='screen',
    )

    control_allocation_node = Node(
        package=hex_desc_pkg_name,
        executable='control_allocation_node',
        output='screen',
        parameters=[control_allocation_params],
    )

    return LaunchDescription([
        spawn_launch,
        controller_node,
        control_allocation_node,
    ])
