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

    declare_world = DeclareLaunchArgument(
        'world',
        default_value='empty.sdf',
        description='World file to load in Gazebo',
    )

    declare_controller_type = DeclareLaunchArgument(
        'controller_type',
        default_value='PD',
        description='Controller type: PD, FeedLin or Adaptive'
    )

    declare_Kp_att = DeclareLaunchArgument(
        'Kp_att',
        default_value='[0.5, 0.5, 0.5]',
        description='Proportional gain for attitude control'
    )

    declare_Kp_pos = DeclareLaunchArgument(
        'Kp_pos',
        default_value='[0.5, 0.5, 0.5]',
        description='Proportional gain for position control'
    )

    declare_Kd = DeclareLaunchArgument(
        'Kd',
        default_value='[0.5, 0.5, 0.5, 0.5, 0.5, 0.5]',
        description='Derivative gain for velocity control'
    )

    # Paths
    hex_pkg = get_package_share_directory(hex_desc_pkg_name)
    ctrl_pkg = get_package_share_directory(package_name)
    spawn_launch_file = os.path.join(hex_pkg, 'launch', 'spawn_robot.launch.py')
    hex_params = os.path.join(hex_pkg, 'config', 'hexacopter_params.yaml')
    hover_control_params = os.path.join(ctrl_pkg, 'config', 'control_params.yaml')

    # Load YAML parameters
    with open(hex_params, 'r') as f:
        all_params = yaml.safe_load(f)

    # Load Control Allocation Parameters
    control_allocation_params = all_params.get('control_allocation_node', {}).get('ros__parameters', {})
    control_allocation_params['allocation_method'] = LaunchConfiguration('allocation_method')
    control_allocation_params['tilt_angle'] = LaunchConfiguration('tilt_angle')

    # Include robot spawner
    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_launch_file),
        launch_arguments={
            'start_pos': "0.0 0.0 0.3", 
            'world': LaunchConfiguration('world'),
            }.items(),
    )

    control_allocation_node = Node(
        package=hex_desc_pkg_name,
        executable='control_allocation_node',
        name='control_allocation_node',
        output='screen',
        parameters=[control_allocation_params],
    )

    # Define controller node
    controller_node = Node(
        package=package_name,
        executable='hover_control_node',
        name='hover_control_node',
        output='screen',
        parameters=[hover_control_params, 
                    {
                        'hover_altitude': LaunchConfiguration('hover_altitude'),
                        'controller_type': LaunchConfiguration('controller_type'),
                        'Kp_att': PythonExpression(LaunchConfiguration('Kp_att')),
                        'Kp_pos': PythonExpression(LaunchConfiguration('Kp_pos')),
                        'Kd': PythonExpression(LaunchConfiguration('Kd')),
                    }],
    )

    return LaunchDescription([
        declare_allocation_method,
        declare_tilt_angle,
        declare_hover_altitude,
        declare_world,
        declare_controller_type,
        declare_Kp_att,
        declare_Kp_pos,
        declare_Kd,
        
        spawn_launch,
        controller_node,
        control_allocation_node,
    ])
