from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os, yaml

hex_desc_pkg_name = 'hexacopter_description'
package_name = 'geometric_controllers'

def generate_launch_description():

    # Declare launch arguments
    declare_allocation_method = DeclareLaunchArgument(
        'allocation_method', default_value='fixed_tilt',
        description='Method of control allocation (fixed_tilt or variable_tilt)'
    )

    declare_tilt_angle = DeclareLaunchArgument(
        'tilt_angle', default_value='0.57',
        description='Tilt angle for fixed tilt allocation (radians)'
    )

    declare_world = DeclareLaunchArgument(
        'world', default_value='empty.sdf',
        description='World file to load in Gazebo'
    )

    declare_path_type = DeclareLaunchArgument(
        'path', default_value='infinity_loop',
        description='Path type: circle, square, infinity_loop, takeoff_land, hover.'
    )

    declare_path_scale = DeclareLaunchArgument(
        'path_scale', default_value='5.0',
        description='Path scaling factor'
    )

    declare_path_period = DeclareLaunchArgument(
        'path_period', default_value='60.0',
        description='Traversal speed along the path'
    )

    declare_path_altitude = DeclareLaunchArgument(
        'path_altitude', default_value='5.0',
        description='Altitude of the generated path'
    )

    declare_path_start_with_hover = DeclareLaunchArgument(
        'path_start_with_hover', default_value='true',
        description='Start with hover before following the path'
    )

    declare_controller_type = DeclareLaunchArgument(
        'controller_type',
        default_value='Adaptive',
        description='Controller type: PD, FeedLin or Adaptive'
    )

    declare_potential_type = DeclareLaunchArgument(
        'potential_type',
        default_value='liealgebra',
        description='Controller type: liealgebra or separate',
        choices=['liealgebra', 'separate'],
    )

    declare_Kp_att = DeclareLaunchArgument(
        'Kp_att',
        default_value='[5.5,5.5,5.5]',
        description='Proportional gain for attitude control'
    )

    declare_Kp_pos = DeclareLaunchArgument(
        'Kp_pos',
        default_value='[35.5,35.5,65.28]',
        description='Proportional gain for position control'
    )

    declare_Kd = DeclareLaunchArgument(
        'Kd',
        default_value='[2.05,2.05,2.05,20.0,20.5,20.55]',
        description='Derivative gain for velocity control'
    )

    # Paths
    hex_pkg = get_package_share_directory(hex_desc_pkg_name)
    ctrl_pkg = get_package_share_directory(package_name)
    spawn_launch_file = os.path.join(hex_pkg, 'launch', 'spawn_robot_adaptive.launch.py')
    hex_params = os.path.join(hex_pkg, 'config', 'hexacopter_params.yaml')
    control_params = os.path.join(ctrl_pkg, 'config', 'control_params.yaml')

    # Load control allocation parameters
    with open(hex_params, 'r') as f:
        all_params = yaml.safe_load(f)

    control_allocation_params = all_params.get('control_allocation_node', {}).get('ros__parameters', {})
    control_allocation_params['allocation_method'] = LaunchConfiguration('allocation_method')
    control_allocation_params['tilt_angle'] = LaunchConfiguration('tilt_angle')

    # Include robot spawner
    spawn_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_launch_file),
        launch_arguments={
            'start_pos': "0.0 0.0 0.1",
            'world': LaunchConfiguration('world'),
        }.items()
    )

    # Control allocation node
    control_allocation_node = Node(
        package=hex_desc_pkg_name,
        executable='control_allocation_node',
        name='control_allocation_node',
        output='screen',
        parameters=[control_allocation_params],
    )

    # Path-following controller node
    adaptive_path_following_node = Node(
        package=package_name,
        executable='adaptive_path_following_node',
        name='adaptive_path_following_node',
        output='screen',
        parameters=[control_params, {
            'path': LaunchConfiguration('path'),
            'path_scale': LaunchConfiguration('path_scale'),
            'path_period': LaunchConfiguration('path_period'),
            'path_altitude': LaunchConfiguration('path_altitude'),
            'path_start_with_hover': LaunchConfiguration('path_start_with_hover'),
            'controller_type': LaunchConfiguration('controller_type'),
            'potential_type': LaunchConfiguration('potential_type'),
            'Kp_att': PythonExpression(LaunchConfiguration('Kp_att')),
            'Kp_pos': PythonExpression(LaunchConfiguration('Kp_pos')),
            'Kd': PythonExpression(LaunchConfiguration('Kd')),
        }],
    )

    return LaunchDescription([
        declare_allocation_method,
        declare_tilt_angle,
        declare_world,
        declare_path_type,
        declare_path_scale,
        declare_path_period,
        declare_path_altitude,
        declare_controller_type,
        declare_potential_type,
        declare_Kp_att,
        declare_Kp_pos,
        declare_Kd,
        declare_path_start_with_hover,

        spawn_launch,
        control_allocation_node,
        adaptive_path_following_node,
    ])
