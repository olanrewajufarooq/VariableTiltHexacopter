from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os

geo_ctrl_pkg_name = 'geometric_controllers'
pkg_name = 'state_estimation'

def generate_launch_description():

    # Estimator Specific Launch Arguments
    declare_estimator_mode = DeclareLaunchArgument(
        'estimator_mode', 
        default_value='robust', 
        description='Estimator mode: robust or full', 
        choices=['robust', 'full']
    )

    # Declare launch arguments
    declare_allocation_method = DeclareLaunchArgument('allocation_method', default_value='fixed_tilt', description='Method of control allocation (fixed_tilt, variable_tilt, optimized)')
    declare_tilt_angle = DeclareLaunchArgument('tilt_angle', default_value='0.57', description='Tilt angle for fixed tilt allocation (radians)')
    declare_world = DeclareLaunchArgument('world', default_value='industrial_warehouse_edited.sdf', description='World file to load in Gazebo')
    declare_path_type = DeclareLaunchArgument('path', default_value='infinity_loop', description='Path type: circle, square, infinity_loop, takeoff_land, hover.')
    declare_path_scale = DeclareLaunchArgument('path_scale', default_value='3.0', description='Path scaling factor')
    declare_path_period = DeclareLaunchArgument('path_period', default_value='120.0', description='Traversal speed along the path')
    declare_path_altitude = DeclareLaunchArgument('path_altitude', default_value='5.5', description='Altitude of the generated path')
    declare_path_start_with_hover = DeclareLaunchArgument('path_start_with_hover', default_value='true', description='Start with hover before following the path')
    declare_controller_type = DeclareLaunchArgument('controller_type', default_value='PD', description='Controller type: PD, FeedLin, FeedForward or Adaptive', choices=['PD', 'FeedLin', 'FeedForward', 'Adaptive'])
    declare_potential_type = DeclareLaunchArgument('potential_type', default_value='liealgebra', description='Controller type: liealgebra or separate', choices=['liealgebra', 'separate'])
    declare_Kp_att = DeclareLaunchArgument('Kp_att', default_value='[5.5,5.5,5.5]', description='Proportional gain for attitude control')
    declare_Kp_pos = DeclareLaunchArgument('Kp_pos', default_value='[35.5,35.5,65.28]', description='Proportional gain for position control')
    declare_Kd = DeclareLaunchArgument('Kd', default_value='[2.05,2.05,2.05,20.0,20.5,20.55]', description='Derivative gain for velocity control')

    # Paths
    ctrl_pkg = get_package_share_directory(geo_ctrl_pkg_name)
    est_pkg = get_package_share_directory(pkg_name)
    est_params = os.path.join(est_pkg, 'config', 'estimator_params.yaml')


    path_following_launch_file = os.path.join( ctrl_pkg, 'launch', 'path_following.launch.py' )

    simulator_and_controller_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path_following_launch_file),
        launch_arguments={
            'allocation_method': LaunchConfiguration('allocation_method'),
            'tilt_angle': LaunchConfiguration('tilt_angle'),
            'world': LaunchConfiguration('world'),
            'path': LaunchConfiguration('path'),
            'path_scale': LaunchConfiguration('path_scale'),
            'path_period': LaunchConfiguration('path_period'),
            'path_altitude': LaunchConfiguration('path_altitude'),
            'path_start_with_hover': LaunchConfiguration('path_start_with_hover'),
            'controller_type': LaunchConfiguration('controller_type'),
            'potential_type': LaunchConfiguration('potential_type'),
            'Kp_att': LaunchConfiguration('Kp_att'),
            'Kp_pos': LaunchConfiguration('Kp_pos'),
            'Kd': LaunchConfiguration('Kd'),
        }.items()
    )

    # State estimator node
    state_estimator_node = Node(
        package=pkg_name,
        executable='state_estimator_node',
        name='state_estimator_node',
        output='screen',
        parameters=[est_params, {
            'mode': LaunchConfiguration('estimator_mode'),
        }],
    )

    return LaunchDescription([
        declare_estimator_mode,
        declare_allocation_method,
        declare_tilt_angle,
        declare_world,
        declare_path_type,
        declare_path_scale,
        declare_path_period,
        declare_path_altitude,
        declare_controller_type,
        declare_Kp_att,
        declare_Kp_pos,
        declare_Kd,
        declare_path_start_with_hover,
        declare_potential_type,

        simulator_and_controller_node,
        state_estimator_node,
    ])
