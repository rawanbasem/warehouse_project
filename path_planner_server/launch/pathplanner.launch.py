import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    nav_dir = get_package_share_directory('path_planner_server')

    use_sim_time_param = LaunchConfiguration('use_sim_time')

    planner_yaml = PathJoinSubstitution([
        nav_dir, 'config',
        PythonExpression(["'planner_sim.yaml' if '", use_sim_time_param, "'.lower() == 'true' else 'planner_real.yaml'"])
    ])

    controller_yaml = PathJoinSubstitution([
        nav_dir, 'config',
        PythonExpression(["'controller_sim.yaml' if '", use_sim_time_param, "'.lower() == 'true' else 'controller_real.yaml'"])
    ])

    bt_navigator_yaml = PathJoinSubstitution([
        nav_dir, 'config',
        PythonExpression(["'bt_navigator_sim.yaml' if '", use_sim_time_param, "'.lower() == 'true' else 'bt_navigator_real.yaml'"])
    ])

    recovery_yaml = PathJoinSubstitution([
        nav_dir, 'config',
        PythonExpression(["'recoveries_sim.yaml' if '", use_sim_time_param, "'.lower() == 'true' else 'recoveries_real.yaml'"])
    ])

    rviz_config_path = os.path.join(nav_dir, 'rviz', 'navigation.rviz')

    filters_yaml = PathJoinSubstitution([
        nav_dir, 'config',
        PythonExpression(["'filters.yaml' if '", use_sim_time_param, "'.lower() == 'true' else 'filters_real.yaml'"])
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true, real hardware clock if false'
        ),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml, {'use_sim_time': use_sim_time_param}]
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml, {'use_sim_time': use_sim_time_param}]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml, {'use_sim_time': use_sim_time_param}]
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[recovery_yaml, {'use_sim_time': use_sim_time_param}]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
            parameters=[{'use_sim_time': use_sim_time_param}]
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='filter_mask_server',
            output='screen',
            emulate_tty=True,
            parameters=[filters_yaml, {'use_sim_time': use_sim_time_param}]
        ),

        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='costmap_filter_info_server',
            output='screen',
            emulate_tty=True,
            parameters=[filters_yaml, {'use_sim_time': use_sim_time_param}]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_pathplanning',
            output='screen',
            parameters=[
                {'autostart': True},
                {'use_sim_time': use_sim_time_param},
                {'node_names': [
                    'planner_server', 
                    'controller_server', 
                    'behavior_server', 
                    'bt_navigator',
                    'filter_mask_server',
                    'costmap_filter_info_server'
                ]}
            ]
        )
    ])