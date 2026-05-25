import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('map_server')
    
    map_file_arg = LaunchConfiguration('map_file')

    auto_sim_time = PythonExpression([
        "False if 'real' in '", map_file_arg, "' else True"
    ])
    
    map_yaml_path = PathJoinSubstitution([
        pkg_share,
        'config',
        map_file_arg
    ])
    
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'map_display.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value='warehouse_map_sim.yaml',
            description='Name of the map yaml file to load'
        ),

        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                {'yaml_filename': map_yaml_path},
                {'use_sim_time': auto_sim_time} 
            ]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_mapper',
            output='screen',
            parameters=[
                {'use_sim_time': auto_sim_time}, 
                {'autostart': True},
                {'node_names': ['map_server']}
            ]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': auto_sim_time}] 
        )
    ])