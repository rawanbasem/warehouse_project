import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import Node

def generate_launch_description():
    loc_share = get_package_share_directory('localization_server')
    map_share = get_package_share_directory('map_server')
    
    # ⚠️ FIXED: Define the LaunchConfiguration FIRST so everything below can use it
    map_file_param = LaunchConfiguration('map_file')
    
    # Now this expression evaluates perfectly without throwing a NameError
    nav2_yaml = PythonExpression([
        "'", os.path.join(loc_share, 'config', 'amcl_config_real.yaml'), "' if 'real' in '", map_file_param, "' else '", os.path.join(loc_share, 'config', 'amcl_config_sim.yaml'), "'"
    ])
    
    rviz_config_file = os.path.join(loc_share, 'rviz', 'localization.rviz')
    
    map_absolute_path = PathJoinSubstitution([
        map_share,
        'config',
        map_file_param
    ])

    auto_sim_time = PythonExpression([
        "False if 'real' in '", map_file_param, "' else True"
    ])

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
            parameters=[{'use_sim_time': auto_sim_time}, 
                        {'yaml_filename': map_absolute_path}] 
        ),
            
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            arguments=['--ros-args', '--log-level', 'WARN'],
            parameters=[nav2_yaml, {'use_sim_time': auto_sim_time}]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': auto_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server', 'amcl']}]
        ),
        
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'ERROR'],
            parameters=[{'use_sim_time': auto_sim_time}]
        ),
    ])