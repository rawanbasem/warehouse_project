import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PythonExpression, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    
    use_sim_time_toggle = LaunchConfiguration('use_sim_time')
    pkg_share = get_package_share_directory('cartographer_slam')
    
    config_dir = os.path.join(pkg_share, 'config')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'mapping.rviz')
    
    # Dynamically select the LUA profile based on runtime environment toggles
    cartographer_config_basename = PythonExpression([
        "'cartographer_sim.lua' if ", 
        use_sim_time_toggle, 
        " else 'cartographer_real.lua'"
    ])

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='True',
            description='Use simulation (Gazebo) clock if True'
        ),

        Node(
            package='cartographer_ros', 
            executable='cartographer_node', 
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_toggle}],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', cartographer_config_basename
            ]
        ),

        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='occupancy_grid_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time_toggle}],
            arguments=['-resolution', '0.05', '-publish_period_sec', '1.0']
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file, '--ros-args', '--log-level', 'ERROR'],
            parameters=[{'use_sim_time': use_sim_time_toggle}]
        ),
    ])