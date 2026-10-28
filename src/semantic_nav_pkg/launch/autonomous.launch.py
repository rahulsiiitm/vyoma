import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('semantic_nav_pkg')
    parameters = os.path.join(share, 'config', 'autopilot_params.yaml')
    nav2_parameters = os.path.join(share, 'config', 'nav2_params.yaml')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    model_path = LaunchConfiguration('model_path')
    run_policy = LaunchConfiguration('run_policy')
    training = LaunchConfiguration('training')

    common = [parameters, {'use_sim_time': use_sim_time}]
    return LaunchDescription([
        RegisterEventHandler(OnProcessExit(on_exit=[
            EmitEvent(event=Shutdown(reason='An autonomy process exited; stopping the stack'))])),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('run_policy', default_value='true'),
        DeclareLaunchArgument('training', default_value='false',
                              description='Observation-only policy node; actions come from DroneEnv'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['0.12', '0.03', '0.242', '0', '0', '0', 'base_link', 'camera_link']),
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[nav2_parameters, {'use_sim_time': use_sim_time}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_planner', output='screen',
             parameters=[{'use_sim_time': use_sim_time, 'autostart': True,
                          'node_names': ['planner_server']}]),
        Node(package='semantic_nav_pkg', executable='depth_sector_node', parameters=common),
        Node(package='semantic_nav_pkg', executable='global_planner_node', parameters=common),
        Node(package='semantic_nav_pkg', executable='local_target_node', parameters=common),
        Node(package='semantic_nav_pkg', executable='mission_manager_node', parameters=common),
        Node(package='semantic_nav_pkg', executable='safety_supervisor_node', parameters=common),
        Node(package='semantic_nav_pkg', executable='px4_adapter_node', parameters=common),
        Node(package='rl_navigation_pkg', executable='policy_node', output='screen',
             parameters=[parameters, {'use_sim_time': use_sim_time, 'model_path': model_path,
                                     'publish_action': PythonExpression(["'", training, "' != 'true'"])}],
             condition=IfCondition(run_policy)),
        Node(package='rviz2', executable='rviz2', name='rviz_visualizer',
             arguments=['-d', os.path.join(share, 'config', 'default.rviz')],
             condition=IfCondition(use_rviz)),
    ])
