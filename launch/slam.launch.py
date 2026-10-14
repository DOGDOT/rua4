import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_rua4 = get_package_share_directory('rua4')
    pkg_tb4_nav = get_package_share_directory('turtlebot4_navigation')

    # Arguments
    launch_sim_arg = DeclareLaunchArgument(
        'launch_sim', default_value='false',
        choices=['true', 'false'],
        description='Tự động khởi chạy mô phỏng Gazebo nếu chưa chạy')
    world_arg = DeclareLaunchArgument(
        'world', default_value='warehouse',
        description='Môi trường mô phỏng (warehouse, maze, depot)')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        choices=['true', 'false'],
        description='Mở RViz2 hiển thị quá trình xây dựng bản đồ')
    sync_arg = DeclareLaunchArgument(
        'sync', default_value='true',
        choices=['true', 'false'],
        description='Chế độ SLAM đồng bộ (sync) hay bất đồng bộ (async)')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Sử dụng simulation clock')
    params_file_arg = DeclareLaunchArgument(
        'params',
        default_value=PathJoinSubstitution([pkg_rua4, 'config', 'slam.yaml']),
        description='File cấu hình SLAM Toolbox')

    # Simulation include (nếu launch_sim=true, scoped=True để không rò rỉ cấu hình rviz)
    from launch.actions import GroupAction
    sim_launch = GroupAction(
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg_rua4, 'launch', 'sim.launch.py'])
                ),
                launch_arguments={
                    'world': LaunchConfiguration('world'),
                    'rviz': 'false',
                    'use_sim_time': LaunchConfiguration('use_sim_time')
                }.items(),
            )
        ],
        scoped=True,
        condition=IfCondition(LaunchConfiguration('launch_sim'))
    )

    # SLAM Toolbox include
    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_tb4_nav, 'launch', 'slam.launch.py'])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'sync': LaunchConfiguration('sync'),
            'params': LaunchConfiguration('params')
        }.items()
    )

    # RViz2 cấu hình cho SLAM
    rviz_config = PathJoinSubstitution([pkg_rua4, 'config', 'rviz', 'rua4_slam.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        additional_env={
            'QT_QPA_PLATFORM': 'xcb',
            'WAYLAND_DISPLAY': '',
        },
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    rmw_env = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')

    ld = LaunchDescription()
    ld.add_action(rmw_env)
    ld.add_action(launch_sim_arg)
    ld.add_action(world_arg)
    ld.add_action(rviz_arg)
    ld.add_action(sync_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(params_file_arg)

    ld.add_action(sim_launch)
    ld.add_action(slam_toolbox_launch)
    ld.add_action(rviz_node)

    return ld
