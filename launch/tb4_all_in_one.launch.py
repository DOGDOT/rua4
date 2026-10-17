import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import LaunchConfigurationEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    rmw_env = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    pkg_rua4 = get_package_share_directory('rua4')

    mode_arg = DeclareLaunchArgument(
        'mode', default_value='sim',
        choices=['sim', 'slam', 'localization', 'nav2'],
        description='Chế độ chạy: sim (chỉ mô phỏng), slam (xây map), localization (định vị), nav2 (tự hành)')
    world_arg = DeclareLaunchArgument(
        'world', default_value='warehouse',
        description='Môi trường Gazebo (warehouse, maze, depot)')
    map_arg = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([pkg_rua4, 'maps', 'map.yaml']),
        description='File map.yaml (dùng cho localization và nav2)')
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='true',
        choices=['true', 'false'],
        description='Khởi chạy RViz2')

    launch_sim_arg = DeclareLaunchArgument(
        'launch_sim', default_value='true',
        choices=['true', 'false'],
        description='Tự động khởi chạy Gazebo nếu chưa chạy (true), hoặc kết nối Gazebo đang chạy sẵn (false)')

    # Mode 1: sim
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_rua4, 'launch', 'sim.launch.py'])
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'rviz': LaunchConfiguration('rviz')
        }.items(),
        condition=LaunchConfigurationEquals('mode', 'sim')
    )

    # Mode 2: slam
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_rua4, 'launch', 'slam.launch.py'])
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'launch_sim': LaunchConfiguration('launch_sim'),
            'rviz': LaunchConfiguration('rviz')
        }.items(),
        condition=LaunchConfigurationEquals('mode', 'slam')
    )

    # Mode 3: localization
    loc_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_rua4, 'launch', 'localization.launch.py'])
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'map': LaunchConfiguration('map'),
            'launch_sim': LaunchConfiguration('launch_sim'),
            'rviz': LaunchConfiguration('rviz')
        }.items(),
        condition=LaunchConfigurationEquals('mode', 'localization')
    )

    # Mode 4: nav2
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_rua4, 'launch', 'nav2.launch.py'])
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'map': LaunchConfiguration('map'),
            'launch_sim': LaunchConfiguration('launch_sim'),
            'rviz': LaunchConfiguration('rviz')
        }.items(),
        condition=LaunchConfigurationEquals('mode', 'nav2')
    )

    ld = LaunchDescription()
    ld.add_action(rmw_env)
    ld.add_action(mode_arg)
    ld.add_action(world_arg)
    ld.add_action(map_arg)
    ld.add_action(rviz_arg)
    ld.add_action(launch_sim_arg)

    ld.add_action(sim_launch)
    ld.add_action(slam_launch)
    ld.add_action(loc_launch)
    ld.add_action(nav2_launch)

    return ld
