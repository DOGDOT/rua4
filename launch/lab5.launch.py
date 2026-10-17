import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file chuyên biệt cho Bài thực hành 05 (lab5.launch.py).
    CHỈ khởi chạy các node ứng dụng của Bài 05:
      1. yolo_detector (nhận diện vật thể thời gian thực từ camera RGB)
      2. nav_commander (điều hướng tự động qua chuỗi >= 3 Goal)
    Lưu ý: Không khởi chạy lại Gazebo, Map Server, AMCL hay Nav2 để tránh trùng lặp node.
    """
    pkg_rua4 = get_package_share_directory('rua4')

    rmw_env = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    qt_qpa_env = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')

    # Các tham số cấu hình cho Node YOLO
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/oakd/rgb/preview/image_raw',
        description='Topic ảnh RGB từ camera TurtleBot 4'
    )
    annotated_topic_arg = DeclareLaunchArgument(
        'annotated_topic',
        default_value='/yolo/annotated_image',
        description='Topic publish ảnh đã vẽ bounding box YOLO để xem trên RViz2'
    )
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='yolov8n.pt',
        description='Đường dẫn hoặc tên trọng số YOLO (yolov8n.pt, yolo11n.pt...)'
    )
    conf_threshold_arg = DeclareLaunchArgument(
        'conf_threshold',
        default_value='0.45',
        description='Ngưỡng độ tin cậy (confidence threshold) của YOLO'
    )
    show_window_arg = DeclareLaunchArgument(
        'show_window',
        default_value='true',
        choices=['true', 'false'],
        description='Hiển thị cửa sổ OpenCV (cv2.imshow)'
    )

    # Các tham số cấu hình cho Node Điều hướng tự động (nav_commander)
    run_nav_arg = DeclareLaunchArgument(
        'run_nav',
        default_value='true',
        choices=['true', 'false'],
        description='Tự động chạy luôn node nav_commander (true) hay chỉ chạy yolo_detector (false)'
    )
    preset_arg = DeclareLaunchArgument(
        'preset',
        default_value='warehouse',
        choices=['warehouse', 'maze', 'custom'],
        description='Preset danh sách Goal trong config/lab5_goals.yaml'
    )
    goals_file_arg = DeclareLaunchArgument(
        'goals_file',
        default_value=PathJoinSubstitution([pkg_rua4, 'config', 'lab5_goals.yaml']),
        description='Đường dẫn file YAML chứa danh sách Goal'
    )
    goal_timeout_arg = DeclareLaunchArgument(
        'goal_timeout_sec',
        default_value='120.0',
        description='Thời gian tối đa (giây) cho mỗi Goal'
    )
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='Sử dụng đồng hồ mô phỏng Gazebo (use_sim_time)'
    )

    # 1. Node nhận diện vật thể YOLO
    yolo_node = Node(
        package='rua4',
        executable='yolo_detector',
        name='rua4_yolo_detector',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'image_topic': LaunchConfiguration('image_topic'),
            'annotated_topic': LaunchConfiguration('annotated_topic'),
            'model_path': LaunchConfiguration('model_path'),
            'conf_threshold': LaunchConfiguration('conf_threshold'),
            'show_window': LaunchConfiguration('show_window'),
        }],
        additional_env={
            'QT_QPA_PLATFORM': 'xcb',
            'WAYLAND_DISPLAY': '',
        }
    )

    # 2. Node điều hướng tự động qua chuỗi Goal (khởi chạy sau 3 giây để YOLO sẵn sàng trước)
    nav_commander_node = Node(
        package='rua4',
        executable='nav_commander',
        name='rua4_nav_commander',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'preset': LaunchConfiguration('preset'),
            'goals_file': LaunchConfiguration('goals_file'),
            'goal_timeout_sec': LaunchConfiguration('goal_timeout_sec'),
        }],
        condition=IfCondition(LaunchConfiguration('run_nav'))
    )

    delayed_nav_commander = TimerAction(
        period=3.0,
        actions=[nav_commander_node]
    )

    ld = LaunchDescription()
    ld.add_action(rmw_env)
    ld.add_action(qt_qpa_env)
    ld.add_action(image_topic_arg)
    ld.add_action(annotated_topic_arg)
    ld.add_action(model_path_arg)
    ld.add_action(conf_threshold_arg)
    ld.add_action(show_window_arg)
    ld.add_action(run_nav_arg)
    ld.add_action(preset_arg)
    ld.add_action(goals_file_arg)
    ld.add_action(goal_timeout_arg)
    ld.add_action(use_sim_time_arg)

    ld.add_action(yolo_node)
    ld.add_action(delayed_nav_commander)

    return ld
