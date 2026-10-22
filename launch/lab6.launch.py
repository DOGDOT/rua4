from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file chuyên biệt cho Bài tập 06 (lab6.launch.py).
    Khởi chạy đồng thời 2 node của hệ thống Giao tiếp Phi Ngôn ngữ & Điều khiển Cử chỉ Tay:
      1. gesture_recognition_node (Đọc webcam bằng OpenCV, nhận diện 06 cử chỉ bằng MediaPipe,
         publish /detected_gesture)
      2. robot_behavior_node (Subscribe /detected_gesture, điều khiển hành vi TurtleBot 4
         qua /cmd_vel)
    Lưu ý: Khởi chạy Gazebo trước ở Terminal 1 bằng lệnh:
      ros2 launch rua4 sim.launch.py world:=warehouse
    """
    pkg_rua4 = get_package_share_directory('rua4')

    rmw_env = SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp')
    qt_qpa_env = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')

    camera_id_arg = DeclareLaunchArgument(
        'camera_id',
        default_value='0',
        description='Chỉ số thiết bị webcam laptop/USB (mặc định: 0 -> /dev/video0)',
    )
    gesture_topic_arg = DeclareLaunchArgument(
        'gesture_topic',
        default_value='/detected_gesture',
        description='Topic giao tiếp kết quả nhận diện cử chỉ tay (std_msgs/msg/String)',
    )
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/cmd_vel',
        description='Topic điều khiển vận tốc TurtleBot 4 (geometry_msgs/msg/Twist)',
    )
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value=PathJoinSubstitution([pkg_rua4, 'config', 'gesture_recognizer.task']),
        description='Đường dẫn file trọng số MediaPipe Gesture Recognizer',
    )
    conf_threshold_arg = DeclareLaunchArgument(
        'conf_threshold',
        default_value='0.55',
        description='Ngưỡng độ tin cậy tối thiểu để chấp nhận cử chỉ [0.0 - 1.0]',
    )
    require_confirm_for_wave_arg = DeclareLaunchArgument(
        'require_confirm_for_wave',
        default_value='true',
        choices=['true', 'false'],
        description='Yêu cầu xác nhận Thumb_Up cho cử chỉ Wave (true) hoặc thực hiện ngay (false)',
    )
    show_window_arg = DeclareLaunchArgument(
        'show_window',
        default_value='true',
        choices=['true', 'false'],
        description='Hiển thị cửa sổ webcam OpenCV',
    )

    gesture_node = Node(
        package='rua4',
        executable='gesture_recognition_node',
        name='gesture_recognition_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'camera_id': LaunchConfiguration('camera_id'),
            'gesture_topic': LaunchConfiguration('gesture_topic'),
            'model_path': LaunchConfiguration('model_path'),
            'conf_threshold': LaunchConfiguration('conf_threshold'),
            'show_window': LaunchConfiguration('show_window'),
        }],
        additional_env={
            'QT_QPA_PLATFORM': 'xcb',
            'WAYLAND_DISPLAY': '',
        },
    )

    behavior_node = Node(
        package='rua4',
        executable='robot_behavior_node',
        name='robot_behavior_node',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'gesture_topic': LaunchConfiguration('gesture_topic'),
            'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
            'require_confirm_for_wave': LaunchConfiguration('require_confirm_for_wave'),
        }],
    )

    ld = LaunchDescription()
    ld.add_action(rmw_env)
    ld.add_action(qt_qpa_env)
    ld.add_action(camera_id_arg)
    ld.add_action(gesture_topic_arg)
    ld.add_action(cmd_vel_topic_arg)
    ld.add_action(model_path_arg)
    ld.add_action(conf_threshold_arg)
    ld.add_action(require_confirm_for_wave_arg)
    ld.add_action(show_window_arg)
    ld.add_action(gesture_node)
    ld.add_action(behavior_node)

    return ld
