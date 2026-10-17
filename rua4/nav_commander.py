#!/usr/bin/env python3
"""
Node điều hướng tự động qua chuỗi mục tiêu (Nav2 Multi-Goal Commander) cho TurtleBot 4.
Thực hiện Yêu cầu 2 - Bài thực hành 05:
  - Sử dụng nav2_simple_commander (Action /navigate_to_pose) trong hệ tọa độ 'map'.
  - Đọc danh sách các Goal (x, y, yaw) từ file cấu hình config/lab5_goals.yaml
    hoặc theo preset ('warehouse', 'maze', 'custom') / tham số dòng lệnh.
  - Kiểm tra trạng thái Nav2 Active và cảnh báo nếu có node khác phát xung đột /cmd_vel.
  - Xử lý đầy đủ timeout, thất bại, từ chối goal, mất kết nối Action Server.
  - In bảng tổng kết trạng thái từng Goal rõ ràng trên Terminal.
"""

import os
import sys
import math
import time
import argparse
from typing import List, Tuple, Dict, Any

import rclpy
from rclpy.parameter import Parameter
from rclpy.utilities import remove_ros_args
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError

try:
    import yaml
except ImportError:
    yaml = None


# Tập điểm dự phòng (fallback) nếu không đọc được file YAML
FALLBACK_PRESETS: Dict[str, List[Dict[str, Any]]] = {
    'warehouse': [
        {'name': 'Goal 1 - Hành lang kho A', 'x': 2.0, 'y': 0.0, 'yaw': 0.0},
        {'name': 'Goal 2 - Gian hàng B', 'x': 3.5, 'y': 2.0, 'yaw': 1.57},
        {'name': 'Goal 3 - Điểm trung chuyển', 'x': 1.0, 'y': 2.5, 'yaw': 3.14},
        {'name': 'Goal 4 - Trở về vị trí xuất phát', 'x': 0.0, 'y': 0.0, 'yaw': 0.0},
    ],
    'maze': [
        {'name': 'Goal 1 - Ngã rẽ thứ nhất', 'x': 1.5, 'y': 0.0, 'yaw': 0.0},
        {'name': 'Goal 2 - Hành lang mê cung', 'x': 1.5, 'y': 1.5, 'yaw': 1.57},
        {'name': 'Goal 3 - Góc trên mê cung', 'x': 1.5, 'y': 3.0, 'yaw': 3.14},
        {'name': 'Goal 4 - Trở về vạch xuất phát', 'x': 0.0, 'y': 0.0, 'yaw': 0.0},
    ],
    'custom': [
        {'name': 'Goal 1 - Điểm kiểm tra số 1', 'x': 2.0, 'y': 0.0, 'yaw': 0.0},
        {'name': 'Goal 2 - Điểm kiểm tra số 2', 'x': 3.5, 'y': 2.0, 'yaw': 1.57},
        {'name': 'Goal 3 - Điểm kiểm tra số 3', 'x': 1.0, 'y': 2.5, 'yaw': 3.14},
    ],
}


def resolve_default_goals_file() -> str:
    """Tìm đường dẫn đến config/lab5_goals.yaml (ưu tiên thư mục workspace src nếu tồn tại)."""
    ws_candidate = '/home/gdon/ur_ws/src/rua4/config/lab5_goals.yaml'
    if os.path.isfile(ws_candidate):
        return ws_candidate
    local_candidate = os.path.abspath(
        os.path.join(os.path.dirname(__file__), '..', 'config', 'lab5_goals.yaml')
    )
    if os.path.isfile(local_candidate):
        return local_candidate
    try:
        pkg_share = get_package_share_directory('rua4')
        share_candidate = os.path.join(pkg_share, 'config', 'lab5_goals.yaml')
        if os.path.isfile(share_candidate):
            return share_candidate
    except PackageNotFoundError:
        pass
    return ws_candidate


def _parse_simple_goals_yaml(yaml_path: str) -> Dict[str, Any]:
    """Bộ đọc YAML dự phòng thuần Python cho config/lab5_goals.yaml khi thiếu PyYAML."""
    result: Dict[str, Any] = {
        'frame_id': 'map',
        'use_sim_time': True,
        'default_preset': 'warehouse',
        'goal_timeout_sec': 120.0,
        'wait_between_goals_sec': 2.0,
        'stop_on_failure': False,
        'set_initial_pose': False,
        'initial_pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        'presets': {},
    }
    current_section = None
    current_preset = None
    current_item: Dict[str, Any] = {}

    with open(yaml_path, 'r', encoding='utf-8') as f:
        for raw_line in f:
            line = raw_line.split('#', 1)[0].rstrip()
            if not line.strip():
                continue
            indent = len(line) - len(line.lstrip(' '))
            stripped = line.strip()

            if indent == 0 and ':' in stripped:
                key, val = [p.strip() for p in stripped.split(':', 1)]
                val = val.strip('"').strip("'")
                if key in ('presets', 'initial_pose'):
                    current_section = key
                    current_preset = None
                else:
                    current_section = None
                    if val.lower() in ('true', 'false'):
                        result[key] = (val.lower() == 'true')
                    else:
                        try:
                            result[key] = float(val) if '.' in val else int(val)
                        except ValueError:
                            result[key] = val
            elif current_section == 'initial_pose' and indent >= 2 and ':' in stripped:
                key, val = [p.strip() for p in stripped.split(':', 1)]
                result['initial_pose'][key] = float(val)
            elif current_section == 'presets':
                if indent == 2 and stripped.endswith(':'):
                    current_preset = stripped[:-1].strip()
                    result['presets'][current_preset] = []
                elif indent >= 4 and current_preset:
                    if stripped.startswith('- '):
                        current_item = {}
                        result['presets'][current_preset].append(current_item)
                        stripped = stripped[2:].strip()
                    if ':' in stripped:
                        k, v = [p.strip() for p in stripped.split(':', 1)]
                        v = v.strip('"').strip("'")
                        if k in ('x', 'y', 'yaw'):
                            current_item[k] = float(v)
                        else:
                            current_item[k] = v

    if not result['presets']:
        result['presets'] = FALLBACK_PRESETS
    return result


def load_goals_config(yaml_path: str) -> Dict[str, Any]:
    """Đọc file cấu hình YAML chứa danh sách Goal và các thiết lập điều hướng."""
    if yaml_path and os.path.isfile(yaml_path):
        if yaml is not None:
            try:
                with open(yaml_path, 'r', encoding='utf-8') as f:
                    data = yaml.safe_load(f)
                if isinstance(data, dict):
                    return data
            except Exception as exc:
                print(f"[CẢNH BÁO] Lỗi đọc YAML bằng PyYAML ('{yaml_path}'): {exc}")
        try:
            return _parse_simple_goals_yaml(yaml_path)
        except Exception as exc:
            print(f"[CẢNH BÁO] Không thể đọc file cấu hình '{yaml_path}': {exc}")
    return {
        'frame_id': 'map',
        'use_sim_time': True,
        'default_preset': 'warehouse',
        'goal_timeout_sec': 120.0,
        'wait_between_goals_sec': 2.0,
        'stop_on_failure': False,
        'set_initial_pose': False,
        'initial_pose': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
        'presets': FALLBACK_PRESETS,
    }


def create_pose(
    navigator: BasicNavigator,
    x: float,
    y: float,
    yaw: float = 0.0,
    frame_id: str = 'map'
) -> PoseStamped:
    """Tạo PoseStamped trong hệ tọa độ 'map' từ (x, y, yaw) với biến đổi Quaternion chuẩn."""
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0

    # Chuyển đổi góc quay yaw (rad) quanh trục Z thành Quaternion (x=0, y=0, z, w)
    pose.pose.orientation.x = 0.0
    pose.pose.orientation.y = 0.0
    pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
    pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
    return pose


def parse_arguments(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Điều hướng tự động TurtleBot 4 qua chuỗi Goal (Nav2 Simple Commander)'
    )
    parser.add_argument(
        '--preset',
        choices=['warehouse', 'maze', 'custom'],
        default=None,
        help='Tập mục tiêu mẫu trong config/lab5_goals.yaml (mặc định: warehouse)'
    )
    parser.add_argument(
        '--goals-file',
        type=str,
        default=None,
        help='Đường dẫn tới file YAML chứa danh sách Goal (mặc định: config/lab5_goals.yaml)'
    )
    parser.add_argument('--x', type=float, default=None, help='Tọa độ X nếu gửi 1 mục tiêu lẻ')
    parser.add_argument('--y', type=float, default=None, help='Tọa độ Y nếu gửi 1 mục tiêu lẻ')
    parser.add_argument('--yaw', type=float, default=0.0, help='Góc hướng mục tiêu (radians)')
    parser.add_argument(
        '--timeout',
        type=float,
        default=None,
        help='Thời gian tối đa (giây) cho mỗi Goal trước khi báo timeout (mặc định: 120s)'
    )
    parser.add_argument(
        '--wait',
        type=float,
        default=None,
        help='Thời gian nghỉ (giây) giữa hai Goal liên tiếp (mặc định: 2.0s)'
    )
    parser.add_argument(
        '--num-goals',
        type=int,
        default=None,
        help='Số lượng Goal muốn chạy từ danh sách (mặc định: chạy toàn bộ danh sách >= 3 Goal)'
    )
    parser.add_argument(
        '--stop-on-failure',
        action='store_true',
        default=False,
        help='Dừng ngay chuỗi điều hướng nếu có 1 Goal thất bại'
    )
    parser.add_argument(
        '--set-initial-pose',
        action='store_true',
        default=False,
        help='Gửi initial pose (x=0, y=0, yaw=0) cho AMCL trước khi điều hướng'
    )
    parser.add_argument(
        '--shutdown-nav2',
        action='store_true',
        default=False,
        help='Tắt toàn bộ Lifecycle của Nav2 sau khi kết thúc (mặc định: False để giữ Nav2 hoạt động)'
    )
    clean_args = remove_ros_args(args=argv)[1:]
    parsed, _ = parser.parse_known_args(clean_args)
    return parsed


def main(args=None):
    cli_args = parse_arguments(sys.argv if args is None else args)

    rclpy.init(args=args)
    navigator = BasicNavigator(node_name='rua4_nav_commander')

    # Khai báo ROS 2 parameters để hỗ trợ cấu hình qua launch file
    default_yaml = resolve_default_goals_file()
    navigator.declare_parameter('goals_file', default_yaml)
    navigator.declare_parameter('preset', 'warehouse')
    navigator.declare_parameter('goal_timeout_sec', 120.0)
    navigator.declare_parameter('wait_between_goals_sec', 2.0)
    navigator.declare_parameter('stop_on_failure', False)
    navigator.declare_parameter('set_initial_pose', False)

    yaml_path = cli_args.goals_file or navigator.get_parameter('goals_file').value
    cfg = load_goals_config(yaml_path)

    use_sim_time = bool(cfg.get('use_sim_time', True))
    navigator.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, use_sim_time)])

    frame_id = str(cfg.get('frame_id', 'map'))
    preset_name = (
        cli_args.preset
        or navigator.get_parameter('preset').value
        or str(cfg.get('default_preset', 'warehouse'))
    )
    goal_timeout_sec = (
        cli_args.timeout
        if cli_args.timeout is not None
        else float(cfg.get('goal_timeout_sec', navigator.get_parameter('goal_timeout_sec').value))
    )
    wait_between_sec = (
        cli_args.wait
        if cli_args.wait is not None
        else float(cfg.get('wait_between_goals_sec', navigator.get_parameter('wait_between_goals_sec').value))
    )
    stop_on_failure = (
        cli_args.stop_on_failure
        or bool(navigator.get_parameter('stop_on_failure').value)
        or bool(cfg.get('stop_on_failure', False))
    )
    do_set_init_pose = (
        cli_args.set_initial_pose
        or bool(navigator.get_parameter('set_initial_pose').value)
        or bool(cfg.get('set_initial_pose', False))
    )

    # Xây dựng danh sách Goal: (name, x, y, yaw)
    goals: List[Tuple[str, float, float, float]] = []
    if cli_args.x is not None and cli_args.y is not None:
        goals.append(("Mục tiêu tùy chỉnh (CLI)", float(cli_args.x), float(cli_args.y), float(cli_args.yaw)))
    else:
        presets_dict = cfg.get('presets', FALLBACK_PRESETS)
        raw_list = presets_dict.get(preset_name, FALLBACK_PRESETS.get(preset_name, FALLBACK_PRESETS['warehouse']))
        for idx, item in enumerate(raw_list, 1):
            g_name = str(item.get('name', f'Goal {idx}'))
            g_x = float(item.get('x', 0.0))
            g_y = float(item.get('y', 0.0))
            g_yaw = float(item.get('yaw', 0.0))
            goals.append((g_name, g_x, g_y, g_yaw))

    if cli_args.num_goals is not None and cli_args.num_goals > 0:
        goals = goals[:cli_args.num_goals]

    print("=" * 72)
    print("   [rua4] BỘ ĐIỀU KHIỂN TỰ ĐỘNG NAV2 (AUTONOMOUS MULTI-GOAL NAVIGATION)")
    print("=" * 72)
    print(f"  + File cấu hình Goal : {yaml_path if os.path.isfile(yaml_path) else 'Built-in Fallback'}")
    print(f"  + Preset đang dùng   : {preset_name} ({len(goals)} mục tiêu)")
    print(f"  + Hệ tọa độ (Frame)  : {frame_id}")
    print(f"  + Timeout mỗi Goal   : {goal_timeout_sec:.1f} giây")
    print(f"  + Chế độ dừng khi lỗi: {'Có' if stop_on_failure else 'Không (báo lỗi và chuyển Goal kế tiếp)'}")
    print("-" * 72)

    if not goals:
        print("[LỖI] Danh sách Goal trống! Vui lòng kiểm tra config/lab5_goals.yaml.")
        navigator.destroy_node()
        rclpy.shutdown()
        return

    # Tùy chọn gửi Initial Pose trước khi chờ Nav2 active
    if do_set_init_pose:
        init_cfg = cfg.get('initial_pose', {'x': 0.0, 'y': 0.0, 'yaw': 0.0})
        ix = float(init_cfg.get('x', 0.0))
        iy = float(init_cfg.get('y', 0.0))
        iyaw = float(init_cfg.get('yaw', 0.0))
        print(f"Đang thiết lập Initial Pose cho AMCL tại x={ix:.2f}, y={iy:.2f}, yaw={iyaw:.2f} rad...")
        init_pose = create_pose(navigator, ix, iy, iyaw, frame_id=frame_id)
        navigator.setInitialPose(init_pose)

    print("Đang kiểm tra trạng thái Nav2 stack (chờ amcl & bt_navigator active)...")
    try:
        navigator.waitUntilNav2Active()
    except Exception as exc:
        print(f"[LỖI NGHIÊM TRỌNG] Không thể kết nối hoặc kích hoạt Nav2 stack: {exc}")
        navigator.destroy_node()
        rclpy.shutdown()
        return

    print("[THÀNH CÔNG] Hệ thống Localization (AMCL) và Nav2 đã sẵn sàng!")

    # Kiểm tra xem có node teleop nào đang phát lệnh cạnh tranh trên /cmd_vel hay không
    cmd_vel_pubs = navigator.count_publishers('/cmd_vel')
    if cmd_vel_pubs > 2:
        print(
            f"[CẢNH BÁO] Phát hiện {cmd_vel_pubs} publishers trên topic /cmd_vel! "
            "Hãy đảm bảo bạn ĐÃ TẮT teleop_twist_keyboard để không tranh chấp vận tốc với Nav2."
        )

    summary_records = []

    try:
        for idx, (name, gx, gy, gyaw) in enumerate(goals, 1):
            deg_yaw = math.degrees(gyaw)
            qz = math.sin(gyaw / 2.0)
            qw = math.cos(gyaw / 2.0)

            print("\n" + "-" * 72)
            print(f"[GOAL {idx}/{len(goals)}] BẮT ĐẦU DI CHUYỂN ĐẾN: {name}")
            print(
                f"  + Tọa độ (frame '{frame_id}'): x = {gx:.2f} m, y = {gy:.2f} m, "
                f"yaw = {gyaw:.2f} rad ({deg_yaw:.1f} deg)"
            )
            print(f"  + Quaternion tương ứng      : qz = {qz:.4f}, qw = {qw:.4f}")
            print("-" * 72)

            target_pose = create_pose(navigator, gx, gy, gyaw, frame_id=frame_id)
            start_wall_t = time.time()

            try:
                accepted = navigator.goToPose(target_pose)
            except Exception as exc:
                print(f"[LỖI ACTION SERVER] Mất kết nối hoặc lỗi khi gửi Goal '{name}': {exc}")
                summary_records.append((idx, name, gx, gy, gyaw, 0.0, "ACTION_ERROR"))
                if stop_on_failure:
                    print("[DỪNG CHUỖI] Dừng điều hướng do cờ --stop-on-failure được bật.")
                    break
                continue

            if accepted is False:
                print(
                    f"[BỊ TỪ CHỐI] Action Server '/navigate_to_pose' từ chối Goal '{name}'! "
                    "Kiểm tra xem tọa độ có nằm ngoài bản đồ hoặc trùng vật cản không."
                )
                summary_records.append((idx, name, gx, gy, gyaw, 0.0, "REJECTED"))
                if stop_on_failure:
                    print("[DỪNG CHUỖI] Dừng điều hướng do cờ --stop-on-failure được bật.")
                    break
                continue

            step_counter = 0
            timed_out = False
            while not navigator.isTaskComplete():
                step_counter += 1
                elapsed = time.time() - start_wall_t

                if elapsed > goal_timeout_sec:
                    print(
                        f"  [TIMEOUT] Quá thời gian cho phép ({goal_timeout_sec:.1f}s) tại '{name}'! "
                        "Đang gửi lệnh hủy mục tiêu hiện tại..."
                    )
                    navigator.cancelTask()
                    timed_out = True
                    break

                feedback = navigator.getFeedback()
                if feedback and step_counter % 5 == 0:
                    dist = feedback.distance_remaining
                    recoveries = getattr(feedback, 'number_of_recoveries', 0)
                    print(
                        f"  -> Đang di chuyển tới Goal {idx}/{len(goals)} | "
                        f"Còn lại: {dist:.2f} m | Thời gian: {elapsed:.1f}s | "
                        f"Recovery: {recoveries}"
                    )
                time.sleep(0.5)

            elapsed_total = time.time() - start_wall_t
            result = navigator.getResult()

            if timed_out:
                status_str = "TIMEOUT"
                print(f"[KẾT QUẢ GOAL {idx}] THẤT BẠI DO TIMEOUT ({elapsed_total:.1f}s): {name}")
            elif result == TaskResult.SUCCEEDED:
                status_str = "SUCCEEDED"
                print(f"[KẾT QUẢ GOAL {idx}] HOÀN THÀNH XUẤT SẮC ({elapsed_total:.1f}s): {name}!")
            elif result == TaskResult.CANCELED:
                status_str = "CANCELED"
                print(f"[KẾT QUẢ GOAL {idx}] ĐÃ BỊ HỦY ({elapsed_total:.1f}s): {name}.")
            elif result == TaskResult.FAILED:
                status_str = "FAILED"
                print(
                    f"[KẾT QUẢ GOAL {idx}] THẤT BẠI ({elapsed_total:.1f}s): Không thể tới '{name}' "
                    "(Đường đi bị chặn hoặc vượt quá số lần Recovery)."
                )
            else:
                status_str = "UNKNOWN"
                print(f"[KẾT QUẢ GOAL {idx}] TRẠNG THÁI KHÔNG XÁC ĐỊNH ({elapsed_total:.1f}s): {name}.")

            summary_records.append((idx, name, gx, gy, gyaw, elapsed_total, status_str))

            if status_str != "SUCCEEDED" and stop_on_failure:
                print("[DỪNG CHUỖI] Dừng điều hướng sớm do phát hiện lỗi và cờ stop_on_failure=True.")
                break

            if idx < len(goals) and wait_between_sec > 0:
                print(f"  .. Nghỉ {wait_between_sec:.1f}s tại chỗ trước khi chuyển sang Goal tiếp theo ..")
                time.sleep(wait_between_sec)

    except KeyboardInterrupt:
        print("\n[NGẮT BỞI NGƯỜI DÙNG] Đang hủy tác vụ điều hướng hiện tại...")
        try:
            navigator.cancelTask()
        except Exception:
            pass

    # Bảng tổng kết kết quả
    succeeded_count = sum(1 for r in summary_records if r[6] == "SUCCEEDED")
    print("\n" + "=" * 72)
    print("   BẢNG TỔNG KẾT KẾT QUẢ ĐIỀU HƯỚNG TỰ ĐỘNG (BÀI THỰC HÀNH 05)")
    print("=" * 72)
    for idx, name, gx, gy, gyaw, duration, status_str in summary_records:
        print(
            f"  Goal {idx}: {name:<32} | "
            f"({gx:5.2f}, {gy:5.2f}, yaw={gyaw:4.2f}) | "
            f"{duration:5.1f}s | [{status_str}]"
        )
    print("-" * 72)
    print(f"  Tổng số Goal hoàn thành: {succeeded_count}/{len(goals)}")
    print("=" * 72 + "\n")

    if cli_args.shutdown_nav2:
        navigator.lifecycleShutdown()
    else:
        navigator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
