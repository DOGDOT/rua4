#!/usr/bin/env python3
"""
ROS 2 Node: robot_behavior_node (Bài tập 06 - Package rua4)
Điều khiển hành vi TurtleBot 4 trong Gazebo dựa trên cử chỉ tay nhận diện từ AI.

Bảng ánh xạ cử chỉ tay -> Hành vi của TurtleBot 4:
  1. Vẫy tay (Wave)               : Chọn hành vi xoay trái–phải để chào lại
  2. Giơ 1 ngón tay (Pointing_Up) : Chọn hành vi tiến–lùi 4 lần
  3. Giơ 2 ngón tay (Victory)     : Chọn hành vi di chuyển theo quỹ đạo tròn
  4. Giơ ngón cái (Thumb_Up)      : Xác nhận và thực hiện hành vi đã chọn
  5. Xòe bàn tay (Open_Palm)      : Robot dừng ngay lập tức
  6. Nắm tay (Closed_Fist)        : Hủy hành vi hiện tại và trở về trạng thái chờ

Đặc tính an toàn & chống nhiễu:
  - Robot chỉ thực hiện hành vi sau khi nhận được cử chỉ xác nhận (Thumb_Up).
  - Cơ chế chống nhiễu 2 tầng: Đếm số bản tin liên tiếp ổn định + Khóa sườn lên (Edge-Trigger Latch)
    + Thời gian chờ chống dội (Cooldown Timer) tránh kích hoạt lệnh liên tục.
  - Tự động dừng khẩn cấp khi nhận cử chỉ Open_Palm, hủy về trạng thái chờ khi nhận Closed_Fist,
    hoặc dừng an toàn khi mất tín hiệu điều khiển quá ngưỡng watchdog_timeout_sec.
  - Giám sát cảm biến LiDAR (/scan) để ngăn ngừa va chạm trong quá trình thực hiện hành vi.
"""

import argparse
import math
from typing import Dict, List, Optional
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


# Chuẩn hóa tên cử chỉ nhận được từ topic /detected_gesture
GESTURE_ALIASES: Dict[str, str] = {
    'wave': 'Wave',
    'vay_tay': 'Wave',
    'iloveyou': 'Wave',
    'pointing_up': 'Pointing_Up',
    'pointing up': 'Pointing_Up',
    '1_finger': 'Pointing_Up',
    'victory': 'Victory',
    'peace': 'Victory',
    '2_fingers': 'Victory',
    'thumb_up': 'Thumb_Up',
    'thumbs_up': 'Thumb_Up',
    'thumbs up': 'Thumb_Up',
    'confirm': 'Thumb_Up',
    'open_palm': 'Open_Palm',
    'open palm': 'Open_Palm',
    'stop': 'Open_Palm',
    'closed_fist': 'Closed_Fist',
    'closed fist': 'Closed_Fist',
    'cancel': 'Closed_Fist',
    'none': 'None',
    'unknown': 'None',
    '': 'None',
}

BEHAVIOR_DESCRIPTIONS: Dict[str, str] = {
    'Wave': 'Chao lai (Xoay trai - phai)',
    'Pointing_Up': 'Tien - Lui 4 lan',
    'Victory': 'Di chuyen quy dao tron',
}


class RobotBehaviorNode(Node):
    """Node ROS 2 nhận cử chỉ từ /detected_gesture và điều khiển TurtleBot 4 qua /cmd_vel."""

    STATE_IDLE = 'IDLE'
    STATE_SELECTED = 'SELECTED'
    STATE_EXECUTING = 'EXECUTING'
    STATE_STOPPED = 'STOPPED'

    def __init__(self, cli_args: argparse.Namespace):
        super().__init__('robot_behavior_node')

        self.declare_parameter('gesture_topic', cli_args.gesture_topic)
        self.declare_parameter('cmd_vel_topic', cli_args.cmd_vel_topic)
        self.declare_parameter('status_topic', cli_args.status_topic)
        self.declare_parameter('require_confirm_for_wave', not cli_args.wave_immediate)
        self.declare_parameter('confirm_frames', cli_args.confirm_frames)
        self.declare_parameter('cooldown_sec', cli_args.cooldown_sec)
        self.declare_parameter('watchdog_timeout_sec', cli_args.watchdog_timeout_sec)
        self.declare_parameter('linear_speed', cli_args.linear_speed)
        self.declare_parameter('angular_speed', cli_args.angular_speed)
        self.declare_parameter('circle_linear_speed', cli_args.circle_linear_speed)
        self.declare_parameter('circle_angular_speed', cli_args.circle_angular_speed)
        self.declare_parameter('safe_obstacle_dist', cli_args.safe_obstacle_dist)

        self.gesture_topic: str = str(self.get_parameter('gesture_topic').value)
        self.cmd_vel_topic: str = str(self.get_parameter('cmd_vel_topic').value)
        self.status_topic: str = str(self.get_parameter('status_topic').value)
        self.require_confirm_for_wave: bool = bool(
            self.get_parameter('require_confirm_for_wave').value
        )
        self.confirm_frames: int = max(1, int(self.get_parameter('confirm_frames').value))
        self.cooldown_sec: float = float(self.get_parameter('cooldown_sec').value)
        self.watchdog_timeout_sec: float = float(self.get_parameter('watchdog_timeout_sec').value)
        self.linear_speed: float = float(self.get_parameter('linear_speed').value)
        self.angular_speed: float = float(self.get_parameter('angular_speed').value)
        self.circle_linear_speed: float = float(self.get_parameter('circle_linear_speed').value)
        self.circle_angular_speed: float = float(self.get_parameter('circle_angular_speed').value)
        self.safe_obstacle_dist: float = float(self.get_parameter('safe_obstacle_dist').value)

        # Publisher /cmd_vel điều khiển vận tốc TurtleBot 4 và /robot_behavior_status báo trạng thái
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        # Subscriber /detected_gesture
        self.gesture_sub = self.create_subscription(
            String,
            self.gesture_topic,
            self._gesture_callback,
            10,
        )

        # Subscriber /scan để kiểm tra an toàn chống va chạm (tùy chọn bổ trợ)
        scan_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self.front_min_dist: float = 99.0
        self.rear_min_dist: float = 99.0
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            scan_qos,
        )

        # Trạng thái máy hữu hạn (Finite State Machine)
        self.state: str = self.STATE_IDLE
        self.selected_behavior: Optional[str] = None
        self.active_behavior: Optional[str] = None
        self.behavior_start_time: float = 0.0
        self.last_logged_cycle: int = 0

        # Biến chống nhiễu (Debounce / Edge-Trigger / Cooldown)
        self.candidate_gesture: str = 'None'
        self.candidate_count: int = 0
        self.last_triggered_gesture: str = 'None'
        self.last_trigger_time: float = 0.0
        self.last_message_time: float = time.monotonic()

        # Timer vòng lặp điều khiển 20 Hz (0.05s)
        self.control_timer = self.create_timer(0.05, self._control_loop)

        self.get_logger().info('=' * 70)
        self.get_logger().info('KHỞI CHẠY NODE ĐIỀU KHIỂN HÀNH VI TURTLEBOT 4 (robot_behavior_node)')
        self.get_logger().info(f'  - Subscribe Topic      : {self.gesture_topic} (std_msgs/msg/String)')
        self.get_logger().info(f'  - Publish Topic        : {self.cmd_vel_topic} (geometry_msgs/msg/Twist)')
        self.get_logger().info(f'  - Confirm Frames       : {self.confirm_frames} bản tin liên tiếp')
        self.get_logger().info(f'  - Cooldown Time        : {self.cooldown_sec:.2f} s')
        self.get_logger().info(f'  - Watchdog Timeout     : {self.watchdog_timeout_sec:.2f} s')
        self.get_logger().info(
            '  - Quy trình điều khiển : Chọn hành vi (Wave / Pointing_Up / Victory) '
            '-> Xác nhận bằng Thumb_Up -> Thực thi (Open_Palm: Dừng, Closed_Fist: Hủy)'
        )
        self.get_logger().info('=' * 70)
        self._publish_status('IDLE (San sang - Hay chon cu chi)')

    @staticmethod
    def _normalize_gesture(raw_str: str) -> str:
        """Chuẩn hóa chuỗi cử chỉ từ message String."""
        if not raw_str:
            return 'None'
        token = raw_str.strip().split('|')[0].split('(')[0].strip().lower()
        return GESTURE_ALIASES.get(token, 'None')

    def _scan_callback(self, msg: LaserScan) -> None:
        """Cập nhật khoảng cách vật cản phía trước và phía sau từ LiDAR /scan."""
        if not msg.ranges:
            return
        n = len(msg.ranges)
        front_vals: List[float] = []
        rear_vals: List[float] = []
        for i, r in enumerate(msg.ranges):
            if not math.isfinite(r) or r < msg.range_min or r > msg.range_max:
                continue
            angle = msg.angle_min + i * msg.angle_increment
            # Chuẩn hóa góc về [-pi, pi]
            angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(angle) <= math.radians(25.0):
                front_vals.append(r)
            elif abs(angle) >= math.radians(155.0):
                rear_vals.append(r)

        self.front_min_dist = min(front_vals) if front_vals else 99.0
        self.rear_min_dist = min(rear_vals) if rear_vals else 99.0

    def _publish_cmd_vel(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        """Phát lệnh vận tốc Twist lên /cmd_vel (kèm kiểm tra an toàn chống va chạm)."""
        if linear_x > 0.0 and self.front_min_dist < self.safe_obstacle_dist:
            self.get_logger().warn(
                f'[AN TOÀN LIDAR] Vật cản phía trước cách {self.front_min_dist:.2f}m '
                f'(< {self.safe_obstacle_dist:.2f}m) -> Ngắt tiến để tránh va chạm!',
                throttle_duration_sec=1.5,
            )
            linear_x = 0.0
        elif linear_x < 0.0 and self.rear_min_dist < self.safe_obstacle_dist:
            self.get_logger().warn(
                f'[AN TOÀN LIDAR] Vật cản phía sau cách {self.rear_min_dist:.2f}m '
                f'(< {self.safe_obstacle_dist:.2f}m) -> Ngắt lùi để tránh va chạm!',
                throttle_duration_sec=1.5,
            )
            linear_x = 0.0

        twist = Twist()
        twist.linear.x = float(linear_x)
        twist.angular.z = float(angular_z)
        self.cmd_vel_pub.publish(twist)

    def _stop_robot_immediately(self) -> None:
        """Gửi liên tiếp lệnh dừng vận tốc = 0 lên /cmd_vel."""
        twist = Twist()
        for _ in range(3):
            self.cmd_vel_pub.publish(twist)

    def _publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def _gesture_callback(self, msg: String) -> None:
        """Xử lý bản tin cử chỉ từ /detected_gesture với bộ lọc chống nhiễu."""
        now = time.monotonic()
        self.last_message_time = now
        gesture = self._normalize_gesture(msg.data)

        # Khi không có cử chỉ (None): giải phóng khóa sườn lên để cho phép nhận lại cử chỉ sau đó
        if gesture == 'None':
            self.candidate_gesture = 'None'
            self.candidate_count = 0
            self.last_triggered_gesture = 'None'
            return

        # ƯU TIÊN CAO NHẤT: Cử chỉ dừng khẩn cấp (Open_Palm) và hủy hành vi (Closed_Fist)
        # Chỉ cần 2 frame liên tiếp để phản ứng tức thì mà vẫn lọc được gai nhiễu đơn lẻ
        if gesture == self.candidate_gesture:
            self.candidate_count += 1
        else:
            self.candidate_gesture = gesture
            self.candidate_count = 1

        required_count = 2 if gesture in ('Open_Palm', 'Closed_Fist') else self.confirm_frames
        if self.candidate_count < required_count:
            return

        # Cơ chế Edge-Trigger Latch: Không kích hoạt lặp lại cùng 1 cử chỉ nếu người dùng vẫn giữ nguyên tay
        if gesture == self.last_triggered_gesture:
            return

        # Cơ chế Cooldown cho các lệnh chọn và xác nhận
        if gesture not in ('Open_Palm', 'Closed_Fist'):
            if (now - self.last_trigger_time) < self.cooldown_sec:
                return

        self.last_triggered_gesture = gesture
        self.last_trigger_time = now
        self._handle_verified_gesture(gesture, now)

    def _handle_verified_gesture(self, gesture: str, now: float) -> None:
        """Thực thi chuyển trạng thái FSM khi cử chỉ đã qua bộ lọc chống nhiễu."""
        # 1. Xòe bàn tay (Open_Palm) -> Dừng robot ngay lập tức
        if gesture == 'Open_Palm':
            self._stop_robot_immediately()
            self.active_behavior = None
            self.state = self.STATE_STOPPED
            self.get_logger().warn(
                '[DỪNG KHẨN CẤP - Open_Palm] Đã dừng TurtleBot 4 ngay lập tức!'
            )
            if self.selected_behavior:
                self._publish_status(
                    f"STOPPED (Da dung - San sang Xac nhan lai: {BEHAVIOR_DESCRIPTIONS[self.selected_behavior]})"
                )
            else:
                self._publish_status('STOPPED (Robot da dung ngay lap tuc)')
            return

        # 2. Nắm tay (Closed_Fist) -> Hủy hành vi hiện tại và trở về trạng thái chờ
        if gesture == 'Closed_Fist':
            self._stop_robot_immediately()
            prev_active = self.active_behavior or self.selected_behavior or 'Khong co'
            self.active_behavior = None
            self.selected_behavior = None
            self.state = self.STATE_IDLE
            self.get_logger().info(
                f'[HỦY HÀNH VI - Closed_Fist] Đã hủy ({prev_active}) -> Trở về trạng thái chờ (IDLE).'
            )
            self._publish_status('IDLE (Da huy hanh vi - Trang thai cho)')
            return

        # 3. Các cử chỉ chọn hành vi: Wave, Pointing_Up, Victory
        if gesture in ('Wave', 'Pointing_Up', 'Victory'):
            if self.state == self.STATE_EXECUTING:
                self.get_logger().info(
                    f'[ĐANG CHẠY] Robot đang thực hiện {self.active_behavior}. '
                    'Hãy Xòe tay (Open_Palm) hoặc Nắm tay (Closed_Fist) trước khi đổi hành vi.'
                )
                return

            # Nếu cấu hình cho phép Wave chạy ngay không cần xác nhận (--wave-immediate)
            if gesture == 'Wave' and not self.require_confirm_for_wave:
                self.selected_behavior = 'Wave'
                self.active_behavior = 'Wave'
                self.behavior_start_time = now
                self.state = self.STATE_EXECUTING
                self.get_logger().info(
                    '[THỰC HIỆN NGAY - Wave] Vẫy tay -> Robot xoay trái-phải chào lại!'
                )
                self._publish_status('EXECUTING: Chao lai (Xoay trai - phai)')
                return

            self.selected_behavior = gesture
            self.state = self.STATE_SELECTED
            desc = BEHAVIOR_DESCRIPTIONS[gesture]
            self.get_logger().info(
                f'[ĐÃ CHỌN HÀNH VI] Cử chỉ: {gesture} -> "{desc}". '
                'Hãy giơ NGÓN CÁI (Thumb_Up) để XÁC NHẬN và thực hiện!'
            )
            self._publish_status(f'SELECTED: {desc} (Cho Thumb_Up xac nhan)')
            return

        # 4. Giơ ngón cái (Thumb_Up) -> Xác nhận và thực hiện hành vi đã chọn
        if gesture == 'Thumb_Up':
            if self.state == self.STATE_EXECUTING:
                self.get_logger().info(
                    f'[ĐANG THỰC HIỆN] Hành vi {self.active_behavior} đang chạy.'
                )
                return

            if self.selected_behavior is None:
                self.get_logger().warn(
                    '[CHƯA CHỌN HÀNH VI] Nhận được Thumb_Up nhưng chưa có hành vi nào được chọn! '
                    'Hãy thực hiện Wave (Vẫy tay), Pointing_Up (1 ngón) hoặc Victory (2 ngón) trước.'
                )
                self._publish_status('IDLE (Hay chon Wave / 1 Ngon / 2 Ngon truoc!)')
                return

            self.active_behavior = self.selected_behavior
            self.behavior_start_time = now
            self.last_logged_cycle = 0
            self.state = self.STATE_EXECUTING
            desc = BEHAVIOR_DESCRIPTIONS[self.active_behavior]
            self.get_logger().info(
                f'[XÁC NHẬN - Thumb_Up] Bắt đầu thực hiện hành vi: {self.active_behavior} ({desc})!'
            )
            self._publish_status(f'EXECUTING: {desc}')

    def _control_loop(self) -> None:
        """Vòng lặp 20 Hz phát lệnh vận tốc /cmd_vel theo hành vi đã xác nhận."""
        now = time.monotonic()

        # Kiểm tra mất tín hiệu điều khiển (Watchdog Timeout) khi robot đang di chuyển
        if self.state == self.STATE_EXECUTING:
            silence_sec = now - self.last_message_time
            if silence_sec > self.watchdog_timeout_sec:
                self._stop_robot_immediately()
                self.active_behavior = None
                self.state = self.STATE_STOPPED
                self.get_logger().error(
                    f'[MẤT TÍN HIỆU ĐIỀU KHIỂN] Không nhận được dữ liệu trên {self.gesture_topic} '
                    f'trong {silence_sec:.2f}s (> {self.watchdog_timeout_sec:.2f}s) -> Dừng robot an toàn!'
                )
                self._publish_status('STOPPED (Mat tin hieu dieu khien - Da dung an toan)')
                return

        if self.state != self.STATE_EXECUTING or self.active_behavior is None:
            return

        elapsed = now - self.behavior_start_time

        # ------------------------------------------------------------------
        # HÀNH VI 1: VẪY TAY (Wave) -> Robot xoay trái–phải 2 nhịp để chào lại
        # ------------------------------------------------------------------
        if self.active_behavior == 'Wave':
            w = self.angular_speed  # mặc định 0.9 rad/s
            # Nhịp 1: Trái (0.8s) -> Phải (1.6s) -> Trái (1.6s) -> Giữa (0.8s) => Tổng 4.8s
            if elapsed < 0.8:
                self._publish_cmd_vel(0.0, +w)
            elif elapsed < 2.4:
                self._publish_cmd_vel(0.0, -w)
            elif elapsed < 4.0:
                self._publish_cmd_vel(0.0, +w)
            elif elapsed < 4.8:
                self._publish_cmd_vel(0.0, -w)
            else:
                self._finish_active_behavior('Hoàn tất hành vi Chào lại (Xoay trái–phải).')
            return

        # ------------------------------------------------------------------
        # HÀNH VI 2: GIƠ 1 NGÓN TAY (Pointing_Up) -> Tiến–lùi 4 lần
        # ------------------------------------------------------------------
        if self.active_behavior == 'Pointing_Up':
            move_dur = 1.6   # Mỗi lượt tiến/lùi kéo dài 1.6s (~0.40m với v=0.25m/s)
            pause_dur = 0.35 # Nghỉ 0.35s khi đảo chiều để chuyển động êm ái
            cycle_dur = 2 * (move_dur + pause_dur)  # 3.9s / chu kỳ
            total_cycles = 4
            total_dur = total_cycles * cycle_dur

            if elapsed >= total_dur:
                self._finish_active_behavior('Hoàn tất hành vi Tiến–lùi đủ 4 lần.')
                return

            cycle_idx = int(elapsed // cycle_dur) + 1
            in_cycle_t = elapsed % cycle_dur

            if cycle_idx != self.last_logged_cycle:
                self.last_logged_cycle = cycle_idx
                self.get_logger().info(f'  -> [Tiến–Lùi] Đang thực hiện lần {cycle_idx}/{total_cycles}...')
                self._publish_status(f'EXECUTING: Tien - Lui (Lan {cycle_idx}/{total_cycles})')

            if in_cycle_t < move_dur:
                self._publish_cmd_vel(+self.linear_speed, 0.0)
            elif in_cycle_t < move_dur + pause_dur:
                self._publish_cmd_vel(0.0, 0.0)
            elif in_cycle_t < 2 * move_dur + pause_dur:
                self._publish_cmd_vel(-self.linear_speed, 0.0)
            else:
                self._publish_cmd_vel(0.0, 0.0)
            return

        # ------------------------------------------------------------------
        # HÀNH VI 3: GIƠ 2 NGÓN TAY (Victory) -> Di chuyển theo quỹ đạo tròn
        # ------------------------------------------------------------------
        if self.active_behavior == 'Victory':
            v = self.circle_linear_speed   # mặc định 0.22 m/s
            w = self.circle_angular_speed  # mặc định 0.55 rad/s -> Bán kính R = v/w = 0.40 m
            circle_period = (2.0 * math.pi) / max(0.05, abs(w))  # ~11.42s cho 1 vòng tròn 360 độ

            if elapsed < circle_period:
                progress_pct = int(min(100.0, (elapsed / circle_period) * 100.0))
                if progress_pct // 25 != self.last_logged_cycle:
                    self.last_logged_cycle = progress_pct // 25
                    self._publish_status(f'EXECUTING: Quy dao tron ({progress_pct}%)')
                self._publish_cmd_vel(v, w)
            else:
                self._finish_active_behavior('Hoàn tất hành vi Di chuyển theo quỹ đạo tròn (360 độ).')
            return

    def _finish_active_behavior(self, log_msg: str) -> None:
        """Kết thúc hành vi hiện tại, dừng robot và trở về trạng thái chờ (IDLE)."""
        self._stop_robot_immediately()
        self.get_logger().info(f'[HOÀN TẤT] {log_msg} -> Trở về trạng thái chờ (IDLE).')
        self.active_behavior = None
        self.selected_behavior = None
        self.state = self.STATE_IDLE
        self._publish_status('IDLE (Hoan tat - San sang chon hanh vi moi)')

    def destroy_node(self) -> None:
        try:
            self._stop_robot_immediately()
        except Exception:
            pass
        super().destroy_node()


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='ROS 2 Node điều khiển hành vi TurtleBot 4 bằng cử chỉ tay (rua4)'
    )
    parser.add_argument(
        '--gesture-topic',
        type=str,
        default='/detected_gesture',
        help='Topic nhận cử chỉ tay (mặc định: /detected_gesture)',
    )
    parser.add_argument(
        '--cmd-vel-topic',
        type=str,
        default='/cmd_vel',
        help='Topic phát lệnh vận tốc điều khiển robot (mặc định: /cmd_vel)',
    )
    parser.add_argument(
        '--status-topic',
        type=str,
        default='/robot_behavior_status',
        help='Topic phát trạng thái hành vi hiện tại của robot',
    )
    parser.add_argument(
        '--wave-immediate',
        action='store_true',
        help='Nếu bật, cử chỉ Wave sẽ chào lại ngay lập tức mà không cần chờ Thumb_Up xác nhận',
    )
    parser.add_argument(
        '--confirm-frames',
        type=int,
        default=3,
        help='Số bản tin cử chỉ liên tiếp cần thiết để xác nhận ổn định (mặc định: 3)',
    )
    parser.add_argument(
        '--cooldown-sec',
        type=float,
        default=0.8,
        help='Thời gian chống dội giữa 2 lệnh chọn/xác nhận liên tiếp (mặc định: 0.8s)',
    )
    parser.add_argument(
        '--watchdog-timeout-sec',
        type=float,
        default=1.5,
        help='Thời gian tối đa mất tín hiệu điều khiển trước khi tự động dừng robot (mặc định: 1.5s)',
    )
    parser.add_argument(
        '--linear-speed',
        type=float,
        default=0.25,
        help='Vận tốc thẳng khi thực hiện Tiến–lùi (mặc định: 0.25 m/s)',
    )
    parser.add_argument(
        '--angular-speed',
        type=float,
        default=0.90,
        help='Vận tốc góc khi xoay trái–phải chào lại (mặc định: 0.90 rad/s)',
    )
    parser.add_argument(
        '--circle-linear-speed',
        type=float,
        default=0.22,
        help='Vận tốc dài khi đi quỹ đạo tròn (mặc định: 0.22 m/s)',
    )
    parser.add_argument(
        '--circle-angular-speed',
        type=float,
        default=0.55,
        help='Vận tốc góc khi đi quỹ đạo tròn (mặc định: 0.55 rad/s -> R=0.40m)',
    )
    parser.add_argument(
        '--safe-obstacle-dist',
        type=float,
        default=0.28,
        help='Khoảng cách an toàn LiDAR tối thiểu để tự động ngắt tiến/lùi (mặc định: 0.28m)',
    )
    cleaned_argv = rclpy.utilities.remove_ros_args(args=argv)
    return parser.parse_args(cleaned_argv[1:])


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    cli_args = parse_args(args)
    node = None
    try:
        node = RobotBehaviorNode(cli_args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
