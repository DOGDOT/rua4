#!/usr/bin/env python3
"""
ROS 2 Node: gesture_recognition_node (Bài tập 06 - Package rua4)
Nhận diện cử chỉ tay qua Webcam Laptop bằng MediaPipe Gesture Recognizer & OpenCV.

Chức năng:
  1. Đọc hình ảnh thời gian thực từ webcam laptop/USB bằng OpenCV (cv2.VideoCapture).
  2. Sử dụng MediaPipe Gesture Recognizer (gesture_recognizer.task) kết hợp phân tích
     động học chuỗi khung hình (landmark trajectory) để nhận diện 06 cử chỉ tay:
       - Wave        : Vẫy tay (Robot xoay trái-phải để chào lại)
       - Pointing_Up : Giơ 1 ngón tay (Chọn hành vi tiến-lùi 4 lần)
       - Victory     : Giơ 2 ngón tay (Chọn hành vi di chuyển theo quỹ đạo tròn)
       - Thumb_Up    : Giơ ngón cái (Xác nhận và thực hiện hành vi đã chọn)
       - Open_Palm   : Xòe bàn tay giữ yên (Robot dừng ngay lập tức)
       - Closed_Fist : Nắm tay (Hủy hành vi hiện tại và trở về trạng thái chờ)
  3. Lọc nhiễu bằng ngưỡng độ tin cậy (Confidence Threshold) + Bỏ phiếu đa số trên cửa sổ trượt
     (Sliding Window Majority Voting) trước khi phát lệnh.
  4. Hiển thị trực quan khung xương bàn tay, tên cử chỉ, độ tin cậy (%), FPS và trạng thái robot
     trên cửa sổ OpenCV.
  5. Publish kết quả nhận diện lên topic /detected_gesture (std_msgs/msg/String).
"""

import argparse
from collections import Counter, deque
import math
import os
import sys
import time
import urllib.request
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

try:
    from ament_index_python.packages import get_package_share_directory
except Exception:
    get_package_share_directory = None

try:
    import mediapipe as mp
    from mediapipe.tasks import python as mp_python
    from mediapipe.tasks.python import vision as mp_vision
    MEDIAPIPE_AVAILABLE = True
except ImportError:
    mp = None
    mp_python = None
    mp_vision = None
    MEDIAPIPE_AVAILABLE = False


MODEL_URL = (
    'https://storage.googleapis.com/mediapipe-models/'
    'gesture_recognizer/gesture_recognizer/float16/1/gesture_recognizer.task'
)

# Danh sách kết nối 21 khớp bàn tay chuẩn của MediaPipe để vẽ khung xương
HAND_CONNECTIONS: List[Tuple[int, int]] = [
    (0, 1), (1, 2), (2, 3), (3, 4),        # Ngón cái
    (0, 5), (5, 6), (6, 7), (7, 8),        # Ngón trỏ
    (5, 9), (9, 10), (10, 11), (11, 12),   # Ngón giữa
    (9, 13), (13, 14), (14, 15), (15, 16), # Ngón áp út
    (13, 17), (0, 17), (17, 18), (18, 19), (19, 20),  # Ngón út & lòng bàn tay
]

# Bảng thông tin hiển thị cho 06 cử chỉ mục tiêu
GESTURE_INFO: Dict[str, Tuple[str, str, Tuple[int, int, int]]] = {
    'Wave': (
        'Vay tay (Wave)',
        'Chao lai: Xoay trai - phai',
        (255, 190, 0),    # Xanh dương sáng / Vàng cam
    ),
    'Pointing_Up': (
        'Gio 1 ngon (Pointing Up)',
        'Chon: Tien - Lui 4 lan',
        (255, 200, 50),   # Xanh lam
    ),
    'Victory': (
        'Gio 2 ngon (Victory)',
        'Chon: Di chuyen quy dao tron',
        (220, 100, 255),  # Tím hồng
    ),
    'Thumb_Up': (
        'Gio ngon cai (Thumbs Up)',
        'XAC NHAN & Thuc hien hanh vi',
        (60, 220, 60),    # Xanh lá
    ),
    'Open_Palm': (
        'Xoe ban tay (Open Palm)',
        'DUNG NGAY LAP TUC (STOP)',
        (0, 60, 255),     # Đỏ
    ),
    'Closed_Fist': (
        'Nam tay (Closed Fist)',
        'HUY hanh vi -> Trang thai cho',
        (0, 165, 255),    # Cam
    ),
    'None': (
        'Khong co cu chi (None)',
        'Dang cho tin hieu tay...',
        (160, 160, 160),  # Xám
    ),
}


def resolve_default_model_path() -> str:
    """Tìm đường dẫn file gesture_recognizer.task trong workspace, install hoặc cache."""
    candidates: List[str] = []

    if get_package_share_directory is not None:
        try:
            share_dir = get_package_share_directory('rua4')
            candidates.append(os.path.join(share_dir, 'config', 'gesture_recognizer.task'))
        except Exception:
            pass

    here = os.path.dirname(os.path.abspath(__file__))
    pkg_root = os.path.abspath(os.path.join(here, '..'))
    candidates.append(os.path.join(pkg_root, 'config', 'gesture_recognizer.task'))
    candidates.append('/home/gdon/ur_ws/src/rua4/config/gesture_recognizer.task')

    cache_path = os.path.expanduser('~/.cache/rua4/gesture_recognizer.task')
    candidates.append(cache_path)

    for path in candidates:
        if os.path.isfile(path) and os.path.getsize(path) > 100_000:
            return path

    return cache_path


def ensure_model_downloaded(model_path: str, logger=None) -> str:
    """Tải tự động gesture_recognizer.task nếu chưa có trên đĩa."""
    if os.path.isfile(model_path) and os.path.getsize(model_path) > 100_000:
        return model_path

    os.makedirs(os.path.dirname(os.path.abspath(model_path)), exist_ok=True)
    msg = f"Đang tải MediaPipe Gesture Recognizer model từ {MODEL_URL} -> {model_path} ..."
    if logger:
        logger.info(msg)
    else:
        print(msg)

    urllib.request.urlretrieve(MODEL_URL, model_path)
    return model_path


class GestureRecognitionNode(Node):
    """Node ROS 2 đọc webcam, nhận diện cử chỉ bằng MediaPipe và publish /detected_gesture."""

    def __init__(self, cli_args: argparse.Namespace):
        super().__init__('gesture_recognition_node')

        self.declare_parameter('camera_id', cli_args.camera_id)
        self.declare_parameter('gesture_topic', cli_args.gesture_topic)
        self.declare_parameter('status_topic', cli_args.status_topic)
        self.declare_parameter('model_path', cli_args.model_path)
        self.declare_parameter('conf_threshold', cli_args.conf_threshold)
        self.declare_parameter('smoothing_window', cli_args.smoothing_window)
        self.declare_parameter('mirror', not cli_args.no_mirror)
        self.declare_parameter('show_window', not cli_args.no_show_window)
        self.declare_parameter('frame_width', cli_args.width)
        self.declare_parameter('frame_height', cli_args.height)

        self.camera_id: int = int(self.get_parameter('camera_id').value)
        self.gesture_topic: str = str(self.get_parameter('gesture_topic').value)
        self.status_topic: str = str(self.get_parameter('status_topic').value)
        self.model_path: str = str(self.get_parameter('model_path').value)
        self.conf_threshold: float = float(self.get_parameter('conf_threshold').value)
        self.smoothing_window: int = max(3, int(self.get_parameter('smoothing_window').value))
        self.mirror: bool = bool(self.get_parameter('mirror').value)
        self.show_window: bool = bool(self.get_parameter('show_window').value)
        self.frame_width: int = int(self.get_parameter('frame_width').value)
        self.frame_height: int = int(self.get_parameter('frame_height').value)

        if not MEDIAPIPE_AVAILABLE:
            self.get_logger().error(
                "Chưa cài đặt thư viện 'mediapipe'! "
                "Vui lòng chạy lệnh: pip install mediapipe 'numpy<2'"
            )
            raise RuntimeError("Missing 'mediapipe' library")

        # Publisher gửi kết quả nhận diện cử chỉ (/detected_gesture - std_msgs/msg/String)
        self.gesture_pub = self.create_publisher(String, self.gesture_topic, 10)

        # Subscriber nhận trạng thái phản hồi từ robot_behavior_node để hiển thị lên HUD webcam
        self.robot_status_text: str = 'IDLE (Dang cho chon hanh vi)'
        self.status_sub = self.create_subscription(
            String,
            self.status_topic,
            self._status_callback,
            10,
        )

        # Khởi tạo MediaPipe Gesture Recognizer
        self.recognizer = None
        self.mp_hands_fallback = None
        self._init_mediapipe()

        # Mở Webcam Laptop/USB
        self.cap = self._open_webcam(self.camera_id)

        # Bộ đệm lọc nhiễu (Sliding Window Majority Voting) & phát hiện cử chỉ Vẫy tay (Wave)
        self.raw_gesture_window: deque = deque(maxlen=self.smoothing_window)
        self.conf_window: deque = deque(maxlen=self.smoothing_window)
        self.wave_history: deque = deque(maxlen=14)  # Lưu (timestamp, x_pos, angle_deg, is_open_hand)
        self.wave_latch_until: float = 0.0
        self.wave_conf_latch: float = 0.90
        self.open_hand_first_seen: float = 0.0

        # Thống kê FPS & log chống lặp
        self.last_frame_time: float = time.monotonic()
        self.fps_ema: float = 0.0
        self.last_logged_gesture: str = 'None'
        self.last_log_time: float = 0.0
        self.window_name: str = 'TurtleBot 4 - AI Hand Gesture Recognition (rua4)'

        # Thư mục lưu ảnh chụp tự động phục vụ báo cáo
        self.results_dir = self._resolve_results_dir()
        self.saved_gestures = set()
        self.last_snapshot_time: float = 0.0

        # Timer đọc webcam và nhận diện ở tần số ~20 Hz (0.05s)
        self.timer = self.create_timer(0.05, self._process_frame)

        self.get_logger().info('=' * 68)
        self.get_logger().info('KHỞI CHẠY NODE NHẬN DIỆN CỬ CHỈ TAY AI (gesture_recognition_node)')
        self.get_logger().info(f'  - Webcam ID           : {self.camera_id}')
        self.get_logger().info(f'  - Publish Topic       : {self.gesture_topic} (std_msgs/msg/String)')
        self.get_logger().info(f'  - Confidence Thresh   : {self.conf_threshold}')
        self.get_logger().info(f'  - Smoothing Window    : {self.smoothing_window} frames')
        self.get_logger().info(
            '  - 06 Cử chỉ hỗ trợ    : Wave, Pointing_Up, Victory, Thumb_Up, Open_Palm, Closed_Fist'
        )
        self.get_logger().info('=' * 68)

    def _status_callback(self, msg: String) -> None:
        if msg.data:
            self.robot_status_text = msg.data

    def _resolve_results_dir(self) -> str:
        ws_results = '/home/gdon/ur_ws/src/rua4/results'
        if os.path.isdir('/home/gdon/ur_ws/src/rua4'):
            os.makedirs(ws_results, exist_ok=True)
            return ws_results
        here = os.path.dirname(os.path.abspath(__file__))
        local_results = os.path.abspath(os.path.join(here, '..', 'results'))
        try:
            os.makedirs(local_results, exist_ok=True)
            return local_results
        except Exception:
            return '/tmp'

    def _init_mediapipe(self) -> None:
        """Khởi tạo MediaPipe GestureRecognizer (hoặc fallback sang mp.solutions.hands)."""
        try:
            resolved_model = ensure_model_downloaded(self.model_path, self.get_logger())
            base_options = mp_python.BaseOptions(model_asset_path=resolved_model)
            options = mp_vision.GestureRecognizerOptions(
                base_options=base_options,
                running_mode=mp_vision.RunningMode.IMAGE,
                num_hands=1,
                min_hand_detection_confidence=0.5,
                min_hand_presence_confidence=0.5,
                min_tracking_confidence=0.5,
            )
            self.recognizer = mp_vision.GestureRecognizer.create_from_options(options)
            self.get_logger().info(f'Đã nạp MediaPipe GestureRecognizer từ: {resolved_model}')
        except Exception as exc:
            self.get_logger().warn(
                f'Không nạp được GestureRecognizer task ({exc}), chuyển sang chế độ MediaPipe Hands.'
            )
            self.mp_hands_fallback = mp.solutions.hands.Hands(
                static_image_mode=False,
                max_num_hands=1,
                min_detection_confidence=0.5,
                min_tracking_confidence=0.5,
            )

    def _open_webcam(self, preferred_id: int) -> Optional[cv2.VideoCapture]:
        """Mở webcam với cơ chế tự động thử V4L2 và các cổng /dev/video0..3."""
        candidate_ids = [preferred_id] + [i for i in range(4) if i != preferred_id]
        for cam_id in candidate_ids:
            for backend in (cv2.CAP_V4L2, cv2.CAP_ANY):
                cap = cv2.VideoCapture(cam_id, backend)
                if cap is not None and cap.isOpened():
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
                    cap.set(cv2.CAP_PROP_FPS, 30)
                    ret, test_frame = cap.read()
                    if ret and test_frame is not None and test_frame.size > 0:
                        self.camera_id = cam_id
                        self.get_logger().info(
                            f'Đã kết nối thành công Webcam ID {cam_id} '
                            f'({test_frame.shape[1]}x{test_frame.shape[0]}).'
                        )
                        return cap
                    cap.release()

        self.get_logger().warn(
            'Không tìm thấy thiết bị webcam (/dev/video*). '
            'Hãy kiểm tra kết nối webcam hoặc quyền truy cập /dev/video0.'
        )
        return None

    @staticmethod
    def _count_extended_fingers(landmarks) -> Tuple[int, Dict[str, bool]]:
        """
        Kiểm tra trạng thái duỗi của 5 ngón tay từ 21 điểm mốc (landmarks).
        Trả về (tổng số ngón duỗi, dict trạng thái từng ngón).
        """
        wrist = landmarks[0]

        def dist_sq(a, b) -> float:
            return (a.x - b.x) ** 2 + (a.y - b.y) ** 2

        # 4 ngón dài: đầu ngón (tip) cao hơn khớp PIP và xa cổ tay hơn khớp PIP
        index_open = (landmarks[8].y < landmarks[6].y) and (dist_sq(landmarks[8], wrist) > dist_sq(landmarks[6], wrist))
        middle_open = (landmarks[12].y < landmarks[10].y) and (dist_sq(landmarks[12], wrist) > dist_sq(landmarks[10], wrist))
        ring_open = (landmarks[16].y < landmarks[14].y) and (dist_sq(landmarks[16], wrist) > dist_sq(landmarks[14], wrist))
        pinky_open = (landmarks[20].y < landmarks[18].y) and (dist_sq(landmarks[20], wrist) > dist_sq(landmarks[18], wrist))

        # Ngón cái: khoảng cách từ tip (4) tới khớp MCP ngón út (17) lớn hơn từ IP (3) tới (17)
        thumb_open = dist_sq(landmarks[4], landmarks[17]) > dist_sq(landmarks[3], landmarks[17]) * 1.15

        states = {
            'thumb': thumb_open,
            'index': index_open,
            'middle': middle_open,
            'ring': ring_open,
            'pinky': pinky_open,
        }
        count = sum(1 for v in states.values() if v)
        return count, states

    def _detect_wave_or_refine(
        self,
        raw_category: str,
        raw_score: float,
        landmarks,
        now: float,
    ) -> Tuple[str, float]:
        """
        Phân biệt cử chỉ Vẫy tay (Wave) và Xòe bàn tay đứng yên (Open_Palm),
        đồng thời chuẩn hóa sang 06 cử chỉ mục tiêu của Bài tập 6.
        """
        num_open, states = self._count_extended_fingers(landmarks)

        # Tính tọa độ ngang đại diện của bàn tay (trung bình đầu ngón giữa 12 và khớp bàn tay 9)
        x_rep = 0.6 * landmarks[12].x + 0.4 * landmarks[9].x
        dx_wrist = landmarks[12].x - landmarks[0].x
        dy_wrist = landmarks[0].y - landmarks[12].y
        angle_deg = math.degrees(math.atan2(dx_wrist, max(1e-4, abs(dy_wrist))))

        # Bàn tay đang mở (ít nhất 3 ngón duỗi hoặc MediaPipe nhận là Open_Palm / ILoveYou)
        is_open_hand = (num_open >= 3) or (raw_category in ('Open_Palm', 'ILoveYou'))

        self.wave_history.append((now, x_rep, angle_deg, is_open_hand))
        # Chỉ giữ các mẫu trong 0.55 giây gần nhất
        while self.wave_history and (now - self.wave_history[0][0] > 0.55):
            self.wave_history.popleft()

        # 1. Nếu người dùng giơ cử chỉ ILoveYou (Ngón cái + trỏ + út), ánh xạ trực tiếp sang Wave
        if raw_category == 'ILoveYou' and raw_score >= self.conf_threshold:
            self.wave_latch_until = now + 0.65
            self.wave_conf_latch = max(raw_score, 0.88)
            return 'Wave', self.wave_conf_latch

        # 2. Kiểm tra chuyển động vẫy tay qua lại trái-phải (Dynamic Wave)
        open_samples = [item for item in self.wave_history if item[3]]
        is_moving_horizontally = False
        if len(open_samples) >= 4:
            xs = [item[1] for item in open_samples]
            angs = [item[2] for item in open_samples]
            total_dx = sum(abs(xs[i] - xs[i - 1]) for i in range(1, len(xs)))
            span_x = max(xs) - min(xs)
            total_dang = sum(abs(angs[i] - angs[i - 1]) for i in range(1, len(angs)))
            span_ang = max(angs) - min(angs)

            # Đang có chuyển động ngang (bắt đầu vẫy tay)
            if total_dx >= 0.032 or total_dang >= 9.0:
                is_moving_horizontally = True

            # Đủ biên độ vẫy tay -> kích hoạt Wave
            if (total_dx >= 0.070 and span_x >= 0.028) or (total_dang >= 18.0 and span_ang >= 8.0):
                motion_boost = min(0.99, 0.82 + total_dx * 0.8)
                self.wave_latch_until = now + 0.70
                self.wave_conf_latch = max(raw_score, motion_boost)
                return 'Wave', self.wave_conf_latch

        # Duy trì trạng thái Wave mượt mà khi đang vẫy tay liên tục
        if is_open_hand and now < self.wave_latch_until:
            return 'Wave', self.wave_conf_latch

        # 3. Nếu bàn tay mở (Open_Palm): đảm bảo người dùng giữ yên ~0.18s để không nhầm lúc mới giơ tay vẫy
        if raw_category == 'Open_Palm':
            if self.open_hand_first_seen == 0.0:
                self.open_hand_first_seen = now
            if is_moving_horizontally and (now - self.open_hand_first_seen < 0.28):
                return 'None', 0.0
            return 'Open_Palm', raw_score
        else:
            self.open_hand_first_seen = 0.0

        # 4. Các cử chỉ tĩnh chuẩn của MediaPipe Gesture Recognizer
        if raw_category in ('Pointing_Up', 'Victory', 'Thumb_Up', 'Closed_Fist'):
            return raw_category, raw_score

        # 5. Bộ phân loại hình học dự phòng nếu raw_category == 'None' nhưng các khớp ngón tay rất rõ ràng
        if states['index'] and not states['middle'] and not states['ring'] and not states['pinky']:
            return 'Pointing_Up', max(raw_score, 0.78)
        if states['index'] and states['middle'] and not states['ring'] and not states['pinky']:
            return 'Victory', max(raw_score, 0.80)
        if (
            states['thumb']
            and not states['index']
            and not states['middle']
            and not states['ring']
            and not states['pinky']
            and landmarks[4].y < landmarks[2].y
        ):
            return 'Thumb_Up', max(raw_score, 0.78)
        if num_open == 0:
            return 'Closed_Fist', max(raw_score, 0.78)

        return 'None', 0.0

    def _classify_frame(self, frame_bgr: np.ndarray, now: float) -> Tuple[str, float, Optional[List]]:
        """Chạy MediaPipe trên khung hình BGR và trả về (gesture_name, confidence, landmarks)."""
        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        if self.recognizer is not None:
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=frame_rgb)
            result = self.recognizer.recognize(mp_image)
            if not result.hand_landmarks:
                self.wave_history.clear()
                self.open_hand_first_seen = 0.0
                return 'None', 0.0, None

            landmarks = result.hand_landmarks[0]
            raw_cat = 'None'
            raw_score = 0.0
            if result.gestures and result.gestures[0]:
                top_gesture = result.gestures[0][0]
                raw_cat = top_gesture.category_name or 'None'
                raw_score = float(top_gesture.score or 0.0)

            gesture_name, score = self._detect_wave_or_refine(raw_cat, raw_score, landmarks, now)
            if score < self.conf_threshold:
                return 'None', score, landmarks
            return gesture_name, score, landmarks

        # Fallback: MediaPipe Hands
        if self.mp_hands_fallback is not None:
            res = self.mp_hands_fallback.process(frame_rgb)
            if not res.multi_hand_landmarks:
                self.wave_history.clear()
                self.open_hand_first_seen = 0.0
                return 'None', 0.0, None
            landmarks = res.multi_hand_landmarks[0].landmark
            num_open, _ = self._count_extended_fingers(landmarks)
            raw_cat = 'Open_Palm' if num_open >= 4 else 'None'
            gesture_name, score = self._detect_wave_or_refine(raw_cat, 0.85, landmarks, now)
            return gesture_name, score, landmarks

        return 'None', 0.0, None

    def _smooth_gesture(self, gesture_name: str, confidence: float) -> Tuple[str, float]:
        """Lọc nhiễu bằng cửa sổ trượt (Majority Voting)."""
        self.raw_gesture_window.append(gesture_name)
        self.conf_window.append(confidence)

        counts = Counter(self.raw_gesture_window)
        most_common_gesture, freq = counts.most_common(1)[0]

        # Yêu cầu ít nhất quá nửa cửa sổ trượt đồng thuận để đổi sang cử chỉ mới
        min_consensus = max(2, (len(self.raw_gesture_window) + 1) // 2)
        if most_common_gesture != 'None' and freq >= min_consensus:
            matching_confs = [
                c for g, c in zip(self.raw_gesture_window, self.conf_window) if g == most_common_gesture
            ]
            avg_conf = sum(matching_confs) / max(1, len(matching_confs))
            return most_common_gesture, avg_conf

        return 'None', 0.0

    def _draw_hand_overlay(
        self,
        canvas: np.ndarray,
        landmarks,
        gesture_name: str,
        confidence: float,
    ) -> None:
        """Vẽ khung xương 21 khớp bàn tay, khung giới hạn và HUD thông tin lên ảnh webcam."""
        h, w = canvas.shape[:2]
        title_vn, action_vn, accent_color = GESTURE_INFO.get(
            gesture_name, GESTURE_INFO['None']
        )

        if landmarks is not None:
            pts: List[Tuple[int, int]] = []
            for lm in landmarks:
                px = int(min(max(lm.x * w, 0), w - 1))
                py = int(min(max(lm.y * h, 0), h - 1))
                pts.append((px, py))

            # Vẽ các đoạn nối xương bàn tay
            for i_start, i_end in HAND_CONNECTIONS:
                if i_start < len(pts) and i_end < len(pts):
                    cv2.line(canvas, pts[i_start], pts[i_end], accent_color, 2, cv2.LINE_AA)

            # Vẽ các khớp ngón tay
            for idx, (px, py) in enumerate(pts):
                radius = 6 if idx in (4, 8, 12, 16, 20) else 4
                cv2.circle(canvas, (px, py), radius, (255, 255, 255), -1, cv2.LINE_AA)
                cv2.circle(canvas, (px, py), radius, accent_color, 1, cv2.LINE_AA)

            # Vẽ Bounding Box quanh bàn tay
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            pad = 22
            x1, y1 = max(0, min(xs) - pad), max(0, min(ys) - pad)
            x2, y2 = min(w - 1, max(xs) + pad), min(h - 1, max(ys) + pad)
            cv2.rectangle(canvas, (x1, y1), (x2, y2), accent_color, 2)

            if gesture_name != 'None':
                tag = f"{gesture_name} ({confidence * 100:.1f}%)"
                (tw, th), _ = cv2.getTextSize(tag, cv2.FONT_HERSHEY_SIMPLEX, 0.62, 2)
                tag_y = max(y1 - 8, th + 10)
                cv2.rectangle(
                    canvas,
                    (x1, tag_y - th - 8),
                    (x1 + tw + 12, tag_y + 4),
                    accent_color,
                    -1,
                )
                cv2.putText(
                    canvas,
                    tag,
                    (x1 + 6, tag_y - 2),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.62,
                    (15, 15, 15),
                    2,
                    cv2.LINE_AA,
                )

        # Vẽ HUD Header trên cùng
        overlay = canvas.copy()
        cv2.rectangle(overlay, (0, 0), (w, 108), (20, 20, 20), -1)
        cv2.rectangle(overlay, (0, h - 36), (w, h), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.78, canvas, 0.22, 0, canvas)

        cv2.putText(
            canvas,
            f"Cu chi: {title_vn}",
            (14, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.75,
            accent_color,
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            f"Do tin cay: {confidence * 100:.1f}%   |   FPS: {self.fps_ema:.1f}",
            (14, 58),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (235, 235, 235),
            1,
            cv2.LINE_AA,
        )
        cv2.putText(
            canvas,
            f"Hanh vi: {action_vn}   |   Robot: {self.robot_status_text}",
            (14, 88),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.54,
            (120, 245, 150),
            1,
            cv2.LINE_AA,
        )

        # Thanh hướng dẫn 06 cử chỉ ở đáy màn hình
        footer = (
            "[Wave] Chao | [1 Ngon] Tien-Lui 4x | [2 Ngon] Vong tron | "
            "[Thumb Up] Xac nhan | [Open Palm] Dung | [Fist] Huy"
        )
        cv2.putText(
            canvas,
            footer,
            (10, h - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.42,
            (220, 220, 220),
            1,
            cv2.LINE_AA,
        )

    def _process_frame(self) -> None:
        now = time.monotonic()
        dt = max(1e-4, now - self.last_frame_time)
        self.last_frame_time = now
        inst_fps = 1.0 / dt
        self.fps_ema = inst_fps if self.fps_ema == 0.0 else (0.85 * self.fps_ema + 0.15 * inst_fps)

        if self.cap is None or not self.cap.isOpened():
            self.cap = self._open_webcam(self.camera_id)
            if self.cap is None:
                return

        ret, frame = self.cap.read()
        if not ret or frame is None:
            self.get_logger().warn('Không đọc được khung hình từ webcam!', throttle_duration_sec=3.0)
            return

        if self.mirror:
            frame = cv2.flip(frame, 1)

        raw_gesture, raw_conf, landmarks = self._classify_frame(frame, now)
        stable_gesture, stable_conf = self._smooth_gesture(raw_gesture, raw_conf)

        # Publish kết quả nhận diện lên /detected_gesture (std_msgs/msg/String)
        msg = String()
        msg.data = stable_gesture
        self.gesture_pub.publish(msg)

        # In log ra Terminal khi có cử chỉ mới hoặc định kỳ mỗi 2.0s
        if stable_gesture != 'None' and (
            stable_gesture != self.last_logged_gesture or (now - self.last_log_time) >= 2.0
        ):
            title_vn, action_vn, _ = GESTURE_INFO.get(stable_gesture, GESTURE_INFO['None'])
            self.get_logger().info(
                f"[DETECTED] {stable_gesture} ({title_vn}) | "
                f"Confidence: {stable_conf * 100:.1f}% -> {action_vn}"
            )
            self.last_logged_gesture = stable_gesture
            self.last_log_time = now
        elif stable_gesture == 'None':
            self.last_logged_gesture = 'None'

        # Vẽ giao diện trực quan lên khung hình webcam
        self._draw_hand_overlay(frame, landmarks, stable_gesture, stable_conf)

        # Lưu ảnh minh họa tự động cho từng loại cử chỉ vào results/
        if stable_gesture != 'None' and stable_conf >= 0.65:
            if stable_gesture not in self.saved_gestures or (now - self.last_snapshot_time > 3.0):
                try:
                    cv2.imwrite(
                        os.path.join(self.results_dir, f'gesture_{stable_gesture.lower()}.jpg'),
                        frame,
                    )
                    cv2.imwrite(
                        os.path.join(self.results_dir, 'gesture_latest_frame.jpg'),
                        frame,
                    )
                    self.saved_gestures.add(stable_gesture)
                    self.last_snapshot_time = now
                except Exception:
                    pass

        if self.show_window:
            try:
                cv2.imshow(self.window_name, frame)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord('q'), 27):
                    self.get_logger().info('Người dùng nhấn Q/ESC -> Đóng node nhận diện cử chỉ.')
                    rclpy.shutdown()
            except Exception as exc:
                self.get_logger().warn(
                    f'Không thể mở cửa sổ cv2.imshow ({exc}) -> Tắt hiển thị cửa sổ.'
                )
                self.show_window = False

    def destroy_node(self) -> None:
        if self.cap is not None:
            try:
                self.cap.release()
            except Exception:
                pass
        if self.recognizer is not None:
            try:
                self.recognizer.close()
            except Exception:
                pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='ROS 2 Node nhận diện cử chỉ tay bằng MediaPipe Gesture Recognizer (rua4)'
    )
    parser.add_argument(
        '--camera-id',
        type=int,
        default=0,
        help='Chỉ số thiết bị webcam (mặc định: 0 tương ứng /dev/video0)',
    )
    parser.add_argument(
        '--gesture-topic',
        type=str,
        default='/detected_gesture',
        help='Topic publish tên cử chỉ nhận diện (mặc định: /detected_gesture)',
    )
    parser.add_argument(
        '--status-topic',
        type=str,
        default='/robot_behavior_status',
        help='Topic nhận trạng thái hành vi từ robot_behavior_node',
    )
    parser.add_argument(
        '--model-path',
        type=str,
        default=resolve_default_model_path(),
        help='Đường dẫn file gesture_recognizer.task của MediaPipe',
    )
    parser.add_argument(
        '--conf-threshold',
        type=float,
        default=0.55,
        help='Ngưỡng độ tin cậy tối thiểu [0.0 - 1.0] (mặc định: 0.55)',
    )
    parser.add_argument(
        '--smoothing-window',
        type=int,
        default=5,
        help='Kích thước cửa sổ trượt lọc nhiễu (mặc định: 5 frames)',
    )
    parser.add_argument(
        '--width',
        type=int,
        default=640,
        help='Chiều rộng khung hình webcam (mặc định: 640)',
    )
    parser.add_argument(
        '--height',
        type=int,
        default=480,
        help='Chiều cao khung hình webcam (mặc định: 480)',
    )
    parser.add_argument(
        '--no-mirror',
        action='store_true',
        help='Tắt lật gương ngang khung hình webcam',
    )
    parser.add_argument(
        '--no-show-window',
        action='store_true',
        help='Chạy ẩn không bật cửa sổ cv2.imshow',
    )
    cleaned_argv = rclpy.utilities.remove_ros_args(args=argv)
    return parser.parse_args(cleaned_argv[1:])


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    cli_args = parse_args(args)
    node = None
    try:
        node = GestureRecognitionNode(cli_args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
