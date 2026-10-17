#!/usr/bin/env python3
"""
Node nhận diện vật thể thời gian thực từ camera RGB của TurtleBot 4 bằng Ultralytics YOLO.
Thực hiện Yêu cầu 3 - Bài thực hành 05:
  1. Subscribe dữ liệu ảnh RGB từ camera TurtleBot 4 (mặc định: /oakd/rgb/preview/image_raw).
  2. Dùng cv_bridge chuyển đổi sensor_msgs/msg/Image sang ảnh OpenCV (BGR8).
  3. Đưa ảnh qua mô hình YOLO (mặc định: yolov8n.pt hoặc yolo11n.pt) của thư viện Ultralytics.
  4. Vẽ bounding box, tên class và độ tin cậy (confidence) lên ảnh.
  5. In thông tin vật thể phát hiện được lên Terminal (có giới hạn tần suất chống spam log).
  6. Hiển thị bằng cửa sổ OpenCV (cv2.imshow) và đồng thời publish ra topic /yolo/annotated_image
     để quan sát trực tiếp trong RViz2.
  7. Chạy độc lập trong tiến trình riêng, không làm nghẽn vòng lặp điều hướng của Nav2.
"""

import os
import sys
import time
import argparse
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Image

try:
    import numpy as np
except ImportError as exc:
    print(f"[LỖI] Thiếu thư viện numpy: {exc}. Hãy cài đặt: pip install 'numpy<2'")
    sys.exit(1)

try:
    import cv2
except ImportError as exc:
    print(
        f"[LỖI] Thiếu thư viện OpenCV (cv2): {exc}.\n"
        "Hãy cài đặt bằng: sudo apt install python3-opencv hoặc pip install opencv-python"
    )
    sys.exit(1)

try:
    from cv_bridge import CvBridge, CvBridgeError
    HAS_CV_BRIDGE = True
except Exception as exc:
    CvBridge = None
    CvBridgeError = Exception
    HAS_CV_BRIDGE = False
    print(
        f"[CẢNH BÁO] Không thể nạp cv_bridge chuẩn ({exc}). "
        "Node sẽ tự động sử dụng bộ chuyển đổi dự phòng NumPy <-> ROS Image."
    )

try:
    from ultralytics import YOLO
    HAS_ULTRALYTICS = True
except ImportError:
    YOLO = None
    HAS_ULTRALYTICS = False


# Bảng màu BGR hiển thị bounding box theo class ID
BOX_COLORS: List[Tuple[int, int, int]] = [
    (0, 255, 0),    # Xanh lá
    (255, 144, 30), # Xanh dương sáng
    (0, 165, 255),  # Cam
    (255, 0, 255),  # Tím hồng
    (0, 255, 255),  # Vàng
    (255, 255, 0),  # Cyan
    (128, 0, 255),  # Hồng đậm
    (0, 128, 255),  # Cam đậm
]


class YoloDetectorNode(Node):
    """ROS 2 Node nhận diện vật thể từ camera RGB TurtleBot 4 bằng YOLO."""

    def __init__(self, cli_args: argparse.Namespace):
        super().__init__('rua4_yolo_detector')

        # Khai báo các ROS 2 Parameters (cho phép ghi đè qua CLI hoặc launch file)
        self.declare_parameter('image_topic', '/oakd/rgb/preview/image_raw')
        self.declare_parameter('annotated_topic', '/yolo/annotated_image')
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('conf_threshold', 0.45)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('device', '')
        self.declare_parameter('show_window', True)
        self.declare_parameter('qos_reliability', 'best_effort')
        self.declare_parameter('log_interval_sec', 2.0)
        self.declare_parameter('save_snapshots', True)
        self.declare_parameter('snapshot_dir', '/home/gdon/ur_ws/src/rua4/results')

        # Ưu tiên tham số CLI nếu người dùng truyền trực tiếp, ngược lại lấy từ ROS Parameter
        self.image_topic: str = (
            cli_args.image_topic
            if cli_args.image_topic is not None
            else str(self.get_parameter('image_topic').value)
        )
        self.annotated_topic: str = (
            cli_args.annotated_topic
            if cli_args.annotated_topic is not None
            else str(self.get_parameter('annotated_topic').value)
        )
        self.model_path: str = (
            cli_args.model
            if cli_args.model is not None
            else str(self.get_parameter('model_path').value)
        )
        self.conf_threshold: float = (
            cli_args.conf
            if cli_args.conf is not None
            else float(self.get_parameter('conf_threshold').value)
        )
        self.iou_threshold: float = (
            cli_args.iou
            if cli_args.iou is not None
            else float(self.get_parameter('iou_threshold').value)
        )
        self.device: str = (
            cli_args.device
            if cli_args.device is not None
            else str(self.get_parameter('device').value)
        )
        if cli_args.no_show_window:
            self.show_window = False
        elif cli_args.show_window:
            self.show_window = True
        else:
            self.show_window = bool(self.get_parameter('show_window').value)

        # Kiểm tra biến môi trường DISPLAY nếu bật cửa sổ OpenCV
        if self.show_window and not os.environ.get('DISPLAY') and not os.environ.get('WAYLAND_DISPLAY'):
            self.get_logger().warn(
                "Không tìm thấy biến môi trường DISPLAY. Tự động tắt cv2.imshow (vẫn publish /yolo/annotated_image)."
            )
            self.show_window = False

        self.qos_reliability: str = (
            cli_args.qos
            if cli_args.qos is not None
            else str(self.get_parameter('qos_reliability').value)
        ).lower()
        self.log_interval_sec: float = (
            cli_args.log_interval
            if cli_args.log_interval is not None
            else float(self.get_parameter('log_interval_sec').value)
        )
        self.save_snapshots: bool = (
            False
            if cli_args.no_save_snapshots
            else bool(self.get_parameter('save_snapshots').value)
        )
        self.snapshot_dir: str = str(self.get_parameter('snapshot_dir').value)

        # Khởi tạo CvBridge
        self.bridge = CvBridge() if HAS_CV_BRIDGE else None

        # Kiểm tra và tải mô hình Ultralytics YOLO
        if not HAS_ULTRALYTICS:
            self.get_logger().error(
                "Chưa cài đặt thư viện 'ultralytics'! "
                "Vui lòng chạy lệnh: pip install 'numpy<2' ultralytics"
            )
            raise RuntimeError("Missing 'ultralytics' package")

        self._load_yolo_model()

        # Thiết lập QoS cho subscriber nhận ảnh camera:
        # BEST_EFFORT + depth=1 tương thích với cả publisher BEST_EFFORT lẫn RELIABLE
        # và luôn lấy frame mới nhất, tránh tích tụ độ trễ khi robot đang di chuyển.
        reliability_policy = (
            ReliabilityPolicy.RELIABLE
            if self.qos_reliability == 'reliable'
            else ReliabilityPolicy.BEST_EFFORT
        )
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=reliability_policy,
            durability=DurabilityPolicy.VOLATILE,
        )

        # QoS RELIABLE cho topic ảnh đầu ra để xem mượt trên RViz2 và rqt_image_view
        pub_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.image_sub = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            sensor_qos,
        )
        self.annotated_pub = self.create_publisher(
            Image,
            self.annotated_topic,
            pub_qos,
        )

        # Biến trạng thái phục vụ thống kê và chống spam log
        self._is_processing: bool = False
        self.frame_count: int = 0
        self.start_time: float = time.time()
        self.last_log_time: float = 0.0
        self.last_heartbeat_time: float = time.time()
        self.last_detected_signature: Tuple[str, ...] = ()
        self.last_snapshot_time: float = 0.0
        self.saved_first_frame: bool = False
        self.window_name: str = "TurtleBot 4 - YOLO Object Detection (rua4)"

        # Timer kiểm tra định kỳ xem đã nhận được dữ liệu từ topic camera chưa
        self.watchdog_timer = self.create_timer(5.0, self._check_camera_stream)

        self._print_startup_banner()

    def _load_yolo_model(self) -> None:
        """Tải trọng số YOLO và xử lý lỗi rõ ràng nếu thiếu file hoặc mất kết nối."""
        print("=" * 72)
        print(f"Đang nạp mô hình Ultralytics YOLO từ: '{self.model_path}'...")
        try:
            self.model = YOLO(self.model_path)
            class_names = list(self.model.names.values()) if hasattr(self.model, 'names') else []
            sample_classes = ", ".join(class_names[:10]) + ("..." if len(class_names) > 10 else "")
            print(f"[THÀNH CÔNG] Đã tải model '{self.model_path}' ({len(class_names)} classes).")
            if sample_classes:
                print(f"  + Các class hỗ trợ (MS COCO): {sample_classes}")
                print(
                    "  + Lưu ý: Model pretrained COCO nhận diện 80 lớp vật thể thông dụng "
                    "(person, chair, bench, potted plant, tv, bottle, cup, suitcase, truck...). "
                    "Các vật thể công nghiệp đặc thù ngoài tập COCO (như bình chữa cháy, kệ kho thô) "
                    "sẽ không nằm trong danh mục chuẩn nếu chưa fine-tune riêng."
                )
        except Exception as exc:
            self.get_logger().error(
                f"Không thể tải model YOLO '{self.model_path}': {exc}\n"
                "Hãy kiểm tra kết nối mạng để tải trọng số lần đầu hoặc chỉ định đường dẫn file .pt hợp lệ."
            )
            raise

    def _print_startup_banner(self) -> None:
        print("=" * 72)
        print("   [rua4] NODE NHẬN DIỆN VẬT THỂ CAMERA RGB VỚI YOLO (LAB 05)")
        print("=" * 72)
        print(f"  + Topic ảnh đầu vào  : {self.image_topic} (QoS: {self.qos_reliability.upper()})")
        print(f"  + Topic ảnh kết quả  : {self.annotated_topic} (Dùng để xem trên RViz2)")
        print(f"  + Model trọng số     : {self.model_path}")
        print(f"  + Ngưỡng Confidence  : {self.conf_threshold:.2f} | IoU: {self.iou_threshold:.2f}")
        print(f"  + Cửa sổ OpenCV GUI  : {'Bật (cv2.imshow)' if self.show_window else 'Tắt (Headless/RViz2 mode)'}")
        print(f"  + Lưu ảnh kết quả    : {'Bật (' + self.snapshot_dir + ')' if self.save_snapshots else 'Tắt'}")
        print("=" * 72)
        print("Đang lắng nghe khung hình từ camera TurtleBot 4...\n")

    def _check_camera_stream(self) -> None:
        """Cảnh báo và gợi ý danh sách topic ảnh thực tế nếu chưa nhận được frame nào."""
        if self.frame_count > 0:
            return
        topic_list = self.get_topic_names_and_types()
        available_image_topics = [
            name for name, types in topic_list
            if 'sensor_msgs/msg/Image' in types and name != self.annotated_topic
        ]
        if available_image_topics:
            self.get_logger().warn(
                f"Chưa nhận được ảnh trên '{self.image_topic}'. "
                f"Các topic sensor_msgs/msg/Image hiện có trong hệ thống: {available_image_topics}. "
                f"Bạn có thể đổi topic bằng: ros2 run rua4 yolo_detector --image-topic {available_image_topics[0]}"
            )
        else:
            self.get_logger().warn(
                f"Chưa nhận được ảnh trên '{self.image_topic}' và chưa thấy topic Image nào đang phát. "
                "Hãy kiểm tra Gazebo Sim đã khởi chạy với model:=standard (có camera OAK-D) chưa."
            )

    def _ros_img_to_cv2(self, msg: Image) -> Optional[np.ndarray]:
        """Chuyển đổi sensor_msgs/msg/Image sang ảnh BGR8 OpenCV với xử lý lỗi an toàn."""
        if self.bridge is not None:
            try:
                return self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as bridge_exc:
                # Dự phòng bằng NumPy nếu cv_bridge gặp xung đột ABI với NumPy
                self.get_logger().debug(f"Fallback NumPy conversion do lỗi cv_bridge: {bridge_exc}")

        try:
            enc = msg.encoding.lower()
            raw = np.frombuffer(msg.data, dtype=np.uint8)
            if enc in ('bgr8', '8uc3'):
                return raw.reshape((msg.height, msg.width, 3)).copy()
            elif enc == 'rgb8':
                rgb = raw.reshape((msg.height, msg.width, 3))
                return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            elif enc == 'rgba8':
                rgba = raw.reshape((msg.height, msg.width, 4))
                return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)
            elif enc == 'bgra8':
                bgra = raw.reshape((msg.height, msg.width, 4))
                return cv2.cvtColor(bgra, cv2.COLOR_BGRA2BGR)
            elif enc in ('mono8', '8uc1'):
                gray = raw.reshape((msg.height, msg.width))
                return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
            else:
                self.get_logger().error(f"Định dạng ảnh '{msg.encoding}' không được hỗ trợ.")
                return None
        except Exception as exc:
            self.get_logger().error(f"Lỗi chuyển đổi ROS Image sang OpenCV: {exc}")
            return None

    def _cv2_to_ros_img(self, cv_img: np.ndarray, header) -> Optional[Image]:
        """Chuyển đổi ảnh BGR8 OpenCV sang sensor_msgs/msg/Image."""
        if self.bridge is not None:
            try:
                out_msg = self.bridge.cv2_to_imgmsg(cv_img, encoding='bgr8')
                out_msg.header = header
                return out_msg
            except Exception:
                pass

        try:
            out_msg = Image()
            out_msg.header = header
            out_msg.height = int(cv_img.shape[0])
            out_msg.width = int(cv_img.shape[1])
            out_msg.encoding = 'bgr8'
            out_msg.is_bigendian = 0
            out_msg.step = int(cv_img.shape[1] * 3)
            out_msg.data = np.ascontiguousarray(cv_img).tobytes()
            return out_msg
        except Exception as exc:
            self.get_logger().error(f"Lỗi chuyển đổi OpenCV sang ROS Image: {exc}")
            return None

    def image_callback(self, msg: Image) -> None:
        """Callback xử lý mỗi khung hình từ camera RGB."""
        if self._is_processing:
            return
        self._is_processing = True

        try:
            t0 = time.time()
            cv_image = self._ros_img_to_cv2(msg)
            if cv_image is None or cv_image.size == 0:
                return

            self.frame_count += 1
            elapsed_total = max(time.time() - self.start_time, 1e-3)
            avg_fps = self.frame_count / elapsed_total

            # Chạy suy luận YOLO
            results = self.model.predict(
                source=cv_image,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                device=self.device if self.device else None,
                verbose=False,
            )

            annotated_frame = cv_image.copy()
            detections_info: List[Tuple[str, float, Tuple[int, int, int, int]]] = []

            if results and len(results) > 0:
                result = results[0]
                boxes = getattr(result, 'boxes', None)
                if boxes is not None and len(boxes) > 0:
                    for box in boxes:
                        xyxy = box.xyxy[0].tolist()
                        x1, y1, x2, y2 = [int(round(v)) for v in xyxy]
                        cls_id = int(box.cls[0].item())
                        conf = float(box.conf[0].item())
                        cls_name = str(self.model.names.get(cls_id, f"class_{cls_id}"))

                        detections_info.append((cls_name, conf, (x1, y1, x2, y2)))

                        color = BOX_COLORS[cls_id % len(BOX_COLORS)]
                        cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), color, 2)

                        label = f"{cls_name} {conf:.2f}"
                        (tw, th), baseline = cv2.getTextSize(
                            label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2
                        )
                        label_y1 = max(y1 - th - baseline - 6, 0)
                        label_y2 = label_y1 + th + baseline + 6
                        cv2.rectangle(
                            annotated_frame,
                            (x1, label_y1),
                            (x1 + tw + 8, label_y2),
                            color,
                            -1,
                        )
                        cv2.putText(
                            annotated_frame,
                            label,
                            (x1 + 4, label_y2 - baseline - 2),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.55,
                            (0, 0, 0),
                            2,
                            cv2.LINE_AA,
                        )

            infer_ms = (time.time() - t0) * 1000.0

            # Vẽ thanh trạng thái ở góc trên ảnh để dễ quan sát trên RViz2 và OpenCV
            status_text = (
                f"YOLO ({os.path.basename(self.model_path)}) | "
                f"{avg_fps:.1f} FPS ({infer_ms:.0f}ms) | "
                f"Detected: {len(detections_info)}"
            )
            cv2.rectangle(annotated_frame, (0, 0), (annotated_frame.shape[1], 28), (20, 20, 20), -1)
            cv2.putText(
                annotated_frame,
                status_text,
                (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.52,
                (0, 255, 180),
                1,
                cv2.LINE_AA,
            )

            # In kết quả nhận diện ra Terminal (chống lặp log quá dày)
            now = time.time()
            if detections_info:
                current_sig = tuple(sorted(d[0] for d in detections_info))
                if (
                    current_sig != self.last_detected_signature
                    or (now - self.last_log_time) >= self.log_interval_sec
                ):
                    summary_str = ", ".join(
                        f"{cls_name} ({conf * 100:.1f}% tại [{b[0]},{b[1]},{b[2]},{b[3]}])"
                        for cls_name, conf, b in detections_info
                    )
                    print(
                        f"[YOLO DETECTED] Frame #{self.frame_count} ({avg_fps:.1f} FPS) -> "
                        f"Phát hiện {len(detections_info)} vật thể: {summary_str}"
                    )
                    self.last_log_time = now
                    self.last_detected_signature = current_sig
                    self.last_heartbeat_time = now
            else:
                self.last_detected_signature = ()
                if (now - self.last_heartbeat_time) >= 10.0:
                    print(
                        f"[YOLO STATUS] Đã xử lý {self.frame_count} frames (~{avg_fps:.1f} FPS, "
                        f"{infer_ms:.1f} ms/frame) từ '{self.image_topic}' - "
                        "Hiện chưa có vật thể thuộc tập COCO trong khung hình."
                    )
                    self.last_heartbeat_time = now

            # Lưu ảnh minh chứng tự động vào thư mục results/
            if self.save_snapshots:
                self._maybe_save_snapshot(annotated_frame, bool(detections_info), now)

            # Publish ảnh đã vẽ bounding box ra topic /yolo/annotated_image
            out_msg = self._cv2_to_ros_img(annotated_frame, msg.header)
            if out_msg is not None:
                self.annotated_pub.publish(out_msg)

            # Hiển thị cửa sổ OpenCV nếu được bật và môi trường hỗ trợ GUI
            if self.show_window:
                try:
                    cv2.imshow(self.window_name, annotated_frame)
                    key = cv2.waitKey(1) & 0xFF
                    if key == ord('q'):
                        self.get_logger().info("Người dùng nhấn 'q' trên cửa sổ OpenCV. Tắt hiển thị cửa sổ.")
                        cv2.destroyAllWindows()
                        self.show_window = False
                except Exception as gui_exc:
                    self.get_logger().warn(
                        f"Không thể mở cửa sổ OpenCV GUI ({gui_exc}). "
                        f"Chuyển sang chế độ chỉ publish qua topic '{self.annotated_topic}'."
                    )
                    self.show_window = False

        except Exception as exc:
            self.get_logger().error(f"Lỗi trong chu trình xử lý ảnh YOLO: {exc}")
        finally:
            self._is_processing = False

    def _maybe_save_snapshot(self, frame: np.ndarray, has_detections: bool, now: float) -> None:
        """Lưu ảnh chụp khung hình đã qua xử lý để phục vụ kiểm thử và chèn báo cáo."""
        try:
            os.makedirs(self.snapshot_dir, exist_ok=True)
            if not self.saved_first_frame:
                first_path = os.path.join(self.snapshot_dir, 'yolo_latest_frame.jpg')
                cv2.imwrite(first_path, frame)
                self.saved_first_frame = True

            if has_detections and (now - self.last_snapshot_time) >= 3.0:
                det_path = os.path.join(self.snapshot_dir, 'yolo_latest_detection.jpg')
                cv2.imwrite(det_path, frame)
                self.last_snapshot_time = now
        except Exception:
            pass


def parse_arguments(argv: List[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description='Node nhận diện vật thể từ camera RGB của TurtleBot 4 bằng Ultralytics YOLO'
    )
    parser.add_argument(
        '--image-topic',
        type=str,
        default=None,
        help='Topic ảnh RGB đầu vào (mặc định: /oakd/rgb/preview/image_raw)'
    )
    parser.add_argument(
        '--annotated-topic',
        type=str,
        default=None,
        help='Topic publish ảnh đã vẽ bounding box (mặc định: /yolo/annotated_image)'
    )
    parser.add_argument(
        '--model',
        type=str,
        default=None,
        help='Đường dẫn hoặc tên trọng số YOLO (mặc định: yolov8n.pt, hỗ trợ yolo11n.pt)'
    )
    parser.add_argument(
        '--conf',
        type=float,
        default=None,
        help='Ngưỡng độ tin cậy confidence (mặc định: 0.45)'
    )
    parser.add_argument(
        '--iou',
        type=float,
        default=None,
        help='Ngưỡng IoU cho NMS (mặc định: 0.45)'
    )
    parser.add_argument(
        '--device',
        type=str,
        default=None,
        help='Thiết bị chạy suy luận: cpu, 0, cuda:0 (mặc định: tự động)'
    )
    parser.add_argument(
        '--qos',
        choices=['best_effort', 'reliable'],
        default=None,
        help='Chính sách QoS Reliability cho subscriber camera (mặc định: best_effort)'
    )
    parser.add_argument(
        '--log-interval',
        type=float,
        default=None,
        help='Khoảng cách thời gian tối thiểu (giây) giữa 2 lần in cùng kết quả lên Terminal (mặc định: 2.0s)'
    )
    parser.add_argument(
        '--show-window',
        action='store_true',
        default=False,
        help='Bật hiển thị cửa sổ cv2.imshow'
    )
    parser.add_argument(
        '--no-show-window',
        action='store_true',
        default=False,
        help='Tắt cửa sổ cv2.imshow (chỉ publish ra topic /yolo/annotated_image)'
    )
    parser.add_argument(
        '--no-save-snapshots',
        action='store_true',
        default=False,
        help='Không tự động lưu ảnh chụp kết quả vào thư mục results/'
    )
    clean_args = remove_ros_args(args=argv)[1:]
    parsed, _ = parser.parse_known_args(clean_args)
    return parsed


def main(args=None):
    cli_args = parse_arguments(sys.argv if args is None else args)
    rclpy.init(args=args)

    node = None
    try:
        node = YoloDetectorNode(cli_args)
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n[THÔNG BÁO] Đang dừng node nhận diện vật thể YOLO...")
    except RuntimeError as exc:
        print(f"[DỪNG NODE] {exc}")
    finally:
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
