#!/usr/bin/env python3
"""
Kiểm tra các topic cơ bản và các topic phục vụ Bài thực hành 05 của TurtleBot 4:
  - Cơ bản (Phần 1): /scan, /odom, /cmd_vel
  - Mở rộng (--lab5): /map, /amcl_pose, /tf, /oakd/rgb/preview/image_raw
"""

import sys
import time
import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage


class TopicChecker(Node):
    def __init__(self, check_lab5: bool = False, image_topic: str = '/oakd/rgb/preview/image_raw'):
        super().__init__('rua4_topic_checker')
        self.check_lab5 = check_lab5
        self.image_topic = image_topic

        # Best effort QoS for sensor scans & camera (matching Gazebo bridge)
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Reliable QoS for odom & tf
        reliable_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        # Transient Local QoS for latched topics (/map, /amcl_pose)
        latched_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.scan_count = 0
        self.odom_count = 0
        self.latest_scan = None
        self.latest_odom = None

        self.map_msg = None
        self.amcl_msg = None
        self.image_count = 0
        self.latest_image = None
        self.tf_frames = set()

        self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)
        self.create_subscription(Odometry, '/odom', self.odom_callback, reliable_qos)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        if self.check_lab5:
            self.create_subscription(OccupancyGrid, '/map', self.map_callback, latched_qos)
            self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_callback, latched_qos)
            self.create_subscription(TFMessage, '/tf', self.tf_callback, reliable_qos)
            self.create_subscription(Image, self.image_topic, self.image_callback, sensor_qos)

    def scan_callback(self, msg: LaserScan):
        self.scan_count += 1
        self.latest_scan = msg

    def odom_callback(self, msg: Odometry):
        self.odom_count += 1
        self.latest_odom = msg

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg

    def amcl_callback(self, msg: PoseWithCovarianceStamped):
        self.amcl_msg = msg

    def tf_callback(self, msg: TFMessage):
        for t in msg.transforms:
            self.tf_frames.add(f"{t.header.frame_id} -> {t.child_frame_id}")

    def image_callback(self, msg: Image):
        self.image_count += 1
        self.latest_image = msg


def main(args=None):
    parser = argparse.ArgumentParser(description='Kiểm tra các topic cảm biến, Localization và Camera của TurtleBot 4')
    parser.add_argument(
        '--lab5',
        action='store_true',
        default=False,
        help='Kiểm tra thêm /map, /amcl_pose, /tf và camera RGB (/oakd/rgb/preview/image_raw) cho Bài thực hành 05'
    )
    parser.add_argument(
        '--image-topic',
        type=str,
        default='/oakd/rgb/preview/image_raw',
        help='Tên topic camera RGB cần kiểm tra (mặc định: /oakd/rgb/preview/image_raw)'
    )
    clean_args = remove_ros_args(args=sys.argv if args is None else args)[1:]
    cli_args, _ = parser.parse_known_args(clean_args)

    rclpy.init(args=args)
    checker = TopicChecker(check_lab5=cli_args.lab5, image_topic=cli_args.image_topic)

    print("=" * 68)
    if cli_args.lab5:
        print("   [rua4] KIỂM TRA HỆ THỐNG BÀI THỰC HÀNH 05 (SIM + AMCL + CAMERA)")
    else:
        print("   [rua4] KIỂM TRA CÁC TOPIC CƠ BẢN CỦA TURTLEBOT 4")
    print("=" * 68)
    print("Đang lắng nghe dữ liệu từ hệ thống ROS 2 trong 4 giây...")

    start_time = time.time()
    while rclpy.ok() and (time.time() - start_time) < 4.0:
        rclpy.spin_once(checker, timeout_sec=0.1)

    sub_count = checker.cmd_vel_pub.get_subscription_count()
    if not cli_args.lab5:
        # Chỉ gửi thử nghiệm Twist=0 ở Phần 1 (không phát xung vào /cmd_vel khi đang kiểm tra Lab 5)
        stop_msg = Twist()
        checker.cmd_vel_pub.publish(stop_msg)

    print("\n" + "=" * 68)
    print("   KẾT QUẢ KIỂM TRA TOPIC HỆ THỐNG")
    print("=" * 68)

    # 1. /scan
    if checker.scan_count > 0 and checker.latest_scan is not None:
        valid_ranges = [r for r in checker.latest_scan.ranges if not (sys.float_info.max == r or r != r)]
        print(f"[PASS] /scan")
        print(f"       + Đã nhận: {checker.scan_count} messages (~{checker.scan_count/4.0:.1f} Hz)")
        print(f"       + Frame ID: '{checker.latest_scan.header.frame_id}'")
        print(f"       + Số tia quét: {len(checker.latest_scan.ranges)}")
        if valid_ranges:
            print(f"       + Khoảng cách quét: {min(valid_ranges):.2f}m -> {max(valid_ranges):.2f}m")
    else:
        print("[FAIL] /scan - Chưa nhận được dữ liệu LiDAR từ Gazebo.")

    print("-" * 68)

    # 2. /odom
    if checker.odom_count > 0 and checker.latest_odom is not None:
        pos = checker.latest_odom.pose.pose.position
        print(f"[PASS] /odom")
        print(f"       + Đã nhận: {checker.odom_count} messages (~{checker.odom_count/4.0:.1f} Hz)")
        print(f"       + Frame ID: '{checker.latest_odom.header.frame_id}' -> '{checker.latest_odom.child_frame_id}'")
        print(f"       + Tọa độ hiện tại: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}")
    else:
        print("[FAIL] /odom - Chưa nhận được dữ liệu Odometry.")

    print("-" * 68)

    # 3. /cmd_vel
    print(f"[PASS] /cmd_vel")
    print(f"       + Số node subscriber đang lắng nghe /cmd_vel: {sub_count}")

    if cli_args.lab5:
        print("-" * 68)
        # 4. /map
        if checker.map_msg is not None:
            info = checker.map_msg.info
            print(f"[PASS] /map (nav_msgs/msg/OccupancyGrid)")
            print(
                f"       + Kích thước: {info.width}x{info.height} ô | "
                f"Độ phân giải: {info.resolution:.3f} m/pixel"
            )
            print(
                f"       + Gốc tọa độ (Origin): x={info.origin.position.x:.2f}, "
                f"y={info.origin.position.y:.2f}"
            )
        else:
            print("[FAIL] /map - Chưa nhận được bản đồ từ map_server (Kiểm tra localization/nav2 launch).")

        print("-" * 68)
        # 5. /amcl_pose & /tf
        if checker.amcl_msg is not None:
            apos = checker.amcl_msg.pose.pose.position
            print(f"[PASS] /amcl_pose (Định vị AMCL)")
            print(f"       + Pose ước lượng trên map: x={apos.x:.3f}, y={apos.y:.3f}")
        else:
            print(
                "[WARN] /amcl_pose - Chưa nhận được pose từ AMCL "
                "(Hãy đặt '2D Pose Estimate' trên RViz2 hoặc di chuyển nhẹ robot)."
            )

        has_map_odom = any('map -> odom' in pair for pair in checker.tf_frames)
        print(f"[{'PASS' if has_map_odom else 'WARN'}] /tf (Biến đổi hệ tọa độ)")
        print(f"       + Liên kết 'map -> odom' : {'ĐÃ CÓ' if has_map_odom else 'CHƯA CÓ'}")
        if checker.tf_frames:
            sample_tf = ", ".join(sorted(checker.tf_frames)[:5])
            print(f"       + Các nhánh TF ghi nhận : {sample_tf}")

        print("-" * 68)
        # 6. Camera RGB
        if checker.image_count > 0 and checker.latest_image is not None:
            img = checker.latest_image
            print(f"[PASS] {cli_args.image_topic} (sensor_msgs/msg/Image)")
            print(
                f"       + Đã nhận: {checker.image_count} frames (~{checker.image_count/4.0:.1f} Hz) | "
                f"Kích thước: {img.width}x{img.height} | Encoding: '{img.encoding}'"
            )
        else:
            print(f"[FAIL] {cli_args.image_topic} - Chưa nhận được khung hình từ camera RGB.")
            print("       + Gợi ý: Kiểm tra camera_bridge hoặc chạy 'ros2 topic list | grep image'.")

    print("=" * 68)
    all_ok = (checker.scan_count > 0) and (checker.odom_count > 0)
    if cli_args.lab5:
        all_ok = all_ok and (checker.map_msg is not None) and (checker.image_count > 0)

    if all_ok:
        print(">> TẤT CẢ CÁC TOPIC KIỂM TRA ĐỀU HOẠT ĐỘNG TỐT! <<")
    else:
        print(">> MỘT SỐ TOPIC CHƯA NHẬN ĐƯỢC DỮ LIỆU. VUI LÒNG XEM CHI TIẾT Ở TRÊN <<")
    print("=" * 68 + "\n")

    checker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
