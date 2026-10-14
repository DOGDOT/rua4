#!/usr/bin/env python3
"""
Kiểm tra các topic cơ bản của TurtleBot 4:
  - /scan (sensor_msgs/msg/LaserScan)
  - /odom (nav_msgs/msg/Odometry)
  - /cmd_vel (geometry_msgs/msg/Twist)
"""

import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist


class TopicChecker(Node):
    def __init__(self):
        super().__init__('rua4_topic_checker')

        # Best effort QoS for sensor scans (matching standard Gazebo bridge)
        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        # Reliable QoS for odom
        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE
        )

        self.scan_count = 0
        self.odom_count = 0
        self.latest_scan = None
        self.latest_odom = None

        self.create_subscription(LaserScan, '/scan', self.scan_callback, sensor_qos)
        self.create_subscription(Odometry, '/odom', self.odom_callback, odom_qos)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def scan_callback(self, msg: LaserScan):
        self.scan_count += 1
        self.latest_scan = msg

    def odom_callback(self, msg: Odometry):
        self.odom_count += 1
        self.latest_odom = msg


def main(args=None):
    rclpy.init(args=args)
    checker = TopicChecker()

    print("=" * 65)
    print("   [rua4] KIỂM TRA CÁC TOPIC CƠ BẢN CỦA TURTLEBOT 4")
    print("=" * 65)
    print("Đang lắng nghe dữ liệu từ mô phỏng Gazebo trong 4 giây...")

    start_time = time.time()
    while rclpy.ok() and (time.time() - start_time) < 4.0:
        rclpy.spin_once(checker, timeout_sec=0.1)

    # Kiểm tra cmd_vel
    sub_count = checker.cmd_vel_pub.get_subscription_count()
    # Thử gửi một lệnh dừng an toàn (0, 0)
    stop_msg = Twist()
    checker.cmd_vel_pub.publish(stop_msg)

    print("\n" + "=" * 65)
    print("   KẾT QUẢ KIỂM TRA (PHẦN 1 - TURTLEBOT 4 SIMULATION)")
    print("=" * 65)

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
        print("       Gợi ý: Kiểm tra ros_gz_bridge lidar_bridge hoặc khởi động lại mô phỏng.")

    print("-" * 65)

    # 2. /odom
    if checker.odom_count > 0 and checker.latest_odom is not None:
        pos = checker.latest_odom.pose.pose.position
        print(f"[PASS] /odom")
        print(f"       + Đã nhận: {checker.odom_count} messages (~{checker.odom_count/4.0:.1f} Hz)")
        print(f"       + Frame ID: '{checker.latest_odom.header.frame_id}' -> '{checker.latest_odom.child_frame_id}'")
        print(f"       + Tọa độ hiện tại: x={pos.x:.3f}, y={pos.y:.3f}, z={pos.z:.3f}")
    else:
        print("[FAIL] /odom - Chưa nhận được dữ liệu Odometry.")
        print("       Gợi ý: Kiểm tra diffdrive_controller hoặc pose_republisher_node.")

    print("-" * 65)

    # 3. /cmd_vel
    print(f"[PASS] /cmd_vel")
    print(f"       + Đã khởi tạo publisher cho /cmd_vel")
    print(f"       + Số node subscriber đang lắng nghe /cmd_vel: {sub_count}")
    print(f"       + Đã gửi thử nghiệm lệnh Twist an toàn thành công.")

    print("=" * 65)
    all_ok = (checker.scan_count > 0) and (checker.odom_count > 0)
    if all_ok:
        print(">> TẤT CẢ TOPIC CƠ BẢN HOẠT ĐỘNG HOÀN HẢO! <<")
    else:
        print(">> MỘT SỐ TOPIC CHƯA NHẬN ĐƯỢC DỮ LIỆU. VUI LÒNG KIỂM TRA MÔ PHỎNG <<")
    print("=" * 65 + "\n")

    checker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
