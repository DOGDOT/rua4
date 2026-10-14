#!/usr/bin/env python3
"""
Tự động gửi mục tiêu Navigation (Nav2 Goal) cho TurtleBot 4 (Phần 4)
Có thể chọn các goal tại các khu vực khác nhau hoặc nhập tọa độ tùy ý.
"""

import sys
import time
import argparse
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


def create_pose(navigator: BasicNavigator, x: float, y: float, yaw: float = 0.0) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = 0.0

    # Chuyển đổi yaw thành quaternion z, w
    import math
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main():
    parser = argparse.ArgumentParser(description='Gửi mục tiêu di chuyển tự động (Nav2 Goal) cho TurtleBot 4')
    parser.add_argument('--x', type=float, default=None, help='Tọa độ X mục tiêu')
    parser.add_argument('--y', type=float, default=None, help='Tọa độ Y mục tiêu')
    parser.add_argument('--yaw', type=float, default=0.0, help='Góc hướng mục tiêu (radians)')
    parser.add_argument('--preset', choices=['warehouse', 'maze', 'custom'], default='warehouse',
                        help='Tập điểm mẫu tại các khu vực khác nhau')
    args = parser.parse_args()

    rclpy.init()
    navigator = BasicNavigator()

    print("=" * 65)
    print("   [rua4] BỘ ĐIỀU KHIỂN TỰ ĐỘNG NAV2 (AUTONOMOUS NAVIGATION)")
    print("=" * 65)
    print("Đang chờ Nav2 stack sẵn sàng (Lifecycle active)...")
    navigator.waitUntilNav2Active()
    print("[THÀNH CÔNG] Nav2 đã sẵn sàng nhận lệnh lập kế hoạch đường đi!")

    # Xác định danh sách mục tiêu
    goals = []
    if args.x is not None and args.y is not None:
        goals.append(("Mục tiêu tùy chỉnh", args.x, args.y, args.yaw))
    elif args.preset == 'warehouse':
        goals = [
            ("Khu vực 1 - Hành lang kho A", 2.0, 0.0, 0.0),
            ("Khu vực 2 - Gian hàng B", 3.5, 2.0, 1.57),
            ("Khu vực 3 - Điểm trung chuyển", 1.0, 2.5, 3.14),
            ("Khu vực 4 - Trở về vị trí ban đầu (Dock)", 0.0, 0.0, 0.0),
        ]
    elif args.preset == 'maze':
        goals = [
            ("Khu vực 1 - Ngã rẽ thứ nhất", 1.5, 0.0, 0.0),
            ("Khu vực 2 - Gian phòng mê cung", 1.5, 1.5, 1.57),
            ("Khu vực 3 - Lối ra mê cung", 0.0, 1.5, 3.14),
            ("Khu vực 4 - Trở về vạch xuất phát", 0.0, 0.0, 0.0),
        ]

    for idx, (name, gx, gy, gyaw) in enumerate(goals, 1):
        print("\n" + "-" * 65)
        print(f"[{idx}/{len(goals)}] BẮT ĐẦU DI CHUYỂN ĐẾN: {name}")
        print(f"      Tọa độ mục tiêu: x={gx:.2f}, y={gy:.2f}, yaw={gyaw:.2f} rad")
        print("      Robot tự lập kế hoạch toàn cục (Global Path) và tránh vật cản...")
        print("-" * 65)

        target_pose = create_pose(navigator, gx, gy, gyaw)
        navigator.goToPose(target_pose)

        i = 0
        while not navigator.isTaskComplete():
            i += 1
            feedback = navigator.getFeedback()
            if feedback and i % 5 == 0:
                dist = feedback.distance_remaining
                print(f"  -> Đang di chuyển... Khoảng cách còn lại: {dist:.2f} m")
            time.sleep(0.5)

        result = navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            print(f"[HOÀN THÀNH] Robot đã đến đích thành công: {name}!")
        elif result == TaskResult.CANCELED:
            print(f"[ĐÃ HỦY] Mục tiêu {name} bị hủy bỏ.")
            break
        elif result == TaskResult.FAILED:
            print(f"[THẤT BẠI] Không thể đến mục tiêu {name} (Có thể bị kẹt hoặc đường đi bị chặn).")
            break

        time.sleep(2.0)

    print("\n" + "=" * 65)
    print("   HOÀN THÀNH CHUỖI NHIỆM VỤ NAV2!")
    print("=" * 65 + "\n")

    navigator.lifecycleShutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
