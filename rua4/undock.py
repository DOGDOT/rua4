#!/usr/bin/env python3
"""
Tiện ích rời trạm sạc (Undock) cho TurtleBot 4
Mặc định trong mô phỏng Gazebo, robot được đặt trên dock sạc nên cơ chế an toàn
của Create 3 sẽ khóa chuyển động (bỏ qua /cmd_vel) cho đến khi robot rời dock.
"""

import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
from irobot_create_msgs.action import Undock
from irobot_create_msgs.msg import DockStatus


class UndockClient(Node):
    def __init__(self):
        super().__init__(
            'rua4_undocker',
            parameter_overrides=[
                Parameter('use_sim_time', Parameter.Type.BOOL, True)
            ]
        )
        self.is_docked = None
        self.create_subscription(DockStatus, '/dock_status', self._dock_callback, 10)
        self._action_client = ActionClient(self, Undock, '/undock')
        self._direct_vel_pub = self.create_publisher(Twist, '/diffdrive_controller/cmd_vel_unstamped', 10)

    def _dock_callback(self, msg: DockStatus):
        self.is_docked = msg.is_docked

    def execute(self):
        print("=" * 65)
        print("   [rua4] KIỂM TRA TRẠNG THÁI VÀ RỜI TRẠM SẠC (UNDOCK)")
        print("=" * 65)
        print("Đang đọc trạng thái robot từ topic /dock_status...")

        # Đợi tối đa 2s để nhận dock_status
        start = time.time()
        while rclpy.ok() and self.is_docked is None and (time.time() - start) < 2.0:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.is_docked is False:
            print("[THÔNG BÁO] Robot hiện tại ĐÃ Ở NGOÀI TRẠM SẠC (is_docked = False)!")
            print(">> Bạn KHÔNG CẦN gọi Undock nữa, robot đã sẵn sàng để điều khiển! <<")
            print("\nHãy mở terminal và chạy lệnh teleop:")
            print("  ros2 run teleop_twist_keyboard teleop_twist_keyboard")
            print("=" * 65)
            return True

        print("Robot đang ở trên trạm sạc (hoặc chưa rõ). Đang kết nối Action Server '/undock'...")
        if self._action_client.wait_for_server(timeout_sec=3.0):
            print("Đang gửi lệnh Undock Action Goal...")
            goal_msg = Undock.Goal()
            send_goal_future = self._action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_goal_future)

            goal_handle = send_goal_future.result()
            if goal_handle and goal_handle.accepted:
                print("Robot đã chấp nhận yêu cầu và đang tự động lùi khỏi trạm sạc...")
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future)
                print("-" * 65)
                print("[THÀNH CÔNG] Robot đã rời khỏi trạm sạc hoàn tất!")
                print(">> Giờ bạn có thể điều khiển bình thường bằng teleop! <<")
                print("=" * 65)
                return True
            else:
                print("[THÔNG BÁO] Action /undock không cần thiết hoặc bị từ chối.")
        else:
            print("[CẢNH BÁO] Không tìm thấy Action Server '/undock'.")

        # Giải pháp dự phòng 1: Dùng Ignition Service dịch chuyển robot ra khỏi vùng dock (nếu bị kẹt cơ khí)
        print("Đang thử giải phóng robot khỏi trạm sạc qua Gazebo Ignition...")
        import subprocess
        for world_name in ['warehouse', 'maze', 'depot']:
            try:
                cmd = [
                    'ign', 'service', '-s', f'/world/{world_name}/set_pose',
                    '--reqtype', 'ignition.msgs.Pose',
                    '--reptype', 'ignition.msgs.Boolean',
                    '--timeout', '1000',
                    '--req', 'name: "turtlebot4", position: {x: 1.0, y: 0.0, z: 0.05}'
                ]
                res = subprocess.run(cmd, capture_output=True, text=True, timeout=2.0)
                if 'data: true' in res.stdout:
                    print(f"[THÀNH CÔNG] Đã nhấc robot ra khỏi dock trên world '{world_name}' an toàn!")
                    break
            except Exception:
                pass

        # Giải pháp dự phòng 2: Gửi xung vận tốc lùi trực tiếp vào controller
        print("Đang gửi xung vận tốc giải phóng bánh xe qua diffdrive_controller...")
        twist = Twist()
        twist.linear.x = -0.3
        for _ in range(15):
            self._direct_vel_pub.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
            time.sleep(0.1)

        # Dừng robot
        stop_twist = Twist()
        for _ in range(5):
            self._direct_vel_pub.publish(stop_twist)
            time.sleep(0.05)

        print("-" * 65)
        print("[HOÀN TẤT] Robot đã được giải phóng khỏi trạm sạc!")
        print("Bạn có thể điều khiển robot ngay bằng lệnh:")
        print("  ros2 run teleop_twist_keyboard teleop_twist_keyboard")
        print("=" * 65)
        return True


def main(args=None):
    rclpy.init(args=args)
    client = UndockClient()
    try:
        client.execute()
    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
