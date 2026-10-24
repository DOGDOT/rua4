# Package `rua4` - TurtleBot 4 Simulation, SLAM, Localization, Nav2, YOLO & Hand Gesture HRI

Package ROS 2 Humble cho **TurtleBot 4** trong môi trường mô phỏng **Gazebo Sim (Ignition)**, tích hợp đầy đủ chuỗi bài thực hành từ cơ bản đến nâng cao theo trình tự:
1. **Phần 1 – Simulation**: Khởi chạy robot trong Gazebo, kiểm tra `/scan`, `/odom`, `/cmd_vel`, hiển thị trực quan trên RViz2.
2. **Phần 2 – Xây dựng bản đồ (SLAM)**: Khảo sát môi trường bằng SLAM Toolbox + Teleop bàn phím, xuất file `map.yaml` và `map.pgm`.
3. **Phần 3 – Localization**: Nạp bản đồ đã lưu (`map.yaml`), định vị robot bằng AMCL (`2D Pose Estimate`), đối chiếu vị trí thực trong Gazebo.
4. **Phần 4 – Autonomous Navigation (Nav2)**: Tự động dẫn đường đến các mục tiêu (`Nav2 Goal`) tại các khu vực khác nhau mà không va chạm.
5. **Bài thực hành 05 – Điều hướng Tự động qua chuỗi Goal bằng Code & Nhận diện Vật thể Camera RGB với YOLO**:
   - Nạp bản đồ đã lưu (`maps/map.yaml`), điều hướng tự động tuần tự qua chuỗi Goal (`Goal 1 → Goal 2 → Goal 3`) bằng `nav_commander.py` song song với nhận diện vật thể bằng **Ultralytics YOLOv8** (`yolo_detector.py`).
6. **Bài tập 06 – Giao tiếp Phi Ngôn ngữ & Điều khiển TurtleBot 4 bằng Cử chỉ Tay (MediaPipe + OpenCV)**:
   - **`gesture_recognition_node`**: Đọc hình ảnh từ webcam laptop bằng OpenCV, sử dụng **MediaPipe Gesture Recognizer** (`gesture_recognizer.task`) nhận diện **06 cử chỉ tay**, hiển thị tên cử chỉ + độ tin cậy (%) + trạng thái robot trên cửa sổ webcam và publish lên `/detected_gesture` (`std_msgs/msg/String`).
   - **`robot_behavior_node`**: Subscribe `/detected_gesture`, lọc nhiễu 2 tầng, thực thi quy trình **Chọn hành vi → Xác nhận (`Thumbs Up`) → Thực thi** và điều khiển TurtleBot 4 trong Gazebo qua `/cmd_vel` (`geometry_msgs/msg/Twist`).

---

## 📁 Cấu trúc Thư mục Package

```
rua4/
├── package.xml                     # Khai báo metadata & dependencies (rclpy, std_msgs, geometry_msgs...)
├── setup.py                        # Cấu hình cài đặt Python package & các console_scripts
├── setup.cfg
├── rua4/                           # Module mã nguồn Python
│   ├── __init__.py
│   ├── check_topics.py             # Kiểm tra /scan, /odom, /cmd_vel và mở rộng (--lab5)
│   ├── save_map.py                 # Tiện ích lưu bản đồ map.yaml & map.pgm (Phần 2)
│   ├── nav_commander.py            # Node điều hướng tự động qua chuỗi >= 3 Goal (Bài 05)
│   ├── yolo_detector.py            # Node nhận diện vật thể camera RGB bằng YOLO (Bài 05)
│   ├── gesture_recognition_node.py # Node AI nhận diện 06 cử chỉ tay qua webcam (Bài 06)
│   ├── robot_behavior_node.py      # Node điều khiển hành vi TurtleBot 4 theo cử chỉ (Bài 06)
│   └── undock.py                   # Tiện ích rời trạm sạc tự động
├── launch/                         # Các file launch chuyên biệt
│   ├── sim.launch.py               # Launch Gazebo + Spawn TurtleBot 4 (Phần 1 & Bài 06)
│   ├── slam.launch.py              # Launch SLAM Toolbox + RViz2 (Phần 2)
│   ├── localization.launch.py      # Launch AMCL + Map Server với maps/map.yaml (Phần 3)
│   ├── nav2.launch.py              # Launch AMCL + Map Server + Nav2 full stack + RViz2 (Phần 4 & Bài 05)
│   ├── lab5.launch.py              # Launch riêng yolo_detector + nav_commander (Bài 05)
│   ├── lab6.launch.py              # Launch đồng thời gesture_recognition_node + robot_behavior_node (Bài 06)
│   ├── tb4_all_in_one.launch.py    # File launch tổng hợp (mode:=sim|slam|localization|nav2)
│   ├── turtlebot4_spawn.launch.py
│   └── include/                    # Cấu hình controller Create 3 & TF
├── config/                         # Cấu hình tham số & trọng số mô hình
│   ├── slam.yaml                   # Cấu hình SLAM Toolbox tối ưu scan matching
│   ├── localization.yaml           # Cấu hình AMCL Particle Filter (use_sim_time: True)
│   ├── nav2.yaml                   # Cấu hình Nav2 Planner, Controller, Costmaps
│   ├── lab5_goals.yaml             # Danh sách tọa độ Goal (x, y, yaw) cho Bài thực hành 05
│   ├── gesture_recognizer.task     # Trọng số chuẩn của MediaPipe Gesture Recognizer (Bài 06)
│   └── rviz/                       # Cấu hình giao diện RViz2 riêng cho từng phần
├── maps/                           # Thư mục chứa bản đồ Occupancy Grid (yaml, pgm)
└── results/                        # Kết quả chạy thực tế, ảnh chụp nhận diện YOLO & Cử chỉ tay
    ├── gesture_*.jpg               # Ảnh chụp nhận diện 06 cử chỉ tay thực tế (Bài 06)
    ├── yolo_*.jpg                  # Ảnh nhận diện camera RGB với YOLOv8 (Bài 05)
    └── maps/                       # Bản đồ xuất từ SLAM Toolbox (Phần 2)
```

---

## 🛠️ Yêu cầu Môi trường & Chuẩn bị Build

### 1. Yêu cầu hệ thống
- **Hệ điều hành**: Ubuntu 22.04 LTS *(Ghi chú: Nếu máy đang chạy phiên bản Ubuntu/Linux khác, có thể khởi chạy môi trường Ubuntu 22.04 qua Docker/Distrobox trước khi thực hiện các lệnh bên dưới)*.
- **Middleware**: ROS 2 Humble Hawksbill.
- **Trình mô phỏng & Điều hướng**: Gazebo Sim (Ignition Fortress), RViz2, `turtlebot4_ignition_bringup`, `slam_toolbox`, `navigation2`, `nav2_simple_commander`.
- **Thị giác máy tính & AI**:
  - Bài thực hành 05: `cv_bridge`, `opencv-python`, `ultralytics` (YOLOv8), `numpy<2`.
  - Bài tập 06: Webcam laptop/USB (`/dev/video0`), `opencv-python`, `mediapipe`, `numpy<2`.

### 2. Source môi trường & Build Package
Tại mỗi Terminal mới, di chuyển vào workspace (ví dụ `~/ur_ws`) và nạp môi trường:

```bash
cd ~/ur_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

*(Khi lần đầu tải về hoặc sau khi chỉnh sửa code trong package `rua4`, thực hiện build lại bằng lệnh:)*
```bash
cd ~/ur_ws
colcon build --symlink-install --packages-select rua4
source install/setup.bash
```

---

## 🚀 Hướng dẫn Thực hiện Chi tiết Các Phần 1 – 4

### 📍 Phần 1 – TurtleBot 4 Simulation

#### Bước 1: Khởi chạy mô phỏng và RViz2
Mở **Terminal 1**:
```bash
cd ~/ur_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 launch rua4 sim.launch.py world:=warehouse
```

#### Bước 2: Kiểm tra các topic cơ bản
Mở **Terminal 2**:
```bash
cd ~/ur_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 run rua4 check_topics
```

---

### 🗺️ Phần 2 – Xây dựng bản đồ bằng SLAM Toolbox

*Giữ nguyên Terminal 1 đang chạy mô phỏng Gazebo.*

#### Bước 1: Khởi chạy SLAM Toolbox (Terminal 2)
```bash
cd ~/ur_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 launch rua4 slam.launch.py
```

#### Bước 2: Điều khiển robot quét bản đồ (Terminal 3)
```bash
cd ~/ur_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### Bước 3: Lưu bản đồ (Terminal 4)
```bash
cd ~/ur_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 run rua4 save_map
```

---

### 📍 Phần 3 – Localization

Mở **Terminal 2**:
```bash
cd ~/ur_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 launch rua4 localization.launch.py
```

---

### 🎯 Phần 4 – Navigation với Nav2

Mở **Terminal 2**:
```bash
cd ~/ur_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 launch rua4 nav2.launch.py
```

---

## 🤖 Hướng dẫn Thực hiện Chi tiết BÀI THỰC HÀNH 05 (Localization + Multi-Goal Nav2 + YOLO)

### Bước 1 – Cài đặt thư viện phụ thuộc (chỉ cần chạy 1 lần)
```bash
sudo apt update
sudo apt install -y ros-humble-cv-bridge ros-humble-nav2-simple-commander python3-opencv python3-yaml
pip install "numpy<2" ultralytics
```

### Bước 2 – Khởi chạy Gazebo (Terminal 1) và Nav2 với Bản đồ cũ (Terminal 2)
- **Terminal 1**:
  ```bash
  cd ~/ur_ws
  source /opt/ros/humble/setup.bash && source install/setup.bash
  ros2 launch rua4 sim.launch.py world:=warehouse model:=standard
  ```
- **Terminal 2**:
  ```bash
  cd ~/ur_ws
  source /opt/ros/humble/setup.bash && source install/setup.bash
  ros2 launch rua4 nav2.launch.py
  ```

### Bước 3 – Khởi chạy YOLO Detector & Điều hướng Tự động Đa điểm (Terminal 3)
```bash
cd ~/ur_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 launch rua4 lab5.launch.py preset:=warehouse
```
*(Hoặc chạy tách riêng `ros2 run rua4 yolo_detector` ở Terminal 3 và `ros2 run rua4 nav_commander --preset warehouse` ở Terminal 4).*

---

## 🖐️ Hướng dẫn Thực hiện Chi tiết BÀI TẬP 06 (Giao tiếp Phi Ngôn ngữ & Điều khiển Cử chỉ Tay)

### 1. Bảng Ánh xạ 06 Cử chỉ Tay và Hành vi TurtleBot 4

| STT | Cử chỉ tay | Nhãn trên `/detected_gesture` | Cách thực hiện trước Webcam | Hành vi của TurtleBot 4 |
| :---: | :--- | :--- | :--- | :--- |
| 1 | **Vẫy tay (Wave)** | `Wave` | Xòe bàn tay và vẫy qua lại trái–phải *(hoặc giơ cử chỉ I-Love-You)* | Chọn hành vi: **Robot xoay trái–phải để chào lại** |
| 2 | **Giơ 1 ngón tay (Pointing Up)** | `Pointing_Up` | Giơ 1 ngón trỏ hướng thẳng lên | Chọn hành vi: **Tiến–lùi 4 lần** |
| 3 | **Giơ 2 ngón tay (Victory)** | `Victory` | Giơ 2 ngón trỏ & giữa (chữ V) | Chọn hành vi: **Di chuyển theo quỹ đạo tròn** |
| 4 | **Giơ ngón cái (Thumbs Up)** | `Thumb_Up` | Nắm 4 ngón, giơ ngón cái hướng lên | **Xác nhận và thực hiện** hành vi đã chọn |
| 5 | **Xòe bàn tay (Open Palm)** | `Open_Palm` | Xòe 5 ngón tay và **giữ yên** | **Robot dừng ngay lập tức** (`cmd_vel = 0`) |
| 6 | **Nắm tay (Closed Fist)** | `Closed_Fist` | Nắm chặt 5 ngón tay | **Hủy hành vi hiện tại** và trở về trạng thái chờ (`IDLE`) |

> **Cơ chế an toàn & chống nhiễu**:
> - **Xác nhận 2 bước**: Robot chỉ bắt đầu di chuyển sau khi người dùng đã chọn hành vi (`Wave`, `Pointing_Up`, hoặc `Victory`) **VÀ** giơ ngón cái (`Thumb_Up`) để xác nhận. *(Nếu muốn `Wave` chào lại ngay lập tức không cần chờ `Thumb_Up`, có thể truyền thêm cờ `--wave-immediate`)*.
> - **Chống nhiễu 2 tầng**: Lọc bỏ phiếu đa số trên cửa sổ trượt 5 frame tại `gesture_recognition_node` + Khóa sườn lên (Edge-Trigger Latch) & Cooldown (`0.8s`) tại `robot_behavior_node` để giữ nguyên tay không bị kích hoạt lặp lại lệnh liên tục.
> - **Tự động dừng khi mất tín hiệu**: Nếu `robot_behavior_node` không nhận được bản tin từ `/detected_gesture` quá `1.5s` (webcam ngắt kết nối hoặc tắt node nhận diện), robot lập tức dừng an toàn.

---

### 2. Các bước Cài đặt & Chạy Demo Bài tập 06

#### Bước 0 – Cài đặt thư viện MediaPipe & Build Package (chỉ cần chạy 1 lần)
```bash
pip install "numpy<2" mediapipe opencv-python

cd ~/ur_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select rua4
source install/setup.bash

# Kiểm tra các executable đã đăng ký (phải có gesture_recognition_node và robot_behavior_node):
ros2 pkg executables rua4
```

#### Bước 1 – Khởi chạy TurtleBot 4 trong Gazebo (Terminal 1)
Mở **Terminal 1** và khởi chạy TurtleBot 4 tại vùng sàn trống rộng rãi trong môi trường Gazebo:
```bash
cd ~/ur_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 launch rua4 sim.launch.py world:=warehouse
```

*(Kiểm tra nhanh robot nhận lệnh vận tốc qua `/cmd_vel` trước khi chạy AI nếu cần:)*
```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}, angular: {z: 0.0}}"
```

#### Bước 2 – Khởi chạy Node Nhận diện Cử chỉ Tay qua Webcam (Terminal 2)
Mở **Terminal 2**:
```bash
cd ~/ur_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 run rua4 gesture_recognition_node
```
* Cửa sổ **`TurtleBot 4 - AI Hand Gesture Recognition (rua4)`** sẽ mở lên, hiển thị hình ảnh từ webcam laptop cùng khung xương bàn tay 21 điểm, tên cử chỉ, độ tin cậy (`%`) và trạng thái hiện tại của robot.
* Nếu máy có nhiều camera (ví dụ webcam rời ở `/dev/video2`), truyền thêm `--camera-id 2`.

#### Bước 3 – Khởi chạy Node Điều khiển Hành vi Robot (Terminal 3)
Mở **Terminal 3**:
```bash
cd ~/ur_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 run rua4 robot_behavior_node
```
*(Mẹo: Có thể chạy gộp cả Bước 2 và Bước 3 chỉ bằng 1 lệnh launch duy nhất: `ros2 launch rua4 lab6.launch.py`)*.

#### Bước 4 – Kiểm tra Topic `/detected_gesture` & Kịch bản Thực hiện Demo
Có thể mở thêm 1 terminal phụ để quan sát dữ liệu thời gian thực trên topic `/detected_gesture`:
```bash
ros2 topic echo /detected_gesture
```
---

## 🔧 Xử lý Lỗi Thường Gặp (Troubleshooting)

1. **Lỗi `TypeError: canonicalize_version() got an unexpected keyword argument 'strip_trailing_zero'` khi chạy `colcon build`**:
   - **Nguyên nhân**: `setuptools` bản mới (`>=70`) vô tình được cài vào thư mục người dùng (`~/.local/lib/python3.10/site-packages`), xung đột với thư viện `packaging` của Ubuntu 22.04.
   - **Khắc phục**: Gỡ bỏ setuptools trong user space để sử dụng phiên bản hệ thống chuẩn:
     ```bash
     pip uninstall -y setuptools
     colcon build --symlink-install --packages-select rua4
     ```

2. **Lỗi `AttributeError: _ARRAY_API not found` khi import `cv_bridge`**:
   - **Khắc phục**: Ghim phiên bản numpy `< 2`:
     ```bash
     pip install "numpy<2"
     ```

3. **Cửa sổ Webcam không mở hoặc lỗi `/dev/video0`**:
   - Nếu dùng webcam ngoài (USB), kiểm tra danh sách thiết bị bằng `ls -la /dev/video*` và truyền tham số camera:
     ```bash
     ros2 run rua4 gesture_recognition_node --camera-id 2
     ```
