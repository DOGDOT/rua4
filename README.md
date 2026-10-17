# Package `rua4` - TurtleBot 4 Simulation, SLAM, Localization, Nav2 & YOLO Object Detection

Package ROS 2 Humble cho **TurtleBot 4** trong môi trường mô phỏng **Gazebo Sim (Ignition)**, tích hợp đầy đủ các bài thực hành từ cơ bản đến nâng cao:
1. **Phần 1 – Simulation**: Khởi chạy robot trong Gazebo, kiểm tra `/scan`, `/odom`, `/cmd_vel`, hiển thị trực quan trên RViz2.
2. **Phần 2 – Xây dựng bản đồ (SLAM)**: Khảo sát môi trường bằng SLAM Toolbox + Teleop bàn phím, xuất file `map.yaml` và `map.pgm`.
3. **Phần 3 – Localization**: Nạp bản đồ đã lưu (`map.yaml`), định vị robot bằng AMCL (`2D Pose Estimate`), đối chiếu vị trí thực trong Gazebo.
4. **Phần 4 – Autonomous Navigation (Nav2)**: Tự động dẫn đường đến các mục tiêu (`Nav2 Goal`) tại các khu vực khác nhau mà không va chạm.
5. **Bài thực hành 05 – Localization trên Bản đồ cũ, Điều hướng Tự động qua chuỗi Goal bằng Code & Nhận diện Vật thể Camera RGB với YOLO**:
   - Nạp bản đồ đã lưu (`maps/map.yaml` & `maps/map.pgm`), định vị AMCL ổn định.
   - Điều hướng tự động tuần tự qua ít nhất 3 Goal (`Goal 1 → Goal 2 → Goal 3`) bằng code Python (`nav_commander.py` + `config/lab5_goals.yaml`) không dùng chuột hay teleop.
   - Nhận diện vật thể thời gian thực từ camera RGB (`/oakd/rgb/preview/image_raw`) bằng **Ultralytics YOLO** (`yolo_detector.py`), vẽ bounding box + class + confidence, hiển thị cửa sổ OpenCV và publish lên `/yolo/annotated_image` cho RViz2 song song với quá trình robot di chuyển.

---

## 📁 Cấu trúc Thư mục Package

```
/home/gdon/ur_ws/src/rua4/
├── package.xml                 # Khai báo metadata & dependencies (rclpy, cv_bridge, nav2...)
├── setup.py                    # Cấu hình cài đặt Python package & các console_scripts
├── setup.cfg
├── bao_cao_turtlebot4.tex      # Báo cáo LaTeX tổng hợp (Phần 1–4 & Bài thực hành 05)
├── rua4/                       # Module mã nguồn Python
│   ├── __init__.py
│   ├── check_topics.py         # Kiểm tra /scan, /odom, /cmd_vel và mở rộng (--lab5)
│   ├── save_map.py             # Tiện ích lưu bản đồ map.yaml & map.pgm (Phần 2)
│   ├── nav_commander.py        # Node điều hướng tự động qua chuỗi >= 3 Goal (Phần 4 & Bài 05)
│   ├── yolo_detector.py        # Node nhận diện vật thể camera RGB bằng YOLO (Bài 05)
│   └── undock.py               # Tiện ích rời trạm sạc tự động
├── launch/                     # Các file launch chuyên biệt
│   ├── sim.launch.py           # Launch Gazebo + Spawn TurtleBot 4 (Phần 1)
│   ├── slam.launch.py          # Launch SLAM Toolbox + RViz2 (Phần 2)
│   ├── localization.launch.py  # Launch AMCL + Map Server với maps/map.yaml (Phần 3)
│   ├── nav2.launch.py          # Launch AMCL + Map Server + Nav2 full stack + RViz2 (Phần 4 & Bài 05)
│   ├── lab5.launch.py          # Launch riêng yolo_detector + nav_commander (Bài 05)
│   ├── tb4_all_in_one.launch.py# File launch tổng hợp (mode:=sim|slam|localization|nav2)
│   ├── turtlebot4_spawn.launch.py
│   └── include/                # Cấu hình controller Create 3 & TF
├── config/                     # Cấu hình tham số tối ưu cho TurtleBot 4
│   ├── slam.yaml               # Cấu hình SLAM Toolbox tối ưu scan matching
│   ├── localization.yaml       # Cấu hình AMCL Particle Filter (use_sim_time: True)
│   ├── nav2.yaml               # Cấu hình Nav2 Planner, Controller, Costmaps
│   ├── lab5_goals.yaml         # Danh sách tọa độ Goal (x, y, yaw) cho Bài thực hành 05
│   └── rviz/                   # Cấu hình giao diện RViz2 riêng cho từng phần
│       ├── rua4_sim.rviz       # Fixed frame 'odom', hiển thị RobotModel, TF, /scan, /odom
│       ├── rua4_slam.rviz      # Hiển thị trực tiếp quá trình vẽ bản đồ (/map)
│       └── rua4_nav2.rviz      # Hiển thị Map, AMCL, Costmaps, Path và /yolo/annotated_image
└── maps/                       # Thư mục chứa bản đồ
    ├── map.yaml / map.pgm      # Bản đồ đã lưu từ bài thực hành SLAM trước (604x1005, res 0.05m)
    ├── map_final.yaml / .pgm   # Bản sao lưu bản đồ SLAM hoàn chỉnh
    ├── warehouse.yaml / .pgm   # Bản đồ kho bãi độ phân giải cao (1006x1674, res 0.03m)
    └── maze.yaml / .pgm        # Bản đồ môi trường mê cung (670x669, res 0.03m)
```

---

## 🛠️ Bước Chuẩn bị Môi trường & Build

Mỗi khi mở một Terminal mới để thực hiện các phần, bạn luôn chạy 2 lệnh chuẩn bị sau:

```bash
# 1. Truy cập vào container ROS 2 Humble (nếu dùng Distrobox)
distrobox enter ros2-humble

# 2. Di chuyển vào workspace và source môi trường
cd /home/gdon/ur_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

*(Nếu sửa code trong package `rua4`, bạn build lại bằng lệnh:)*
```bash
colcon build --symlink-install --packages-select rua4
source install/setup.bash
```

---

## 🚀 Hướng dẫn Thực hiện Chi tiết Các Phần 1 – 4

### 📍 Phần 1 – TurtleBot 4 Simulation

#### Bước 1: Khởi chạy mô phỏng và RViz2
Mở **Terminal 1**:
```bash
distrobox enter ros2-humble
cd /home/gdon/ur_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 launch rua4 sim.launch.py world:=warehouse
```
* **Gazebo Sim** sẽ mở môi trường nhà kho với robot TurtleBot 4 xuất hiện ngay trên mặt sàn phẳng (`spawn_dock:=false` mặc định để bánh xe không bị kẹt cơ khí trên dốc sạc).
* Nếu muốn mở kèm RViz2 ở chế độ Simulation, thêm `rviz:=true`.

#### Bước 2: Kiểm tra các topic cơ bản
Mở **Terminal 2**:
```bash
distrobox enter ros2-humble
cd /home/gdon/ur_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

# Cách 1: Dùng công cụ kiểm tra tự động tích hợp sẵn trong package rua4
ros2 run rua4 check_topics

# Cách 2: Kiểm tra thủ công từng topic bằng lệnh tiêu chuẩn của ROS 2
ros2 topic echo /scan --once
ros2 topic echo /odom --once
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1, y: 0.0, z: 0.0}, angular: {z: 0.0}}"
```

---

### 🗺️ Phần 2 – Xây dựng bản đồ bằng SLAM Toolbox

*Giữ nguyên Terminal 1 đang chạy mô phỏng Gazebo.*

#### Bước 1: Khởi chạy SLAM Toolbox
Mở **Terminal 2**:
```bash
distrobox enter ros2-humble
cd /home/gdon/ur_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 launch rua4 slam.launch.py
```

#### Bước 2: Điều khiển robot quét bản đồ (Teleop)
Mở **Terminal 3**:
```bash
distrobox enter ros2-humble
cd /home/gdon/ur_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

#### Bước 3: Lưu bản đồ
Sau khi đã quét khảo sát xong toàn bộ khu vực, mở **Terminal 4**:
```bash
distrobox enter ros2-humble
cd /home/gdon/ur_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

# Cách 1: Dùng công cụ lưu bản đồ của package rua4 (tự động lưu vào thư mục maps/)
ros2 run rua4 save_map

# Cách 2: Lệnh trực tiếp của nav2_map_server
ros2 run nav2_map_server map_saver_cli -t map -f /home/gdon/ur_ws/src/rua4/maps/map --ros-args -p use_sim_time:=true
```

---

### 📍 Phần 3 – Localization

#### Bước 1: Khởi chạy Localization với bản đồ đã lưu
* Tắt SLAM và Teleop (`Ctrl + C`) nếu đang chạy, chỉ giữ Terminal 1 chạy `sim.launch.py world:=warehouse`.
* Mở **Terminal 2**:
```bash
distrobox enter ros2-humble
cd /home/gdon/ur_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 launch rua4 localization.launch.py
```
*(Mặc định nạp file `src/rua4/maps/map.yaml`. Có thể chỉ định file khác: `map:=/home/gdon/ur_ws/src/rua4/maps/warehouse.yaml`)*

#### Bước 2: Định vị vị trí robot trên RViz2
1. Trên thanh công cụ trên cùng của RViz2, chọn **`2D Pose Estimate`**.
2. Click chuột trái vào vị trí robot trong bản đồ tương ứng với vị trí thực tế trong Gazebo, giữ và kéo chuột theo hướng đầu robot.
3. Quan sát các điểm hạt xanh (`/particle_cloud` của AMCL) xuất hiện xung quanh robot và tia quét Laser khớp với viền tường của bản đồ.

---

### 🎯 Phần 4 – Navigation với Nav2

> **Lưu ý quan trọng**: `nav2.launch.py` đã bao gồm cả **Localization (`map_server` + `amcl`)** và **Nav2 Stack**. Vì vậy, hãy tắt `localization.launch.py` trước khi chạy `nav2.launch.py` để tránh khởi tạo trùng node!

Mở **Terminal 2**:
```bash
distrobox enter ros2-humble
cd /home/gdon/ur_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 launch rua4 nav2.launch.py
```

---

## 🤖 Hướng dẫn Thực hiện Chi tiết BÀI THỰC HÀNH 05 (Localization + Multi-Goal Nav2 + YOLO)

### Bước A – Chuẩn bị môi trường & Cài đặt thư viện

1. **Vào môi trường ROS 2 Humble và source workspace**:
```bash
distrobox enter ros2-humble
cd /home/gdon/ur_ws
source /opt/ros/humble/setup.bash
```

2. **Cài đặt các gói phụ thuộc (chỉ cần chạy 1 lần)**:
```bash
# Các gói ROS 2 và hệ thống qua apt:
sudo apt update
sudo apt install -y ros-humble-cv-bridge ros-humble-nav2-simple-commander python3-opencv python3-yaml

# Thư viện Ultralytics YOLO qua pip (Ghim numpy<2 để tương thích hoàn toàn với cv_bridge trên ROS 2 Humble):
pip install "numpy<2" ultralytics
```

3. **Xác minh phiên bản và khả năng import thư viện**:
```bash
python3 -c "import rclpy, cv2, yaml, numpy, ultralytics; from cv_bridge import CvBridge; from nav2_simple_commander.robot_navigator import BasicNavigator; print('NumPy:', numpy.__version__, '| OpenCV:', cv2.__version__, '| Ultralytics:', ultralytics.__version__, '-> ALL OK!')"
```

---

### Bước F – Build Package và Kiểm tra Executables

Thực hiện build package `rua4` và kiểm tra danh sách node đã đăng ký:
```bash
cd /home/gdon/ur_ws
colcon build --symlink-install --packages-select rua4
source install/setup.bash

# Kiểm tra các executable của package rua4 (phải có check_topics, save_map, nav_commander, yolo_detector, undock):
ros2 pkg executables rua4
```

---

### Bước B – Khởi chạy Mô phỏng Gazebo (Terminal 1)

Mở **Terminal 1** và khởi chạy TurtleBot 4 bản `standard` (có gắn LiDAR và camera RGB-D OAK-D) trong world `warehouse` đúng với bản đồ `maps/map.yaml` đã lưu:
```bash
distrobox enter ros2-humble
cd /home/gdon/ur_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 launch rua4 sim.launch.py world:=warehouse model:=standard
```

---

### Bước C – Khởi chạy Localization (AMCL) và Nav2 với Bản đồ cũ (Terminal 2)

Mở **Terminal 2** và khởi chạy hệ thống Localization + Nav2 sử dụng bản đồ cũ `maps/map.yaml` (mặc định):
```bash
distrobox enter ros2-humble
cd /home/gdon/ur_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 launch rua4 nav2.launch.py map:=/home/gdon/ur_ws/src/rua4/maps/map.yaml
```
*(Lưu ý: Không chạy đồng thời `localization.launch.py` và `nav2.launch.py` vì `nav2.launch.py` đã nạp sẵn `map_server` và `amcl`).*

#### Kiểm tra trạng thái Localization, TF và Camera trước khi chạy tự hành:
Mở **Terminal 3** và chạy lệnh kiểm tra tổng hợp Bài 05:
```bash
distrobox enter ros2-humble
cd /home/gdon/ur_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

# Kiểm tra tự động /scan, /odom, /cmd_vel, /map, /amcl_pose, /tf và /oakd/rgb/preview/image_raw:
ros2 run rua4 check_topics --lab5

# Hoặc kiểm tra thủ công bằng CLI chuẩn của ROS 2:
ros2 topic list
ros2 topic info /oakd/rgb/preview/image_raw -v
ros2 topic echo /amcl_pose --once
```
* Trong `config/localization.yaml`, tham số `set_initial_pose: true` đã tự động khởi tạo vị trí robot tại `(x=0.0, y=0.0, yaw=0.0)` khớp với vị trí spawn trong Gazebo.
* Nếu thấy tia quét LiDAR trên RViz2 bị lệch so với tường bản đồ, chọn công cụ **`2D Pose Estimate`** trên thanh công cụ RViz2, click vào gốc `(0, 0)` trên bản đồ và kéo mũi tên dọc theo hướng đầu xe trong Gazebo.

---

### Bước D – Khởi chạy Node Nhận diện Vật thể YOLO (Terminal 3)

Trong **Terminal 3**, khởi chạy node `yolo_detector`:
```bash
distrobox enter ros2-humble
cd /home/gdon/ur_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 run rua4 yolo_detector
```
* **Các tham số tùy chọn của `yolo_detector`**:
  ```bash
  # Thay đổi topic camera, model trọng số hoặc ngưỡng confidence:
  ros2 run rua4 yolo_detector --image-topic /oakd/rgb/preview/image_raw --model yolov8n.pt --conf 0.45

  # Chạy ở chế độ không mở cửa sổ cv2.imshow (chỉ xem qua topic /yolo/annotated_image trên RViz2):
  ros2 run rua4 yolo_detector --no-show-window
  ```
* **Quan sát kết quả**:
  * Cửa sổ **`TurtleBot 4 - YOLO Object Detection (rua4)`** sẽ bật lên hiển thị luồng video trực tiếp từ camera RGB của robot kèm thanh trạng thái FPS, thời gian suy luận (ms) và các khung bounding box + nhãn class + độ tin cậy.
  * Đồng thời trên **RViz2**, khung **YOLO Annotated Image** (subscribe topic `/yolo/annotated_image`) cũng hiển thị trực tiếp hình ảnh đã gắn nhãn.
  * Ảnh chụp khung hình mới nhất và ảnh phát hiện vật thể được tự động lưu tại `/home/gdon/ur_ws/src/rua4/results/yolo_latest_frame.jpg` và `yolo_latest_detection.jpg` để phục vụ làm báo cáo.

---

### Bước E – Khởi chạy Điều hướng Tự động qua Chuỗi Goal bằng Code (Terminal 4)

> **Quy tắc nghiệm thu Bài 05**: Trong lượt demo tự hành, **KHÔNG** dùng chuột bấm `Nav2 Goal` trên RViz2 và **KHÔNG** bật `teleop_twist_keyboard`. Giữ nguyên Terminal 3 đang chạy `yolo_detector`.

Mở **Terminal 4** và chạy node điều hướng tự động `nav_commander`:
```bash
distrobox enter ros2-humble
cd /home/gdon/ur_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

# Chạy chuỗi Goal của môi trường warehouse từ file config/lab5_goals.yaml:
ros2 run rua4 nav_commander --preset warehouse
```

* Robot sẽ tự động kiểm tra trạng thái Nav2 Active và lần lượt di chuyển qua các mục tiêu trong hệ tọa độ `map`:
  1. **Goal 1 – Hành lang kho A**: `(x = 2.0, y = 0.0, yaw = 0.0 rad)`
  2. **Goal 2 – Gian hàng B**: `(x = 3.5, y = 2.0, yaw = 1.57 rad)`
  3. **Goal 3 – Điểm trung chuyển**: `(x = 1.0, y = 2.5, yaw = 3.14 rad)`
  4. **Goal 4 – Trở về vị trí xuất phát**: `(x = 0.0, y = 0.0, yaw = 0.0 rad)`
* Nếu chỉ muốn chạy đúng 3 Goal đầu tiên:
  ```bash
  ros2 run rua4 nav_commander --preset warehouse --num-goals 3
  ```

#### 💡 Cách thay thế: Chạy đồng thời cả YOLO và Điều hướng bằng 1 lệnh Launch (Thay cho Bước D + E)
Sau khi đã bật Gazebo (Terminal 1) và Nav2 (Terminal 2), bạn có thể mở **Terminal 3** và chạy duy nhất file `lab5.launch.py`:
```bash
ros2 launch rua4 lab5.launch.py preset:=warehouse
```
*(Lệnh này sẽ khởi chạy `yolo_detector` trước và sau 3 giây tự động kích hoạt `nav_commander` mà không khởi tạo lại Gazebo hay Nav2).*

---

## 📐 Hướng dẫn Lấy Tọa độ `(x, y, yaw)` Thực tế từ RViz2 để Chỉnh sửa `config/lab5_goals.yaml`

Nếu bạn muốn thay đổi vị trí các Goal trong file `config/lab5_goals.yaml`:
1. Khi RViz2 đang mở, bật một terminal phụ và chạy:
   ```bash
   ros2 topic echo /goal_pose --once
   ```
2. Dùng công cụ **`Nav2 Goal`** trên RViz2 chấm và kéo hướng tại vị trí mong muốn trên bản đồ (chỉ dùng để lấy số liệu trước khi chạy demo).
3. Đọc `position.x`, `position.y` và `orientation.z`, `orientation.w` trên terminal. Góc `yaw` (radian) được tính bởi công thức:
   $$\text{yaw} = 2 \cdot \text{atan2}(z, w)$$
   *(Mẹo nhanh: hướng $+X$ có $\text{yaw} = 0.0$, hướng $+Y$ có $\text{yaw} = 1.57$, hướng $-X$ có $\text{yaw} = 3.14$, hướng $-Y$ có $\text{yaw} = -1.57$).*
4. Điền tọa độ vào mục `custom` hoặc `warehouse` trong `/home/gdon/ur_ws/src/rua4/config/lab5_goals.yaml`.

---

## 🔧 Xử lý Lỗi Thường Gặp (Troubleshooting)

1. **Lỗi `AttributeError: _ARRAY_API not found` khi import `cv_bridge`**:
   - **Nguyên nhân**: `pip install ultralytics` tự động nâng cấp `numpy` lên bản `2.x`, không tương thích với gói `ros-humble-cv-bridge` biên dịch với `numpy 1.x`.
   - **Khắc phục**: Hạ cấp `numpy` về `<2`:
     ```bash
     pip install "numpy<2"
     ```
     *(Ngoài ra, `yolo_detector.py` của `rua4` đã tích hợp sẵn bộ chuyển đổi dự phòng bằng NumPy thuần nên vẫn hoạt động bình thường ngay cả khi `cv_bridge` gặp lỗi ABI).*

2. **Node `yolo_detector` báo chưa nhận được ảnh trên `/oakd/rgb/preview/image_raw`**:
   - Kiểm tra xem Gazebo có được khởi chạy với `model:=standard` hay không (`model:=lite` không có camera OAK-D).
   - Kiểm tra danh sách topic ảnh đang hoạt động bằng `ros2 topic list | grep image` và truyền vào `--image-topic <tên_topic>`.

3. **Model YOLO chạy nhưng ít phát hiện vật thể trong kho mặc định**:
   - **Giải thích**: Trọng số pretrained `yolov8n.pt` được huấn luyện trên tập **MS COCO (80 classes)** gồm các đối tượng đời sống (`person`, `chair`, `bench`, `tv`, `bottle`, `cup`, `suitcase`, `potted plant`, `truck`...). Các kệ sắt công nghiệp hoặc tường kho thô không thuộc 80 lớp COCO (và hoàn toàn không có lớp `bình chữa cháy` trong COCO chuẩn).
   - **Cách kiểm thử nhận diện trực quan trong Gazebo**: Trong giao diện Gazebo Ignition, bạn có thể mở thẻ **Resource Spawner / Fuel** (hoặc Insert model) để đặt thêm mô hình người (`Standing person`), ghế (`Chair`) hoặc xe nâng/thùng hàng trước mặt đường đi của robot (tại khoảng tọa độ $x \in [1.5, 3.5], y \in [0.0, 2.5]$ ngoài hành lang di chuyển) để YOLO phát hiện với độ tin cậy rất cao ($> 0.80$).
