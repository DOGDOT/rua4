# Package `rua4` - TurtleBot 4 Simulation, SLAM Toolbox & Nav2 Autonomous Navigation

Package ROS 2 Humble cho **TurtleBot 4** trong môi trường mô phỏng **Gazebo Sim (Ignition)**, thực hiện toàn bộ 4 phần của bài tập bằng các câu lệnh ROS 2 thủ công trên terminal:
1. **Phần 1 – Simulation**: Khởi chạy robot trong Gazebo, kiểm tra `/scan`, `/odom`, `/cmd_vel`, hiển thị trực quan trên RViz2.
2. **Phần 2 – Xây dựng bản đồ (SLAM)**: Khảo sát môi trường bằng SLAM Toolbox + Teleop bàn phím, xuất file `map.yaml` và `map.pgm`.
3. **Phần 3 – Localization**: Nạp bản đồ đã lưu, định vị robot bằng AMCL (`2D Pose Estimate`), đối chiếu vị trí thực trong Gazebo.
4. **Phần 4 – Autonomous Navigation (Nav2)**: Tự động dẫn đường đến các mục tiêu (`Nav2 Goal`) tại các khu vực khác nhau mà không va chạm.

---

## 📁 Cấu trúc Thư mục Package

```
/home/gdon/ur_ws/src/rua4/
├── package.xml                 # Khai báo metadata & dependencies
├── setup.py                    # Cấu hình cài đặt Python package & các console script
├── setup.cfg
├── rua4/                       # Module mã nguồn Python
│   ├── __init__.py
│   ├── check_topics.py         # Node tự động kiểm tra /scan, /odom, /cmd_vel (Phần 1)
│   ├── save_map.py             # Tiện ích lưu bản đồ map.yaml & map.pgm (Phần 2)
│   └── nav_commander.py        # [Tùy chọn] Tự động gửi chuỗi Nav2 Goal qua Python (Phần 4)
├── launch/                     # Các file launch chuyên biệt cho từng phần
│   ├── sim.launch.py           # Launch Gazebo + RViz2 (Phần 1)
│   ├── slam.launch.py          # Launch SLAM Toolbox + RViz2 (Phần 2)
│   ├── localization.launch.py  # Launch AMCL + Map Server (Phần 3)
│   ├── nav2.launch.py          # Launch Nav2 full stack + RViz2 (Phần 4)
│   └── tb4_all_in_one.launch.py# File launch tổng hợp (mode:=sim|slam|localization|nav2)
├── config/                     # Cấu hình tham số tối ưu cho TurtleBot 4
│   ├── slam.yaml               # Cấu hình SLAM Toolbox tối ưu scan matching
│   ├── localization.yaml       # Cấu hình AMCL Particle Filter
│   ├── nav2.yaml               # Cấu hình Nav2 Planner, Controller, Costmaps
│   └── rviz/                   # Cấu hình giao diện RViz2 riêng cho từng phần
│       ├── rua4_sim.rviz       # Fixed frame 'odom', hiển thị RobotModel, TF, /scan, /odom
│       ├── rua4_slam.rviz      # Hiển thị trực tiếp quá trình vẽ bản đồ (/map)
│       └── rua4_nav2.rviz      # Hiển thị Costmaps, Global/Local Plan, Nav2 Goal
└── maps/                       # Thư mục chứa bản đồ
    ├── map.yaml / map.pgm      # Bản đồ mặc định và bản đồ sau khi lưu
    ├── warehouse.yaml / .pgm   # Bản đồ môi trường kho bãi
    └── maze.yaml / .pgm        # Bản đồ môi trường mê cung
```

---

## 🛠️ Bước Chuẩn bị Môi trường & Build

Mỗi khi mở một Terminal mới để thực hiện các phần, bạn luôn chạy 2 lệnh chuẩn bị sau:

```bash
# 1. Truy cập vào container ROS 2 Humble
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

## 🚀 Hướng dẫn Thực hiện Chi tiết Từng Phần (Chạy thủ công)

### 📍 Phần 1 – TurtleBot 4 Simulation

#### Bước 1: Khởi chạy mô phỏng và RViz2
Mở **Terminal 1**:
```bash
distrobox enter ros2-humble
cd /home/gdon/ur_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 launch rua4 sim.launch.py world:=warehouse
```
* **Gazebo Sim** sẽ mở môi trường nhà kho với robot TurtleBot 4 xuất hiện ngay trên mặt sàn phẳng.
* **Lưu ý đặc biệt**: Bản cập nhật của `rua4` đã **tắt hoàn toàn trạm sạc Dock** (`spawn_dock:=false` mặc định), robot đứng tự do trên sàn mở, **bánh xe không bao giờ bị kẹt cơ khí trên dốc sạc**.
* **RViz2** tự động mở giao diện được cấu hình riêng: hiển thị mô hình 3D của robot, hệ trục tọa độ TF, tia quét laser **`/scan`** và mũi tên biểu diễn odometry **`/odom`**.

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
* Cửa sổ RViz2 sẽ hiển thị bản đồ đang được vẽ và cập nhật liên tục trên topic `/map`.

#### Bước 2: Điều khiển robot quét bản đồ (Teleop)
Mở **Terminal 3**:
```bash
distrobox enter ros2-humble
cd /home/gdon/ur_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

# Khởi chạy điều khiển bàn phím:
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

> 💡 **MẸO GIẢI PHÓNG NẾU ROBOT BỊ KẸT TRẠM SẠC (từ phiên bản cũ còn sót lại):**
> Nếu bạn thấy robot vẫn còn ở trên trạm sạc hoặc bánh xe bị khóa, chạy ngay lệnh cứu hộ tự động của package `rua4`:
> ```bash
> ros2 run rua4 undock
> ```
> Lệnh này sẽ tự động kiểm tra, gửi action undock, và dịch chuyển robot ra khỏi trạm sạc đặt xuống sàn phẳng an toàn.
> Hoặc điều khiển trực tiếp qua topic controller để bỏ qua mọi cơ chế khóa an toàn:
> ```bash
> ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/diffdrive_controller/cmd_vel_unstamped
> ```

* **Phím điều khiển**:
  * `i`: Tiến | `k`: Dừng | `,`: Lùi
  * `j`: Xoay trái | `l`: Xoay phải
  * `q` / `z`: Tăng / Giảm tốc độ tịnh tiến (khuyên dùng khoảng `0.2 - 0.25 m/s`)
  * `w` / `x`: Tăng / Giảm tốc độ xoay (khuyên dùng khoảng `0.5 rad/s`)
* **Lưu ý để điều khiển được và bản đồ đẹp**:
  * Cửa sổ terminal chạy `teleop_twist_keyboard` **phải đang được chọn (Active focus)** thì mới nhận phím gõ.
  * Lái robot chậm, đặc biệt khi vào cua để thuật toán scan-matching ghép tia laser chuẩn xác.
  * Cho robot đi hết các hành lang chính và quay lại vị trí xuất phát để SLAM kích hoạt cơ chế **Loop Closure** khớp toàn bộ bản đồ.

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
* Bản đồ được tạo ra gồm 2 file tại `src/rua4/maps/`:
  * `map.yaml`
  * `map.pgm`

---

### 📍 Phần 3 – Localization

#### Bước 1: Khởi động lại mô phỏng
* Nhấn `Ctrl + C` ở Terminal 2 và 3 để tắt SLAM và Teleop.
* Khởi động lại mô phỏng ở Terminal 1 (hoặc tắt đi bật lại để đưa robot về vị trí ban đầu tại dock).

#### Bước 2: Khởi chạy Localization với bản đồ vừa tạo
Mở **Terminal 2**:
```bash
distrobox enter ros2-humble
cd /home/gdon/ur_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 launch rua4 localization.launch.py
```
*(Mặc định nạp file `src/rua4/maps/map.yaml`. Có thể chỉ định file khác: `map:=/duong/dan/map.yaml`)*

#### Bước 3: Định vị vị trí robot trên RViz2
1. Trên thanh công cụ trên cùng của RViz2, chọn **`2D Pose Estimate`**.
2. Click chuột trái vào vị trí robot trong bản đồ tương ứng với vị trí thực tế trong Gazebo, giữ và kéo chuột theo hướng đầu robot.
3. Quan sát các điểm hạt xanh (`/particlecloud` của AMCL) xuất hiện xung quanh robot.
4. Mở teleop (`ros2 run teleop_twist_keyboard teleop_twist_keyboard`) lái robot xoay nhẹ tại chỗ: các hạt AMCL sẽ hội tụ sát vào thân xe, tia quét Laser màu đỏ sẽ khớp chính xác với viền tường của bản đồ.

---

### 🎯 Phần 4 – Navigation với Nav2

#### Bước 1: Khởi chạy hệ thống Nav2
Mở **Terminal 2**:
```bash
distrobox enter ros2-humble
cd /home/gdon/ur_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 launch rua4 nav2.launch.py
```
* Hệ thống sẽ tự động khởi động **Localization (AMCL + Map Server)** và **Nav2 Stack (Planner, Controller, Costmaps, Behavior Tree)** cùng giao diện RViz2 tích hợp sẵn bảng điều khiển Navigation 2.

#### Bước 2: Ra lệnh mục tiêu bằng RViz2 (Nav2 Goal)
1. Kiểm tra vị trí robot đã khớp bản đồ chưa (nếu chưa, dùng `2D Pose Estimate` như Phần 3).
2. Nhấn chọn công cụ **`Nav2 Goal`** (hoặc `2D Goal Pose`) trên thanh công cụ RViz2.
3. Click vào bất kỳ vị trí hợp lệ nào trên bản đồ (trong vùng màu xám nhạt) và kéo chuột để chỉ định hướng quay khi đến đích.
4. **Quan sát robot tự hành**:
   * Robot tự vẽ đường đi toàn cục **Global Path**.
   * Chi phí chướng ngại vật hiển thị trên **Global & Local Costmap**.
   * Robot tự động di chuyển theo quỹ đạo, linh hoạt né tránh các vật cản và dừng lại chính xác tại tọa độ đích mà không va chạm.
5. Tiếp tục đặt các mục tiêu tiếp theo ở **các khu vực khác nhau** theo đúng yêu cầu bài tập.

#### Bước 3 (Tùy chọn nâng cao): Chạy chuỗi mục tiêu tự động bằng Python script
Nếu bạn muốn gửi tuần tự nhiều mục tiêu ở các khu vực khác nhau qua script Python (không cần bấm chuột trên RViz2):
Mở **Terminal 3**:
```bash
distrobox enter ros2-humble
cd /home/gdon/ur_ws
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 run rua4 nav_commander --preset warehouse
```
Robot sẽ lần lượt tự di chuyển qua 4 khu vực:
1. Hành lang kho A
2. Gian hàng B
3. Điểm trung chuyển
4. Trở về vị trí xuất phát (Dock)

---

## ⚙️ Tùy biến Môi trường Mô phỏng

TurtleBot 4 hỗ trợ 3 môi trường Gazebo có sẵn:
- `warehouse`: Nhà kho công nghiệp (mặc định)
- `maze`: Mê cung có tường và góc cua phức tạp
- `depot`: Khu tập kết hàng hóa

Để đổi môi trường khi chạy lệnh, thêm tham số `world:=<tên_world>`, ví dụ:
```bash
ros2 launch rua4 sim.launch.py world:=maze
ros2 launch rua4 slam.launch.py world:=maze
```
