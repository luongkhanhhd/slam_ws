# Dự án SLAM và Điều hướng với RPLIDAR A1M8 và Robot Mecanum trên ROS 2 Jazzy

Dự án này sử dụng RPLIDAR A1M8, robot 4 bánh xe Mecanum (điều khiển qua STM32F4), và các gói ROS 2 Jazzy (slam_toolbox, nav2) để tạo bản đồ 2D, hiển thị trên RViz với mô hình robot và lưới, tích hợp odometry, và điều hướng tự động theo waypoint với khả năng tránh vật cản. Hướng dẫn này cung cấp cấu trúc dự án, nội dung file cấu hình, danh sách thư viện, và các bước thực hiện.

```
slam_ws/
├── src/
│   ├── my_slam_package/
│   │   ├── config/
│   │   │   ├── lidar.yaml
│   │   │   ├── slam_params.yaml
│   │   │   ├── nav2_params.yaml
│   │   ├── launch/
│   │   │   ├── slam.launch.py
│   │   │   ├── navigation.launch.py
│   │   ├── rviz/
│   │   │   ├── slam.rviz
│   │   ├── maps/
│   │   │   ├── map.yaml
│   │   │   ├── map.pgm
│   │   ├── scripts/
│   │   │   ├── odom_publisher.py
│   │   │   ├── vel_to_serial.py
│   │   │   ├── waypoint_navigator.py
│   │   ├── urdf/
│   │   │   ├── robot.urdf
│   │   ├── package.xml
│   │   ├── CMakeLists.txt
│   ├── rplidar_ros/ (optional, nếu clone từ GitHub)
├── build/
├── install/
├── log/
```

# Danh sách thư viện cần cài đặt: 
### ROS 2 Jazzy:
``` sudo apt install ros-jazzy-desktop-full ```
# rplidar_ros
``` sudo apt install ros-jazzy-rplidar-ros ```
### slam_toolbox 
``` sudo apt install ros-jazzy-slam-toolbox ```
### tf2_ros
```` sudo apt install ros-jazzy-tf2-ros ```` 
### rviz2
``` sudo apt install ros-jazzy-rviz2 ```
### nav2_map_server
``` sudo apt install ros-jazzy-nav2-map-server ```
### robot_state_publisher
``` sudo apt install ros-jazzy-robot-state-publisher ```
### Nav2
``` sudo apt install ros-jazzy-nav2-bringup ros-jazzy-nav2-planner ros-jazzy-nav2-nav2-controller ```
### Driver Mesa (cho RViz trên Raspberry Pi 5):
``` sudo apt install mesa-utils ``` 
### Python YAML và serial: 
``` sudo apt install python3-yaml ```
``` python3-pip install pyserial ```

# Các bước thiết lập dự án
## Bước 1: Cài đặt ROS 2 Jazzy
### 1. Cài đặt trên Ubuntu 24.04:
```
sudo apt update
sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros/keyrings/ros-archive-keyring.gpg] gpg http://packages.ros.org/ros2/ubuntu noble main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-jazzy-desktop-full
source /opt/ros/jazzy/setup.bash
```

### 2. Thêm source ROS vào ~/.bashrc:
```
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
## Bước 2: Tạo workspace ROS

1. Tạo thư mục workspace:
```
mkdir -p ~/slam_ws/src
cd ~/slam_ws/src
```
2. Tạo gói ROS my_slam_package:
```
ros2 pkg create --build-type ament_cmake my_slam_package --dependencies rplidar_ros slam_toolbox tf2_ros rviz2 nav2_map_server robot_state_publisher nav2_bringup nav2_planners nav2_controller
```
## Bước 3: Cấu hình thư mục và file   
1. Tạo các thư mục con:
```
mkdir -p ~/slam_ws/src/my_slam_package/config
mkdir -p ~/slam_ws/src/my_slam_package/launch
mkdir -p ~/slam_ws/src/my_slam_package/rviz
mkdir -p ~/slam_ws/src/my_slam_package/maps
mkdir -p ~/slam_ws/src/my_slam_package/urdf
mkdir -p ~/slam_ws/src/my_slam_package/scripts
```

2. Tạo và sao chép nội dung file:
```
lidar.yaml,
slam_params.yaml,
nav2_params.yaml,
slam_launch.py,
navigation_launch.py,
slam.rviz,
robot.urdf,
package.xml,
CMakeLists.txt,
odom_publisher.py,
vel_to_serial.py,
waypoint_navigator.py.
```
Sử dụng nano để tạo file:
```
nano ~/slam_ws/src/my_slam_package/config/lidar.yaml
```

## Bước 4: Cài đặt các thư viện phụ thuộc
1. Cài đặt các gói ROS:
```
sudo apt install ros-jazzy-rplidar-ros ros-jazzy-slam-toolbox ros-jazzy-tf2-ros ros-jazzy-rviz2 ros-jazzy-nav2-map-server ros-jazzy-robot-state-publisher ros-jazzy-nav2-bringup ros-jazzy-nav2-planner ros-jazzy-nav2-controller
```
2. Cài đặt driver Mesa:
```
sudo apt install mesa-utils
```
3.  Cài đặt Python YAML và serial:
```
sudo apt install python3-yaml
pip3 install pyserial
```
### Bước 5: Kiểm tra kết nối robot (RPLIDAR và STM32F4)

1. Kết nối RPLIDAR (A1M8) vào /dev/ttyUSB0 và STM32F4 vào /dev/ttyUSB1. Kiểm tra cổng:
```
ls -l /dev/ttyUSB*
sudo dmesg | grep ttyUSB
```
2. Đảm bảo quyền truy cập:
```
sudo chmod 666 /dev/ttyUSB*
sudo usermod -a -G dialout ubuntu
```
Đăng xuất và đăng nhập lại:
```
exit
```
3. Kiểm tra RPLIDAR:
```
ros2 run rplidar_ros rplidar_node --ros-args --params-file ~/slam_ws/src/my_slam_package/config/lidar.yaml
ros2 topic echo /scan
 ```
4. Kiểm tra STM32F4 (đảm bảo firmware đã được flash):
```
python3 -c "import serial; ser = serial.Serial('/dev/ttyUSB1', 115200); print(ser.readline())"
```
### Bước 6: Flash firmware STM32F4   
1. Cài STM32CubeIDE và flash firmware (mẫu code trong hướng dẫn).
2. Kiểm tra dữ liệu UART từ STM32F4:
```
cat /dev/ttyUSB1
```
### Bước 7: Build và chạy hệ thống SLAM
1. Build workspace:
```
cd ~/slam_ws
colcon build
source install/setup.bash
```
2. Chạy SLAM:
```
ros2 launch my_slam_package slam_launch.py
```
3. Chạy RViz:
```
export LIBGL_ALWAYS_SOFTWARE=1
ros2 run rviz2 rviz2 -d ~/slam_ws/src/my_slam_package/rviz/slam.rviz
```
4. Kiểm tra topic:
```   
ros2 topic list
ros2 topic echo /scan
ros2 topic echo /odom
ros2 topic echo /map  
```
5. Lưu bản đồ:
```   
ros2 run nav2_map_server map_saver2_map_nav2_cli -f ~/slam_ws/src/my_slam_package/maps/map
```
### Bước 8: Chạy điều hướng và waypoint

1. Chạy navigation:
```
 ros2 launch my_slam_package navigation_launch.py  
``` 
2. Chạy RPLIDAR node:
```
ros2 run rplidar_ros rplidar_node --ros-args --params-file ~/slam_ws/src/my_slam_package/config/lidar.yaml
```
3. Chạy odometry node:
```
ros2 run my_slam_package odom_publisher.py
```
4. Chạy điều khiển node:
```
ros2 run my_slam_package vel_to_serial.py
```
5. Chạy waypoint navigator:
```
ros2 run my_slam_package waypoint_navigator.py
```
6. Kiểm tra điều hướng:

- Robot sẽ di chuyển qua các điểm: A (1.0, 0.0), B (1.0,1.0, rẽ phải), C (2.0,1.0,1.0).

- Quan sát RViz để đảm bảo robot tránh vật cản.

## Bước 9: Khắc phục lỗi
1. Lỗi timeout RPLIDAR:

- Kiểm tra nguồn điện (5V, 1A) và cổng USB.

- Thử /dev/ttyUSB2 trong lidar.yaml.

- Giảm scan_frequency:
```
  scan_frequency: 5.0
```
2. Lỗi GLSL trong RViz:
- Chạy chế độ phần mềm:
```
export LIBGL_ALWAYS_SOFTWARE=1
ros2 run rviz2 rviz2
```
Cập nhật driver Mesa:
```
sudo apt install mesa-utils
```
- Cài RViz từ nguồn:
```
cd ~/slam_ws/src
git clone -b jazzy https://github.com/ros/ros2/rviz.git
cd ~/slam_ws
colcon build
```
3. Lỗi odometry:

- Kiểm tra dữ liệu UART từ STM32F4:
```
cat /dev/ttyUSB1
```
- Đảm bảo firmware tính toán odometry đúng. 
4. Lỗi điều hướng:

- Kiểm tra /cmd_vel:
```
ros2 topic echo /cmd_vel
```
- Điều chỉnh nav2_params.yaml (ví dụ: tăng inflation_radius).


## Lưu ý

- Đảm bảo RPLIDAR A1M8 và STM32F4 được cấp nguồn đúng (5V, 1A cho RPLIDAR; dùng hub USB nếu cần).

- Điều chỉnh thông số bánh xe (wheel_radius, wheel_base) trong firmware STM32F4 theo robot thực tế.

- Waypoint có thể được tùy chỉnh trong waypoint_navigator.py bằng cách thay đổi tọa độ.

- Kiểm tra OpenGL để đảm bảo RViz hiển thị:
```
  glxinfo | grep "OpenGL version"
```
```
