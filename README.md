# Dự án SLAM và Điều hướng với RPLIDAR A1M8 và Robot Mecanum trên ROS 2 Jazzy

Dự án này sử dụng RPLIDAR A1M8, robot 4 bánh xe Mecanum (điều khiển qua STM32F4), và các gói ROS 2 Jazzy (slam_toolbox, nav2) để tạo bản đồ 2D, hiển thị trên RViz với mô hình robot và lưới, tích hợp odometry, và điều hướng tự động theo waypoint với khả năng tránh vật cản. Hướng dẫn này cung cấp cấu trúc dự án, nội dung file cấu hình, danh sách thư viện, và các bước thực hiện.

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


# Danh sách thư viện cần cài đặt: 
### ROS 2 Jazzy:
sudo apt install ros-jazzy-desktop-full
# rplidar_ros
sudo apt install ros-jazzy-rplidar-ros
### slam_toolbox 
sudo apt install ros-jazzy-slam-toolbox
### tf2_ros
sudo apt install ros-jazzy-tf2-ros
### rviz2
sudo apt install ros-jazzy-rviz2
### nav2_map_server
sudo apt install ros-jazzy-nav2-map-server
### robot_state_publisher
sudo apt install ros-jazzy-robot-state-publisher
### Nav2
sudo apt install ros-jazzy-nav2-bringup ros-jazzy-nav2-planner ros-jazzy-nav2-nav2-controller
### Driver Mesa (cho RViz trên Raspberry Pi 5):
sudo apt install mesa-utils
### Python YAML và serial:
sudo apt install python3-yaml
python3-pip install pyserial

# Các bước thiết lập dự án
## Bước 1: Cài đặt ROS 2 Jazzy
### 1. Cài đặt trên Ubuntu 24.04:

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

### 2. Thêm source ROS vào ~/.bashrc:

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

source ~/.bashrc

## Bước 2: Tạo workspace ROS

1. Tạo thư mục workspace:
mkdir -p ~/slam_ws/src

cd ~/slam_ws/src

2. Tạo gói ROS my_slam_package:
ros2 pkg create --build-type ament_cmake my_slam_package --dependencies rplidar_ros slam_toolbox tf2_ros rviz2 nav2_map_server robot_state_publisher nav2_bringup nav2_planners nav2_controller

## Bước 3: Cấu hình thư mục và file   
1. Tạo các thư mục con:
mkdir -p ~/slam_ws/src/my_slam_package/config

mkdir -p ~/slam_ws/src/my_slam_package/launch

mkdir -p ~/slam_ws/src/my_slam_package/rviz

mkdir -p ~/slam_ws/src/my_slam_package/maps

mkdir -p ~/slam_ws/src/my_slam_package/urdf

mkdir -p ~/slam_ws/src/my_slam_package/scripts

2. Tạo và sao chép nội dung file:
lidar.yaml, slam_params.yaml, nav2_params.yaml, slam_launch.py, navigation_launch.py, slam.rviz, robot.urdf, package.xml, CMakeLists.txt, odom_publisher.py, vel_to_serial.py, waypoint_navigator.py.

Sử dụng nano để tạo file:
nano ~/slam_ws/src/my_slam_package/config/lidar.yaml

## Bước 4: Cài đặt các thư viện phụ thuộc
1. Cài đặt các gói ROS:

sudo apt install ros-jazzy-rplidar-ros ros-jazzy-slam-toolbox ros-jazzy-tf2-ros ros-jazzy-rviz2 ros-jazzy-nav2-map-server ros-jazzy-robot-state-publisher ros-jazzy-nav2-bringup ros-jazzy-nav2-planner ros-jazzy-nav2-controller

2.  Cài đặt driver Mesa:
sudo apt install mesa-utils

3.  Cài đặt Python YAML và serial:

sudo apt install python3-yaml

pip3 install pyserial
  
    



















