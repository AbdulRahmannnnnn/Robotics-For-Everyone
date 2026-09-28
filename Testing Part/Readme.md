### Setup Environment ROS2
```sh
nano ~/.bashrc
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
source /opt/ros/jazzy/setup.bash
source ~/microros_ws/install/local_setup.bash
export ROS_DOMAIN_ID=0
source ~/.bashrc

```
### Command to run ROS2 and Micro-ros
### Install Micro ROS Agent
```sh
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir uros_ws && cd uros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```
```sh
cd ~/microros_ws
source install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 -b 115200 -v 6
```
if the ESP32 can't connect to micro ros agent, please try to push reset button on ESP32
```sh
ros2 topic pub /sound_control std_msgs/msg/Int32 "{data: 1}" --once
ros2 topic pub /led_control std_msgs/msg/Int32MultiArray "{layout: {dim: [], data_offset: 0}, data: [3, 1, 500, 500]}" --once
ros2 topic echo /battery_status
```
### Install Teleoperation Keyboard
```sh
sudo apt update
sudo apt install ros-${ROS_DISTRO}-teleop-twist-keyboard
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
### RPLiDAR A1 
```sh
cd ~/ros2_ws/src
git clone --single-branch -b ros2 https://github.com/Slamtec/rplidar_ros.git
cd ..
colcon build --symlink-install
source install/local_setup.bash
sudo chmod 777 /dev/ttyUSB1
ros2 launch rplidar_ros view_rplidar_a1_launch.py
ros2 launch rplidar_ros rplidar_a1_launch.py serial_port:=/dev/ttyUSB1
```
### Install Madgwick Filter Node

we will use madgwick filter to filter and make the imu data more clean and accurate
```sh
sudo apt update
sudo apt install ros-${ROS_DISTRO}-imu-madgwick-filter
```
after install madgwick filter, let's make the launch file before run this node, with launch file we can change the topic node from imy and output topic from this node
```sh
mkdir ros2/ws/src/mpu9250_imu/launch

```
```sh
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter_madgwick',
            output='screen',
            parameters=[{
                'use_mag': False,              # Ubah ke True jika Anda mempublikasikan topik /imu/mag (Magnetometer MPU9250)
                'gain': 0.1,                   # Responsivitas filter (default: 0.1). Semakin besar, semakin responsif tapi noise naik.
                'zeta': 0.0,                   # Koreksi gyro drift
                'publish_tf': True,            # Set True jika ingin menyertakan transformasi koordinat TF ke sistem robot
                'fixed_frame': 'imu_link',     # Sesuaikan dengan frame_id dari sensor MPU9250 Anda
                'remove_gravity_vector': True  # Menghilangkan efek gravitasi bumi pada data akselerasi linear output
            }],
            remappings=[
                # Memetakan input default Madgwick (/imu/data_raw) membaca topik dari MPU9250 Anda (/imu/raw)
                ('/imu/data_raw', '/imu/raw'),
                # Output hasil filter orientasi (quaternion) akan dipublikasikan ke /imu/data
                ('/imu/data', '/imu/filtered')
            ]
        )
    ])
```
### Install RVIZ IMU Plugin

this plugin can be used to show imu data on rviz by display data type, the imu data type option will not be showed before install the plugin
```sh
sudo apt update
sudo apt install ros-${ROS_DISTRO}-rviz2-imu-plugin
```
### Install EKF Robot L0calization
```sh
sudo apt update
sudo apt install ros-jazzy-robot-localization
```
### Setup Remote PC/Laptop
- Install ROS2
- export ROS_DOMAIN=0, make same with Rasberry PI
- nano ~/.bashrc