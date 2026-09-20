# SLAM Differential Mobile Robot

<p align="center">
    <img src="/images/slam.png"
         alt="SLAM Differential Mobile Robot"
         width="90%" />
</p>

<!-- Badges -->
<p align="center">
    <a href="https://docs.ros.org/en/jazzy/">
        <img src="https://img.shields.io/badge/ROS%202-Jazzy%20Jalisco-22314E?logo=ros&logoColor=white&style=flat"
             alt="ROS 2 Jazzy Jalisco" />
    </a>
    <a href="https://ubuntu.com/download/desktop">
        <img src="https://img.shields.io/badge/Ubuntu-24.04%20LTS-E95420?logo=ubuntu&logoColor=white&style=flat"
             alt="Ubuntu 24.04 LTS Noble Numbat" />
    </a>
    <a href="https://docs.nav2.org/jazzy/">
        <img src="https://img.shields.io/badge/Nav2-Jazzy-22314E?logo=ros&logoColor=white&style=flat"
             alt="Navigation2 Jazzy" />
    </a>
    <a href="https://github.com/SteveMacenski/slam_toolbox">
        <img src="https://img.shields.io/badge/SLAM%20Toolbox-ROS%202-22314E?logo=ros&logoColor=white&style=flat"
             alt="SLAM Toolbox" />
    </a>
</p>

## New Using ROS2?
- Install ROS2 Jazzy Jalisco

## Getting Started
### Prerequisites
1. Ubuntu Noble 24.04
2. [Install ROS2 Jazzy Jalisco](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)
3. [Install Micro ROS2 Agent](https://github.com/micro-ros/micro_ros_setup)
```bash
mkdir -p ~/microros_ws/src
cd ~/microros_ws/src
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build
source ~/microros_ws/install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
```
4. [Install Navigation 2(Nav2)](https://docs.nav2.org/jazzy/getting_started/build_and_install/)

<p align="center">
    <img src="/images/nav2.png"
         alt="SLAM Differential Mobile Robot"
         width="90%" />
</p>

```bash
source /opt/ros/jazzy/setup.bash
# Update your package list
sudo apt update

# Install the core Nav2 packages and bringup tools
sudo apt install ros-$ROS_DISTRO-navigation2 ros-$ROS_DISTRO-nav2-bringup
```
If you want to run the pre configured Turtlebot simulation to test your setup:
```bash
sudo apt install ros-$ROS_DISTRO-nav2-minimal-tb\*
```
5. [Install SLAM Toolbox](https://docs.ros.org/en/jazzy/p/slam_toolbox/)
```bash
# Update your package list
sudo apt update

# Install SLAM Toolbox
sudo apt install ros-$ROS_DISTRO-slam-toolbox
```
6. [Install Teleoperation Keyboard](https://docs.ros.org/en/jazzy/p/teleop_twist_keyboard/)
```bash
sudo apt install ros-$ROS_DISTRO-teleop-twist-keyboard
#Publishing to a different topic (in this case my_cmd_vel)
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=my_cmd_vel
```
7. Install 

### Setup Environment

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone
colcon build
source install/setup.bash
```

### Setup Micro ROS2 Agent

```bash
mkdir -p ~micro_ros2_ws/src
cd ~/micro_ros2_ws/src
source install/setup.bash
```