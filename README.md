# PX4 + ROS2 Autonomous UAV
Tested for Ubuntu 20.04
## Prerequisites
#### ROS2 Environment
Follow the steps of the [ROS2 Foxy Documentation](https://docs.ros.org/en/foxy/Installation.html).
#### PX4 SITL Gazebo-Classic Environment
Follow the steps of the [PX4 Documentation](https://docs.px4.io/main/en/dev_setup/dev_env_linux_ubuntu.html#simulation-and-nuttx-pixhawk-targets).
It is assumed that the PX4 Code is cloned to ```~/```.
Additionally install the necessary dependencies:
```bash
sudo apt install ros-foxy-gazebo-ros-pkgs libgazebo-dev
```
Build the PX4 Autopilot for SITL Gazebo-Classic:
```bash
cd ~/PX4-Autopilot
DONT_RUN=1 make px4_sitl gazebo-classic
```
#### Build px4_autonomous
```bash
cd ~/ros2_ws/
git clone git@github.com:AImotion-Flight/px4_autonomous.git --recursive src/
colcon build
source install/setup.bash
```
## PX4 Autonomous
Launch the SITL simulation with:
```
ros2 launch px4_autonomous px4_autonomous_sitl_launch.py
```
Launch Offboard node:
```bash
ros2 launch px4_autonomous px4_autonomous_launch.py
```
