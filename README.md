# PX4 + ROS2 Autonomous UAV
## Run the SITL simulation
### Install PX4 SITL Gazebo Environment
Clone the PX4 Autopilot:
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive ~/
```
Build SITL Gazebo:
```
cd ~/PX4-Autopilot
make px4_sitl gazebo
```
### Install px4_autonomous
:warning: This step requires a working [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) installation.

Clone this repository into your ROS2 workspace:
```
cd ~/ros2_ws
git clone git@github.com:AImotion-Flight/px4_autonomous.git --recursive
colcon build
source install/setup.bash
```
### Launching the SITL simulation
Finally the simulation can be started with:
```
ros2 launch px4_autonomous px4_autonomous_sitl_launch.py
```
