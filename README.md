# PX4 + ROS2 Autonomous UAV
## Run the SITL simulation
### Get the PX4 Source Code
Clone the PX4 Autopilot:
```
git clone https://github.com/PX4/PX4-Autopilot.git --recursive ~/PX4-Autopilot
```
### Install px4_autonomous
:warning: This step requires a fully set up ROS2 Humble workspace.

Clone this repository into your ROS2 workspace:
```
cd ~/ros2_ws/
git clone git@github.com:AImotion-Flight/px4_autonomous.git --recursive src/
colcon build
source install/setup.bash
```
### Launching the SITL simulation (Gazebo Classic)
Build SITL Gazebo Classic:
```
cd ~/PX4-Autopilot
make px4_sitl gazebo
```
After the successful build close Gazebo and PX4. Start the simulation with:
```
ros2 launch px4_autonomous px4_autonomous_sitl_classic_launch.py
```
