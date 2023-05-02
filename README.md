# PX4 + ROS2 Autonomous UAV
## Run the SITL simulation
### Get the PX4 Source Code
Clone the PX4 Autopilot:
```
git clone git@github.com:AImotion-Flight/PX4-Autopilot.git --recursive ~/PX4-Autopilot
```
### Install px4_autonomous
:warning: This step requires a fully set up ROS2 Humble workspace.

Clone this repository into your ROS2 workspace:
```
cd ~/ros2_ws/
git clone -b multiagent git@github.com:AImotion-Flight/px4_autonomous.git --recursive src/
colcon build
source install/setup.bash
```
### Launching the SITL simulation (Gazebo Classic)
Build SITL Gazebo Classic:
```
cd ~/PX4-Autopilot
DONT_RUN=1 make px4_sitl gazebo
```
Start the SITL simulation with:
```
ros2 launch px4_autonomous px4_autonomous_sitl_launch.py
```
Launch PX4 offboard:
```bash
ros2 launch px4_autonomous px4_autonomous_launch.py
# Note: If PX4 is not automatically switching to offboard mode, open QGC and select the mode manually
```
