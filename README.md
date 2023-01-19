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
After the successfull build close 
### Install px4_autonomous
:warning: This step requires a fully set up ROS2 Humble worspace.

Clone this repository into your ROS2 workspace:
```
cd ~/ros2_ws/src
git clone git@github.com:AImotion-Flight/px4_autonomous.git --recursive .
colcon build
source install/setup.bash
```
### Launching the SITL simulation
Finally the simulation can be started with:
```
source ~/PX4-Autopilot/Tools/simulation/gazebo-classic/setup_gazebo.bash ~/PX4-Autopilot ~/PX4-Autopilot/build/px4_sitl_default
ros2 launch px4_autonomous px4_autonomous_sitl_launch.py
```
