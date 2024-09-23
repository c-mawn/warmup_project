

<!--  DELETE THE LINES ABOVE THIS AND WRITE YOUR PROJECT README BELOW -->

---
# Warmup Project


Programs a Neato to execute the following behaviors: 
- teleop
- draw a square
- follow walls
- follow people
- avoid obstacles

Connect to a physical Neato or use Gazebo to simulate Neato. 

## Requirements

> Follow **[this tutorial](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)** to install Ros2 humble-desktop\
> Follow **[this tutorial](https://gazebosim.org/docs/latest/getstarted/)** to install Gazebo Fortress

1. You are using Linux or Mac OS machine. Windows does not support our teleop function
2. Does not work with Anaconda

## Install
Install `ros2_ws` and `neato_packages`
```
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone git@github.com:comprobo24/neato_packages.git
cd ~/ros2_ws
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```
To install this packagee, 
```
cd ~/ros2_ws/src
git clone git@github.com:c-mawn/warmup_project.git
```

## Usage

Build packages using
```
colcon build --symlink-install
source ~/ros2_ws/install/setup.bash
```
Run an executable using
```
ros2 run package_name executable_name
```
For example, to run the teleop package
```
ros2 run warmup_project teleop
```

