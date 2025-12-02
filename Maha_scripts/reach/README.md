# Pronobis_reach

This repository contains a **customized version of the Reach ROS2 stack** with modifications to support a KUKA iiwa14 robot. It includes:

- Custom robot URDF, SRDF, kinematics, joint limits, and MoveIt configuration files.
- Project-specific scripts and resources.

## Dependencies


- ROS2 Humble/Foxy (or compatible distribution)  
- MoveIt2  
- Reach ROS2  

Make sure you have both this repo and `reach_ros2` cloned and built in your ROS2 workspace (`reach_ws`).

## Setup Instructions

1. Clone this repo and the official `reach_ros2` repo into your ROS2 workspace:

```bash

cd ~/reach_ws/src
git clone <this-repo-url>
git clone https://github.com/Interbotix/reach_ros2.git

```

2. Setup the Robot:

```bash
ros2 launch reach_ros setup.launch.py robot_description_file:=/home/maha/reach_ws/src/lbr_description/urdf/iiwa14/iiwa14.xacro

```

3. Start the Reach ROS2 Node:

```bash
ros2 launch reach_ros start.launch.py \
robot_description_file:=/home/maha/reach_ws/src/lbr_description/urdf/iiwa14/iiwa14.xacro \
robot_description_semantic_file:=/home/maha/reach_ws/src/lbr_description/urdf/iiwa14/iiwa14.srdf \
robot_description_kinematics_file:=/home/maha/reach_ws/src/lbr_description/urdf/iiwa14/kinematics.yaml \
robot_description_joint_limits_file:=/home/maha/reach_ws/src/lbr_description/urdf/iiwa14/joint_limits.yaml \
config_file:=/home/maha/reach_ws/src/lbr_description/config/config_file1.yaml \
config_name:=config_file1

