<img src=https://pronobis.crta-robotics.com/_ipx/w_896/gallery/secondary_blue.svg alt="project logo" height="350">

# PRONOBIS robot interface

**Required software:**
- KUKA sunrise workbench - med 1.15 with FRI support
- ROS2 Humble
- Python 3.10
- numpy 1.26.4
- scipy
- open cv 4.12.0
- YOLOv8
- scikit library (for polaris kamera)


**Robot ROS suport**

This project uses KUKA LBR robots, to use them with ROS you will need to install open source support package. We are using [lbr_stack](https://github.com/lbr-stack/lbr_fri_ros2_stack/tree/humble), which you can install from [our fork of the repository](https://github.com/matija9/lbr_fri_ros2_stack) with all of the properties set up for best compatibility with the PRONOBIS project. 

## Setup

Clone this repository into your home directory
```bash
git clone git@github.com:CRTA-Lab/KUKA_LBRMed_servoing.git
```

### Robot side setup

>[!TIP]
>This part is done on Windows PC

Open Sunrise Workbench with KUKA_LBRMed_servoing folder as workspace.

If neccessary adjust exact robot beeing used and its configuration

In file *com.FRI.LBRServer.java* add the IP adress of the PC where you will run the ROS packages to the list of the IP adresses

Install the SUnrse setup to the robot and syncronise the programs.


### ROS side setup

Build and source ros packages

```bash
cd ~/KUKA_LBRMed_servoing/ros_ws/
colcon build --symlink-install
source install/local_setup.bash
```

## Usage
Start ultrasound acquisition program on a Windows PC

Run *LBRServer* app on robot's teach pendant

Launch ROS nodes for the whole procedure
```bash
ros2 launch lbr_prostate_sweep sweep.launch.py
```

>[!WARNING]
>It is reccomended to always first run the robot side application because other way arround sometimes causes an error in robot's controller due to sending move comands before FRI is fully set up

## Note
This module is part of the PRONOBIS project, one of CRTA's (Regional Center of Excellence for Robotic Technology) projects. Please check out the [project website](https://pronobis.crta-robotics.com/) as well as [CRTA website](https://crta-robotics.com/en) in order to learn more. Reach out to us if you would like to collaborate.

<img src=https://crta-robotics.com/wp-content/uploads/2025/01/logo-231x300.webp alt="CRTA logo" height="150">
<img src=https://crta-robotics.com/wp-content/uploads/2025/10/Logo_eng-768x239.png alt="FSB logo" height="150">

<img src=https://pronobis.crta-robotics.com/_ipx/w_300/EU.svg alt="EU logo" height="150">