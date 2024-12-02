# Tracking the marker with KUKA LBR Med using Polaris Vega ST camera

Required software:
- KUKA sunrise workbench - med 1.15 with FRI support
- ROS2 Humble
- Python 3.10
- numpy 1.26.4
- scipy
- scikit library

## Installation
**KUKA Sunrise**

Install KUKA Sunrise using it's installation wizard, and install all of the availible additional software.

**ROS**

Install ROS2 Humble using [official guide](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)

**Camera support**

Your Ubuntu instalation most likely comes with appropriate version of Python already installed. You can check that by running
```bash
apt policy python3
```
You should ge output something like:
```
python3:
  Installed: 3.10.6-1~22.04.1
  Candidate: 3.10.6-1~22.04.1
  Version table:
 *** 3.10.6-1~22.04.1 500
        500 http://hr.archive.ubuntu.com/ubuntu jammy-updates/main amd64 Packages
        100 /var/lib/dpkg/status
     3.10.6-1~22.04 500
        500 http://security.ubuntu.com/ubuntu jammy-security/main amd64 Packages
     3.10.4-0ubuntu2 500
        500 http://hr.archive.ubuntu.com/ubuntu jammy/main amd64 Packages

```
Any subversion of Python 3.10 will work fine, and most of lower wersion should as well.

Your installation of ROS should also come with numpy with appropriate version installed, you can check that by running:
```bash
pip list | grep numpy
```
where you should get output:
```
numpy                                1.21.5
```
any version (right column) starting with 1 should be alright.

If you do not have numpy installed you can do so with:
```bash 
pip install numpy==1.26.4
```
And you also need to install additional packages:

scipy
```bash
pip install scipy
```
scikit
```bash
pip install scikit-surgerynditracker
```

**Robot ROS suport**

To use this robot with ROS you will need to install open source support package. For installation follow the instructions on [their page](https://github.com/lbr-stack/lbr_fri_ros2_stack/tree/humble)

## Usage

### KUKA Sunrise

Befor starting it is advised to backup all of the data on the robot because the process will erase it

Load the project with File > Import. Select General > Existing projects into workspace. MarkerNavigation folder is root folder of this project.

Next the project needs to be installed to the robot. Open StationSetup.cat and go to configuration tab. Ther you need to set robot's IP adress (You can find it on the smartPad under Station > Information > Station Server IP). Also if your robot is not mounted flat on the flor be sure to set its mounting orientation. Now you can install the setup onto the robot by going to Installation tab and clicking install.**All of the data currently on the robot will be delited so be sure to backup any project currently on the robot.**

To enable the robot you will probably need to activate it's safety configuration. For that on smartPad go to Station > Safety > Activation and select activation. The default KUKA's password is argus.

Due to this being medical version of LBR robot it has some safety checks that need to be performed at every startup of the robot. To performe those checks you need to run 2 apps, BrakeTestMonitorSampleApp and PositionAndGSMReferencing. BrakeTestMonitorSampleApp also needs to be perforemed every 8 hours that robot is on.

### Camera

Camera needs to be connected to the PC with the ethernet cable trough power ower ethernet (PoE) device. When the camera is on it will "beep" twice and a green LED will ligh up on the front of the camera.

Comunication with the camera is done trough camera_extraction_data.py program. In the code you need to check if IP adresses of the robot and the camera match actual values and change them if they don't. Rest of the code should be good as is.

### Tracking

For the tracking to start run ROS interface on the robot's controller and launch ROS packages on the PC