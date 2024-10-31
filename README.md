# Tracking the marker with KUKA LBR Med using Polaris Vega ST camera

Required software:
- KUKA sunrise workbench - med 1.15
- Python 3.10
- numpy 1.26.4
- scipy
- scikit library

## Installation
**KUKA Sunrise**

Install KUKA Sunrise using it's installation wizard, and install all of the availible additional software.

**Camera support**

To use the camera installation of Python 3.10 is needed because the library used for the communication with the camera requires numpy version not newer than 1.26.4 which in turn is not supported by Python versions newer than 3.10. You can download it from https://www.python.org/downloads/release/python-31011/

numpy
```bash 
pip install numpy==1.26.4
```
scipy
```bash
pip install scipy
```
scikit
```bash
pip install scikit-surgerynditracker
```

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

To acheave the tracking of the marker you first need to start the ServoTrackingTest app on the robot's controller and then the camera_extraction_data.py on the PC. Both programs are stoped by stoping app on the robot.