# Tracking the marker with KUKA LBR Med using Polaris Vega ST camera

Required software:
- KUKA sunrise workbench - med 1.15
- Python 3.10 (not newer)
- numpy 1.26.4 (not newer) and scipy
- scikit library

## Installation
**KUKA Sunrise**

KUKA Sunrise Wokrbench - Med 1.15 needs to be installed on the PC using installation wizard (versions 2.x arent compaible with the robot in the lab). Once it is istalled run the program and create your workspace where your projects will be stored. You will need to install additional software, for that go to Help > Install new software..., and intstall all of the packages in Connectivity, Extensions, Med, Sunrise and Sunrise.Safety folders.

**Camera support**

To use the camera installation of Python 3.10 is needed because the library used for the communication with the camera requires numpy version not newer than 1.26.4 which in turn is not supported by Python versions newer than 3.10. Python official pages no loger offer an installer of this version so the easiest way to do it is an installation from Microsoft Store. 

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

## Kuka sunrise

To load this project go to File > Import. There select General and then Existing projects into workspace. There you need to select project root folder, in this case MarkerNavigation folder.
Once the project is loaded into workbench you first need to install this project on to the robot. For that you will need to open StationSetup.cat. There you can chose which robot you use, what software to install and configure the robot settings. This project was made for KUKA LBR Med 7 so if you are using any other robot be sure to change that in topology tab. In Software tab check if you have all of the software avalible. In configuration tab set robot's IP adress (You can find robot's IP adress on the smartPad under Station > Information > Station Server IP) and mounting orientation if robot is not mounted on the floor. Once it is all set go to the Installation tab and click on install, this will install the new setup on the robot. **All of the data currently on the robot will be delited so be sure to backup any project currently on the robot**. After the installation the robot will reboot. To enable the robot the safety configuration will probably need to be activated. For that on smartPad go to Station > Safety > Activation and select activation. The default KUKA's password is argus. Now the robot is ready for programing. To bring all of the code currently installed you need to syncronise the priject (the icon with 2 arrows in the toolbar)

Due to this being medical version of LBR robot it has some safety checks that need to be performed at every startup of the robot. First thing that has to be done is a Brake test, a sequence that tests the mechanical brakes of the robot for possible defects. This test needs to be performed at each startup of the robot and then every 8 hours after that. To perform the break test run BrakeTestMonitorSampleApp, it will take a couple of minutes for the robot to perform the test. Other thing that needs to be done is joint referencing in which the accuracy of the joint sensors is tested. This test needs to be performed at the botup of the robot, after the brake test. For that you need to run PossitionAndGSMReferencing application, which will again take a couple of minutes to perform. After that an user application can be run.

To start the tracking the marker start ServoTrackingTest application. The application will wait for the TCP/IP connection and will start servo controll of the robot. Since the robot moves along the joint motion path it could be unpredictable so be sure to jog the robot behif the marker to avoid potential crashes. Currently the robot will position itself at (0,0,-30,0,0,0) in the coordinate sistem of the mobile marker.

## Camera

Camera needs to be connected to the PC with the ethernet cable trough power ower ethernet (PoE) device. When the camera is on it will "beep" twice and a green LED will ligh up on the front of the camera.

Check in the code if the IP adresses of the robot and the camera are the same as the ones in the camera_data_extraction.py program, everything else is 

If python and all of the packages are installed then camera_data_extraction.py can be run. This program will establish the connection with the camera and then wait on hte user input before connectiong to the robot. Besure that the robot application is started before python program tries to connect to it, othervise it will raise an error.