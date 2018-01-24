# FireAnt\_ROS

ROS - package for BrainyAnt database communication

## Description

FireAnt is a ROS package written in python that allows control of a Raspberry Pi device through the www.brainyant.com platform.

## System Requirements

- Hardware
  - Raspberry Pi v3.0
  - Raspicam v2.1
- Software
  - Raspbian Stretch OS
  - Python 2.7
- Other
  - WiFi or ethernet internet connection
  - Google or Facebook account

## Create an account

  - Go to [brainyant.com](www.brainyant.com)
  - Create an account
  - Create a new robot entry. Include a video stream.
  - Download the __auth.json__ file

## Install dependencies

  - Install ROS (lunar) for Debian 9
    - [ROS lunar Debain](http://wiki.ros.og/Installation/Debian)
    
  - Install Pyrebase
    - Install pip if not installed: `sudo apt-get install python-pip`
    - Get pyrebase: `pip install pyrebase`
    - Apply this [fix](https://gist.github.com/codeAshu/f6384203706e989b0d38db6e0a9d11e7) so Pyrebase works with python2.7 
  
  - Install ffmpeg: `sudo apt-get install ffmpeg`

  - Get FireAnt package
    `git clone https://github.com/BrainyAnt/FireAnt_ROS.git`

  - Copy the **auth.json** file in the *config* folder of this package
    
  - Build package
    `catkin_make --pkg fireant_ros`

## Use
  
  Edit files in _scripts_ folder to give functionality to robot. __control\_lobe.py__ receives the control data which can be used to give commands to actuators. **sensor\_lobe.py** can be used to read sensors upon request.

## Run
  Run the launch file. This starts a ROS core and the three nodes (lobes) in the *scripts* folder.
  `roslaunch fireant_ros fireant_ros.launch`
