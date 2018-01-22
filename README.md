# FireAnt ROS - package for BrainyAnt database communication

# Description
FireAnt is a ROS package written in python that allows control of a Raspberry Pi device through the www.brainyant.com platform.

# System Requirements
  - Raspberry Pi v3.0
  - Raspicam v2.1
  - Raspbian Stretch OS
  - Python 2.7
  - WiFi or ethernet internet connection
  - Google or Facebook account

# Installation
  Install ROS (kinetic)
    http://wiki.ros.og/Installation/Debian
    sudo apt-get install ros-kinetic-ros-base
  
  Install Pyrebase
    - Install pip if not installed:
      sudo apt-get install python-pip
    - Get pyrebase:
      pip install pyrebase
    - Apply fix for python 2.7
      https://gist.github.com/codeAshu/f6384203706e989b0d38db6e0a9d11e7
  
  Get FireAnt package
    - git clone
  
  Build package
    - catkin_make --pkg ant

# Use
  Run launch file
  $ roslaunch ant ___________.launch
