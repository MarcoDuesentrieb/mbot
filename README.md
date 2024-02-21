# mbot
mBot metapackage containing: [mbot_msgs](https://github.com/MarcoDuesentrieb/mbot/tree/main/mbot_msgs), [mbot_bringup](https://github.com/MarcoDuesentrieb/mbot/tree/main/mbot_bringup)

## Introduction

This package is part of the *"EMG Robot"*-Project.

##  Installation and usage

1. Go to the `src` directory of your ROS workspace and clone the repository:
 ```bash
 git clone https://github.com/MarcoDuesentrieb/mbot.git
 ```

2. Go one level back to your ROS workspace, then build and install the package:

 ```bash
catkin_make_isolated --pkg mbot --install -DCMAKE_INSTALL_PREFIX=/opt/ros/melodic
 ```

3. Now you can run different nodes or launch different launch files, e.g. the demo application:

 ```bash
roslaunch mbot_bringup mbot_demo.launch
 ```

## mbot_bringup

Package designed to interface with the various hardware components of the mBot, including the x728 UPS, mCore, and Arduino Nano 33 IoT.

## mbot_msgs

Contains the `mbot_msgs` message definitions that are used to exchange mBot related data between different nodes.




