# telebot_controller

## Overview

This node allows communication between a telebot_controller [composed of an IMU sensor, a potentiometer and two push buttons for mode selection] and a Kinova Gen3 robot. It runs the Publishers that publish IMU Data and Potentiometer values.

## Usage

* Ensure both the "controller" and "agent" are connected to the same virtual network. We suggest [Husarnet's VPN service](https://husarion.com/tutorials/ros-tutorials/6-robot-network/).
* Run this terminal command to launch the push_button node:
```
    rosrun telebot_controller push_button.py
```
* Run this terminal command to launch the gripper_publisher node:
```
    rosrun telebot_controller gripper_out.py
```
  

