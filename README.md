# ROS Master for ESP-Hover

## Table of Contents
1. [Overview](#overview)
2. [Nodes Overview](#nodes-overview)
   - [joy_node](#joy_node)
   - [drone_controller](#drone_controller)
   - [socket_node](#socket_node)
   - [rqt_ez_publisher](#rqt_ez_publisher)
   - [rqt_multiplot](#rqt_multiplot)
3. [Drone Controller Node Code Structure](#drone-controller-node-code-structure)
4. [License](#license)

## Overview
This repository contains the ROS master nodes used to control the ESP-Hover drone. The system utilizes multiple nodes to handle joystick inputs, drone commands, TCP connections, PID gain adjustments, and data plotting.

## Nodes Overview

### joy_node
The `joy_node` is responsible for receiving key events from the joystick.

### drone_controller
The `drone_controller` node translates joystick events into drone commands and sends them to the drone over WiFi.

### socket_node
The `socket_node` establishes a TCP connection with the drone.

### rqt_ez_publisher
The `rqt_ez_publisher` node sets the PID gains and sends them to the `drone_controller` node.

### rqt_multiplot
The `rqt_multiplot` node is very helpful for tuning the PID gains, as it allows plotting the drone's attitude and PID values.

## Drone Controller Node Code Structure
The code structure for the `drone_controller` node is organized as follows:

```plaintext
[src]
    ├── CMakeLists.txt -> Build configuration files for the project.
    ├── [drone_controller] -> Main directory for the drone_controller node
        ├── CMakeLists.txt
        ├── [config] -> Contains configuration files, such as rqt_multiplot.xml.
            └── rqt_multiplot.xml
        ├── [include]
            └── [drone_controller] 
        ├── [launch] -> Launch files for starting the ROS nodes, including drone_system.launch.
            └── drone_system.launch
        ├── [msg] -> custom message types used by the node.
            ├── AttitudeArray.msg -> Message type for the drone's attitude data.
            ├── FlightRecord.msg -> Message type for flight record data.
            ├── LightFlightRecord.msg ->  Message type for lighter flight record data. (less data)
            └── PID_Gains.msg -> Message type for PID gain values.
        ├── package.xml
        └── [src]
            └── drone_controller.cpp -> Main source file for the drone_controller node.
```
