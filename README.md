# HAUV - Hovering Autonomous Underwater Vehicle

**HAUV** is an innovative compact underwater vehicle designed to enhance underwater exploration and object retrieval. 
Equipped with advanced navigation systems and a suite of sensors, HAUV serves as an essential tool for pre-diving operations, marking objects of interest, and streamlining processes for divers, thereby increasing operational efficiency and safety.


## Hardware

![HAUV Block Diagram](https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/4f9e4e93-6bb8-4646-aa27-57463f9cde8e)

### Block Diagram Description

The hardware setup of HAUV includes an array of sensors, propulsion systems, communication interfaces, and a control unit. The system diagram illustrates the following components:

- **Main PC (UP Board)**: Acts as the central processing unit, running ROS2 for sensor data processing and system integration.
- **RT MCU (ESP32 WROOM)**: Real-time microcontroller for managing lower-level controls and communication.
- **Sensors**: IMU, USBL, Depth & Pressure (BAR100), Temperature & Humidity (BME280), and DVL for precise navigation and environmental monitoring.
- **Propulsion**: 5x Blue Robotics Thrusters controlled by the ESP32 through PWM signals.
- **Camera System**: Provides visual feedback for navigation and object detection.
- **Lights**: Ensures visibility in underwater environments.

### Connection Overview

- USBL, IMU, DVL, and environmental sensors are connected to the main PC for high-bandwidth data processing.
- Thrusters, camera, lights, and pan-tilt servo receive control signals from the ESP32 WROOM, which is interfaced with the main PC via serial communication.
- The ESP32 is also responsible for low-level sensor readings and actuator controls.

## Software Configuration

### On UP Board (Main PC)

1. **ROS2 Foxy Installation**: Follow the official ROS2 documentation to install ROS2 Foxy on Ubuntu running on the UP board.
2. **ROS Agent**: Install `micro-ros-agent` on the UP board to facilitate communication between ROS2 and micro-ROS on the ESP32.

### On ESP32 (RT MCU)

1. **Micro-ROS-Arduino Setup**: Install the micro-ROS library for Arduino on the ESP32 to enable ROS2 communication.
2. **Arduino CLI**: Install the Arduino CLI on the UP board to allow code editing, compilation, and flashing of the ESP32 over SSH.

## Operating Code

### ROS2 Nodes on UP Board

- **Autopilot Node**: Handles vehicle navigation and control.
- **Guidance Node**: Processes sensor data to assist in object detection and mapping.
- **Camera Node**: Manages the camera system for real-time video feedback.

### Typical Commands for Troubleshooting

```bash
ros2 topic list
ros2 topic echo /name_of_topic
journalctl -u ros2_service_name
# Remember to source your ROS2 workspace
source ~/ros2_ws/install/setup.bash
```

## Common Issues

- **VS Code Connection Issues**: Solutions for `XHR Failed` errors and reestablishing a stable connection.
- **Freeing Memory on UP Board**: Commands for cleaning up unused files and managing system resources.


## Acknowledgments

- Prof. Hugo Guterman, project supervisor.
- Ben-Gurion University's, Electrical and Computer Engineering Department.
