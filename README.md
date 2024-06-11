# HAUV - Hovering Autonomous Underwater Vehicle

**HAUV** is a compact underwater vehicle designed to enhance underwater exploration and object retrieval. 
Equipped with advanced navigation systems and a suite of sensors, HAUV serves as an essential tool for pre-diving operations, marking objects of interest, and streamlining processes for divers, thereby increasing operational efficiency and safety.


https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/be1a52d4-17eb-419e-841f-f16a777fe32c


## Hardware

![HAUV Block Diagram](https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/4f9e4e93-6bb8-4646-aa27-57463f9cde8e)

### Block Diagram Description

The hardware setup of HAUV includes an array of sensors, propulsion systems, communication interfaces, and a control unit. The system diagram illustrates the following components:

- **Main PC (UP Board)**: Acts as the central processing unit, running ROS2 for sensor data processing and system integration.
- **RT MCU (ESP32 WROOM)**: Real-time microcontroller for managing lower-level controls and communication.
- **Sensors**: 9-dof IMU (BNO055), Depth & Pressure (BAR100), Temperature & Humidity (BME280), and DVL for precise navigation and environmental monitoring.
- **Propulsion**: 6x Blue Robotics Thrusters controlled by the ESP32 through PWM signals.
- **Camera System**: Provides visual feedback for navigation and object detection.
- **Lights**: Ensures visibility in underwater environments.

### Connection Overview

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
- **Guidance Node**: Handles vehicle navigation and control.
- **Camera Node**: Manages the camera system for real-time video feedback.

### Running the agent:
`ros2 run micro_ros_agent micro_ros_agent serial -b 115200 --dev /dev/ttyUSB0`

### Running RVIZ2 simulation:
- Make sure that the service is up and running, and the guidance node is publishing the motor_data topic.
if not, start the node manually using `ros2 run autopilot_pkg guidance_node`
- Check that the motor_data can be seen. If not, check again for ROS_DOMAIN_ID matching.
- run `ros2 launch rov_sim_pkg rov.launch.py`
- running joystick node using `ros2 run joy joy_node`
- start playing.
Simulation Example:


https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/d82e4049-b91d-40fe-90d1-3eb95b58fe3c


### Using the arduino CLI to flash the esp32:
1. Go to main sketch directory:
`cd ~/rov_ws/src/esp_sketches/rov_esp_main`
2. Edit the desired code using nano or VScode IDE.
3. Compile the code:
`arduino-cli compile --fqbn esp32:esp32:esp32da rov_esp_main.ino`
4. Flash the code onto the esp32:
`arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32da rov_esp_main.ino`


### Typical Commands for Troubleshooting
1. UP:
```bash
ros2 topic list
ros2 topic echo /name_of_topic

sudo systemctl daemon-reload
sudo systemctl start ros2_launch.service
sudo systemctl stop ros2_launch.service
sudo systemctl restart ros2_launch.service
sudo systemctl status ros2_launch.service
sudo journalctl -u ros2_launch.service -f
killall guidance_node

# Remember to source your ROS2 workspace
source ~/ros2_ws/install/setup.bash
```
2. ESP:
monitoring the serial messages:
`screen /dev/ttyUSB1 115200`
if screen isn't terminating, kill manually by:
`ctrl+A, then press K`, or:
`screen -ls` (to get the session id)
`screen -XS <session-id> quit`
   


## Acknowledgments

- Prof. Hugo Guterman, project supervisor.
- Ben-Gurion University's, Electrical and Computer Engineering Department.
