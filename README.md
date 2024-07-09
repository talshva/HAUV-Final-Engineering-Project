# HAUV - Hovering Autonomous Underwater Vehicle

**HAUV** is a compact underwater vehicle designed to enhance underwater exploration and object retrieval. 
Equipped with advanced navigation systems and a suite of sensors, HAUV serves as an essential tool for pre-diving operations, marking objects of interest, and streamlining processes for divers, thereby increasing operational efficiency and safety.

## Hardware
![ROV Diagram](https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/8492f26f-86e4-493d-8b80-7392e1fb8db5)

### Block Diagram Description

The hardware setup of HAUV includes an array of sensors, propulsion systems, communication interfaces, and a control unit. The system diagram illustrates the following components:

- **Main PC (UP Board)**: Acts as the central processing unit, running ROS2 for sensor data processing and system integration.
- **RT MCU (ESP32 WROOM)**: Real-time microcontroller for managing lower-level controls and communication.
- **Sensors**: 9-dof IMU (BNO055), Depth & Pressure (BAR100), Temperature & Humidity (BME280), and DVL for precise navigation and environmental monitoring.
- **Propulsion**: 6x Blue Robotics Thrusters controlled by the ESP32 through PWM signals.
- **Camera System**: Provides visual feedback for navigation and object detection.
- **Lights**: Ensures visibility in underwater environments.

### Connection Overview

- Thrusters, lights, and pan-tilt servo receive control signals from the ESP32 WROOM, which is interfaced with the main PC via serial communication.
- The ESP32 is also responsible for low-level sensor readings and actuator controls.

## Operating Code

### Running the agent:
On the UP board, run:
`ros2 run micro_ros_agent micro_ros_agent serial -b 115200 --dev /dev/ttyUSB0`
after the agent is running, reset the esp32 to let it automatically connect to the agent.

### ROS2 Nodes on UP Board
After successfully connecting to the agent, the esp32 should publish sensor data. 
To check the data validity, use `ros2 topic echo /esp32/<name-of-sensor-topic>`.
To send motor commands to the esp32 and control the ROV, run the following nodes:
- **Guidance Node**: Handles vehicle navigation and control. (`ros2 run autopilot guidance_node`).
- **Joystick Node**: Automatically detects the joystick and publish commands on `/joy` topic (`ros2 run joy joy_node`);

To receive camera data and send it to remote PC via TCP:
- **Camera Node**: Manages the camera system for real-time video feedback (`ros2 run camera_pkg camera_node`);


## Timeline and Work Progress
The timeline documents the steps taken to develop and implement HAUV:

### Initial Planning and Model Characterization
1. **Define and Characterize the Model**:
   - Determine necessary sensors, operational depth, speed, and environment requirements.

2. **Hardware and MCU Selection**:
   - Opt for the ESP32 MCU due to its robust library support and community, and its ample PWM output pins, ensuring it can handle sensor data and motor commands efficiently.

### Software Framework and Initial Testing
1. **Software Installation**:
   - Installed Ubuntu 20.04 Jammy, ROS2 Foxy, Arduino CLI, and Micro-ROS-Agent.
   - Flashed the ESP32 with micro-ROS-Arduino and set up basic publisher-subscriber example code over serial communication.
   
   ![Initial Setup](https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/83639ce9-3ba4-4aba-ab67-78f8ffdfe51a)

2. **Motor and PWM Integration**:
   - Connected the T200 motors, lights, and a servo to the ESP32 PWM outputs.
   - Sent publish commands from the UP board to verify the operation of multiple PWMs simultaneously, while also testing the ROS framework.
   
   ![PWM Verification](https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/980f8fde-eba6-4f22-a2f8-1a80e6792300)

3. **IMU Integration and Initial Movement Simulation**:
   - Integrated the MPU6050 to measure HAUV gyro/acceleration data.
   - Simulated ROV movement using motor speed and MPU acceleration values to calculate yaw, pitch, and roll.
   
   ![IMU Integration](https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/4f9a05bb-01a2-4b5e-82c6-5f0618d48bf6)

### Structural Design and Motor Testing
1. **First Setup with Basic Frame**:
   - Attached 4 horizontal motors and 1 vertical motor to a wooden plate.
   - Verified control in all directions.
   
   ![First Setup](https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/550c35b5-7a5b-4ee3-b198-ca422e674704)

2. **SolidWorks Design and Physical Build**:
   - Designed a representative model in SolidWorks, positioning motors as intended.
   - Built the model using aluminum profiles.
   
   ![SolidWorks Model](https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/4eb4419e-30ca-46c7-8fd0-c56b70a8d7e6)
   ![Model Build](https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/452fb7f9-53f0-420c-80ba-b1bd9afd695c)
   ![Wiring](https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/98da028a-8ae3-42d4-87f7-94673a4631b2)

### Advanced Sensor Integration and Autonomous Control
1. **Advanced IMU (BNO055) Integration**:
   - Integrated the BNO055 for improved yaw, pitch, and roll compensation.
   
   ![BNO055 Integration](https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/b1dcd281-737d-478e-b8a2-e48cd91a614e)

2. **Pathfinder OEM DVL Integration**:
   - Configured the DVL for precise underwater navigation.
   - Tested DVL data accuracy in various scenarios.
   
   ![DVL Integration](https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/b4049c02-31d9-4bf4-a882-99dd7b1478cd)
   ![DVL Depth Test](https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/59ab2e93-2b21-4b62-8523-20c145469a1e)
   ![DVL Data Accuracy](https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/70ba83be-acbd-4e48-92b7-cc2e006fbb0e)

### Final Testing and Simulation
1. **Overall Testing and Autonomous Mode**:
   - Conducted comprehensive tests to verify all control modes and sensor integrations.
   
   ![Final Test](https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/ec95252b-0728-4c9e-8f26-35af209ef355)

2. **RVIZ2 Simulation**:
   - Ensured the simulation service was operational.
   - Ran the guidance node and joystick node for simulation in RVIZ2.
   
   ![Simulation Example](https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/062ddd23-0c9b-471f-b78c-63565cd50323)


### Running RVIZ2 simulation:
- Make sure that the service is up and running, and the guidance node is publishing the motor_data topic.
if not, start the node manually using `ros2 run autopilot_pkg guidance_node`
- Check that the motor_data can be seen. If not, check again for ROS_DOMAIN_ID matching.
- Copy the rov_sim_pkg to rov_ws on the UP board.
- build using `colcon build`
- run `ros2 launch rov_sim_pkg rov.launch.py`
- running joystick node using `ros2 run joy joy_node`
- start playing.
Simulation Example:

https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/062ddd23-0c9b-471f-b78c-63565cd50323


## Software Configuration

### On UP Board (Main PC)

1. **ROS2 Foxy Installation**: Follow the official ROS2 documentation to install ROS2 Foxy on Ubuntu running on the UP board.
2. **ROS Agent**: Install `micro-ros-agent` on the UP board to facilitate communication between ROS2 and micro-ROS on the ESP32.

### On ESP32 (RT MCU)

1. **Micro-ROS-Arduino Setup**: Install the micro-ROS library for Arduino on the ESP32 to enable ROS2 communication.
https://github.com/micro-ROS/micro_ros_arduino
You can find the latest version for micro-ros-arduino-foxy in the releases tab.

3. **Arduino CLI**: Install the Arduino CLI on the UP board to allow code editing, compilation, and flashing of the ESP32 over UART via SSH.
Don't forget to download the relevant libraries to the Arudino folder in the home directory of the UP board.


### Using the arduino CLI to flash the esp32:
1. Go to main sketch directory:
`cd ~/rov_ws/src/esp_sketches/rov_esp_main`
2. Edit the desired code using nano or VScode IDE.
3. Compile the code:
`arduino-cli compile --fqbn esp32:esp32:esp32da rov_esp_main.ino`
4. Flash the code onto the esp32:
`arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32da rov_esp_main.ino`
if upload failed, check that the USB port is not used by something else.

### Typical Commands for Troubleshooting
1. UP:
```bash
ros2 topic list
ros2 topic echo /name_of_topic

```
### future Implementation
After getting everything to work, we ideally prefer to run all nodes on UP startup, using a system service.
These are some troubleshooting commands for the service:
``` bash
sudo systemctl daemon-reload
sudo systemctl start ros2_launch.service
sudo systemctl stop ros2_launch.service
sudo systemctl restart ros2_launch.service
sudo systemctl status ros2_launch.service
sudo journalctl -u ros2_launch.service -f
```
### Additional Technical Details:
- kill nodes directly using `killall guidance_node`
- Remember to source your ROS2 workspace: `source ~/ros2_ws/install/setup.bash`
- When debugging the esp32 using external UART, we can monitor the serial messages:
`screen /dev/ttyUSB1 115200`
if screen isn't terminating, kill manually by:
`ctrl+A, then press K`, or:
`screen -ls` (to get the session id)
`screen -XS <session-id> quit`
  

## Acknowledgments

- Prof. Hugo Guterman, project supervisor.
- Ben-Gurion University's, Electrical and Computer Engineering Department.
