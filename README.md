# HAUV - Hovering Autonomous Underwater Vehicle




**HAUV** is a compact underwater vehicle designed to enhance underwater exploration and object retrieval. 
Equipped with advanced navigation systems and a suite of sensors, HAUV serves as an essential tool for pre-diving operations, marking objects of interest, and streamlining processes for divers, thereby increasing operational efficiency and safety.



## Operating Code

In general, all nessessary nodes (including the agent) should be automaticaly run on startup. For manual operation:

### Running the agent:
On the UP board, run:
`ros2 run micro_ros_agent micro_ros_agent serial -b 115200 --dev /dev/ttyUSB0`
after the agent is running, reset the esp32 to let it automatically connect to the agent.

### ROS2 Nodes on UP Board
After successfully connecting to the agent, the esp32 should publish sensor data. 
To check the data validity, use `ros2 topic echo /esp32/<name-of-sensor-topic>`.
To send motor commands to the esp32 and control the ROV, run the following nodes:
- **Guidance Node**: Handles vehicle navigation and control. (`ros2 run autopilot guidance_node`).
- **DVL Node**: received ethernet DVL data and publish commands on `/dvl/velocity_data` topic (`ros2 run autopilot dvl_node`).
- **Joystick Node**: Automatically detects the joystick and publish commands on `/joy` topic (`ros2 run joy joy_node`).

-note: 
      Make sure that the DVL is configured to ping and send data on startup. If not, use the Teledyne Tool to send a `CS` command on the desired port.
      To send commands to the DVL using the TRDI Toolz, connect to 192.168.168.102 with port 1033. The DVL is configured to send binary data to port 1034, and string data to port 1037 (Can be configured in 192.168.168.102 on a web             browser).
      Make sure that the user settings are loaded with `CR` command before start pinging with the CS command. For more help type `?` in the Tool's Terminal, or look in in the datasheets.


To receive camera data (sending Image messages as a ros2 topic):
- **Camera Node**: Manages the camera system for real-time video feedback (`ros2 run camera_pkg camera_node`);


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

3. Edit the desired code using nano or VScode IDE.
4. Compile the code:
   
`arduino-cli compile --fqbn esp32:esp32:esp32da rov_esp_main.ino`

5. Flash the code onto the esp32:
   
`arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32da rov_esp_main.ino`

if upload failed, check that the USB port is not used by something else.

### Typical Commands for Troubleshooting
1. UP:
```bash
ros2 topic list
ros2 topic echo /name_of_topic

```
### Automatic Startup Implementation
In general, we ideally prefer to run all nodes on UP startup, using a system service.
These are some troubleshooting commands for the service:
``` bash
sudo systemctl daemon-reload
sudo systemctl start rov_nodes.service
sudo systemctl stop rov_nodes.service
sudo systemctl restart rov_nodes.service
sudo systemctl status rov_nodes.service
sudo journalctl -u rov_nodes.service -f
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




## Hardware

![Components](https://github.com/user-attachments/assets/38843e3d-361e-4e97-9a5d-140aa9777527)


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

## Timeline and Work Progress

1. **Defining and Characterizing the Model**: The initial step was to define and characterize the model. This involved determining the necessary sensors and the conditions in which the HAUV would operate, including depth, speed, and environment.

2. **Choosing the MCU**: To ensure the UP board handled only high-level integration, I opted to use a real-time MCU for sensor data readout and motor control. After comparing various MCUs, including the STM32 Nucleo, I chose the ESP32 due to its extensive libraries, robust community support, and numerous PWM output pins.

3. **Setting Up the Software Framework**: Next, I set up the software framework. This included installing Ubuntu 20.04 (Jammy), ROS2 Foxy, Arduino CLI, and micro-ros-agent. I then flashed the ESP32 with micro-ros-arduino and set up a basic publisher-subscriber example code over serial communication.  
   ![image](https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/83639ce9-3ba4-4aba-ab67-78f8ffdfe51a)

4. **Connecting Components and Verifying PWM Operation**: I proceeded to connect the T200 motors, lights, and a servo to the ESP32 PWM outputs. I sent publish commands from the UP board to verify the operation of multiple PWMs simultaneously, while also checking the ROS framework.  
   ![video](https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/980f8fde-eba6-4f22-a2f8-1a80e6792300)

5. **Integrating the MPU6050**: Subsequently, I integrated the MPU6050 to measure the HAUV's gyro/acceleration data and compensate accordingly. I simulated the ROV movement using motor speeds and MPU acceleration values to calculate yaw, pitch, and roll.  
   ![video](https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/4f9a05bb-01a2-4b5e-82c6-5f0618d48bf6)

6. **First Physical Setup**: My first physical setup involved attaching four motors and one vertical motor to a wooden plate to check all directional controls.  
   ![video](https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/550c35b5-7a5b-4ee3-b198-ca422e674704)

7. **Designing a More Representative Model**: This was followed by designing a more representative model in SolidWorks, ensuring all motors were located in the desired positions. I used aluminum profiles found in the lab, and built a SolidWorks model, which was then constructed in the lab.
8.

   https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/6fc3f76a-386f-4c2c-b053-d802bc5e5e01
  
  ![Bulding the Model](https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/c2f5511f-dc08-4e59-a307-5f374cd1c080)
   ![Wiring the Model](https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/98da028a-8ae3-42d4-87f7-94673a4631b2)

10. **Stabilizing the HAUV**: The next goal was to stabilize the HAUV without any unwanted rotation. I attempted to calculate counterforces based on MPU6050 gyro and acceleration data but encountered noise issues. I then switched to using the BNO055 for its fused data capabilities.
    Using the mpu6050:
    
   ![video](https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/82a6e954-5ed4-43ee-b376-077be81b91f1)

   Overall test with the bno055:
   
   ![video](https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/b1dcd281-737d-478e-b8a2-e48cd91a614e)

12. **Integrating the Pathfinder OEM DVL**: To address x, y, and z velocities, I integrated the Pathfinder OEM DVL by Teledyne. After wiring the DVL for power and communication, I configured, calibrated, and received its data over the Ethernet protocol. Initial tests showed promising accuracy in mm resolution for x and y coordinates.
    
   ![image](https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/b4049c02-31d9-4bf4-a882-99dd7b1478cd)  

   DVL Depth Test using a bucket:

   ![video](https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/59ab2e93-2b21-4b62-8523-20c145469a1e)  

   DVL Real-time Location Test:

   ![video](https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/039a02d6-3f1c-48c4-8025-09734b80c4e4)  

   Test Results:

   ![image](https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/70ba83be-acbd-4e48-92b7-cc2e006fbb0e)

14. **Final Overall Test**: The final overall test involved checking the self-control mode, switching to autonomous mode, and verifying the BNO055 compensation for yaw, pitch, and roll, and the DVL compensation for x, y, and z movements.  

   ![video](https://github.com/talshva/HAUV-Final-Engineering-Project/assets/82408347/ec95252b-0728-4c9e-8f26-35af209ef355)


![tests+cam](https://github.com/user-attachments/assets/a61eaffa-4ed9-42ba-8984-1a3d4e673387)


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



  

## Acknowledgments

- Prof. Hugo Guterman, project supervisor.
- Ben-Gurion University's, Electrical and Computer Engineering Department.
