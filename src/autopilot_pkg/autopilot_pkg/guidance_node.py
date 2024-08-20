import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3
from nav_msgs.msg import Odometry
import math

class GuidanceNode(Node):
    def __init__(self):
        super().__init__('guidance_node')
        self.get_logger().info(f"Guidance Node has been started!")
        
        # Publishers
        self.motors_publisher = self.create_publisher(Twist, '/motor_data', 10)
        self.lights_publisher = self.create_publisher(Vector3, '/lights_servo_data', 10)
        self.odometry_publisher = self.create_publisher(Odometry, '/odometry', 10)
        
        # Subscribers
        self.orientation_subscription = self.create_subscription(Twist, '/esp32/bno055_data', self.orientation_callback, 10)
        self.dvl_subscription = self.create_subscription(Twist, '/dvl/velocity_data', self.dvl_callback, 10)
        self.joy_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.bar100_subscription = self.create_subscription(Vector3, '/esp32/bar100_data', self.bar100_callback, 10)
        
        # Timers
        self.motor_light_pub_timer = self.create_timer(0.05, self.motor_light_publisher)  # 20 Hz

        # Variables
        self.start_time = self.get_clock().now()
        self.orientation = {"x": 0.0, "y": 0.0, "z": 0.0}  # roll, pitch, yaw
        self.dvl_velocities = {"vx": 0.0, "vy": 0.0, "vz": 0.0}
        self.motor_velocity = {"motor1": 1500, "motor2": 1500, "motor3": 1500, "motor4": 1500, "motor5": 1500, "cam_servo": 90, "motor6": 1500}
        self.light_intensity = {"light_single": 1100, "light_couple": 1100}
        self.joystick_axes = [0.0] * 6  # Initialize joystick axes array
        self.joystick_buttons = [0] * 6
        self.joy_cam_up = 0.0
        self.joy_cam_down = 0.0
        self.joystick_mode = True  # Start in joystick mode by default
        self.previous_button_states = [False] * 6  # Track previous button states for debouncing

        # Target orientation in autonomous mode
        self.target_orientation = {"x": 0.0, "y": 0.0, "z": 0.0}

        # PID constants for autonomous control (these values need to be tuned)
        self.kp_yaw = 15.0
        self.kd_yaw = 2.0
        self.kp_pitch = 15.0
        self.kd_pitch = 2.0
        self.kp_roll = 15.0
        self.kd_roll = 2.0
        # Initialize state estimate for odometry
        self.state_estimate = [0.0, 0.0, 0.0]  # x, y, z position        

    def orientation_callback(self, msg):
        self.orientation["x"] = msg.linear.x #yaw
        self.orientation["y"] = msg.linear.y #pitch
        self.orientation["z"] = msg.linear.z #roll

    def dvl_callback(self, msg):
        self.dvl_velocities["vx"] = msg.linear.x
        self.dvl_velocities["vy"] = msg.linear.y
        self.dvl_velocities["vz"] = msg.linear.z

    def joy_callback(self, msg):
        # Read joystick inputs
        self.joystick_axes[1] = msg.axes[1]  # forward/reverse
        self.joystick_axes[0] = msg.axes[0]  # left/right
        self.joystick_axes[4] = msg.axes[4]  # ascend/descend
        self.joystick_axes[3] = msg.axes[3]  # yaw left/right
        self.joy_cam_up = msg.axes[5]
        self.joy_cam_down = msg.axes[2]
        self.joystick_buttons[0] = msg.buttons[0]
        self.joystick_buttons[1] = msg.buttons[1]
        self.joystick_buttons[2] = msg.buttons[3]

        # Debounce logic for mode switch button (buttons[5])
        if msg.buttons[5] and not self.previous_button_states[5]:
            self.joystick_mode = not self.joystick_mode
            self.get_logger().info(f'Switched to {"Joystick" if self.joystick_mode else "Autonomous"} mode')
            if not self.joystick_mode:
                self.target_orientation = self.orientation.copy()

        # Debounce logic for light toggle buttons
        if msg.buttons[0] and not self.previous_button_states[0]:
            self.light_intensity["light_single"] = 1900 if self.light_intensity["light_single"] == 1100 else 1100
        if msg.buttons[1] and not self.previous_button_states[1]:
            self.light_intensity["light_couple"] = 1900 if self.light_intensity["light_couple"] == 1100 else 1100

        # Update previous button states
        self.previous_button_states[5] = msg.buttons[5]
        self.previous_button_states[0] = msg.buttons[0]
        self.previous_button_states[1] = msg.buttons[1]

    def bar100_callback(self, msg):
        pass

    def motor_light_publisher(self):
        if self.joystick_mode:
            # Joystick control logic
            cam_servo_pwm = 90
            if self.joy_cam_up != 1:
                cam_servo_pwm = int((1 - self.joy_cam_up) * 45 + 90)  # Maps from 90 to 180
            elif self.joy_cam_down != 1:
                cam_servo_pwm = int((1 + self.joy_cam_down) * 45)  # Maps from 90 to 0

            cam_servo_pwm = max(0, min(180, cam_servo_pwm))  # Clamp between 0 and 180

            # Calculate motor speeds:
            forward_backward = self.joystick_axes[1]
            left_right = self.joystick_axes[0]
            ascend_descend = self.joystick_axes[4]
            yaw = self.joystick_axes[3]

            # Convert joystick input range [-1, 1] to PWM range [1100, 1900]
            scale_factor = 400  # Scale factor for motor speed adjustment

            # Calculate motor speeds based on joystick input and motor angles
            motor1_speed = 1500 + (forward_backward * scale_factor * math.cos(math.radians(45)) +
                                   left_right * scale_factor * math.sin(math.radians(45)) +
                                   yaw * scale_factor)
            motor3_speed = 1500 + (forward_backward * scale_factor * math.cos(math.radians(45)) -
                                   left_right * scale_factor * math.sin(math.radians(45)) -
                                   yaw * scale_factor)
            motor6_speed = 1500 + (-forward_backward * scale_factor * math.cos(math.radians(45)) +
                                   left_right * scale_factor * math.sin(math.radians(45)) -
                                   yaw * scale_factor)
            motor4_speed = 1500 + (-forward_backward * scale_factor * math.cos(math.radians(45)) -
                                   left_right * scale_factor * math.sin(math.radians(45)) +
                                   yaw * scale_factor)
            motor2_speed = 1500 + (ascend_descend * scale_factor)
            motor5_speed = 1500 + (ascend_descend * scale_factor)

            self.motor_velocity["motor1"] = motor1_speed  # front right
            self.motor_velocity["motor2"] = motor2_speed  # rear_vertical
            self.motor_velocity["motor3"] = motor3_speed  # rear_right
            self.motor_velocity["motor4"] = motor4_speed  # rear_left
            self.motor_velocity["motor5"] = motor5_speed  # front_vertical
            self.motor_velocity["motor6"] = motor6_speed  # front_left
            self.motor_velocity["cam_servo"] = cam_servo_pwm
        else:
            # Autonomous control logic
            self.calculate_autonomous_motor_speeds()

        # Publish motor speeds
        msg = Twist()
        msg.linear.x = float(self.motor_velocity["motor1"])
        msg.linear.y = float(self.motor_velocity["motor2"])
        msg.linear.z = float(self.motor_velocity["motor3"])
        msg.angular.x = float(self.motor_velocity["motor4"])
        msg.angular.y = float(self.motor_velocity["motor5"])
        msg.angular.z = float(self.motor_velocity["motor6"])
        self.motors_publisher.publish(msg)

        # Publish light intensities
        light_msg = Vector3()
        light_msg.x = float(self.light_intensity["light_single"])
        light_msg.y = float(self.light_intensity["light_couple"])
        light_msg.z = float(self.motor_velocity["cam_servo"])
        self.lights_publisher.publish(light_msg)

    def calculate_autonomous_motor_speeds(self):
        # Convert target orientation and current orientation from degrees to radians
        target_yaw_rad = math.radians(self.target_orientation["x"])
        target_pitch_rad = math.radians(self.target_orientation["y"])
        
        current_yaw_rad = math.radians(self.orientation["x"])
        current_pitch_rad = math.radians(self.orientation["y"])

        # PID control for maintaining orientation with angle wrap-around handling
        yaw_error = self.angle_wrap(target_yaw_rad - current_yaw_rad)
        pitch_error = self.angle_wrap(target_pitch_rad - current_pitch_rad)

        yaw_adjustment = int((400/(15*math.pi)) * self.kp_yaw * yaw_error)# + self.kd_yaw * current_yaw_rad)
        pitch_adjustment = int((400/(15*math.pi)) * self.kp_pitch * pitch_error)# + self.kd_pitch * current_pitch_rad)

        # self.get_logger().info(f"yaw_error: {yaw_error}")
        # self.get_logger().info(f"yaw_adjustment: {yaw_adjustment}")

        # Use DVL velocities to correct the speeds
        vx = self.dvl_velocities["vx"]*1000
        vy = self.dvl_velocities["vy"]*1000
        vz = self.dvl_velocities["vz"]*1000

        # Calculate motor speeds based on adjustments and DVL velocities
        motor1_speed = 1500 + yaw_adjustment + (vx + vy) / math.sqrt(2)
        motor3_speed = 1500 + yaw_adjustment + (vx - vy) / math.sqrt(2)
        motor6_speed = 1500 - yaw_adjustment - (vx + vy) / math.sqrt(2)
        motor4_speed = 1500 - yaw_adjustment - (vx - vy) / math.sqrt(2)
        motor2_speed = 1500 + pitch_adjustment + vz  # rear_vertical
        motor5_speed = 1500 - pitch_adjustment + vz  # front_vertical

        # Clamp motor speeds to valid range
        motor1_speed = max(1100, min(1900, motor1_speed))
        motor2_speed = max(1100, min(1900, motor2_speed))
        motor3_speed = max(1100, min(1900, motor3_speed))
        motor5_speed = max(1100, min(1900, motor5_speed))
        motor4_speed = max(1100, min(1900, motor4_speed))
        motor6_speed = max(1100, min(1900, motor6_speed))

        self.motor_velocity["motor1"] = motor1_speed
        self.motor_velocity["motor2"] = motor2_speed
        self.motor_velocity["motor3"] = motor3_speed
        self.motor_velocity["motor4"] = motor4_speed
        self.motor_velocity["motor5"] = motor5_speed
        self.motor_velocity["motor6"] = motor6_speed

    def angle_wrap(self, angle):
        """Wrap angle to range [-pi, pi]"""
        return (angle + math.pi) % (2 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    guidance_node = GuidanceNode()
    rclpy.spin(guidance_node)
    guidance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
