import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, Vector3
import math

class GuidanceNode(Node):
    def __init__(self):
        super().__init__('guidance_node')
        # Publishers
        self.motors_publisher = self.create_publisher(Twist, '/motor_data', 10)
        self.lights_publisher = self.create_publisher(Vector3, '/light_data', 10)
        # Subscribers:
        self.subscription = self.create_subscription(Twist, '/esp32/gyro_accel_data', self.gyro_accel_callback, 10)
        self.joy_subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        # Timers:
        self.motor_light_pub_timer = self.create_timer(0.05, self.motor_light_publisher) # 20hz

        # Variables
        self.start_time = self.get_clock().now()
        self.gyro_accel_values = {"linear_x": 0.0, "linear_y": 0.0, "linear_z": 0.0, "angular_x": 0.0, "angular_y": 0.0, "angular_z": 0.0}
        self.motor_velocity = {"motor1": 1500, "motor2": 1500, "motor3": 1500, "motor4": 1500, "motor5": 1500, "cam_servo": 90}
        self.light_intensity = {"light1": 1100, "light2": 1100, "light_couple": 1100}
        #self.joy_speed = 0.0  # Speed control from joystick
        self.joystick_axes = [0.0] * 6  # Initialize joystick axes array
        self.joystick_buttons = [0] * 3
        self.joy_cam = 0.0

    def gyro_accel_callback(self, msg):
        self.gyro_accel_values["linear_x"] = msg.linear.x
        self.gyro_accel_values["linear_y"] = msg.linear.y
        self.gyro_accel_values["linear_z"] = msg.linear.z
        self.gyro_accel_values["angular_x"] = msg.angular.x
        self.gyro_accel_values["angular_y"] = msg.angular.y
        self.gyro_accel_values["angular_z"] = msg.angular.z

    def joy_callback(self, msg):
        # Read joystick inputs:
        self.joystick_axes[4] = msg.axes[4] # forward/backward
        self.joystick_axes[3] = msg.axes[3] # left/right
        self.joystick_axes[5] = msg.axes[5] # Clockwise rotation
        self.joystick_axes[2] = msg.axes[2] # Counter-clockwise rotation
        self.joy_cam = msg.axes[0]
        self.joystick_buttons[0] = msg.buttons[0]
        self.joystick_buttons[1] = msg.buttons[1]
        self.joystick_buttons[2] = msg.buttons[3]

    def motor_light_publisher(self):
        # Map the joystick speed [-1, 1] to motor PWM [1100, 1900]
        #motor_pwm = int((self.joy_speed * 400) + 1500)
        #motor_pwm = max(1100, min(1900, motor_pwm))  # Clamp between 1100 and 1900
        cam_servo_pwm = int((-1 * self.joy_cam * 90) + 90)
        cam_servo_pwm = max(0, min(180, cam_servo_pwm))  # Clamp between 0 and 180

        # Calculate motor speeds:
        # Here we map the joystick range [-1, 1] to PWM range [1100, 1900]. Adjust as necessary.
        # The coefficients are chosen based on the motor position in the X configuration.
        forward_backward = self.joystick_axes[4]
        left_right = self.joystick_axes[3]
        rotate_cw = -self.joystick_axes[5]
        rotate_ccw = self.joystick_axes[2] 
        # Combined yaw input (assuming axes[5] is -1 to 1 where -1 is full CW press)
        yaw_input = rotate_cw + rotate_ccw

        # Convert joystick input range [-1, 1] to PWM range [1100, 1900]
        scale_factor = 400  # Scale factor for motor speed adjustment

        # Calculate motor speeds based on joystick input and motor angles
        motor1_speed = 1500 + (forward_backward * scale_factor * math.cos(math.radians(45)) +
                               left_right * scale_factor * math.sin(math.radians(45)) +
                               yaw_input * scale_factor)
        motor2_speed = 1500 + (forward_backward * scale_factor * math.cos(math.radians(45)) -
                               left_right * scale_factor * math.sin(math.radians(45)) -
                               yaw_input * scale_factor)
        motor3_speed = 1500 + (-forward_backward * scale_factor * math.cos(math.radians(45)) +
                               left_right * scale_factor * math.sin(math.radians(45)) -
                               yaw_input * scale_factor)

        motor4_speed = 1500

        motor5_speed = 1500 + (-forward_backward * scale_factor * math.cos(math.radians(45)) -
                               left_right * scale_factor * math.sin(math.radians(45)) +
                               yaw_input * scale_factor)
        
        self.motor_velocity["motor1"] = motor1_speed # front left
        self.motor_velocity["motor2"] = motor2_speed # front right
        self.motor_velocity["motor3"] = motor3_speed # back right
        self.motor_velocity["motor4"] = motor4_speed # vertical
        self.motor_velocity["motor5"] = motor5_speed # back left
        self.motor_velocity["cam_servo"] = cam_servo_pwm

        # # Calculate light intensity based on absolute difference from 1500
        # light_intensity = int((abs(motor1_speed - 1500) / 400) * 800 + 1100)
        # if motor1_speed == 1500:
        #     light_intensity = 1100
        if self.joystick_buttons[0]:
            if self.light_intensity["light1"] == 1100:
                self.light_intensity["light1"] = 1900
            else:
                self.light_intensity["light1"] = 1100

        if self.joystick_buttons[1]:
            if self.light_intensity["light2"] == 1100:
                self.light_intensity["light2"] = 1900
            else:
                self.light_intensity["light2"] = 1100

        if self.joystick_buttons[2]:
            if self.light_intensity["light_couple"] == 1100:
                self.light_intensity["light_couple"] = 1900
            else:
                self.light_intensity["light_couple"] = 1100

        # Publish motor speeds
        msg = Twist()
        msg.linear.x = float(self.motor_velocity["motor1"])
        msg.linear.y = float(self.motor_velocity["motor2"])
        msg.linear.z = float(self.motor_velocity["motor3"])
        msg.angular.x = float(self.motor_velocity["motor4"])
        msg.angular.y = float(self.motor_velocity["motor5"])
        msg.angular.z = float(self.motor_velocity["cam_servo"])
        self.motors_publisher.publish(msg)

        # Publish light intensities
        msg = Vector3()
        msg.x = float(self.light_intensity["light1"])
        msg.y = float(self.light_intensity["light2"])
        msg.z = float(self.light_intensity["light_couple"])
        self.lights_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    guidance_node = GuidanceNode()
    rclpy.spin(guidance_node)
    guidance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()












# import rclpy
# from rclpy.node import Node
# from std_msgs.msg import Int32
# from geometry_msgs.msg import Twist, Vector3
# import math

# class GuidanceNode(Node):
#     def __init__(self):
#         super().__init__('guidance_node')
#         # Publishers
#         self.motors_publisher = self.create_publisher(Twist, '/motor_data', 10) # motor1_data
#         self.lights_publisher = self.create_publisher(Vector3, '/light_data', 10) #light1_data
#         #Subscribers:
#         self.subscription = self.create_subscription(Twist,'/esp32/gyro_accel_data',self.gyro_accel_callback,10)
#         #Timers:
#         self.motor_light_pub_timer = self.create_timer(0.1, self.motor_light_publisher) # Timer period for 10Hz frequency
#         self.sine_test_timer = self.create_timer(0.02, self.sine_test) # Timer period for 50Hz frequency

#         # Variables
#         self.start_time = self.get_clock().now()
#         self.gyro_accel_values = {"linear_x": 0.0, "linear_y": 0.0, "linear_z": 0.0, "angular_x": 0.0, "angular_y": 0.0, "angular_z": 0.0}
#         self.motor_velocity = {"motor1": 1500, "motor2": 1500, "motor3": 1500, "motor4": 1500, "motor5": 1500, "cam_servo": 90}
#         self.light_intensity = {"light1": 1500, "light2": 1500, "light_couple": 1500}

#     def gyro_accel_callback(self, msg):
#         self.gyro_accel_values["linear_x"] = msg.linear.x
#         self.gyro_accel_values["linear_y"] = msg.linear.y
#         self.gyro_accel_values["linear_z"] = msg.linear.z
#         self.gyro_accel_values["angular_x"] = msg.angular.x
#         self.gyro_accel_values["angular_y"] = msg.angular.y
#         self.gyro_accel_values["angular_z"] = msg.angular.z

#     def motor_light_publisher(self):
#         msg = Twist()
#         msg.linear.x = float(self.motor_velocity["motor1"])
#         msg.linear.y = float(self.motor_velocity["motor2"])
#         msg.linear.z = float(self.motor_velocity["motor3"])
#         msg.angular.x = float(self.motor_velocity["motor4"])
#         msg.angular.y = float(self.motor_velocity["motor5"])
#         msg.angular.z = float(self.motor_velocity["cam_servo"])
#         self.motors_publisher.publish(msg)

#         msg = Vector3()
#         msg.x = float(self.light_intensity["light1"])
#         msg.y = float(self.light_intensity["light2"])
#         msg.z = float(self.light_intensity["light_couple"])
#         self.lights_publisher.publish(msg)


#     def sine_test(self):
#         self.motors_sine()
#         self.lights_sine()


#     def update_plot(self, frame):
#         for i, line in enumerate(self.lines):
#             line.set_ydata(self.y_data[i])
#         self.fig.canvas.draw()
#         self.fig.canvas.flush_events()


#     def motors_sine(self):
#         current_time = self.get_clock().now()
#         elapsed_time = (current_time - self.start_time).nanoseconds / 1e9  # Convert to seconds
#         motor_omega = 2 * math.pi / 6 # Angular frequency for a 6 second period
#         servo_omega = 2 * math.pi / 8  # Angular frequency for a 8 second period 
#         # Publish sine wave values to thrusters and servo motor
#         motor_sine_value = math.sin(motor_omega * elapsed_time)
#         servo_sine_value = math.sin(servo_omega * elapsed_time)
#         self.motor_velocity["motor1"] = int((motor_sine_value * 400) + 1500)  # Scale sine wave to fit between 1100 and 1900
#         self.motor_velocity["motor2"] = int(-(motor_sine_value * 400) + 1500)  # Scale sine wave to fit between 1100 and 1900
#         self.motor_velocity["cam_servo"] = int((servo_sine_value * 90) + 90)   # Scale sine wave to fit between 0 and 180
 

#     def lights_sine(self):
#         current_time = self.get_clock().now()
#         elapsed_time = (current_time - self.start_time).nanoseconds / 1e9  # Convert to seconds
#         light_omega = 2 * math.pi / 12  # Angular frequency for a 12 second period for lights
#       # Publish sine wave to lights
#         light_sine_value = math.sin(light_omega * elapsed_time)  
#         self.light_intensity["light1"] = int((light_sine_value * 400) + 1500)  # Scale sine wave for lights
#         self.light_intensity["light_couple"] = int(-(light_sine_value * 400) + 1500)  # Scale sine wave for lights
 




# def main(args=None):
#     rclpy.init(args=args)
#     guidance_node = GuidanceNode()
#     rclpy.spin(guidance_node)
#     guidance_node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

