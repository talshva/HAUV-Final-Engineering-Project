import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from geometry_msgs.msg import Twist, Vector3
import math

class GuidanceNode(Node):
    def __init__(self):
        super().__init__('guidance_node')
        # Publishers
        self.motors_publisher = self.create_publisher(Twist, '/motor_data', 10) # motor1_data
        self.lights_publisher = self.create_publisher(Vector3, '/light_data', 10) #light1_data
        #Subscribers:
        self.subscription = self.create_subscription(Twist,'/esp32/gyro_accel_data',self.gyro_accel_callback,10)
        #Timers:
        self.motor_light_pub_timer = self.create_timer(0.02, self.motor_light_publisher) # Timer period for 50Hz frequency
        self.sine_test_timer = self.create_timer(0.02, self.sine_test) # Timer period for 50Hz frequency

        # Variables
        self.start_time = self.get_clock().now()
        self.gyro_accel_values = {"linear_x": 0.0, "linear_y": 0.0, "linear_z": 0.0, "angular_x": 0.0, "angular_y": 0.0, "angular_z": 0.0}
        self.motor_velocity = {"motor1": 1500, "motor2": 1500, "motor3": 1500, "motor4": 1500, "motor5": 1500, "cam_servo": 90}
        self.light_intensity = {"light1": 1500, "light2": 1500, "light_couple": 1500}

    def gyro_accel_callback(self, msg):
        self.gyro_accel_values["linear_x"] = msg.linear.x
        self.gyro_accel_values["linear_y"] = msg.linear.y
        self.gyro_accel_values["linear_z"] = msg.linear.z
        self.gyro_accel_values["angular_x"] = msg.angular.x
        self.gyro_accel_values["angular_y"] = msg.angular.y
        self.gyro_accel_values["angular_z"] = msg.angular.z

    def motor_light_publisher(self):
        msg = Twist()
        msg.linear.x = float(self.motor_velocity["motor1"])
        msg.linear.y = float(self.motor_velocity["motor2"])
        msg.linear.z = float(self.motor_velocity["motor3"])
        msg.angular.x = float(self.motor_velocity["motor4"])
        msg.angular.y = float(self.motor_velocity["motor5"])
        msg.angular.z = float(self.motor_velocity["cam_servo"])
        self.motors_publisher.publish(msg)

        msg = Vector3()
        msg.x = float(self.light_intensity["light1"])
        msg.y = float(self.light_intensity["light2"])
        msg.z = float(self.light_intensity["light_couple"])
        self.lights_publisher.publish(msg)


    def sine_test(self):
        self.motors_sine()
        self.lights_sine()


    def update_plot(self, frame):
        for i, line in enumerate(self.lines):
            line.set_ydata(self.y_data[i])
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


    def motors_sine(self):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9  # Convert to seconds
        motor_omega = 2 * math.pi / 6 # Angular frequency for a 6 second period
        servo_omega = 2 * math.pi / 8  # Angular frequency for a 8 second period 
        # Publish sine wave values to thrusters and servo motor
        motor_sine_value = math.sin(motor_omega * elapsed_time)
        servo_sine_value = math.sin(servo_omega * elapsed_time)
        self.motor_velocity["motor1"] = int((motor_sine_value * 400) + 1500)  # Scale sine wave to fit between 1100 and 1900
        self.motor_velocity["motor2"] = int(-(motor_sine_value * 400) + 1500)  # Scale sine wave to fit between 1100 and 1900
        self.motor_velocity["cam_servo"] = int((servo_sine_value * 90) + 90)   # Scale sine wave to fit between 0 and 180
 

    def lights_sine(self):
        current_time = self.get_clock().now()
        elapsed_time = (current_time - self.start_time).nanoseconds / 1e9  # Convert to seconds
        light_omega = 2 * math.pi / 12  # Angular frequency for a 12 second period for lights
      # Publish sine wave to lights
        light_sine_value = math.sin(light_omega * elapsed_time)  
        self.light_intensity["light1"] = int((light_sine_value * 400) + 1500)  # Scale sine wave for lights
        self.light_intensity["light_couple"] = int(-(light_sine_value * 400) + 1500)  # Scale sine wave for lights
 




def main(args=None):
    rclpy.init(args=args)
    guidance_node = GuidanceNode()
    rclpy.spin(guidance_node)
    guidance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

