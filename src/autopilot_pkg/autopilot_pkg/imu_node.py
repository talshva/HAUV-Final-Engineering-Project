import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
import math


class IMUNode(Node):

    def __init__(self):
        super().__init__('imu_node')
        self.subscription = self.create_subscription(Twist, '/esp32/gyro_accel_data', self.imu_callback, 10)
        self.motor_subscription = self.create_subscription(Twist, '/motor_data', self.motor_callback, 10)
        self.publisher = self.create_publisher(Imu, 'imu/imu_data', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize filter state
        self.last_time = self.get_clock().now().nanoseconds
        self.alpha = 0.98  # Complementary filter constant
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # Position state
        self.position_x = 0.0
        self.position_y = 0.0
        self.position_z = 0.0
        self.forward_speed = 0.0

        # Constants
        self.k = 0.005  # Adjust this to scale speed
        self.pwm_stop = 1500

    def imu_callback(self, msg):
        current_time = self.get_clock().now().nanoseconds
        dt = (current_time - self.last_time) / 1e9  # Convert nanoseconds to seconds
        self.last_time = current_time

        # Gyroscope angles
        roll_gyro = self.roll + msg.angular.x * dt
        pitch_gyro = self.pitch + msg.angular.y * dt
        yaw_gyro = self.yaw + msg.angular.z * dt

        # Accelerometer angles
        roll_accel = math.atan2(msg.linear.y, msg.linear.z)
        pitch_accel = math.atan2(-msg.linear.x, math.sqrt(msg.linear.y ** 2 + msg.linear.z))

        # Complementary filter
        self.roll = self.alpha * roll_gyro + (1 - self.alpha) * roll_accel
        self.pitch = self.alpha * pitch_gyro + (1 - self.alpha) * pitch_accel
        self.yaw = yaw_gyro

        # Update position
        self.position_x += self.forward_speed * dt * math.cos(self.yaw)
        self.position_y += self.forward_speed * dt * math.sin(self.yaw)

        # Convert roll, pitch, yaw to quaternion
        qx, qy, qz, qw = self.euler_to_quaternion(self.roll, self.pitch, self.yaw)

        # Create an Imu message
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"
        imu_msg.linear_acceleration.x = msg.linear.x
        imu_msg.linear_acceleration.y = msg.linear.y
        imu_msg.linear_acceleration.z = msg.linear.z
        imu_msg.angular_velocity.x = msg.angular.x
        imu_msg.angular_velocity.y = msg.angular.y
        imu_msg.angular_velocity.z = msg.angular.z
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw
        self.publisher.publish(imu_msg)

        # Publish the transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"  # Reference frame
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.position_x
        t.transform.translation.y = self.position_y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(t)

    def motor_callback(self, msg):
        self.forward_speed = self.k * (msg.linear.x - self.pwm_stop)

    def euler_to_quaternion(self, roll, pitch, yaw):
        """Convert Euler angles to quaternion."""
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return (qx, qy, qz, qw)


def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
