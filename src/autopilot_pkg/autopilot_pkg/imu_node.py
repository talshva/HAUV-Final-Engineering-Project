import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import math

class IMUNode(Node):

    def __init__(self):
        super().__init__('imu_node')
        self.get_logger().info("Starting IMU Node")
        self.subscription = self.create_subscription(Twist, '/esp32/gyro_accel_data', self.imu_callback, 10)
        self.publisher = self.create_publisher(Imu, 'imu/data', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def imu_callback(self, msg):
        self.get_logger().debug("Received IMU data")
        # Create an Imu message
        imu_msg = Imu()
        
        # Set the header
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "imu_link"

        # Set the linear acceleration
        imu_msg.linear_acceleration.x = msg.linear.x
        imu_msg.linear_acceleration.y = msg.linear.y
        imu_msg.linear_acceleration.z = msg.linear.z

        # Set the angular velocity
        imu_msg.angular_velocity.x = msg.angular.x
        imu_msg.angular_velocity.y = msg.angular.y
        imu_msg.angular_velocity.z = msg.angular.z

        # Convert acceleration to orientation
        roll = math.atan2(msg.linear.y, msg.linear.z)
        pitch = math.atan2(-msg.linear.x, math.sqrt(msg.linear.y ** 2 + msg.linear.z ** 2))
        yaw = 0.0  # Yaw is not determined by acceleration

        # Convert roll, pitch, yaw to quaternion
        qx, qy, qz, qw = self.euler_to_quaternion(roll, pitch, yaw)
        imu_msg.orientation.x = qx
        imu_msg.orientation.y = qy
        imu_msg.orientation.z = qz
        imu_msg.orientation.w = qw

        # Publish the IMU message
        self.publisher.publish(imu_msg)
        self.get_logger().debug("Published IMU message")

        # Publish the transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "imu_link"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.05
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().debug("Published transform")

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
