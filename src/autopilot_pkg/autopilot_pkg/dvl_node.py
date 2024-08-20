import socket
from pprint import pprint
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

def parse_system_attitude(data):
    try:
        parts = data.split(',')
        result = {
            'pitch': float(parts[1]),
            'roll': float(parts[2]),
            'heading': float(parts[3])
        }
        return result
    except (ValueError, IndexError) as e:
        print(f"Error parsing system attitude: {e}")
        return None

def parse_timing_scaling(data):
    try:
        parts = data.split(',')
        result = {
            'timestamp': parts[1],
            'salinity': float(parts[2]),
            'temperature': float(parts[3]),
            'depth': float(parts[4]),
            'speed_of_sound': float(parts[5]),
            'bit_result': parts[6]
        }
        return result
    except (ValueError, IndexError) as e:
        print(f"Error parsing timing scaling: {e}")
        return None

def parse_earth_referenced_velocity(data):
    try:
        parts = data.split(',')
        result = {
            'east_velocity': int(parts[1]) / 1000.0,  # Convert mm/s to m/s
            'north_velocity': int(parts[2]) / 1000.0,  # Convert mm/s to m/s
            'upward_velocity': int(parts[3]) / 1000.0,  # Convert mm/s to m/s
            'status': parts[4]
        }
        return result
    except (ValueError, IndexError) as e:
        print(f"Error parsing earth referenced velocity: {e}")
        return None

def parse_earth_referenced_distance(data):
    try:
        parts = data.split(',')
        result = {
            'east_distance': float(parts[1]),
            'north_distance': float(parts[2]),
            'upward_distance': float(parts[3]),
            'range_to_bottom': float(parts[4]),
            'time_since_last_good_velocity': float(parts[5])
        }
        return result
    except (ValueError, IndexError) as e:
        print(f"Error parsing earth referenced distance: {e}")
        return None

def parse_system_health_monitor(data):
    try:
        parts = data.split(',')
        result = {
            'status_leak_sensor_a': parts[1],
            'status_leak_sensor_b': parts[2],
            'leak_sensor_a_raw': parts[3],
            'leak_sensor_b_raw': parts[4],
            'transmit_voltage': float(parts[5].lstrip('*')),
            'transmit_current': float(parts[6].lstrip('*')),
            'transducer_impedance': float(parts[7].lstrip('*'))
        }
        return result
    except (ValueError, IndexError) as e:
        print(f"Error parsing system health monitor: {e}")
        return None

def parse_pd6_message(data):
    lines = data.strip().split('\n')
    parsed_data = {}

    for line in lines:
        line = line.lstrip(':')  # Remove the colon prefix
        try:
            if line.startswith('SA'):
                parsed_data['system_attitude'] = parse_system_attitude(line)
            elif line.startswith('TS'):
                parsed_data['timing_scaling'] = parse_timing_scaling(line)
            elif line.startswith('BE'):
                parsed_data['earth_referenced_velocity'] = parse_earth_referenced_velocity(line)
            elif line.startswith('BD'):
                parsed_data['earth_referenced_distance'] = parse_earth_referenced_distance(line)
            elif line.startswith('HM'):
                parsed_data['system_health_monitor'] = parse_system_health_monitor(line)
        except Exception as e:
            print(f"Error parsing line: {line}, error: {e}")

    return parsed_data

def print_parsed_data(parsed_data):
    pprint(parsed_data)

class DVLNode(Node):

    def __init__(self):
        super().__init__('dvl_node')
        self.get_logger().info(f"DVL Node has been started!")
        self.publisher_ = self.create_publisher(Twist, '/dvl/velocity_data', 10)
        self.HOST = '192.168.168.102'  # DVL IP address
        self.PORT = 1037  # PD6 port
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.HOST, self.PORT))
        self.timer = self.create_timer(0.02, self.timer_callback)  # 50 Hz

    def timer_callback(self):
        try:
            data = self.socket.recv(4096).decode('ascii')  # Adjust buffer size as needed
            if data:
                parsed_data = parse_pd6_message(data)
                # print_parsed_data(parsed_data)
                if parsed_data and 'earth_referenced_velocity' in parsed_data:
                    velocity_data = parsed_data['earth_referenced_velocity']
                    if velocity_data and velocity_data['status'] == 'A':  # Only publish if data is good
                        twist = Twist()
                        twist.linear.x = velocity_data['east_velocity']
                        twist.linear.y = velocity_data['north_velocity']
                        twist.linear.z = velocity_data['upward_velocity']
                        self.publisher_.publish(twist)
                        self.get_logger().info(f"Published velocity: {twist}")
        except Exception as e:
            self.get_logger().error(f"Error in timer_callback: {e}")

def main(args=None):
    rclpy.init(args=args)
    dvl_node = DVLNode()
    rclpy.spin(dvl_node)

    dvl_node.socket.close()
    dvl_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
