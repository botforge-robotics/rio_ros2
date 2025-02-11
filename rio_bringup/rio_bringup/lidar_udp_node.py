#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import socket
import json
import math
from datetime import datetime


class LidarUDPNode(Node):
    def __init__(self):
        super().__init__('lidar_udp_node')

        # Declare parameters
        self.declare_parameter('udp_port', 9999)
        self.declare_parameter('frame_id', 'lidar')
        self.declare_parameter('angle_min', -math.pi)  # -180 degrees
        self.declare_parameter('angle_max', math.pi)   # 180 degrees
        self.declare_parameter('range_min', 0.15)
        self.declare_parameter('range_max', 6.0)

        # Get parameters
        self.udp_port = self.get_parameter('udp_port').value
        self.frame_id = self.get_parameter('frame_id').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value

        # Create UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('', self.udp_port))
        self.sock.setblocking(False)

        # Create publisher
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)

        # Create timer for reading UDP data
        # 100Hz for faster processing
        self.create_timer(0.01, self.timer_callback)

        # Buffer for accumulating UDP data
        self.data_buffer = ""

        self.get_logger().info(
            f'Listening for LIDAR data on UDP port {self.udp_port}')

    def process_json_data(self, json_data):
        try:
            # Create LaserScan message
            scan_msg = LaserScan()
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.header.frame_id = self.frame_id

            # Get scan frequency and distances
            # Default to 10Hz if not provided
            scan_frequency = float(json_data.get('scan_frequency', 10.0))
            distances = json_data.get('distances', [])
            # Reverse the distances array to change direction from 360-0
            distances = distances[::-1]  # Reverse the array
            num_readings = len(distances)

            if num_readings == 0:
                return

            scan_msg.angle_min = self.angle_min
            scan_msg.angle_max = self.angle_max
            scan_msg.angle_increment = (
                self.angle_max - self.angle_min) / num_readings
            scan_msg.time_increment = 1.0 / (scan_frequency * num_readings)
            scan_msg.scan_time = 1.0 / scan_frequency
            scan_msg.range_min = self.range_min
            scan_msg.range_max = self.range_max

            # Convert distances to ranges array with None check
            scan_msg.ranges = []
            for d in distances:
                try:
                    if d is not None:
                        val = float(d)
                        if self.range_min <= val <= self.range_max:
                            scan_msg.ranges.append(val)
                        else:
                            scan_msg.ranges.append(float('inf'))
                    else:
                        scan_msg.ranges.append(float('inf'))
                except (ValueError, TypeError):
                    scan_msg.ranges.append(float('inf'))

            # Publish the scan message
            self.scan_pub.publish(scan_msg)

        except Exception as e:
            self.get_logger().error(f'Error processing JSON data: {e}')

    def find_json_boundaries(self, data):
        """Find the start and end of a complete JSON object in the data."""
        start = data.find('{')
        if start == -1:
            return -1, -1

        # Count braces to handle nested JSON objects
        count = 0
        for i in range(start, len(data)):
            if data[i] == '{':
                count += 1
            elif data[i] == '}':
                count -= 1
                if count == 0:
                    return start, i + 1
        return start, -1

    def timer_callback(self):
        try:
            # Try to receive UDP data
            data, addr = self.sock.recvfrom(2048)
            self.data_buffer += data.decode()

            # Try to find and process complete JSON messages
            while True:
                start, end = self.find_json_boundaries(self.data_buffer)

                if start == -1 or end == -1:
                    # No complete JSON message found
                    break

                # Extract the JSON message
                json_str = self.data_buffer[start:end]
                try:
                    json_data = json.loads(json_str)
                    self.process_json_data(json_data)
                except json.JSONDecodeError as e:
                    self.get_logger().warning(
                        f'Invalid JSON data received: {e}')

                # Remove processed data from buffer
                self.data_buffer = self.data_buffer[end:]

            # Prevent buffer from growing too large
            if len(self.data_buffer) > 16384:  # 16KB limit
                self.get_logger().warning('Buffer overflow, clearing buffer')
                self.data_buffer = ""

        except BlockingIOError:
            # No data available, this is normal
            pass
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = LidarUDPNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
