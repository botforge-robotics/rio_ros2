#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from sensor_msgs.msg import Imu
import sys
from collections import deque
import numpy as np

class ImuViz2D(Node):
    def __init__(self, topic_name):
        super().__init__('imu_viz_2d')
        self.pub = self.create_publisher(Marker, 'imu_viz_2d', 10)
        self.sub = self.create_subscription(
            Imu,
            topic_name,
            self.callback_sphere,
            10)
        
        # Create smoothing buffers with 10 elements (same as C++ version)
        self.x_buffer = deque([0.0] * 10, maxlen=10)
        self.y_buffer = deque([0.0] * 10, maxlen=10)
        
        self.get_logger().info(f'Subscribed to IMU topic: {topic_name}')
        
    def callback_sphere(self, msg):
        # Add new values to the smoothing buffers
        self.x_buffer.append(msg.linear_acceleration.x)
        self.y_buffer.append(msg.linear_acceleration.y)
        
        # Calculate averages for smoothing
        ave_x = sum(self.x_buffer) / len(self.x_buffer)
        ave_y = sum(self.y_buffer) / len(self.y_buffer)
        
        # Create marker message
        imu_marker = Marker()
        
        # Set the frame ID and timestamp
        imu_marker.header.frame_id = 'base_footprint'
        imu_marker.header.stamp = self.get_clock().now().to_msg()
        
        # Set namespace and id
        imu_marker.ns = 'basic_shapes'
        imu_marker.id = 0
        
        # Set marker type and action
        imu_marker.type = Marker.SPHERE
        imu_marker.action = Marker.ADD
        
        # Set the pose of the marker
        imu_marker.pose.position.x = ave_x
        imu_marker.pose.position.y = ave_y
        imu_marker.pose.position.z = 0.0
        imu_marker.pose.orientation.x = 0.0
        imu_marker.pose.orientation.y = 0.0
        imu_marker.pose.orientation.z = 0.0
        imu_marker.pose.orientation.w = 1.0
        
        # Set the scale of the marker
        imu_marker.scale.x = 0.3
        imu_marker.scale.y = 0.3
        imu_marker.scale.z = 0.1
        
        # Set the color
        imu_marker.color.r = 1.0
        imu_marker.color.g = 0.0
        imu_marker.color.b = 1.0
        imu_marker.color.a = 1.0
        
        # Set lifetime (0 = forever)
        imu_marker.lifetime = rclpy.duration.Duration().to_msg()
        
        # Publish the marker
        self.pub.publish(imu_marker)

def main(args=None):
    rclpy.init(args=args)
    
    # Get the topic name from command line arguments
    if len(sys.argv) > 1:
        topic_name = sys.argv[1]
    else:
        topic_name = '/imu/data'  # Default topic if none provided
    
    node = ImuViz2D(topic_name)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
