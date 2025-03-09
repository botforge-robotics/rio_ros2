#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point, Quaternion, Vector3
from sensor_msgs.msg import Imu
from rclpy.duration import Duration
import math

class ImuFrameNode(Node):
    def __init__(self):
        super().__init__('mobile_frame')
        self.marker_pub = self.create_publisher(Marker, 'mobile_frame', 10)
        
        # Subscribe to IMU data
        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        self.get_logger().info('Mobile frame node initialized, waiting for IMU data...')
        
        # Initialize orientation as identity quaternion
        self.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        
    def imu_callback(self, msg):
        # Update orientation from IMU data
        self.orientation = msg.orientation
        
        # Create and publish the mobile phone visualization
        self.publish_mobile_visualization()
        
    def publish_mobile_visualization(self):
        # Create mobile phone body
        phone_body = self.create_phone_body()
        
        # Publish marker
        self.marker_pub.publish(phone_body)
        
    def create_phone_body(self):
        marker = Marker()
        
        # Set the frame ID and timestamp
        marker.header.frame_id = 'base_footprint'
        marker.header.stamp = self.get_clock().now().to_msg()
        
        # Set namespace and id
        marker.ns = 'mobile_phone'
        marker.id = 0
        
        # Set marker type to CUBE for phone body
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        
        # Set the pose
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation = self.orientation
        
        # Set scale for a phone-like shape (width, height, thickness)
        marker.scale.x = 0.08  # width
        marker.scale.y = 0.16  # height
        marker.scale.z = 0.01  # thickness
        
        # Set color to bright cyan
        marker.color.r = 0.0
        marker.color.g = 0.8
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        return marker

def main(args=None):
    rclpy.init(args=args)
    node = ImuFrameNode()
    rclpy.spin(node)
    return 0

if __name__ == '__main__':
    main()
