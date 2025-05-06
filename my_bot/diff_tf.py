#!/usr/bin/env python3

import math
from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import JointState
import tf2_ros
from geometry_msgs.msg import TransformStamped

class DiffTF(Node):
    def __init__(self):
        super().__init__('diff_tf')
        
        # Parameters
        self.rate = 20.0  # the rate at which to publish the transform
        self.ticks_meter = 3831.75  # The number of wheel encoder ticks per meter of travel
        self.base_width = 0.255  # The distance between the wheel centers
        self.base_frame_id = 'base_link'  # the name of the base frame of the robot
        self.odom_frame_id = 'odom'  # the name of the odometry reference frame
        
        self.encoder_min = -2147483648
        self.encoder_max = 2147483647
        self.encoder_low_wrap = (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min
        self.encoder_high_wrap = (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min
        
        # Internal variables
        self.enc_left = None        # wheel encoder readings
        self.enc_right = None
        self.left = 0               # actual values coming back from robot
        self.right = 0
        self.lmult = 0
        self.rmult = 0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0                  # position in xy plane
        self.y = 0
        self.th = 0
        self.dx = 0                 # speeds in x/rotation
        self.dr = 0
        self.then = self.get_clock().now()
        
        # Subscriptions
        self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 10)
        
        # Publishers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 50)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Timer for update loop
        self.create_timer(1.0/self.rate, self.update)
        
    def joint_states_callback(self, msg):
        # Find the index for left and right wheel joint names
        try:
            left_idx = msg.name.index('left_wheel_joint')
            right_idx = msg.name.index('right_wheel_joint')
            
            # Read the encoder values
            self.enc_left = msg.position[left_idx]
            self.enc_right = msg.position[right_idx]
            
            # Add debug logging
            self.get_logger().info(f'Received joint states - Left: {self.enc_left}, Right: {self.enc_right}')
        except ValueError:
            self.get_logger().warn(f'Joint states message did not contain expected joint names. Got: {msg.name}')
            return
            
    def update(self):
        now = self.get_clock().now()
        elapsed = now - self.then
        self.then = now
        elapsed = elapsed.nanoseconds / 1e9
        
        # Calculate odometry
        if self.enc_left is None or self.enc_right is None:
            self.get_logger().warn('No encoder data received yet')
            return
            
        # Convert encoder counts to distance in meters
        dleft = (self.enc_left - self.prev_lencoder) / self.ticks_meter
        dright = (self.enc_right - self.prev_rencoder) / self.ticks_meter
        
        self.prev_lencoder = self.enc_left
        self.prev_rencoder = self.enc_right
        
        # Distance traveled is the average of the two wheels
        d = (dleft + dright) / 2
        # Difference in distance is the angle turned
        th = (dright - dleft) / self.base_width
        
        # Calculate velocities
        self.dx = d / elapsed
        self.dr = th / elapsed
        
        # Add debug logging for odometry calculations
        self.get_logger().debug(f'Odometry update - dx: {self.dx}, dr: {self.dr}, x: {self.x}, y: {self.y}, th: {self.th}')
        
        # Update the pose of the robot
        if d != 0:
            # Calculate distance traveled in x and y
            dx = d * cos(self.th)
            dy = d * sin(self.th)
            self.x += dx
            self.y += dy
        
        if th != 0:
            self.th += th
            # Normalize angle to -pi to +pi
            self.th = math.atan2(math.sin(self.th), math.cos(self.th))
            
        # Create quaternion from yaw
        odom_quat = Quaternion()
        odom_quat.x = 0.0
        odom_quat.y = 0.0
        odom_quat.z = sin(self.th / 2)
        odom_quat.w = cos(self.th / 2)
            
        # Create and publish transform
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation = odom_quat
        self.tf_broadcaster.sendTransform(t)
            
        # Create and publish odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = odom_quat
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = self.dx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.dr
        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = DiffTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

if __name__ == '__main__':
    main()