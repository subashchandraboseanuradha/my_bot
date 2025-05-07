#!/usr/bin/env python3

"""
   diff_tf.py - follows the output of a wheel encoder and
   creates tf and odometry messages.
   some code borrowed from the arbotix diff_controller script
   A good reference: http://rossum.sourceforge.net/papers/DiffSteer/
   
   Modified for ROS2 compatibility.
   
   Originally from http://wiki.ros.org/differential_drive
   
    Copyright (C) 2012 Jon Stephan. 
     
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

import rclpy
from rclpy.node import Node
from math import sin, cos, pi

from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
from std_msgs.msg import Int16

#############################################################################
class DiffTf(Node):
#############################################################################

    #############################################################################
    def __init__(self):
    #############################################################################
        super().__init__('diff_tf')
        self.nodename = self.get_name()
        self.get_logger().info("-I- %s started" % self.nodename)
        
        #### parameters #######
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('ticks_meter', 50.0)
        self.declare_parameter('base_width', 0.245)
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('encoder_min', -32768)
        self.declare_parameter('encoder_max', 32768)
        
        self.rate = self.get_parameter('rate').value  # the rate at which to publish the transform
        self.ticks_meter = float(self.get_parameter('ticks_meter').value)  # The number of wheel encoder ticks per meter of travel
        self.base_width = float(self.get_parameter('base_width').value) # The wheel base width in meters
        
        self.base_frame_id = self.get_parameter('base_frame_id').value # the name of the base frame of the robot
        self.odom_frame_id = self.get_parameter('odom_frame_id').value # the name of the odometry reference frame
        
        self.encoder_min = self.get_parameter('encoder_min').value
        self.encoder_max = self.get_parameter('encoder_max').value
        
        # Calculate wheel wrap parameters
        self.encoder_low_wrap = (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min
        self.encoder_high_wrap = (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min
 
        # internal data
        self.enc_left = None        # wheel encoder readings
        self.enc_right = None
        self.left = 0.0               # actual values coming back from robot
        self.right = 0.0
        self.lmult = 0
        self.rmult = 0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0.0                  # position in xy plane 
        self.y = 0.0
        self.th = 0.0
        self.dx = 0.0                 # speeds in x/rotation
        self.dr = 0.0
        self.then = self.get_clock().now()
        
        # subscriptions
        self.lwheel_sub = self.create_subscription(Int16, "lwheel", self.lwheelCallback, 10)
        self.rwheel_sub = self.create_subscription(Int16, "rwheel", self.rwheelCallback, 10)
        
        # publishers
        self.odomPub = self.create_publisher(Odometry, "odom", 10)
        
        # TF broadcaster
        self.br = tf2_ros.TransformBroadcaster(self)
        
        # Timer for update
        self.timer = self.create_timer(1.0/self.rate, self.update)
        
    #############################################################################
    def update(self):
    #############################################################################
        now = self.get_clock().now()
        elapsed = now - self.then
        self.then = now
        elapsed = elapsed.nanoseconds / 1e9  # Convert to seconds
        
        # calculate odometry
        if self.enc_left is None:
            d_left = 0.0
            d_right = 0.0
        else:
            d_left = float(self.left - self.enc_left) / self.ticks_meter
            d_right = float(self.right - self.enc_right) / self.ticks_meter
        self.enc_left = self.left
        self.enc_right = self.right
       
        # distance traveled is the average of the two wheels 
        d = float(d_left + d_right) / 2.0
        # this approximation works (in radians) for small angles
        th = float(d_right - d_left) / self.base_width
        # calculate velocities
        self.dx = float(d / elapsed if elapsed > 0 else 0.0)
        self.dr = float(th / elapsed if elapsed > 0 else 0.0)
       
         
        if (d != 0.0):
            # calculate distance traveled in x and y
            x = float(cos(th) * d)
            y = float(-sin(th) * d)
            # calculate the final position of the robot
            self.x = float(self.x + (cos(self.th) * x - sin(self.th) * y))
            self.y = float(self.y + (sin(self.th) * x + cos(self.th) * y))
        if (th != 0.0):
            self.th = float(self.th + th)
            
        # publish the odom information
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = float(sin(self.th / 2.0))
        quaternion.w = float(cos(self.th / 2.0))
        
        # TF transform
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.base_frame_id
        t.transform.translation.x = float(self.x)
        t.transform.translation.y = float(self.y)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = float(quaternion.x)
        t.transform.rotation.y = float(quaternion.y)
        t.transform.rotation.z = float(quaternion.z)
        t.transform.rotation.w = float(quaternion.w)
        
        # Send the transform
        self.br.sendTransform(t)
        
        # Odometry message
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.pose.pose.position.x = float(self.x)
        odom.pose.pose.position.y = float(self.y)
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quaternion
        odom.child_frame_id = self.base_frame_id
        odom.twist.twist.linear.x = float(self.dx)
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = float(self.dr)
        self.odomPub.publish(odom)
            
    #############################################################################
    def lwheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        if (enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap):
            self.lmult = self.lmult + 1
            
        if (enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap):
            self.lmult = self.lmult - 1
            
        self.left = float(enc + self.lmult * (self.encoder_max - self.encoder_min)) 
        self.prev_lencoder = enc
        
    #############################################################################
    def rwheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        if(enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap):
            self.rmult = self.rmult + 1
        
        if(enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap):
            self.rmult = self.rmult - 1
            
        self.right = float(enc + self.rmult * (self.encoder_max - self.encoder_min))
        self.prev_rencoder = enc

#############################################################################
#############################################################################
def main(args=None):
    rclpy.init(args=args)
    try:
        diff_tf = DiffTf()
        rclpy.spin(diff_tf)
    except KeyboardInterrupt:
        pass
    finally:
        diff_tf.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 