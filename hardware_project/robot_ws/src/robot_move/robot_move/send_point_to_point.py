#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import serial
import time
import math

class CmdVelToSerial(Node):
    def __init__(self):
        super().__init__('cmdvel_to_serial')
            # Open serial port
        self.serial_port = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)

        
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)


    
        # Robot parameters
        self.wheel_base = 0.27  # distance between wheels (in meters)
        self.wheel_radius = 0.0325  # radius of wheel (in meters)


    def listener_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        L = self.wheel_base
        R = self.wheel_radius

        # Linear velocities for each wheel (m/s)
        v_left = v - (w * L / 2.0)
        v_right = v + (w * L / 2.0)

        # Convert to RPM
        left_rpm = (v_left / (2 * math.pi * R)) * 60.0
        right_rpm = (v_right / (2 * math.pi * R)) * 60.0

        line = f"{right_rpm:.3f} {left_rpm:.3f}\n"

        self.serial_port.write(line.encode('utf-8'))
        

        self.get_logger().info(f'Sent to Arduino from serial from cmd/vel: {line.strip()}')
def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToSerial()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
