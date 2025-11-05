#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import serial
import time

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_ = self.create_publisher(Imu, 'imu', 10)
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)
           # time.sleep(2)  # wait for serial to start
            self.get_logger().info("Serial connection established.")
        except Exception as e:
            self.get_logger().error(f"Could not open serial port: {e}")
            raise

        self.timer = self.create_timer(0.05, self.timer_callback)  # 20 Hz

    def timer_callback(self):
        try:
            line = self.serial_port.readline().decode('utf-8').strip()
            parts = [float(x) for x in line.split(',')]
            if len(parts) != 12:
                self.get_logger().warn(f"Invalid data length: {len(parts)}")
                return

            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu_link"

            # Linear acceleration
            imu_msg.linear_acceleration.x = parts[0]
            imu_msg.linear_acceleration.y = parts[1]
            imu_msg.linear_acceleration.z = parts[2]

            # Angular velocity
            imu_msg.angular_velocity.x = parts[3]
            imu_msg.angular_velocity.y = parts[4]
            imu_msg.angular_velocity.z = parts[5]

            # Orientation
            imu_msg.orientation.w = parts[6]
            imu_msg.orientation.x = parts[7]
            imu_msg.orientation.y = parts[8]
            imu_msg.orientation.z = parts[9]

            self.publisher_.publish(imu_msg)

        except Exception as e:
            self.get_logger().warn(f"Error parsing line: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImuPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

