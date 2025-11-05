#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import math

class ArduinoOdomNode(Node):
    def __init__(self):
        super().__init__('arduino_odom_node')

        # Create odometry publisher
        self.publisher_ = self.create_publisher(Odometry, 'odom', 10)

        # TF broadcaster for odom -> base_link
        self.tf_broadcaster = TransformBroadcaster(self)

        # Serial connection to Arduino
        self.serial_port = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)  # Update if needed

        # Position and heading
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # Time for integration
        self.last_time = self.get_clock().now()

        # Timer to read serial and publish odometry at 20ms
        self.timer = self.create_timer(0.02, self.read_and_publish)

        # Wheel info
        self.wheel_diameter = 0.065  # meters
        self.wheel_base = 0.27       # distance between wheels (meters)
        self.rpm_to_mps = (math.pi * self.wheel_diameter) / 60.0  # RPM to m/s

    def read_and_publish(self):
        if self.serial_port.in_waiting > 0:
            line = self.serial_port.readline().decode('utf-8').strip()
            try:
                values = [float(v) for v in line.split(',')]
                if len(values) != 2:
                    self.get_logger().warn(f'Expected 2 floats, got: {values}')
                    return

                right_rpm, left_rpm= values
                vel_l = left_rpm * self.rpm_to_mps
                vel_r = right_rpm * self.rpm_to_mps

                v = (vel_r + vel_l) / 2.0
                omega = (vel_r - vel_l) / self.wheel_base

                now = self.get_clock().now()
                dt = (now - self.last_time).nanoseconds / 1e9
                self.last_time = now

                delta_x = v * math.cos(self.theta) * dt
                delta_y = v * math.sin(self.theta) * dt
                delta_theta = omega * dt

                self.x += delta_x
                self.y += delta_y
                self.theta += delta_theta

                # Create Odometry message
                odom_msg = Odometry()
                odom_msg.header.stamp = now.to_msg()
                odom_msg.header.frame_id = 'odom'
                odom_msg.child_frame_id = 'base_link'

                odom_msg.pose.pose.position.x = self.x
                odom_msg.pose.pose.position.y = self.y
                odom_msg.pose.pose.position.z = 0.0

                q = self.euler_to_quaternion(0, 0, self.theta)
                odom_msg.pose.pose.orientation = q

                odom_msg.twist.twist.linear.x = v
                odom_msg.twist.twist.angular.z = omega

                self.publisher_.publish(odom_msg)

                # Publish transform from odom -> base_link
                t = TransformStamped()
                t.header.stamp = now.to_msg()
                t.header.frame_id = 'odom'
                t.child_frame_id = 'base_link'
                t.transform.translation.x = self.x
                t.transform.translation.y = self.y
                t.transform.translation.z = 0.0
                t.transform.rotation = q

                self.tf_broadcaster.sendTransform(t)

            except ValueError:
                self.get_logger().warn(f'Could not parse line: {line}')

    def euler_to_quaternion(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy
        return q

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()













########################################################################################################################################################################################################
#!/usr/bin/env python
# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry
# from geometry_msgs.msg import Quaternion
# import serial
# import math

# class ArduinoOdomNode(Node):
#     def __init__(self):
#         super().__init__('arduino_odom_node')

#         # Create odometry publisher
#         self.publisher_ = self.create_publisher(Odometry, 'odom', 10)

#         # Serial connection to Arduino
#         self.serial_port = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)  # Adjust as needed

#         # Position and heading
#         self.x = 0.0
#         self.y = 0.0
#         self.theta = 0.0

#         # Time for integration
#         self.last_time = self.get_clock().now()

#         # Timer to read serial and publish odometry at  20 ms
#         self.timer = self.create_timer(0.02, self.read_and_publish)

#         # Wheel info
#         self.wheel_diameter = 0.065  # meters
#         self.wheel_base = 0.27       # distance between wheels (meters)
#         self.rpm_to_mps = (math.pi * self.wheel_diameter) / 60.0  # conversion factor

#     def read_and_publish(self):
#         if self.serial_port.in_waiting > 0:
#             line = self.serial_port.readline().decode('utf-8').strip()
#             try:
#                 # Parse 2 comma-separated float values
#                 values = [float(v) for v in line.split(',')]
#                 if len(values) != 2:
#                     self.get_logger().warn(f'Expected 2 floats, got: {values}')
#                     return

#                 left_rpm, right_rpm = values
#                 vel_l = left_rpm * self.rpm_to_mps
#                 vel_r = right_rpm * self.rpm_to_mps

#                 # Compute linear and angular velocity
#                 v = (vel_r + vel_l) / 2.0
#                 omega = (vel_r - vel_l) / self.wheel_base

#                 # Time step
#                 now = self.get_clock().now()
#                 dt = (now - self.last_time).nanoseconds / 1e9
#                 self.last_time = now

#                 # Integrate position
#                 delta_x = v * math.cos(self.theta) * dt
#                 delta_y = v * math.sin(self.theta) * dt
#                 delta_theta = omega * dt

#                 self.x += delta_x
#                 self.y += delta_y
#                 self.theta += delta_theta

#                 # Publish Odometry message
#                 odom_msg = Odometry()
#                 odom_msg.header.stamp = now.to_msg()
#                 odom_msg.header.frame_id = 'odom'
#                 odom_msg.child_frame_id = 'base_link'

#                 odom_msg.pose.pose.position.x = self.x
#                 odom_msg.pose.pose.position.y = self.y
#                 odom_msg.pose.pose.position.z = 0.0

#                 q = self.euler_to_quaternion(0, 0, self.theta)
#                 odom_msg.pose.pose.orientation = q

#                 odom_msg.twist.twist.linear.x = v
#                 odom_msg.twist.twist.angular.z = omega

#                 self.publisher_.publish(odom_msg)

#             except ValueError:
#                 self.get_logger().warn(f'Could not parse line: {line}')

#     def euler_to_quaternion(self, roll, pitch, yaw):
#         cy = math.cos(yaw * 0.5)
#         sy = math.sin(yaw * 0.5)
#         cp = math.cos(pitch * 0.5)
#         sp = math.sin(pitch * 0.5)
#         cr = math.cos(roll * 0.5)
#         sr = math.sin(roll * 0.5)

#         q = Quaternion()
#         q.w = cr * cp * cy + sr * sp * sy
#         q.x = sr * cp * cy - cr * sp * sy
#         q.y = cr * sp * cy + sr * cp * sy
#         q.z = cr * cp * sy - sr * sp * cy
#         return q

# def main(args=None):
#     rclpy.init(args=args)
#     node = ArduinoOdomNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


































