#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time
#ros2 topic pub /robot_command std_msgs/String "data: 'Move Forward'"

class SerialWriterPublisher(Node):
    def __init__(self):
        super().__init__('serial_writer_publishe')
            # Open serial port
        self.serial_port = serial.Serial('/dev/ttyUSB1', 115200, timeout=1)

        self.subscription = self.create_subscription(
            String,
            '/robot_command',                 # Topic name
            self.listener_callback,           # Callback function
            10                                # QoS queue size
        )






        # Set up publisher to ROS topic
        self.publisher_ = self.create_publisher(String, 'send_vel_to_arduino', 10)

        
        #time.sleep(0.5)  # Give time for Arduino to reset
                                                                                                                                                      
        # Timer to send and publish message every 2 seconds
        # self.timer = self.create_timer(0.5, self.send_and_publish)

    # def send_and_publish(self):
    #     message = "10 10\n"
    #     self.serial_port.write(message.encode('utf-8'))

    #     # Publish to ROS topic
    #     msg = String()
    #     msg.data = message.strip()
    #     self.publisher_.publish(msg)

       # self.get_logger().info(f'Sent to Arduino and published: {msg.data}')


    def listener_callback(self, msg):
        #command = msg.data.strip() + '\n'   # Make sure to end with newline
        command = msg.data.strip()
        result = "" #80 60   80 is right 60 is left
        if command == "Turn Left because more distance":
              result = "50 -50"
        elif command == "Turn Right because more distance":
             result = "-50 50"
        elif command == "Just stop":
             result = "0 0"    
        elif command == "Turn Right to avoid obstacle on the front-left":
             result = "-50 50"
        elif command == "Turn Left to avoid obstacle on the front-right":
             result = "50 -50"  


        elif command == "Go Forwards turning slightly right to avoid obstacle on the left":
             result = "45 50"              
        elif command == "Go Forwards turning slightly left to avoid obstacle on the right":
             result = "50 45" 
        else:
             result = "50 50"    #self.get_logger().info(f"Received unknown command: {command}")
        
#colcon build --symlink-install
 #ros2 launch sllidar_ros2 view_sllidar_a1_launch.py   
 #colcon build --packages-select robot_move              
        self.serial_port.write((result+'\n').encode('utf-8'))
        
        # Optionally re-publish it
        feedback_msg = String()
        feedback_msg.data = (result+'\n').strip()
        self.publisher_.publish(feedback_msg)

        self.get_logger().info(f'Sent to Arduino: {feedback_msg.data}')
def main(args=None):
    rclpy.init(args=args)
    node = SerialWriterPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()



#ros2 topic pub /robot_command std_msgs/String "data: 'go forward'"


