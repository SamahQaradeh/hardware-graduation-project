#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile
import random
from std_msgs.msg import String

class ObstacleDetectorNode(Node):
    def __init__(self, node_name="obstacle_detector_node"):
        self._node_name = node_name
        super().__init__(self._node_name)

        # create the subscriber object
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laserscan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )
        self.command_publisher = self.create_publisher(String, '/robot_command',10)

        self.get_logger().info(self._node_name + " Ready...")

    def laserscan_callback(self, msg):
        # Define the sectors
        sectors = {
            "Right_Rear": (0, 90),
            "Right": (90, 126),
            "Front_Right": (127, 162),
            "Front": (163, 198),
            "Front_Left": (199, 224),
            "Left": (225, 270),
            "Left_Rear": (270, 358)
        }

        # Initialize the minimum distances for each sector
        min_distances = {key: float('inf') for key in sectors.keys()}

        # Find the minimum distance in each sector
        for sector, (start_idx, end_idx) in sectors.items():
            # Ensure the index range is within bounds and not empty
            if start_idx < len(msg.ranges) and end_idx < len(msg.ranges):
                sector_ranges = msg.ranges[start_idx:end_idx + 1]
                if sector_ranges:
                    min_distances[sector] = min(sector_ranges)

        # Log the minimum distances
        for sector, min_distance in min_distances.items():
            self.get_logger().info(f'{sector}: {min_distance:.2f} meters')

        # Define the threshold for obstacle detection
        obstacle_threshold = 0.30  # meters

        # Determine detected obstacles
        detections = {sector: min_distance < obstacle_threshold for sector, min_distance in min_distances.items()}

        # Determine suggested action based on detection and ordered Cinditions by priority
        # Priority 1: Front detection, Priority 2: Side detections, Priority 3: Rear detections
        # Priority 1

        # if detections["Front"] :
        #     # Choose the direction with more clearance
        #     if min_distances["Front_Left"] > min_distances["Front_Right"]:
        #         action = "Turn Left because more distance"
        #     else:
        #         action = "Turn Right because more distance"
        if detections["Front_Left"] and not detections["Front_Right"]:
            action = "Turn Right to avoid obstacle on the front-left"
        elif detections["Front_Right"] and not detections["Front_Left"]:
            action = "Turn Left to avoid obstacle on the front-right"
        # Priority 2
        elif detections["Front_Right"] and detections["Front_Left"]:
           #Choose the direction with more clearance
            if min_distances["Left"] > min_distances["Right"]:
                action = "Turn Left because more distance"
            else:
                action = "Turn Right because more distance"
        elif detections["Front"]:
             if min_distances["Left"] > min_distances["Right"]:
                action = "Turn Left because more distance"
             else:
                action = "Turn Right because more distance"

        
        elif detections["Left"]:
            action = "Go Forwards turning slightly right to avoid obstacle on the left"
        elif detections["Right"]:
            action = "Go Forwards turning slightly left to avoid obstacle on the right"
        # Priority 3
        elif detections["Right_Rear"]:
            action = "Go Forwards, BUT DONT reverse Right"
        elif detections["Left_Rear"]:
            action = "Go Forwards, BUT DONT reverse left"
        else:
            action = "Go Forwards"

        # Log the suggested action
        self.get_logger().info(f'Suggested action: {action}')
        self.command_publisher.publish(String(data=action))
        

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


















