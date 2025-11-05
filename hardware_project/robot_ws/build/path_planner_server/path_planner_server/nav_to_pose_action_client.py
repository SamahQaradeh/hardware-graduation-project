#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
import math
import time


class NavToPoseActionClient(Node):

    def __init__(self):
        super().__init__('nav_to_pose_auto_map')

        # Action client
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Subscribe to map topic
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.subscription  # prevent unused warning

        # Publisher for visualization
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoint_markers', 10)

        self.map_received = False
        self.current_goal_index = 0
        self.waypoints = []

    def map_callback(self, msg: OccupancyGrid):
        if self.map_received:
            return

        self.map_received = True
        self.get_logger().info('Map received. Generating waypoints...')

        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        resolution = msg.info.resolution
        width = msg.info.width
        height = msg.info.height

        # Define full map boundaries
        x_min = origin_x
        x_max = origin_x + width * resolution
        y_min = origin_y
        y_max = origin_y + height * resolution
        step = 0.5  # meters between waypoints

        self.waypoints = self.generate_zigzag_waypoints(x_min, x_max, y_min, y_max, step)

        # Wait for Nav2 action server
        self.get_logger().info("Waiting for Nav2 action server...")
        self._action_client.wait_for_server()

        # Visualize waypoints
        self.visualize_waypoints()

        # Start navigation
        self.send_next_goal()

    def generate_zigzag_waypoints(self, x_min, x_max, y_min, y_max, step):
        waypoints = []
        direction = 1
        y = y_min
        while y <= y_max:
            x_range = list(self.frange(x_min, x_max, step))
            if direction == -1:
                x_range = list(reversed(x_range))
            for x in x_range:
                waypoints.append((x, y))
            y += step
            direction *= -1
        return waypoints

    def frange(self, start, stop, step):
        while start <= stop:
            yield round(start, 2)
            start += step

    def send_next_goal(self):
        if self.current_goal_index >= len(self.waypoints):
            self.get_logger().info("All waypoints completed.")
            rclpy.shutdown()
            return

        x, y = self.waypoints[self.current_goal_index]
        self.get_logger().info(f"Sending waypoint {self.current_goal_index + 1}: x={x}, y={y}")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  # facing forward

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected.')
            rclpy.shutdown()
            return

        self.get_logger().info('Goal accepted.')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Reached waypoint {self.current_goal_index + 1}")
        self.current_goal_index += 1
        time.sleep(1.0)
        self.send_next_goal()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Current pose: {feedback.current_pose.pose.position}")

    def visualize_waypoints(self):
        marker_array = MarkerArray()
        for i, (x, y) in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.05
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.7
            marker.color.b = 1.0
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = NavToPoseActionClient()
    rclpy.spin(node)