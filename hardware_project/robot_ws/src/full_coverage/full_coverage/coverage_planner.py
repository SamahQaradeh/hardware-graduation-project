#!/usr/bin/env python3
import os
import yaml
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped

class CoverageNavigator(Node):

    
    def __init__(self):
        super().__init__('coverage_navigator')
        self.origin_point = None  # Store origin for later
        self.appended_origin = False  # Flag to make sure it's only added once

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.waypoints = []
        self.current_waypoint = 0
        self.retry_count = 0  # ← track how many retries we’ve made
        self.MAX_RETRIES = 3  # You can set this higher or lower
        # Load map and generate path
        self.load_map_and_generate_path()
        
        # Start navigation if waypoints exist
        if self.waypoints:
            self.send_next_waypoint()

    def load_map_and_generate_path(self):
        """Load map and generate sparse zigzag path"""
        try:
            map_yaml_path = os.path.expanduser('/home/samah/robot_ws/src/map_server/config/room_area.yaml')
            robot_size = 0.4  # meters
            overlap = 0.1     # meters

            with open(map_yaml_path, 'r') as f:
                map_data = yaml.safe_load(f)

            map_img_path = os.path.join(os.path.dirname(map_yaml_path), map_data['image'])
            map_img = cv2.imread(map_img_path, cv2.IMREAD_GRAYSCALE)

            if map_img is None:
                self.get_logger().error("Failed to load map image!")
                return

            height, width = map_img.shape
            resolution = map_data['resolution']
            origin_x = map_data['origin'][0]
            origin_y = map_data['origin'][1]
            step_px = int((robot_size - overlap) / resolution)

            _, bin_map = cv2.threshold(map_img, 250, 255, cv2.THRESH_BINARY)
            bin_map = bin_map // 255  # to 0/1
              
             # === DILATE OBSTACLES FOR 30cm BUFFER ===
            dilation_distance_m = 0.35 # 30 cm
            dilation_radius_px = int(dilation_distance_m / resolution)
            kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * dilation_radius_px + 1, 2 * dilation_radius_px + 1))
            dilated_obstacles = cv2.dilate(1 - bin_map, kernel)  # Dilate obstacles (invert first)
            bin_map = 1 - dilated_obstacles  # Invert back: 1 = free, 0 = obstacle

              


            direction = 1
            for row in range(0, height, step_px):
                cols = range(0, width) if direction == 1 else range(width - 1, -1, -1)
                valid_cols = [col for col in cols if bin_map[row, col] == 1]

                if valid_cols:
                    start_col = valid_cols[0]
                    end_col = valid_cols[-1]

                    for col in [start_col, end_col]:
                        world_x = origin_x + (col * resolution)+0.2
                        world_y = origin_y + ((height - row) * resolution)+0.2
                        self.waypoints.append((world_x, world_y))

                direction *= -1

            self.origin_point = (origin_x, origin_y)
            self.get_logger().info(f"Generated {len(self.waypoints)} optimized waypoints")

        except Exception as e:
            self.get_logger().error(f"Error during map loading: {str(e)}")

    def send_next_waypoint(self):
        """Send next waypoint to navigation stack"""
          

         # Append origin once at the end
        if not self.appended_origin and self.origin_point:
          self.waypoints.append(self.origin_point)
          self.appended_origin = True
          self.get_logger().info("Appended origin as final waypoint.")
 


        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info("All waypoints completed!")
            return

        x, y = self.waypoints[self.current_waypoint]
        self.get_logger().info(f"Navigating to waypoint {self.current_waypoint+1}/{len(self.waypoints)}: x={x:.2f}, y={y:.2f}")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = PoseStamped()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.w = 1.0  # No rotation for simplicity

        self.nav_client.wait_for_server()
        self.send_goal_future = self.nav_client.send_goal_async(goal_msg)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Waypoint rejected! Moving to next...")
            self.current_waypoint += 1
            self.send_next_waypoint()
            return

        self.get_logger().info("Waypoint accepted")
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.navigation_result_callback)

    def navigation_result_callback(self, future):
      goal_result = future.result()
      status = goal_result.status
      result = goal_result.result

      if status == 4:  # STATUS_SUCCEEDED
        self.get_logger().info(f"Reached waypoint {self.current_waypoint+1} successfully.")
        self.retry_count = 0  # Reset retry counter
        self.current_waypoint += 1
        self.send_next_waypoint()

      else:
        self.get_logger().warn(f"Failed to reach waypoint {self.current_waypoint+1}. Status: {status}")
        
        if self.retry_count < self.MAX_RETRIES:
          self.retry_count += 1
          self.get_logger().warn(f"Retrying waypoint {self.current_waypoint+1} (Attempt {self.retry_count})...")
          self.send_next_waypoint()
        else:
          self.get_logger().error(f"Max retries reached for waypoint {self.current_waypoint+1}. Skipping...")
          self.retry_count = 0
          self.current_waypoint += 1
          self.send_next_waypoint()
   


    # def navigation_result_callback(self, future):
    #     result = future.result().result
    #     self.get_logger().info(f"Navigation result: {result}")
    #     self.current_waypoint += 1
    #     self.send_next_waypoint()





def main(args=None):
    rclpy.init(args=args)
    navigator = CoverageNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()













