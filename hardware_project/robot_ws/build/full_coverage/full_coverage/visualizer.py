#!/usr/bin/env python3
import os
import yaml
import cv2
import numpy as np

# === CONFIGURATION ===
map_yaml_path = '/home/samah/robot_ws/src/map_server/config/room_area.yaml'
robot_size = 0.4  # meters
overlap = 0.1     # meters
resolution_threshold = 250

# === LOAD MAP YAML ===
with open(map_yaml_path, 'r') as f:
    map_data = yaml.safe_load(f)

map_img_path = os.path.join(os.path.dirname(map_yaml_path), map_data['image'])
resolution = map_data['resolution']
origin_x, origin_y = map_data['origin'][0], map_data['origin'][1]

# === LOAD MAP IMAGE ===
map_img = cv2.imread(map_img_path, cv2.IMREAD_GRAYSCALE)
color_map = cv2.cvtColor(map_img, cv2.COLOR_GRAY2BGR)  # for visualization

height, width = map_img.shape
step_px = int((robot_size - overlap) / resolution)

# === BINARIZE MAP (white = free, black/gray = obstacle/unknown) ===
# Map is free where pixel > threshold
_, bin_map = cv2.threshold(map_img, resolution_threshold, 255, cv2.THRESH_BINARY)
bin_map = bin_map // 255  # 1 = free, 0 = obstacle or unknown

# Save a copy of the original free space for masking later
free_mask = bin_map.copy()
  # Node(
        #     package='full_coverage',
        #     executable='coverage_planner',
        #     name='coverage_planner',
        #     output='screen'
        # ),
# === DILATE OBSTACLES FOR 30cm BUFFER, INSIDE KNOWN AREA ONLY ===
dilation_distance_m = 0.35
dilation_radius_px = int(dilation_distance_m / resolution)
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * dilation_radius_px + 1, 2 * dilation_radius_px + 1))

# Invert bin_map: 1 - bin_map → 1 for obstacles, 0 for free
obstacles = 1 - bin_map
inflated_obstacles = cv2.dilate(obstacles.astype(np.uint8), kernel)

# Constrain inflation to original free space + obstacles only (not outside map)
# I.e., do not let inflation enter pixels that were not part of known space
inflated_obstacles = np.where(free_mask + obstacles == 1, inflated_obstacles, 1 - free_mask)

# === VISUALIZE INFLATED OBSTACLES IN RED ===
overlay = color_map.copy()
overlay[inflated_obstacles == 1] = [50, 50, 50]  # 
visual_map = cv2.addWeighted(color_map, 0.6, overlay, 0.4, 0)

# === UPDATE bin_map FOR PATH PLANNING (avoid inflated areas) ===
bin_map = 1 - inflated_obstacles  # 1 = free, 0 = inflated/obstacle

# === GENERATE SPARSE ZIGZAG WAYPOINTS ===
waypoints = []
direction = 1
for row in range(0, height, step_px):
    cols = range(0, width) if direction == 1 else range(width - 1, -1, -1)
    valid_points = [col for col in cols if bin_map[row, col] == 1]

    if len(valid_points) > 0:
        start = valid_points[0]
        end = valid_points[-1]
        waypoints.append((start, row))
        waypoints.append((end, row))

    direction *= -1

print(f"Generated {len(waypoints)} optimized waypoints")

# === DRAW WAYPOINTS ===
for i, (x, y) in enumerate(waypoints):
    cv2.circle(visual_map, (x, y), 2, (0, 255, 255), -1)  # Yellow
    if i > 0:
        cv2.line(visual_map, waypoints[i - 1], (x, y), (0, 255, 0), 1)  # Green path



origin_x, origin_y = map_data['origin'][0], map_data['origin'][1]
resolution = map_data['resolution']
height, width = map_img.shape  # image shape

# Image coordinates of (0,0) in map frame
px = int(-origin_x / resolution)
py = int(height + (origin_y / resolution))  # origin_y is usually negative
cv2.circle(visual_map, (px, py), 2, (255, 0, 0), -1)  # Blue dot = map frame origin


if waypoints:
    last_wp = waypoints[-1]
    cv2.line(visual_map, (px, py), last_wp, (0,255,0), 1)  # Red line from origin to last waypoint


# === DISPLAY MAP ===
cv2.imshow("Inflation Inside Map Only + Waypoints", cv2.resize(visual_map, (800, 800)))
cv2.waitKey(0)
cv2.destroyAllWindows()
