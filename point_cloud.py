import json
import numpy as np

with open('depth_output.json', 'r') as file:
    # Load the JSON data
    json_data = json.load(file)

# Convert the JSON data to a Python list or array
depth_img = np.array(json_data['depth_img'])
view_matrix = np.array(json_data['view_matrix'])
proj_matrix = np.array(json_data['proj_matrix'])

width, height = 256, 256

min_depth_threshold = 0.1  # Minimum depth threshold for obstacles
max_depth_threshold = 1.0  # Maximum depth threshold for obstacles

# Apply thresholding to create a binary mask
obstacle_mask = np.logical_and(depth_img >= min_depth_threshold, depth_img <= max_depth_threshold)

# Get the obstacle positions from the mask
obstacle_positions = np.where(obstacle_mask)

# Print the obstacle positions or perform further analysis
print(obstacle_positions)