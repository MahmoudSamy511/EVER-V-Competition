import numpy as np
import pandas as pd

# Parameters
X_center = 6.600
Y_center = -28.225
radius = 32
num_waypoints = 360

# Calculate the starting angle
start_angle = np.arctan2(7.325 - Y_center, 60.350 - X_center)

# Generate the waypoints
angles = np.linspace(start_angle, start_angle + 2 * np.pi, num_waypoints)
x_points = X_center + radius * np.cos(angles)
y_points = Y_center + radius * np.sin(angles)

# Create a DataFrame
waypoints = pd.DataFrame({'x': x_points, 'y': y_points})

# Save to CSV
waypoints.to_csv('cicleeee.csv', index=False)
