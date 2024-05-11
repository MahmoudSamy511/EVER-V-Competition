import pandas as pd
import numpy as np

def make_shape_bigger(x, y, scale_factor):
    """
    Make the shape bigger by scaling its coordinates.
    
    Parameters:
        x (array-like): X coordinates of the shape.
        y (array-like): Y coordinates of the shape.
        scale_factor (float): Factor by which to scale the shape.
    
    Returns:
        tuple: Tuple containing the scaled X and Y coordinates.
    """
    scaled_x = x * scale_factor
    scaled_y = y * scale_factor
    return scaled_x, scaled_y

# Read infinity shape path data from CSV
shape_data = pd.read_csv("odom.csv")

# Extract X and Y coordinates from the CSV file
x_original = shape_data['X'].values
y_original = shape_data['Y'].values

# Scale factor to make the shape bigger
scale_factor = 1.5  # Adjust as needed

# Make the shape bigger
x_bigger, y_bigger = make_shape_bigger(x_original, y_original, scale_factor)

# Create a new DataFrame for the bigger shape
bigger_shape_data = pd.DataFrame({'X': x_bigger, 'Y': y_bigger})

# Save the new shape data to a CSV file
bigger_shape_data.to_csv('bigger_infinity_shape_path.csv', index=False)
