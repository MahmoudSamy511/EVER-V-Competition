import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

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
shape_data = pd.read_csv("Mahmoud/odom_data.csv")

# Extract X and Y coordinates from the CSV file
x_original = shape_data['X'].values
y_original = shape_data['Y'].values

# Scale factor to make the shape bigger
scale_factor = 3  # Adjust as needed

# Make the shape bigger
x_bigger, y_bigger = make_shape_bigger(x_original, y_original, scale_factor)

# Plot both shapes on the same figure
plt.figure(figsize=(8, 6))

# Plot original shape
plt.plot(x_original, y_original, label='Original Shape')
# Plot bigger shape
plt.plot(x_bigger, y_bigger, label='Bigger Shape')

plt.xlabel('X')
plt.ylabel('Y')
plt.title('Original and Bigger Shapes')
plt.legend()
plt.grid(True)

plt.tight_layout()
plt.show()

# Optionally, save the new shape data to a CSV file
bigger_shape_data = pd.DataFrame({'X': x_bigger, 'Y': y_bigger})
bigger_shape_data.to_csv('Mahmoud/bigger_infinity_shape_path.csv', index=False)
