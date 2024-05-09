import pandas as pd
import numpy as np
import matplotlib.pyplot as plt




def calculate_infinity_shape(data, a_minor, a_major):
    # Define the range of theta values for the infinity shape
    theta = np.linspace(0, 2*np.pi, len(data))

    # Calculate x and y coordinates of the infinity shape
    x_infinity = (a_minor * np.cos(theta)) / (1 + np.sin(theta)**2)
    y_infinity = (a_major * np.cos(theta) * np.sin(theta)) / (1 + np.sin(theta)**2)
    
    return x_infinity, y_infinity

# Load the CSV file
data = pd.read_csv("odom_data.csv")

# Remove time and z columns
data = data.drop(columns=['Timestamp', 'Z'])

# Define the radii parameters for the original and modified infinity shapes
a_minor_original = 1
a_major_original = 1

a_minor_modified = 1  # Change this value to modify the minor radius of the modified shape
a_major_modified = 12  # Change this value to modify the major radius of the modified shape

# Calculate x and y coordinates of the original and modified infinity shapes
x_infinity_original, y_infinity_original = calculate_infinity_shape(data, a_minor_original, a_major_original)
x_infinity_modified, y_infinity_modified = calculate_infinity_shape(data, a_minor_modified, a_major_modified)

# Define the coordinates for the new shape
x_new_shape = np.linspace(-3, 3, len(data))
y_new_shape = np.sin(x_new_shape)  # Example: sine wave

# Save the coordinates of the new shape to a DataFrame
new_shape_data = pd.DataFrame({'X': x_new_shape, 'Y': y_new_shape})

# Save the DataFrame to a CSV file
new_shape_data.to_csv('new_shape_data.csv', index=False)

# Plot both the original and modified infinity shapes
plt.figure(figsize=(8, 6))
plt.plot(x_infinity_original, y_infinity_original, label='Original Infinity Shape')
plt.plot(x_infinity_modified, y_infinity_modified, label='Modified Infinity Shape')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Comparison of Infinity Shapes')
plt.legend()
plt.grid(True)
plt.show()
