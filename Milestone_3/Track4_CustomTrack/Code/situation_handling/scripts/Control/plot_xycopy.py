import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file into a DataFrame
df = pd.read_csv('/home/egyptianego17/Desktop/EVER/EVER-V-Competition/Milestone_2/Part1: Gaussian Noise & Controller/Open Loop Controller/infinity_shape/odom_data_infinity_shape.csv')

# Extracting 'X' and 'Y' columns as numpy arrays
x = df['X'].to_numpy()
y = df['Y'].to_numpy()

# Plotting
plt.plot(x, y)
plt.xlabel('X-axis label')
plt.ylabel('Y-axis label')
plt.title('X vs Y')

# Ensure the aspect ratio is equal (1:1) for both axes
plt.gca().set_aspect('equal', adjustable='box')

plt.grid(True)
plt.show()
