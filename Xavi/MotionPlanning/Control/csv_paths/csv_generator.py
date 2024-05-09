import csv
import math

# Function to generate coordinates of points on a circle
def generate_circle_coordinates(radius, num_points):
    coordinates = []
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        coordinates.append((x, y ))
    return coordinates

# Parameters
radius = 6  # Radius of the circle in meters
num_points = 100  # Number of points on the circle

# Generate coordinates
circle_coordinates = generate_circle_coordinates(radius, num_points)

# Save coordinates to CSV file
with open('csv_paths/circle_coordinates.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['x', 'y'])
    for coord in circle_coordinates:
        writer.writerow([coord[0], coord[1]])

print("Circle coordinates saved to circle_coordinates.csv")
