import csv

# Function to generate coordinates of points on a straight line
def generate_straight_line_coordinates(length, num_points):
    coordinates = []
    interval = length / (num_points - 1)
    for i in range(num_points):
        x = i * interval
        y = 0  # Assuming the line is along the x-axis
        coordinates.append((x, y))
    return coordinates

# Parameters
length = 75  # Length of the straight line in meters
num_points = 100  # Number of points on the line

# Generate coordinates
straight_line_coordinates = generate_straight_line_coordinates(length, num_points)

# Save coordinates to CSV file
with open('csv_paths/straight_line_coordinates.csv', 'w', newline='') as csvfile:
    writer = csv.writer(csvfile)
    writer.writerow(['x', 'y'])
    for coord in straight_line_coordinates:
        writer.writerow([coord[0], coord[1]])

print("Straight line coordinates saved to straight_line_coordinates.csv")
