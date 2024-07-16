import pandas as pd
import matplotlib.pyplot as plt

def plot_data(original_file, noisy_file, filtered_file):
    # Read the CSV files
    original_data = pd.read_csv(original_file)
    noisy_data = pd.read_csv(noisy_file)
    filtered_data = pd.read_csv(filtered_file)
    
    # Plot X data separately
    plt.figure(figsize=(10, 5))
    plt.plot(original_data['x'], label='Actual X', color='blue')
    plt.title('Actual X data')
    plt.xlabel('Index')
    plt.ylabel('X')
    plt.legend()
    plt.show()

    plt.figure(figsize=(10, 5))
    plt.plot(noisy_data['x'], label='Noisy X', color='orange')
    plt.title('Noisy X data')
    plt.xlabel('Index')
    plt.ylabel('X')
    plt.legend()
    plt.show()

    plt.figure(figsize=(10, 5))
    plt.plot(filtered_data['x'], label='Ground truth X', color='green')
    plt.title('Ground truth X data')
    plt.xlabel('Index')
    plt.ylabel('X')
    plt.legend()
    plt.show()

    # Plot Y data separately
    plt.figure(figsize=(10, 5))
    plt.plot(original_data['y'], label='Actual Y', color='blue')
    plt.title('Actual Y data')
    plt.xlabel('Index')
    plt.ylabel('Y')
    plt.legend()
    plt.show()

    plt.figure(figsize=(10, 5))
    plt.plot(noisy_data['y'], label='Noisy Y', color='orange')
    plt.title('Noisy Y data')
    plt.xlabel('Index')
    plt.ylabel('Y')
    plt.legend()
    plt.show()

    plt.figure(figsize=(10, 5))
    plt.plot(filtered_data['y'], label='Ground truth Y', color='green')
    plt.title('Ground truth Y data')
    plt.xlabel('Index')
    plt.ylabel('Y')
    plt.legend()
    plt.show()

    # Plot Yaw data separately
    plt.figure(figsize=(10, 5))
    plt.plot(original_data['yaw'], label='Actual Yaw', color='blue')
    plt.title('Actual Yaw data')
    plt.xlabel('Index')
    plt.ylabel('Yaw')
    plt.legend()
    plt.show()

    plt.figure(figsize=(10, 5))
    plt.plot(noisy_data['yaw'], label='Noisy Yaw', color='orange')
    plt.title('Noisy Yaw data')
    plt.xlabel('Index')
    plt.ylabel('Yaw')
    plt.legend()
    plt.show()

    plt.figure(figsize=(10, 5))
    plt.plot(filtered_data['yaw'], label='Ground truth Yaw', color='green')
    plt.title('Ground truth Yaw data')
    plt.xlabel('Index')
    plt.ylabel('Yaw')
    plt.legend()
    plt.show()

def main():
    # Define the file paths for original, noisy, and filtered data
    original_file = 'before_anything.csv'
    noisy_file = 'after_noise_adding.csv'
    filtered_file = 'after_filtering.csv'
    
    # Plot the data
    plot_data(original_file, noisy_file, filtered_file)

if __name__ == '__main__':
    main()
