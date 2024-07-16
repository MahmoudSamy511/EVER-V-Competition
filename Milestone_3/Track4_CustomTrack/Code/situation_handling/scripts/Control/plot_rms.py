import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

def plot_data(original_file, noisy_file, filtered_file):
    # Read the CSV files
    original_data = pd.read_csv(original_file)
    noisy_data = pd.read_csv(noisy_file)
    filtered_data = pd.read_csv(filtered_file)
    
    # Calculate the error for each data type
    error_x = original_data['x'] - filtered_data['x']
    error_y = original_data['y'] - filtered_data['y']
    error_yaw = original_data['yaw'] - filtered_data['yaw']
    
    # Create a figure with subplots
    fig, axs = plt.subplots(3, 2, figsize=(15, 15))

    # Colors for plotting each type of data
    colors = {'original': 'blue', 'noisy': 'orange', 'filtered': 'green', 'error': 'red'}

    # Plot X data
    axs[0, 0].plot(original_data['x'], label='Actual X', color=colors['original'])
    axs[0, 0].plot(noisy_data['x'], label='Noisy X', color=colors['noisy'])
    axs[0, 0].plot(filtered_data['x'], label='Ground truth X', color=colors['filtered'])
    axs[0, 0].set_title('X data')
    axs[0, 0].set_xlabel('Index')
    axs[0, 0].set_ylabel('X')
    axs[0, 0].legend()

    # Plot error in X
    axs[0, 1].plot(error_x, label='Error X', color=colors['error'])
    axs[0, 1].set_title('Error in X')
    axs[0, 1].set_xlabel('Index')
    axs[0, 1].set_ylabel('Error X')
    axs[0, 1].legend()

    # Plot Y data
    axs[1, 0].plot(original_data['y'], label='Actual Y', color=colors['original'])
    axs[1, 0].plot(noisy_data['y'], label='Noisy Y', color=colors['noisy'])
    axs[1, 0].plot(filtered_data['y'], label='Ground truth Y', color=colors['filtered'])
    axs[1, 0].set_title('Y data')
    axs[1, 0].set_xlabel('Index')
    axs[1, 0].set_ylabel('Y')
    axs[1, 0].legend()

    # Plot error in Y
    axs[1, 1].plot(error_y, label='Error Y', color=colors['error'])
    axs[1, 1].set_title('Error in Y')
    axs[1, 1].set_xlabel('Index')
    axs[1, 1].set_ylabel('Error Y')
    axs[1, 1].legend()

    # Plot Yaw data
    axs[2, 0].plot(original_data['yaw'], label='Actual Yaw', color=colors['original'])
    axs[2, 0].plot(noisy_data['yaw'], label='Noisy Yaw', color=colors['noisy'])
    axs[2, 0].plot(filtered_data['yaw'], label='Ground truth Yaw', color=colors['filtered'])
    axs[2, 0].set_title('Yaw data')
    axs[2, 0].set_xlabel('Index')
    axs[2, 0].set_ylabel('Yaw')
    axs[2, 0].legend()

    # Plot error in Yaw
    axs[2, 1].plot(error_yaw, label='Error Yaw', color=colors['error'])
    axs[2, 1].set_title('Error in Yaw')
    axs[2, 1].set_xlabel('Index')
    axs[2, 1].set_ylabel('Error Yaw')
    axs[2, 1].legend()
    
    # Add a title to the overall figure
    fig.suptitle('Actual data (Original), Noisy data, Ground truth data (After filtering), and Error')

    # Display the plots
    plt.tight_layout()
    plt.show()

    # Calculate RMS error for each data type
    rms_x = np.sqrt(np.mean(error_x**2))
    rms_y = np.sqrt(np.mean(error_y**2))
    rms_yaw = np.sqrt(np.mean(error_yaw**2))

    print("RMS Error for X data:", rms_x)
    print("RMS Error for Y data:", rms_y)
    print("RMS Error for Yaw data:", rms_yaw)

def main():
    # Define the file paths for original, noisy, and filtered data
    original_file = 'before_anything.csv'
    noisy_file = 'after_noise_adding.csv'
    filtered_file = 'after_filtering.csv'
    
    # Plot the data and calculate RMS error
    plot_data(original_file, noisy_file, filtered_file)

if __name__ == '__main__':
    main()
