import pandas as pd
import matplotlib.pyplot as plt

def plot_data(original_file, noisy_file, filtered_file):
    # Read the CSV files
    original_data = pd.read_csv(original_file)
    noisy_data = pd.read_csv(noisy_file)
    filtered_data = pd.read_csv(filtered_file)
    
    # Create a figure with subplots
    fig, axs = plt.subplots(3, 1, figsize=(10, 15))

    # Colors for plotting each type of data
    colors = {'original': 'blue', 'noisy': 'orange', 'filtered': 'green'}

    # Plot X data
    axs[0].plot(original_data['X'], label='Original X', color=colors['original'])
    axs[0].plot(noisy_data['X'], label='Noisy X', color=colors['noisy'])
    axs[0].plot(filtered_data['X'], label='Filtered X', color=colors['filtered'])
    axs[0].set_title('X data')
    axs[0].set_xlabel('Index')
    axs[0].set_ylabel('X')
    axs[0].legend()

    # Plot Y data
    axs[1].plot(original_data['Y'], label='Original Y', color=colors['original'])
    axs[1].plot(noisy_data['Y'], label='Noisy Y', color=colors['noisy'])
    axs[1].plot(filtered_data['Y'], label='Filtered Y', color=colors['filtered'])
    axs[1].set_title('Y data')
    axs[1].set_xlabel('Index')
    axs[1].set_ylabel('Y')
    axs[1].legend()

    # Plot Yaw data
    axs[2].plot(original_data['Yaw'], label='Original Yaw', color=colors['original'])
    axs[2].plot(noisy_data['Yaw'], label='Noisy Yaw', color=colors['noisy'])
    axs[2].plot(filtered_data['Yaw'], label='Filtered Yaw', color=colors['filtered'])
    axs[2].set_title('Yaw data')
    axs[2].set_xlabel('Index')
    axs[2].set_ylabel('Yaw')
    axs[2].legend()
    
    # Add a title to the overall figure
    fig.suptitle('Original, Noisy, and Filtered Data')
    
    # Display the plots
    plt.tight_layout()
    plt.show()

def main():
    # Define the file paths for original, noisy, and filtered data
    original_file = 'logging/odom_position_data.csv'
    noisy_file = 'noisy_data/odom_noisy_xyz.csv'
    filtered_file = 'filtered_data/filtered_odom_xyz.csv'
    
    # Plot the data
    plot_data(original_file, noisy_file, filtered_file)

if __name__ == '__main__':
    main()
