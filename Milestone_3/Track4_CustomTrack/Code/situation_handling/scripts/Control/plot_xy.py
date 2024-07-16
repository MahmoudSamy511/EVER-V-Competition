import pandas as pd
import matplotlib.pyplot as plt

def plot_data(original_file, noisy_file, filtered_file, fourth_file):
    # Read the CSV files
    original_data = pd.read_csv(original_file)
    noisy_data = pd.read_csv(noisy_file)
    filtered_data = pd.read_csv(filtered_file)
    fourth_data = pd.read_csv(fourth_file)
    
    # Normalize the column names
    fourth_data.rename(columns={'X': 'x', 'Y': 'y'}, inplace=True)

    # Create a figure with subplots
    fig, axs = plt.subplots(1, 1, figsize=(10, 5))  # Adjust to subplot to one plot for clarity; adjust as necessary

    # Colors for plotting each type of data
    colors = {'original': 'blue', 'noisy': 'orange', 'filtered': 'green', 'fourth': 'red'}

    # Plot X versus Y data, ensure data is in NumPy array format
    axs.plot(original_data['x'].to_numpy(), original_data['y'].to_numpy(), label='Original', color=colors['original'])
    axs.plot(noisy_data['x'].to_numpy(), noisy_data['y'].to_numpy(), label='Noisy', color=colors['noisy'])
    axs.plot(filtered_data['x'].to_numpy(), filtered_data['y'].to_numpy(), label='Filtered', color=colors['filtered'])
    axs.plot(fourth_data['x'].to_numpy(), fourth_data['y'].to_numpy(), label='Fourth', color=colors['fourth'])
    
    axs.set_title('X versus Y Comparison')
    axs.set_xlabel('X')
    axs.set_ylabel('Y')
    axs.legend()

    # Add a title to the overall figure
    fig.suptitle('Comparative Plot of Original, Noisy, Filtered, and Fourth Data')

    # Display the plots
    plt.show()

def main():
    original_file = 'before_anything.csv'
    noisy_file = 'after_noise_adding.csv'
    filtered_file = 'after_filtering.csv'
    fourth_file = 'bigger_infinity_shape_path.csv'
    
    plot_data(original_file, noisy_file, filtered_file, fourth_file)

if __name__ == '__main__':
    main()