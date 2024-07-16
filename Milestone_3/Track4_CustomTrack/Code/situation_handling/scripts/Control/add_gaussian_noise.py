import pandas as pd
import numpy as np

def add_gaussian_noise(data, mean=0, std_dev=0.1):
    """Add Gaussian noise to the data."""
    pos_noise_std_dev = 0.05  # Standard deviation for position noise
    noise = np.random.normal(mean, std_dev, data.shape)
    return data + noise

def main():
    # File path of the input data file
    input_file = 'logging/odom_position_data.csv'
    
    # File path of the output data file (with Gaussian noise added)
    output_file = 'noisy_data/odom_noisy_xyz.csv'
    
    # Read the CSV file
    data = pd.read_csv(input_file)


    # Add Gaussian noise to the X, Y, and Yaw columns
    data['X'] = add_gaussian_noise(data['X'])
    data['Y'] = add_gaussian_noise(data['Y'])
    data['Yaw'] = add_gaussian_noise(data['Yaw'])
    
    # Save the noisy data to a new CSV file
    data.to_csv(output_file, index=False)

if __name__ == '__main__':
    main()
