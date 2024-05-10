import pandas as pd
from pykalman import KalmanFilter

def apply_kalman_filter(data):
    # Initialize Kalman Filter parameters
    initial_state_mean = [data['X'].iloc[0], data['Y'].iloc[0], data['Yaw'].iloc[0]]
    initial_state_covariance = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]  # Initial state covariance matrix
    
    # Observation noise covariance
    observation_covariance = [[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]]
    
    # Process noise covariance
    transition_covariance = [[0.1, 0, 0], [0, 0.1, 0], [0, 0, 0.1]]
    
    # Transition matrices
    transition_matrices = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]  
    
    # Observation matrices
    observation_matrices = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]

    # Create the Kalman Filter
    kf = KalmanFilter(
        initial_state_mean=initial_state_mean,
        initial_state_covariance=initial_state_covariance,
        observation_covariance=observation_covariance,
        transition_covariance=transition_covariance,
        transition_matrices=transition_matrices,
        observation_matrices=observation_matrices
    )

    # Apply Kalman filter to the data
    filtered_state_means, filtered_state_covariances = kf.filter(data[['X', 'Y', 'Yaw']])

    # Create a DataFrame to store the filtered results
    filtered_data = pd.DataFrame(filtered_state_means, columns=['X', 'Y', 'Yaw'])

    return filtered_data

def main():
    # File path of the input data file
    input_file = 'noisy_data/odom_noisy_xyz.csv'
    
    # Read the CSV file
    data = pd.read_csv(input_file)
    
    # Apply Kalman filter to the data
    filtered_data = apply_kalman_filter(data)
    
    # Output file path for the filtered data
    output_file = 'filtered_data/filtered_odom_xyz.csv'
    
    # Save the filtered data to a CSV file
    filtered_data.to_csv(output_file, index=False)
    
    print(f'Filtered data has been saved to {output_file}')

if __name__ == '__main__':
    main()
